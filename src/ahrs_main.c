/******************************************************************************
 * FILE: ahrs_main.c
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 *   
 *
 * REVISION: arrange routines to speed up the computational time. Slow down
 *           the Kalman filter update routine at 25Hz while the propagation
 *           is done at 50Hz.
 *
 * LAST REVISED: 7/12/05 Jung Soon Jang
 ******************************************************************************/

#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include "globaldefs.h"
#include "matrix.h"
#include "misc.h"

//prototype definition
void 		AHRS_Algorithm(struct imu *data);
double 		wraparound(double dta);
extern void 	display_message(struct imu *data, int disptime);
extern void 	control_uav(short init_done, short flight_mode);

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//sensor noise characteristics
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//predefined variables
#define		g	9.81		//m/sec^2
#define         g2      19.62   	//2*g
#define		r2d	57.2958         //raidan to degree
#define		d2r     0.01745		//degree to radian
#define         pi      3.141592	
#define         pi2     6.283184	//pi*2

//magnetometer hard-iron calibration
//users need to fill out proper values for their unit
#define		bBy	 0.0     
#define		bBx	 0.0     
#define         sfx      1
#define         sfy      1

//err covariance of accelerometers
#define		var_az  0.45                                 
#define		var_ax	0.45     			
#define         var_ay  0.45                       	


//err covariance of magnetometer heading
#define		var_psi     0.03045725476579   //(10*d2r)^2		
#define         var_psiMag  0.00487387871659   //(4.0*d2r)^2

//sign function
#define         sign(arg) (arg>=0 ? 1:-1)

//global variables
MATRIX aP,aQ,aR,aK,PP,QQ,Fsys,Hj,Iden;
MATRIX xvar,zvar;
MATRIX af,Jcobtr,tmp36,tmp63,tmp33,tmp66,tmpr,tmpr33,Rinv;
MATRIX mat66;

MATRIX RRinv,tmp22;
char   *cnt_status;

//external global variables
extern short screen_on;


void *ahrs_main(void *thread_id)
{
    short  i = 0 /*, j = 0 */;
    int    rc;
    static short control_init =FALSE;
    static short count = 0, enable=FALSE;

#ifndef NCURSE_DISPLAY_OPTION
    printf("[ahrs_main]::thread[%x] initiated...\n", (int)thread_id);
#endif
   
    //initialization of err, measurement, and process cov. matrices
    aP = mat_creat(6,6,ZERO_MATRIX); 
    aQ = mat_creat(6,6,ZERO_MATRIX); for(i=0;i<6;i++) {aQ[i][i] = 1.0e-7; aP[i][i] = 10.0;}  aQ[4][4]=1.0e-13; aQ[5][5]=1.0e-15; // Q = 1.0e-6
    aR = mat_creat(3,3,ZERO_MATRIX); aR[0][0] = var_ax; aR[1][1] = var_ay; aR[2][2] = var_az;
    PP = mat_creat(2,2,ZERO_MATRIX); 
    QQ = mat_creat(2,2,ZERO_MATRIX); for(i=0;i<2;i++) {QQ[i][i] = 1.0e-4; PP[i][i] = 50.0;} //QQ = 1.0e-4
    aK = mat_creat(6,3,ZERO_MATRIX);
    //initialization of state variables
    xvar = mat_creat(6,1,ZERO_MATRIX); xvar[0][0] = 1.0;
    zvar = mat_creat(2,1,ZERO_MATRIX); zvar[0][0] = 0.0; zvar[1][0] = 0.0;
    //initialization of system matrix
    Fsys = mat_creat(6,6,ZERO_MATRIX);
    //initialization of Identity matrix
    Iden = mat_creat(6,6,ZERO_MATRIX); for(i=0;i<6;i++) {Iden[i][i] = 1.0;}
    //initialization of Jacobian matrix
    Hj   = mat_creat(3,6,ZERO_MATRIX);

    //initialization of other matrice used in ahrs
    af    = mat_creat(6,1,ZERO_MATRIX);
    Jcobtr= mat_creat(6,3,ZERO_MATRIX);

    RRinv = mat_creat(2,2,ZERO_MATRIX);
    tmp22 = mat_creat(2,2,ZERO_MATRIX);

    Rinv  = mat_creat(3,3,ZERO_MATRIX);
    tmp36 = mat_creat(3,6,ZERO_MATRIX);
    tmp33 = mat_creat(3,3,ZERO_MATRIX);
    tmp63 = mat_creat(6,3,ZERO_MATRIX);
    tmp66 = mat_creat(6,6,ZERO_MATRIX);
    tmpr  = mat_creat(6,6,ZERO_MATRIX);
    tmpr33= mat_creat(3,3,ZERO_MATRIX);
    mat66 = mat_creat(6,6,ZERO_MATRIX);
   
    sleep(1);
    while (1) {
        //wait until data acquisition is done
        pthread_mutex_lock(&mutex_imu);
        rc  = pthread_cond_wait(&trigger_ahrs, &mutex_imu);
        //run attitude and heading estimation algorithm
        if (rc == 0) { 	   
            AHRS_Algorithm(&imupacket);	   
        }
        pthread_mutex_unlock(&mutex_imu);
        if(!screen_on) snap_time_interval("ahrs",  100, 0);
           
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //control logic: add delay on control trigger to minimize 
        //mode confusion caused by the transmitter power off
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        if (servopacket.chn[4] <= 12000) {
            // if the autopilot is enabled
            enable =  TRUE;  
            count  =  15;
            cnt_status = "MNAV in AutoPilot Mode";
        } else if (servopacket.chn[4] > 12000 && servopacket.chn[4] < 60000) {
            if (count <  0) {
                enable = FALSE;
                control_init = FALSE;
                cnt_status = "MNAV in Manual Mode";
            } else {
                count--;
            }
        }		

        if (enable == TRUE ) {
            control_uav(control_init, 0);
            control_init = TRUE;
        }
    }

    //free memory space
    mat_free(aP);
    mat_free(aQ);
    mat_free(aR);
    mat_free(aK);
    mat_free(PP);
    mat_free(QQ);
    mat_free(xvar);
    mat_free(zvar);
    mat_free(Fsys);
    mat_free(Iden);
    mat_free(Hj);
    mat_free(af);
    mat_free(Jcobtr);
    mat_free(Rinv);
    mat_free(tmp66);
    mat_free(tmp33);
    mat_free(tmp63);
    mat_free(tmp36);
    mat_free(tmpr);
    mat_free(tmpr33);
    mat_free(mat66);

    mat_free(RRinv);
    mat_free(tmp22);
  
    pthread_exit(NULL);
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//extended kalman filter algorithm
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void AHRS_Algorithm(struct imu *data)
{
    static double time=0.;
    static double PPup[2][2],tnow,tprev=0;
    double pc,qc,rc;
    double h[3]={0.,},cPHI,sPHI; 
    double norm,Bxc,Byc,psim, /*invR,*/ diff,diff1,psi_temp;
    double dt,Hdt;
    double Kpsi[2][2],temp[4],det=0;
    double coeff[3]={0,};
    short  i = 0 /*, j = 0 */;
    static short sCheck=1;

    //snap the time interval, dt, of this routine
    tnow = get_Time();
    dt   = tnow - tprev; 
    tprev= tnow;
    if (dt==0) dt = 0.020; 

    Hdt = 0.5*dt;
   
    //take out bias terms
    pc = (data->p - xvar[4][0])*Hdt;  
    qc = (data->q - xvar[5][0])*Hdt;  
    rc = (data->r             )*Hdt;          

    //Fill the system matrix Fsys
    //compute system (or state) transition matrix
    for(i=0;i<6;i++) Fsys[i][i]= 0;
    Fsys[0][1] = -pc; Fsys[0][2] = -qc; Fsys[0][3] = -rc;  
    Fsys[1][0] =  pc; Fsys[1][2] =  rc; Fsys[1][3] = -qc;  
    Fsys[2][0] =  qc; Fsys[2][1] = -rc; Fsys[2][3] =  pc;  
    Fsys[3][0] =  rc; Fsys[3][1] =  qc; Fsys[3][2] = -pc;  
   
    Fsys[0][4] = xvar[1][0]*Hdt; Fsys[0][5] = xvar[2][0]*Hdt;
    Fsys[1][4] =-xvar[0][0]*Hdt; Fsys[1][5] = xvar[3][0]*Hdt;
    Fsys[2][4] =-Fsys[1][5];     Fsys[2][5] = Fsys[1][4];
    Fsys[3][4] = Fsys[0][5];     Fsys[3][5] =-Fsys[0][4];
   
    //mat_mul(Fsys,xvar,af);
    for(i=0;i<4;i++) {
   	af[i][0] = (Fsys[i][0]*xvar[0][0]+Fsys[i][1]*xvar[1][0]
                    +Fsys[i][2]*xvar[2][0]+Fsys[i][3]*xvar[3][0]);
    }
    //af[4]=af[5]=0;	
   
    for(i=0;i<6;i++) Fsys[i][i]+= 1;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //prediction steps
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //propagate state using Euler method
    for(i=0;i<4;i++) {
        xvar[i][0] += af[i][0]; 
    }

    //error covriance propagation: P = Fsys*P*Fsys' + Q
    mat_mul(Fsys,aP,tmp66);
    mat_tran(Fsys,tmpr);
    mat_mul(tmp66,tmpr,aP);
    for(i=0;i<6;i++) aP[i][i] += aQ[i][i];

    coeff[0] = xvar[1][0]*xvar[3][0]-xvar[0][0]*xvar[2][0];
    coeff[1] = xvar[0][0]*xvar[1][0]+xvar[2][0]*xvar[3][0];
    coeff[2] = xvar[0][0]*xvar[0][0]-xvar[1][0]*xvar[1][0]-xvar[2][0]*xvar[2][0]+xvar[3][0]*xvar[3][0];

    if (sCheck) {
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //correction steps
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
        //nonlinear measurement equation of h(x)
      
        h[0]    = -g2*coeff[0];
        h[1]    = -g2*coeff[1];
        h[2]    =  -g*coeff[2];
        
        //compute Jacobian matrix of h(x)
        Hj[0][0] = g2*xvar[2][0]; Hj[0][1] =-g2*xvar[3][0]; Hj[0][2] = g2*xvar[0][0]; Hj[0][3] = -g2*xvar[1][0]; 
        Hj[1][0] =      Hj[0][3]; Hj[1][1] =     -Hj[0][2]; Hj[1][2] =      Hj[0][1]; Hj[1][3] =      -Hj[0][0]; 
        Hj[2][0] =     -Hj[0][2]; Hj[2][1] =     -Hj[0][3]; Hj[2][2] =      Hj[0][0]; Hj[2][3] =       Hj[0][1]; 
   
        //gain matrix aK = aP*Hcobtr*(Hj*aP*Hcobtr + aR)^-1
        mat_tran(Hj,Jcobtr);
        mat_mymul(aP,Jcobtr,tmp63,2);
        mat_mymul(Hj,tmp63,tmp33,2);
        for(i=0;i<3;i++) tmp33[i][i] += aR[i][i];
        mat_inv(tmp33,Rinv);
        mat_mul(tmp63,Rinv,aK);
      
        //state update
        for(i=0;i<6;i++) {
            xvar[i][0] += aK[i][0]*(data->ax - h[0]) + aK[i][1]*(data->ay - h[1]) + aK[i][2]*(data->az - h[2]);
        }
        
        //error covariance matrix update
        // aP = (I - aK*Hj)*aP
        mat_mymul1(aK,Hj,mat66,2); 
        mat_sub(Iden,mat66,tmpr);
        mat_mul(tmpr,aP, tmp66);
        mat_copy(tmp66,aP);
    }
   
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    //scaling of quertonian,||q||^2 = 1
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    norm = 1.0/sqrt(xvar[0][0]*xvar[0][0]+xvar[1][0]*xvar[1][0]+xvar[2][0]*xvar[2][0]+xvar[3][0]*xvar[3][0]);
    for(i=0;i<4;i++) xvar[i][0] = xvar[i][0]*norm;

    //obtain euler angles from quaternion
    data->the = asin(-2*coeff[0]);
    data->phi = atan2(2*coeff[1],coeff[2]);
   
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // second stage kalman filter update to estimate the heading angle
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if ( !sCheck ) {
        //hard-iron calibration
        data->hx = sfx*(data->hx) - bBx;
        data->hy = sfy*(data->hy) - bBy;
   
        //magnetic heading correction due to roll and pitch angle
        cPHI= cos(data->phi);
        sPHI= sin(data->phi);
   
        Bxc = (data->hx)*cos(data->the)+((data->hy)*sPHI+(data->hz)*cPHI)*sin(data->the);
        Byc = (data->hy)*cPHI-(data->hz)*sPHI;

        //heading angles
        psim      = atan2(Byc,-Bxc);
        psi_temp  = atan2(2*(xvar[1][0]*xvar[2][0]+xvar[3][0]*xvar[0][0]),xvar[0][0]*xvar[0][0]+xvar[1][0]*xvar[1][0]-xvar[2][0]*xvar[2][0]-xvar[3][0]*xvar[3][0]);
      
        //error covariance propagation
        PP[0][0] = PPup[0][0] + QQ[0][0];
        PP[1][1] = PPup[1][1] + QQ[1][1];
        PP[0][1] = PPup[0][1];
        PP[1][0] = PPup[1][0];

        //gain update
        tmp22[0][0] = PP[0][0] + var_psiMag;
        tmp22[0][1] = PP[0][0] + time*PP[0][1];
        tmp22[1][0] = PP[0][0] + time*PP[1][0];
        tmp22[1][1] = tmp22[1][0] + time*(PP[0][1] + time*PP[1][1]) + var_psiMag;
        //mat_inv(tmp22,RRinv);
        det = 1.0/(tmp22[0][0]*tmp22[1][1] - tmp22[0][1]*tmp22[1][0]);
        RRinv[0][0] = tmp22[1][1]*det;
        RRinv[1][1] = tmp22[0][0]*det;
        RRinv[0][1] =-tmp22[1][0]*det;
        RRinv[1][0] =-tmp22[0][1]*det;
      
        Kpsi[0][0] = PP[0][0]*RRinv[0][0] + tmp22[0][1]*RRinv[1][0];
        Kpsi[0][1] = PP[0][0]*RRinv[0][1] + tmp22[0][1]*RRinv[1][1];
        temp[0]    = PP[1][0] + time*PP[1][1];
        Kpsi[1][0] = PP[1][0]*RRinv[0][0] + temp[0]*RRinv[1][0];
        Kpsi[1][1] = PP[1][0]*RRinv[0][1] + temp[0]*RRinv[1][1];
      
        //error covariance update
        temp[0]  = 1-Kpsi[0][0]-Kpsi[0][1];
        temp[1]  = Kpsi[0][1]*time;
        temp[2]  =-( Kpsi[1][0]+Kpsi[1][1]);
        temp[3]  = 1-Kpsi[1][1]*time;
   
        PPup[0][0] = temp[0]*PP[0][0]-temp[1]*PP[1][0];    
        PPup[0][1] = temp[0]*PP[0][1]-temp[1]*PP[1][1];
        PPup[1][0] = temp[2]*PP[0][0]+temp[3]*PP[1][0];
        PPup[1][1] = temp[2]*PP[0][1]+temp[3]*PP[1][1];
   
        //state update
        diff = wraparound(psim     - zvar[0][0]);
        diff1= wraparound(psi_temp - zvar[0][0] - time*zvar[1][0]);

        zvar[0][0] += Kpsi[0][0]*diff + Kpsi[0][1]*diff1;
        zvar[1][0] += Kpsi[1][0]*diff + Kpsi[1][1]*diff1;

        //bound heading angle between -180 and 180
        if(zvar[0][0] >  pi) zvar[0][0] -= pi2;
        if(zvar[0][0] < -pi) zvar[0][0] += pi2;

        //heading angle
        data->psi  = zvar[0][0];  
    }

    //time update
    time   = time + dt;
   
    sCheck = !sCheck;

}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// wrap around for -180 and + 180 
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
double wraparound(double dta)
{
    double temp=0;
   
    if (fabs(dta) > pi) {
        temp = dta - sign(dta)*pi2;
        return temp;
    } else {
        return dta;
    }
}

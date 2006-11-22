/******************************************************************************
 * FILE: ahrs.c
 * DESCRIPTION: attitude heading reference system providing the attitude of
 *   	       the vehicle using an extended Kalman filter
 *   
 *
 * REVISION: arrange routines to speed up the computational time. Slow down
 *           the Kalman filter update routine at 25Hz while the propagation
 *           is done at 50Hz.
 *
 * LAST REVISED: 8/31/06 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>

#include "comms/logging.h"
#include "control/control.h"
#include "include/globaldefs.h"
#include "util/matrix.h"
#include "util/timing.h"

#include "util.h"

#include "ahrs.h"
#include "imugps.h"

//prototype definition
void 		AHRS_Algorithm(struct imu *data);
double 		wraparound(double dta);

//predefined variables
#define		g	9.81		//m/sec^2
#define         g2      19.62   	//2*g
#define		r2d	57.2958         //raidan to degree
#define		d2r     0.01745		//degree to radian

// sensor characteristics

// magnetometer hard-iron calibration: users need to fill out proper
// values for their unit if necessary. Use the following link to
// understand how to go about:
// www.ssec.honeywell.com/magnetic/datasheets/amr.pdf
//
// See the top level README file for more information on calibration
//
// default values (no hard iron calibration affects observed, no
// offset, no scaling needed.)
//
// #define bBy 0.0     
// #define bBx 0.0
// #define sfx 1.0
// #define sfy 1.1

// jetta @ passenger feet
#define	bBy 0.65
#define	bBx 0.15
#define sfx (1.0 / 0.500)
#define sfy (1.0 / 0.5)

// err covariance of accelerometers: users must change these values
// depending on the environment under the vehicle is in operation
#define		var_az  0.962361        //(0.1*g)^2
#define		var_ax	0.962361     			
#define         var_ay  0.962361                       	

// err covariance of magnetometer heading
#define		var_psi 0.014924        //(7*d2r)^2		
//sign function
#define         sign(arg) (arg>=0 ? 1:-1)

// global variables
MATRIX aP,aQ,aR,aK,Fsys,Hj,Iden;
MATRIX tmp73,tmp33,tmp77,tmpr,Rinv,mat77;
MATRIX Hpsi,Kpsi,tmp71;
double xs[7]={1,0,0,0,0,0,0};
bool   vgCheck = false;
short  magCheck = 0; 


void *ahrs_thread(void *thread_id)
{
    int    rc;
    bool control_init = false;
    bool enable = false;

#ifndef NCURSE_DISPLAY_OPTION
    if ( display_on ) {
        printf("[ahrs_thread]::thread[%d] initiated...\n", thread_id);
    }
#endif
   
    //initialization of err, measurement, and process cov. matrices
    aP = mat_creat(7,7,ZERO_MATRIX); 
    aQ = mat_creat(7,7,ZERO_MATRIX); 
    aR = mat_creat(3,3,ZERO_MATRIX);
   
    aP[0][0]=aP[1][1]=aP[2][2]=aP[3][3]=1.0e-1; aP[4][4]=aP[5][5]=aP[6][6]=1.0e-1;
    aQ[0][0]=aQ[1][1]=aQ[2][2]=aQ[3][3]=1.0e-8; aQ[4][4]=aQ[5][5]=aQ[6][6]=1.0e-12;
    aR[0][0]=aR[1][1]=aR[2][2]=var_ax;
   
    //initialization of gain matrix
    aK = mat_creat(7,3,ZERO_MATRIX);
    //initialization of state transition matrix
    Fsys = mat_creat(7,7,UNIT_MATRIX);
    //initialization of Identity matrix
    Iden = mat_creat(7,7,UNIT_MATRIX);   
    //initialization of Jacobian matrix
    Hj   = mat_creat(3,7,ZERO_MATRIX);
    //initialization related to heading
    Hpsi  = mat_creat(1,7,ZERO_MATRIX);
    Kpsi  = mat_creat(7,1,ZERO_MATRIX);
    tmp71 = mat_creat(7,1,ZERO_MATRIX);
    //initialization of other matrice used in ahrs
    Rinv  = mat_creat(3,3,ZERO_MATRIX);
    tmp33 = mat_creat(3,3,ZERO_MATRIX);
    tmp73 = mat_creat(7,3,ZERO_MATRIX);
    tmp77 = mat_creat(7,7,ZERO_MATRIX);
    tmpr  = mat_creat(7,7,ZERO_MATRIX);
    mat77 = mat_creat(7,7,ZERO_MATRIX);
   
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
           
        if ( display_on ) snap_time_interval("ahrs",  100, 0);
           
        if ( enable && !vgCheck) {
            control_uav( control_init, 0 );
            control_init = true;
        }

    }

    //free memory space
    mat_free(aP);
    mat_free(aQ);
    mat_free(aR);
    mat_free(aK);
    mat_free(Fsys);
    mat_free(Iden);
    mat_free(Hj);
    mat_free(Rinv);
    mat_free(tmp77);
    mat_free(tmp33);
    mat_free(tmp73);
    mat_free(tmpr);
    mat_free(mat77);
    mat_free(Kpsi);
    mat_free(Hpsi);
    mat_free(tmp71);

    pthread_exit(NULL);
}


//
// extended kalman filter algorithm
//

void AHRS_Algorithm(struct imu *data)
{
    static double tnow,tprev=0;
    double pc,qc,rc;
    double h[3]={0.,},cPHI,sPHI; 
    double norm,Bxc,Byc,invR;
    double dt,Hdt;
    double coeff1[3]={0,},temp[2]={0,};
    short  i=0;

    //snap the time interval, dt, of this routine
    tnow = get_Time();
    dt   = tnow - tprev; 
    tprev= tnow;
    if (dt==0) dt = 0.020; 

    Hdt = 0.5*dt;
   
    /*assign new variables			*/
    pc = (data->p - xs[4])*Hdt;  
    qc = (data->q - xs[5])*Hdt;  
    rc = (data->r - xs[6])*Hdt;  
 
    /*state transition matrix			*/
    Fsys[0][1] = -pc; Fsys[0][2] = -qc; Fsys[0][3] = -rc;  
    Fsys[1][0] =  pc; Fsys[1][2] =  rc; Fsys[1][3] = -qc;  
    Fsys[2][0] =  qc; Fsys[2][1] = -rc; Fsys[2][3] =  pc;  
    Fsys[3][0] =  rc; Fsys[3][1] =  qc; Fsys[3][2] = -pc;  
   
    Fsys[0][4] = xs[1]*Hdt;  Fsys[0][5] = xs[2]*Hdt;  Fsys[0][6] = xs[3]*Hdt;
    Fsys[1][4] =-xs[0]*Hdt;  Fsys[1][5] = xs[3]*Hdt;  Fsys[1][6] =-Fsys[0][5];
    Fsys[2][4] =-Fsys[1][5]; Fsys[2][5] = Fsys[1][4]; Fsys[2][6] = Fsys[0][4];
    Fsys[3][4] = Fsys[0][5]; Fsys[3][5] =-Fsys[0][4]; Fsys[3][6] = Fsys[1][4];
   
   
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    //Extended Kalman filter: prediction step
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    /*propagation of quaternion using gyro measurement
      at a given sampling interval dt                                   */
    xs[0] += -pc*xs[1] - qc*xs[2] - rc*xs[3];
    xs[1] +=  pc*xs[0] - qc*xs[3] + rc*xs[2];
    xs[2] +=  pc*xs[3] + qc*xs[0] - rc*xs[1];
    xs[3] += -pc*xs[2] + qc*xs[1] + rc*xs[0];
   
    //error covriance propagation: P = Fsys*P*Fsys' + Q
    mat_mymul2(Fsys,aP,tmp77,3); 
    mat_mymul3(tmp77,Fsys,aP,3);
    for(i=0;i<7;i++) aP[i][i] += aQ[i][i];

    if (vgCheck) {
        // Pitch and Roll Update at 25 Hz
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //Extended Kalman filter: correction step for pitch and roll
        //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        //nonlinear measurement equation of h(x)
        h[0]    = -g2*(xs[1]*xs[3]-xs[0]*xs[2]);
        h[1]    = -g2*(xs[0]*xs[1]+xs[2]*xs[3]);
        h[2]    =  -g*(xs[0]*xs[0]-xs[1]*xs[1]-xs[2]*xs[2]+xs[3]*xs[3]);
   
        //compute Jacobian matrix of h(x)
        Hj[0][0] = g2*xs[2]; Hj[0][1] =-g2*xs[3]; Hj[0][2] = g2*xs[0]; Hj[0][3] = -g2*xs[1]; 
        Hj[1][0] = Hj[0][3]; Hj[1][1] =-Hj[0][2]; Hj[1][2] = Hj[0][1]; Hj[1][3] = -Hj[0][0]; 
        Hj[2][0] =-Hj[0][2]; Hj[2][1] =-Hj[0][3]; Hj[2][2] = Hj[0][0]; Hj[2][3] =  Hj[0][1]; 

        //gain matrix aK = aP*Hj'*(Hj*aP*Hj' + aR)^-1
        mat_mymul4(aP,Hj,tmp73,3);
        mat_mymul(Hj,tmp73,tmp33,3);
        for(i=0;i<3;i++) tmp33[i][i] += aR[i][i];
        mat_inv(tmp33,Rinv);
        mat_mul(tmp73,Rinv,aK);
      
        //state update
        for(i=0;i<7;i++) {
            xs[i] += aK[i][0]*(data->ax - h[0]) 
                +  aK[i][1]*(data->ay - h[1]) 
                +  aK[i][2]*(data->az - h[2]);
        }
      
        //error covariance matrix update aP = (I - aK*Hj)*aP
        mat_mymul1(aK,Hj,mat77,3); 
        mat_sub(Iden,mat77,tmpr); 
        mat_mymul5(tmpr,aP,tmp77,3);
        mat_copy(tmp77,aP);
    }
   
    if ( ++magCheck == 5 ) {  
        // Heading update at 10 Hz
        if ( vgCheck )
            //avoid both acc and mag updated at the same time:due to
            //computational power
            --magCheck;
        else {	  
            magCheck = 0;	
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            // second stage kalman filter update to estimate the heading angle
            //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            //hard-iron calibration
            // original ... data->hx = sfx*(data->hx) - bBx;
            // original ... data->hy = sfy*(data->hy) - bBy;
            data->hx = sfx*(data->hx - bBx);
            data->hy = sfy*(data->hy - bBy);
   
            //magnetic heading correction due to roll and pitch angle
            cPHI= cos(data->phi);
            sPHI= sin(data->phi);
            Bxc = (data->hx)*cos(data->the)+((data->hy)*sPHI+(data->hz)*cPHI)*sin(data->the);
            Byc = (data->hy)*cPHI-(data->hz)*sPHI;

            //Jacobian
            coeff1[0]= 2*(xs[1]*xs[2]+xs[0]*xs[3]);
            coeff1[1]= 1 - 2*(xs[2]*xs[2]+xs[3]*xs[3]);
            coeff1[2]= 2/(coeff1[0]*coeff1[0]+coeff1[1]*coeff1[1]);
   
            temp[0] = coeff1[1]*coeff1[2];
            temp[1] = coeff1[0]*coeff1[2];
      
            Hpsi[0][0] = xs[3]*temp[0];
            Hpsi[0][1] = xs[2]*temp[0];
            Hpsi[0][2] = xs[1]*temp[0]+2*xs[2]*temp[1];
            Hpsi[0][3] = xs[0]*temp[0]+2*xs[3]*temp[1];
      
            //gain matrix Kpsi = aP*Hpsi'*(Hpsi*aP*Hpsi' + Rpsi)^-1
            mat_mymul4(aP,Hpsi,tmp71,3);
            invR = 1/(Hpsi[0][0]*tmp71[0][0]+Hpsi[0][1]*tmp71[1][0]+Hpsi[0][2]*tmp71[2][0]+Hpsi[0][3]*tmp71[3][0]+var_psi);
            
            //state update
            data->psi = atan2(coeff1[0],coeff1[1]);
            for(i=0;i<7;i++) xs[i] += (invR*tmp71[i][0])*wraparound(atan2(Byc,-Bxc) - data->psi);
      
            //error covariance matrix update aP = (I - Kpsi*Hpsi)*aP
            mat_mymul1(Kpsi,Hpsi,mat77,3); 
            mat_sub(Iden,mat77,tmpr);
            mat_mymul5(tmpr,aP, tmp77,3);
            mat_copy(tmp77,aP);      
        }
    }
   
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
    //scaling of quertonian,||q||^2 = 1
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    norm = 1.0/sqrt(xs[0]*xs[0]+xs[1]*xs[1]+xs[2]*xs[2]+xs[3]*xs[3]);
    for(i=0;i<4;i++) xs[i] = xs[i]*norm;
   
    //obtain euler angles from quaternion
    data->the = asin(-2*(xs[1]*xs[3]-xs[0]*xs[2]));
    data->phi = atan2(2*(xs[0]*xs[1]+xs[2]*xs[3]),1-2*(xs[1]*xs[1]+xs[2]*xs[2]));
    data->psi = atan2(2*(xs[1]*xs[2]+xs[0]*xs[3]),1-2*(xs[2]*xs[2]+xs[3]*xs[3]));
   
    vgCheck = !vgCheck;

}

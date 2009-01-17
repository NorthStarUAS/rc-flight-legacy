/******************************************************************************
 * FILE: decoder.c
 * DESCRIPTION:
 *   
 *   
 *
 * SOURCE: 
 * LAST REVISED: 9/11/05 Jung Soon Jang
 ******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "globaldefs.h"

int main()
{
    struct imu dta;
    struct gps gdta;
    struct nav ndta;
    struct servo sdta;
    struct health hdta;

    // int i = 0;

    FILE *fimu, *fgps, *fnav, *fservo, *fhealth;
    FILE *fwimu, *fwgps, *fwnav, *fwservo, *fwhealth;

    //read file
    if((fimu = fopen("/mnt/cf1/imu.dat","r+b"))==NULL) {
        printf("imu.dat does not exist...error!\n");
        exit(-1);
    }

    if((fgps = fopen("/mnt/cf1/gps.dat","r+b"))==NULL) {
        printf("gps.dat does not exist...error!\n");
        exit(-1);
    }

    if((fnav = fopen("/mnt/cf1/nav.dat","r+b"))==NULL) {
        printf("nav.dat does not exist...error!\n");
        exit(-1);
    }

    if((fservo = fopen("/mnt/cf1/servo.dat","r+b"))==NULL) {
        printf("servo.dat does not exist...error!\n");
        exit(-1);
    }

    if((fhealth = fopen("/mnt/cf1/health.dat","r+b"))==NULL) {
        printf("health.dat does not exist...error!\n");
        exit(-1);
    }

    //create file to dump
    fwimu = fopen("/mnt/cf1/imu.txt","w+t");
    fwgps = fopen("/mnt/cf1/gps.txt","w+t");
    fwnav = fopen("/mnt/cf1/nav.txt","w+t");
    fwservo = fopen("/mnt/cf1/servo.txt","w+t");
    fwhealth = fopen("/mnt/cf1/health.txt","w+t");

    while (!feof(fimu)) {
        fread(&dta,sizeof(struct imu),1,fimu);
        fprintf(fwimu,"%f\t",dta.time);
        fprintf(fwimu,"%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t",
                dta.p,dta.q,dta.r,dta.ax,dta.ay,dta.az);
        fprintf(fwimu,"%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t",
                dta.hx,dta.hy,dta.hz,dta.phi,dta.the,dta.psi);
        fprintf(fwimu,"%6.3f\t%6.3f\t%d\n",
                dta.Ps,dta.Pt,dta.status);
    }

    while (!feof(fgps)) {
        fread(&gdta,sizeof(struct gps),1,fgps);
        fprintf(fwgps,"%f\t%14.10f\t%14.10f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%d\n",
                gdta.time,gdta.lat,gdta.lon,gdta.alt,gdta.vn,gdta.ve,gdta.vd,
                gdta.date);
    }

    while (!feof(fnav)) {
        fread(&ndta,sizeof(struct nav),1,fnav);
        fprintf(fwnav,"%f\t%14.10f\t%14.10f\t%6.3f\t%6.3f\t%6.3f\t%6.3f\t%d\n",
                ndta.time,ndta.lat,ndta.lon,ndta.alt,ndta.vn,ndta.ve,ndta.vd,
                ndta.status);
    }

    while (!feof(fservo)) {
        fread(&sdta,sizeof(struct servo),1,fservo);
        fprintf(fwservo,"%f\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                sdta.time,sdta.chn[0],sdta.chn[1],sdta.chn[2],sdta.chn[3],
                sdta.chn[4],sdta.chn[5],sdta.chn[6],sdta.chn[7],sdta.status);
    }

    while (!feof(fhealth)) {
        fread(&hdta,sizeof(struct health),1,fhealth);
        fprintf(fwhealth,"%f\t%%d\n",
                hdta.time, hdta.command_sequence);
    }


    fclose(fimu);
    fclose(fwimu);
    fclose(fgps);
    fclose(fwgps);
    fclose(fnav);
    fclose(fwnav);
    fclose(fservo);
    fclose(fwservo);
    fclose(fhealth);
    fclose(fwhealth);

    printf("data is now dumped in /mnt/cf1 \n");

    return 0;
}

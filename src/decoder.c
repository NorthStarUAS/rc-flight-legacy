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
    // int i = 0;

    FILE *fimu,*fgps,*fnav, *fwimu, *fwgps, *fwnav;
    /* FIXME: name used, but never set! */
    char *name = NULL;

    //read file
    if((fimu = fopen("/mnt/cf1/imu.dat","r+b"))==NULL) {
        printf("%s does not exist...error!\n",name);
        exit(-1);
    }

    if((fgps = fopen("/mnt/cf1/gps.dat","r+b"))==NULL) {
        printf("%s does not exist...error!\n",name);
        exit(-1);
    }

    if((fnav = fopen("/mnt/cf1/nav.dat","r+b"))==NULL) {
        printf("%s does not exist...error!\n",name);
        exit(-1);
    }

    //create file to dump
    fwimu = fopen("/mnt/cf1/imu.txt","w+t");
    fwgps = fopen("/mnt/cf1/gps.txt","w+t");
    fwnav = fopen("/mnt/cf1/nav.txt","w+t");

    while (!feof(fimu)) {
        fread(&dta,sizeof(struct imu),1,fimu);
        fprintf(fwimu,"%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ",dta.p,dta.q,dta.r,dta.ax,dta.ay,dta.az);
        fprintf(fwimu,"%6.3f %6.3f %6.3f %6.3f %6.3f %6.3f ",dta.hx,dta.hy,dta.hz,dta.phi,dta.the,dta.psi);
        fprintf(fwimu,"%6.3f %6.3f %d %f\n"                 ,dta.Ps,dta.Pt,dta.err_type,dta.time);
    }

    while (!feof(fgps)) {
        fread(&gdta,sizeof(struct gps),1,fgps);
        fprintf(fwgps,"%14.10f %14.10f %6.3f %6.3f %6.3f %6.3f %d %f\n",gdta.lat,gdta.lon,gdta.alt,gdta.vn,gdta.ve,gdta.vd,gdta.ITOW,gdta.time);
    }

    while (!feof(fnav)) {
        fread(&ndta,sizeof(struct nav),1,fnav);
        fprintf(fwnav,"%14.10f %14.10f %6.3f %6.3f %6.3f %6.3f %d %f\n",ndta.lat,ndta.lon,ndta.alt,ndta.vn,ndta.ve,ndta.vd,ndta.err_type,ndta.time);
    }


    fclose(fimu);
    fclose(fwimu);
    fclose(fgps);
    fclose(fwgps);
    fclose(fnav);
    fclose(fwnav);

    printf("data is now dumped in /mnt/cf1 \n");

    return 0;
}

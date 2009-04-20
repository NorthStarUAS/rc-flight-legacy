#ifndef EULERDCM_H
#define EULERDCM_H


int euler2dcm(double *eul, double *dcm);
int dcm2euler(double *dcm, double *eul);

int eulerdiff(double *eul1, double *eul2, double *diff);
int euleradd(double *eul1, double *eul2);


#endif /* EULERDCM_H */

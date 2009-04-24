#ifndef KALMANFILTER_H
#define KALMANFILTER_H


#include "matrix.h"


/*
 * Discrete Kalman Filter
 *
 * Compute Kalmin gain
 * K = PH' * inv(HPH' + R)
 *
 * Update estimate with measurement z
 * x = x + K*Innovationiv
 * 
 *
 * Compute error covariance
 * P = (I - KH)P
 *
 * Project ahead
 * x = phi * x
 * P = phi * P * phi' + Q
 *
 */

typedef struct
{
    int /*bool*/ statefeedback;

    uint n;			/*!< state size n x 1 */
    uint m;			/*!< measurement size m x 1 */	

    ivl_matrix *state;		/*!< state vector(n x 1) */
    ivl_matrix *phi;            /*!< matrix(n x n) relating X(k) to X(k-1) - state transition matrix */
    ivl_matrix *Q;              /*!< covariance matrix(n x n) E[wk wi'] = {Qk for i = k, 0 for i != k */

    ivl_matrix *H;              /*!< matrix(m x n) giving the ideal connextion between measurement and state */
    ivl_matrix *R;              /*!< covariance matrix(n x n) E[vk vi'] = {Rk for i = k, 0 for i != k */

    ivl_matrix *Inn;            /*!< innovation matrix(m x 1) */
    ivl_matrix *K;              /*!< kalman gain */
    ivl_matrix *P;              /*!< matrix(n x n) error covariance */

} KalmanFilter;


KalmanFilter *kalmanfilter_new(uint n, uint m);
void kalmanfilter_free(KalmanFilter **kf);

int kalmanfilter_eval_phi(KalmanFilter *kf, ivl_matrix *F, double dt);/* not used*/
int kalmanfilter_eval_Qphi(KalmanFilter *kf, ivl_matrix *F, ivl_matrix *G, double dt, ivl_matrix *Rw);
int kalmanfilter_project_cov(KalmanFilter *kf);
int kalmanfilter_project_state(KalmanFilter *kf);/*not used */

int kalmanfilter_compute_gain(KalmanFilter *kf);
int kalmanfilter_compute_innovation(KalmanFilter *kf, ivl_matrix *z);/*not used*/
int kalmanfilter_update_cov(KalmanFilter *kf);
int kalmanfilter_update_state(KalmanFilter *kf);


#endif /* KALMANFILTER_H */

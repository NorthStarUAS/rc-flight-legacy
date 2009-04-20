#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "ivl_mem.h"
#include "matrix.h"
#include "kalmanfilter.h"


/*! 
 * \brief Kalman filter
 * 
 * \param n size of state (n x 1) 
 * \param m size of measurement (m x 1)
 * 
 * \return a new allocated KalmanFilter struct
 */
KalmanFilter *kalmanfilter_new(uint n, uint m)
{
    KalmanFilter *kf = NULL;

    kf = (KalmanFilter*)ivl_malloc( sizeof(KalmanFilter) );

    kf->n = n;
    kf->m = m;
    kf->statefeedback = 1 /* true */;

    kf->state = ivl_matrix_new(n,1);
    kf->phi = ivl_matrix_new(n,n);
    kf->Q = ivl_matrix_new(n,n);

    kf->Inn = ivl_matrix_new(m,1);
    kf->H = ivl_matrix_new(m,n);
    kf->R = ivl_matrix_new(m,m);

    kf->K = ivl_matrix_new(n,m);
    kf->P = ivl_matrix_new(n,n);

    return kf;
}

void kalmanfilter_free(KalmanFilter **kf)
{
    ivl_matrix_free( &((*kf)->state) );
    ivl_matrix_free( &((*kf)->phi) );
    ivl_matrix_free( &((*kf)->Q) );
    ivl_matrix_free( &((*kf)->Inn) );
    ivl_matrix_free( &((*kf)->H) );
    ivl_matrix_free( &((*kf)->R) );
    ivl_matrix_free( &((*kf)->K) );
    ivl_matrix_free( &((*kf)->P) );

    ivl_free( *kf );
    *kf = NULL;
}

/*! 
 * \brief Numerically evaluate the state transition matrix
 *
 * PHI = expm( F * dt )
 * 
 * \param kf KalmanFilter structure
 * \param F F matrix
 * \param dt time slice
 * 
 * \return 
 */
int kalmanfilter_eval_phi(KalmanFilter *kf, ivl_matrix *F, double dt)
{
    // return matrix_exp( dt, F, kf->phi );
    return matrix_exp_approx( dt, F, kf->phi );
}


/*! 
 * \brief 
 * 
 %   disrw computes the discrete equivalent of continous noise for the
 %   dynamic system described by
 %
 %                            x_dot = Fx + Gw
 %
 %   w is the driving noise.  Ts is the sampling time and Rwpsd
 %   is the power spectral density of the driving noise.

 A = [ -F | GWG' ] * dt
 [ 0  | F'	]

 B = expm(A) =	[ ... | inv(phi)*Q	]
 [ 0   | phi'		]

 phi = transpose of lower-right partition of B
 Q = phi * (upper right of B)

 *
 * \param kf 
 * \param F 
 * \param G 
 * \param dt 
 * \param Rw 
 */
int kalmanfilter_eval_Qphi( KalmanFilter *kf, ivl_matrix *F, ivl_matrix *G,
			    double dt, ivl_matrix *Rw )
{
    ivl_matrix *A, *B, *C, *D, *E;

    ivl_matrix_view ul;
    ivl_matrix_view ur;
    ivl_matrix_view lr;

    A = ivl_matrix_new( F->nrows * 2, F->ncols * 2 );

    /* upper left = -F */
    ivl_matrix_get_view( A, 0, F->nrows, 0, F->ncols, &ul );
    ivl_matrix_copy( F, &ul );
    ivl_matrix_scalar_mult( &ul, -1.0, NULL );

    /* upper right = GWG' */
    ivl_matrix_get_view( A, 0, F->nrows, F->ncols, F->ncols, &ur);

    C = ivl_matrix_new( G->nrows, Rw->ncols );
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 G->nrows, Rw->ncols, G->ncols,
		 1.0,
		 G->vector, G->ncols,
		 Rw->vector, Rw->ncols,
		 0.0,
		 C->vector,C->ncols);*/
    ivl_matrix_dgemm( 0, 0, 1.0, G, Rw, 0.0, C );

    /* D = C * G', C = GW, G->ncols since G' */
    D = ivl_matrix_new( C->nrows, G->nrows );
    /* cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasTrans,
		 C->nrows, G->nrows, C->ncols,
		 1.0,
		 C->vector, C->ncols,
		 G->vector, G->ncols,
		 0.0,
		 D->vector, D->ncols ); */
    ivl_matrix_dgemm( 0, 1, 1.0, C, G, 0.0, D );

    ivl_matrix_copy( D, &ur );

    /* lower right = F' */
    ivl_matrix_get_view( A, F->nrows, F->nrows, F->ncols, F->ncols, &lr);
    ivl_matrix_copy( F, &lr );
    ivl_matrix_transpose( &lr, NULL );

    B = ivl_matrix_new( A->nrows, A->ncols);
    // matrix_exp(dt,A,B);
    matrix_exp_approx(dt,A,B);

    /* upper right = inv(phi) * Q */
    E = ivl_matrix_new( F->nrows, F->ncols );
    ivl_matrix_get_view( B, 0, F->nrows, E->nrows, E->ncols, &ur);
    ivl_matrix_copy( &ur, E);

    /* lower left = phi' */
    ivl_matrix_get_view( B, F->nrows, F->nrows, kf->phi->nrows, kf->phi->ncols,
			 &lr );
    ivl_matrix_transpose( &lr , kf->phi );

    /* Q = phi * (E) */
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->phi->nrows, E->ncols, kf->phi->ncols,
		 1.0,
		 kf->phi->vector, kf->phi->ncols,
		 E->vector, E->ncols,
		 0.0,
		 kf->Q->vector, kf->Q->ncols );*/
    ivl_matrix_dgemm( 0, 0, 1.0, kf->phi, E, 0.0, kf->Q );
    ivl_matrix_free(&A);
    ivl_matrix_free(&B);
    ivl_matrix_free(&C);
    ivl_matrix_free(&D);
    ivl_matrix_free(&E);

    return 0;
}


/*! 
 * \brief Project error corvariance ahead to next time step
 *
 * P = phi P phi' + Q
 * 
 * \param hf 
 * 
 * \return 
 */
int kalmanfilter_project_cov(KalmanFilter *kf)
{
    ivl_matrix *a;

    a = ivl_matrix_new( kf->phi->nrows, kf->P->ncols );

    /* a = phi * P */
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->phi->nrows, kf->P->ncols, kf->phi->ncols,
		 1.0,
		 kf->phi->vector, kf->phi->ncols,
		 kf->P->vector, kf->P->ncols,
		 0.0,
		 a->vector, a->ncols );*/
    ivl_matrix_dgemm( 0, 0, 1.0, kf->phi, kf->P, 0.0, a );

    /* P = a * phi' + Q */
    /* 1.0 * a * phi' + P, copy Q to P */
    ivl_matrix_copy( kf->Q, kf->P );

    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasTrans,
		 a->nrows, kf->phi->nrows, a->ncols,
		 1.0,
		 a->vector, a->ncols,
		 kf->phi->vector, kf->phi->ncols,
		 1.0,
		 kf->P->vector, kf->P->ncols);*/
    ivl_matrix_dgemm( 0, 1, 1.0, a, kf->phi, 1.0, kf->P );
    
    ivl_matrix_free( &a );

    return 0;
}


/*! 
 * \brief Compute kalman gain for current step
 *
 * K = PH' * inv(HPH' + R)
 * 
 * \param kf 
 * 
 * \return 
 */
int kalmanfilter_compute_gain( KalmanFilter *kf )
{
    ivl_matrix *a,*b,*invb;
    ivl_matrix *c;

    /* a = H*P */
    a = ivl_matrix_new( kf->H->nrows, kf->P->ncols );

    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->H->nrows, kf->P->ncols, kf->H->ncols,
		 1.0,
		 kf->H->vector, kf->H->ncols,
		 kf->P->vector, kf->P->ncols,
		 0.0,
		 a->vector, a->ncols );*/
    ivl_matrix_dgemm( 0, 0, 1.0, kf->H, kf->P, 0.0, a );

    /* b = HP *H' + R = a * H' + R */
    b = ivl_matrix_new( kf->R->nrows, kf->R->ncols );
    ivl_matrix_copy( kf->R, b );

    /*
     * a = m x k
     * kf->H' = k x n -> kf->H = n x k -> k = hk->H->nrows
     * b = m x n
     */
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasTrans,
		 a->nrows, kf->H->nrows, a->ncols,
		 1.0,
		 a->vector, a->ncols,
		 kf->H->vector, kf->H->ncols,
		 1.0,
		 b->vector, b->ncols );*/
    ivl_matrix_dgemm( 0, 1, 1.0, a, kf->H, 1.0, b );

    /* inv( HPH + R ) */
    invb = ivl_matrix_new( b->nrows, b->ncols );
	
    matrix_inv(b, invb);

    /* a = PH' */
    c = ivl_matrix_new( kf->P->nrows, kf->H->nrows );
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasTrans,
		 kf->P->nrows, kf->H->nrows, kf->P->ncols,
		 1.0,
		 kf->P->vector, kf->P->ncols,
		 kf->H->vector, kf->H->ncols,
		 0.0,
		 c->vector, c->ncols);*/
    ivl_matrix_dgemm( 0, 1, 1.0, kf->P, kf->H, 0.0, c );

    /* K = PH' * invb = a * invb */
    /* cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans,
		c->nrows, invb->ncols, kf->K->ncols,
		1.0,
		c->vector, c->ncols,
		invb->vector, invb->ncols,
		0.0,
		kf->K->vector, kf->K->ncols); */
    ivl_matrix_dgemm( 0, 0, 1.0, c, invb, 0.0, kf->K );

    ivl_matrix_free( &a );
    ivl_matrix_free( &b );
    ivl_matrix_free( &invb );
    ivl_matrix_free( &c );

    return 0;
}


/*! 
 * \brief Compute system innovation
 * 
 * z - H*state_estimate
 *
 * \param kf 
 * \param z 
 * 
 * \return 
 */
int kalmanfilter_compute_innovation( KalmanFilter *kf, ivl_matrix *z )
{
    /* Inn = z - H*est = -1.0 * H * est + z */
    ivl_matrix_copy( z, kf->Inn );

    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->H->nrows, kf->state->ncols, kf->H->ncols,
		 -1.0,
		 kf->H->vector, kf->H->ncols,
		 kf->state->vector, kf->state->ncols,
		 1.0,
		 kf->Inn->vector, kf->Inn->ncols );*/
    ivl_matrix_dgemm( 0, 0, -1.0, kf->H, kf->state, 1.0, kf->Inn );

    return 0;
}


/*! 
 * \brief Update state estimate for current step
 * 
 * Xhat = Xhat_minus + K ( Innovation )
 *
 * \param kf 
 * 
 * \return 
 */
int kalmanfilter_update_state( KalmanFilter *kf )
{
    /* state = state + K * Inn */
    ivl_matrix_dgemv( 0, 1.0, kf->K, kf->Inn->vector,
		      kf->statefeedback ? 1.0 : 0.0,
		      kf->state->vector );

    /*    matrix_dgemm(MatrixNoTrans, MatrixNoTrans,*/
    /*            kf->K->nrows,kf->Inn->ncols,kf->K->ncols,*/
    /*            1.0,*/
    /*            kf->K->vector,kf->K->ncols,*/
    /*            kf->Inn->vector,kf->Inn->ncols,*/
    /*            kf->statefeedback ? 1.0 : 0.0,*/
    /*            kf->state->vector,kf->state->ncols);*/

    return 0;
}


/*! 
 * \brief Update error covariance for updated estimate at current time step
 *
 * P = (I - KH)P
 * 
 * \param kf 
 * 
 * \return 
 */
int kalmanfilter_update_cov( KalmanFilter *kf )
{
    ivl_matrix *a, *b;

    a = ivl_matrix_new( kf->K->nrows, kf->H->ncols );

    /* a = I - KH = -1.0 * KH + I */
    ivl_matrix_identity(a);
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->K->nrows, kf->H->ncols, kf->K->ncols,
		 -1.0,
		 kf->K->vector, kf->K->ncols,
		 kf->H->vector, kf->H->ncols,
		 1.0,
		 a->vector, a->ncols );*/
    ivl_matrix_dgemm( 0, 0, -1.0, kf->K, kf->H, 1.0, a );

    /* b = a * P */
    b = ivl_matrix_new( a->nrows, kf->P->ncols );

    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 a->nrows, kf->P->ncols, a->ncols,
		 1.0,
		 a->vector, a->ncols,
		 kf->P->vector, kf->P->ncols,
		 0.0,
		 b->vector, b->ncols );*/
    ivl_matrix_dgemm( 0, 0, 1.0, a, kf->P, 0.0, b );

    ivl_matrix_copy( b, kf->P );

    ivl_matrix_free( &a );
    ivl_matrix_free( &b );

    return 0;
}


/*! 
 * \brief Project state estimate to next time step
 *
 * X = phi * X
 * 
 * \param kf 
 * 
 * \return 
 */
int kalmanfilter_project_state( KalmanFilter *kf )
{
    ivl_matrix *a;

    a = ivl_matrix_new( kf->phi->nrows, kf->state->ncols );

    /* state = phi * state */
    /*cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans,
		 kf->phi->nrows, kf->state->ncols, kf->phi->ncols,
		 1.0,
		 kf->phi->vector, kf->phi->ncols,
		 kf->state->vector, kf->state->ncols,
		 0.0,
		 a->vector, a->ncols );*/
    ivl_matrix_dgemm( 0, 0, 1.0, kf->phi, kf->state, 0.0, a );

    ivl_matrix_copy( a, kf->state );

    ivl_matrix_free( &a );

    return 0;
}


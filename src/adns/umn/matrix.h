#ifndef MATRIX_H
#define MATRIX_H


#include "ivl_matrix.h"

#define MatrixNoTrans 'N'
#define MatrixTrans 'T'
  
int skew_symmetric_3x3(double *v, double *m);
void matrix_mult3x3(int transA, int transB, const double* a, const double* b,
                    double *c);
double matrix_norm_inf(ivl_matrix *m);
int matrix_exp(double a, ivl_matrix *m, ivl_matrix *result);
int matrix_exp_approx( double dt, ivl_matrix *m, ivl_matrix *result );
int matrix_inv(ivl_matrix *m, ivl_matrix *result);

// Compute c = beta*c + alpha*a*b
// note that this software just thinks row major ...
void matrix_dgemm(int transA, int transB, int m, int n, int k,
                  double alpha, double* a, int lda, double* b, int ldb,
                  double beta, double *c, int ldc);

#endif /* MATRIX_H */

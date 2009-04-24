/*! 
 * \file matrix.h
 * \author Bryan Newstrom
 * \date 2006-11-27
 * 
 * \brief interface to a 2d matrix
 */
#ifndef IVL_MATRIX_H
#define IVL_MATRIX_H


enum ivlMatrixType
{
	ivlMATRIX,
	ivlVIEW
};

/*! 
 * \brief simple 2d double matrix
 */
typedef struct ivlMatrix
{
	enum ivlMatrixType type;	/*!< matrix type */
	uint nrows;	/*!< number of rows in matrix */
	uint ncols;	/*!< number of columns in matrix */

	double **matrix;	/*! double[][] interface to matrix */
	double *vector;	/*! the actual allocated matrix as a double array(vector) */

	/* matrix view values */
	struct ivlMatrix *parent;	/*!< matrix that this is a view of */
	uint srow;	/*!< start row of parent */
	uint scol;	/*!< start column of parent */
}ivl_matrix;

ivl_matrix *ivl_matrix_new(uint rows, uint cols);
void ivl_matrix_free( ivl_matrix **m);
ivl_matrix *ivl_matrix_clone(ivl_matrix *m);

/*! 
 * \brief A sub matrix of an existing matrix
 */
typedef ivl_matrix ivl_matrix_view;

int /*bool*/ ivl_matrix_get_view(ivl_matrix *m,
		uint srow, uint nrow, 
		uint scol, uint ncol,
		ivl_matrix_view *v);

double ivl_matrix_get(ivl_matrix *m, uint row, uint col);
void ivl_matrix_set(ivl_matrix *m, uint row, uint col, double v);

ivl_matrix *ivl_matrix_copy(ivl_matrix *m, ivl_matrix *result);

/* math */
ivl_matrix *ivl_matrix_zeros(ivl_matrix *m);
ivl_matrix *ivl_matrix_setall(ivl_matrix *m, double v);
ivl_matrix *ivl_matrix_identity(ivl_matrix *m);

ivl_matrix *ivl_matrix_transpose(ivl_matrix *m, ivl_matrix *result);

ivl_matrix *ivl_matrix_add(ivl_matrix *a, ivl_matrix *b, ivl_matrix *result);
ivl_matrix *ivl_matrix_multiply(ivl_matrix *a, ivl_matrix *b,
				ivl_matrix *result);

ivl_matrix *ivl_matrix_scalar_add(ivl_matrix *m, double scalar,
				  ivl_matrix *result );
ivl_matrix *ivl_matrix_scalar_mult(ivl_matrix *m, double scalar,
				   ivl_matrix *result );
ivl_matrix *ivl_matrix_daxpy(ivl_matrix *x, double scalar, ivl_matrix *y );
void ivl_matrix_dgemv(int /*bool*/ transA, double alpha, ivl_matrix *A,
			     double *x, double beta, double *y );
void ivl_matrix_dgemm( int /*bool*/ transA, int /*bool*/ transB,
		       double alpha, ivl_matrix *A, ivl_matrix *B,
		       double beta, ivl_matrix *C );
void ivl_matrix_print(ivl_matrix *m, int precision, int /*bool*/ sn);


#endif /* IVL_MATRIX_H */

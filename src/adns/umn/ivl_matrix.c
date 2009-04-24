/*! 
 * \file matrix.c
 * \author Bryan Newstrom
 * \date 2006-11-27
 * 
 * \brief implements a simple 2d matrix
 */
#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

#include "ivl_mem.h"
#include "ivl_matrix.h"

#define V(m) ((ivl_matrix_view*)(m))

/*! 
 * \brief Create a new matrix
 * 
 * \param rows number of rows
 * \param cols number of columns
 * 
 * \return pointer to new matrix
 */
ivl_matrix *ivl_matrix_new(uint rows, uint cols)
{
    uint r;
    ivl_matrix *m = NULL;

    m = (ivl_matrix*)ivl_malloc( sizeof(ivl_matrix) );
    m->type = ivlMATRIX;
    m->ncols = cols;
    m->nrows = rows;

    m->matrix = (double**)ivl_malloc(rows * sizeof(double*));
    m->vector = (double*)ivl_malloc( rows * cols * sizeof(double));
	
    for(r = 0; r < rows;r++)
	{
	    m->matrix[r] = &(m->vector)[r * cols];
	}

    return (ivl_matrix*)m;
}

/*! 
 * \brief free memory used in an ivl_matrix
 * 
 * \param *m pointer to ivl_matrix pointer
 */
void ivl_matrix_free( ivl_matrix **m)
{
    if( (*m)->type == ivlMATRIX )
	{
	    ivl_free( (*m)->matrix );
	    ivl_free( (*m)->vector );
	    ivl_free( *m );
	    *m = NULL;
	}
}


/*! 
 * \brief Clone a matrix
 * 
 * \param m matrix to clone
 * 
 * \return clone of matrix
 */
ivl_matrix *ivl_matrix_clone(ivl_matrix *m)
{
    ivl_matrix *clone = NULL;

    clone = ivl_matrix_new( m->nrows, m->ncols);
    ivl_matrix_copy(m,clone);

    return clone;
}


/*! 
 * \brief Create a matrix view
 *
 * Create a view submatrix of a parent matrix.  Must use the get/set functions
 * to access values in a view.
 * 
 * \param m parent matrix
 * \param srow start row of parent
 * \param nrow number of rows in view
 * \param scol start column of paremnt
 * \param ncol number of columns in view
 * \param s pointer to ivl_matrix_view
 * 
 * \return TRUE if view is created, FALSE on error
 */
int /*bool*/ ivl_matrix_get_view(ivl_matrix *m,
				 uint srow, uint nrow, 
				 uint scol, uint ncol,
				 ivl_matrix_view *s)
{
    if( (scol + ncol <= m->ncols) &&
	(srow + nrow <= m->nrows ) )
	{
	    s->type = ivlVIEW;

	    s->srow = srow;
	    s->scol = scol;

	    s->nrows = nrow;
	    s->ncols = ncol;
	    s->matrix = NULL;
	    s->vector = NULL;

	    s->parent = m;

	    return 1 /*true*/;
	}

    return 0 /*false*/;
}


/*! 
 * \brief Get a value from a matrix
 * 
 * \param m pointer to matrix (matrix or view)
 * \param row row of value
 * \param col column of value
 * 
 * \return value at (row,col)
 */
double ivl_matrix_get(ivl_matrix *m, uint row, uint col)
{
    if( m->type == ivlVIEW )
	{
	    return ivl_matrix_get( V(m)->parent,
				   V(m)->srow + row,
				   V(m)->scol + col);
	}

    return m->matrix[row][col];
}

/*! 
 * \brief Set a value in a matrix
 * 
 * \param m pointer to matrix (matrix or view)
 * \param row row of value
 * \param col column of value
 * \param v value to set at (row,col)
 */
void ivl_matrix_set(ivl_matrix *m, uint row, uint col, double v)
{
    if( m->type == ivlVIEW )
	{
	    ivl_matrix_set( V(m)->parent, 
			    V(m)->srow + row,
			    V(m)->scol + col,
			    v);
	    return;
	}

    m->matrix[row][col] = v;
}


/*! 
 * \brief Copy a matrix
 * 
 * \param from pointer to source matrix
 * \param to pointer to dest matrix
 * 
 * \return pointer to dest matrix
 */
ivl_matrix *ivl_matrix_copy(ivl_matrix *from, ivl_matrix *to)
{	
    uint r,c;

    if( from->nrows != to->nrows || from->ncols != to->ncols )
	{
	    printf("matrix copy - matrices are not the same size\n");
	    return NULL;
	}

    if( from->type == ivlMATRIX && to->type == ivlMATRIX )
	{
	    memcpy( to->vector, from->vector, to->nrows * to->ncols * sizeof(double) );
	}
    else
	{
	    for(r=0;r < from->nrows; r++)
		{
		    for(c=0;c<from->ncols; c++)
			{
			    ivl_matrix_set(to,r,c, ivl_matrix_get(from,r,c) );
			}
		}

	}

    return to;
}

/*! 
 * \brief Set all values of a matrix to zero
 * 
 * \param m pointer to matrix
 * 
 * \return pointer to input matrix 
 */
ivl_matrix *ivl_matrix_zeros(ivl_matrix *m)
{
    uint r,c;

    if( m->type == ivlVIEW )
	{
	    for(r=0;r<m->nrows;r++)
		for(c=0;c<m->ncols;c++)
		    ivl_matrix_set(m,r,c,0.0);
	}
    else
	memset( m->vector, 0, m->nrows * m->ncols * sizeof(double) );

    return m;
}

/*! 
 * \brief Set all values of a matrix to a value
 * 
 * \param m pointer to matrix
 * \param v value to set all matrix element
 * 
 * \return pointer to input matrix
 */
ivl_matrix *ivl_matrix_setall(ivl_matrix *m, double v)
{
    uint r,c;

    for(r=0;r<m->nrows;r++)
	for(c=0;c<m->ncols;c++)
	    ivl_matrix_set(m,r,c,v);

    return m;
}

/*! 
 * \brief Set matrix to idenity
 * 
 * \param m pointer to matrix
 * 
 * \return pointer to input matrix
 */
ivl_matrix *ivl_matrix_identity(ivl_matrix *m)
{
    uint s,i;

    if( m->ncols < m->nrows )
	s = m->ncols;
    else
	s = m->nrows;

    ivl_matrix_zeros(m);

    for(i = 0; i < s;i++)
	{	
	    ivl_matrix_set(m,i,i,1.0);
	}	

    return m;

}


/*! 
 * \brief transpose a matrix
 *
 * Computes the transpose of a matrix. Number of rows in input must equal number of columns in output,
 * and number of columns in input must equal number of rows in output.  Will transpose square matrices in
 * place by setting m = result or result = NULL.
 * 
 * \param m pointer to matrix to tranpose
 * \param result pointer to matrix to hold result (NULL for inplace transpose)
 * 
 * \return pointer to result matrix
 */
ivl_matrix *ivl_matrix_transpose(ivl_matrix *m, ivl_matrix *result)
{
    uint r,c;
    double t;

    if( m == result || result == NULL )	{
	/* do transpose in place */
	if( m->nrows != m->ncols ) {
	    printf("matrix transpose: input rows(%d) != output columns(%d) (inplace - matrix not square)\n", m->nrows, m->ncols);
	    return NULL;
	}

	for( r = 1; r < m->nrows; r++) {
	    for(c = 0; c < r; c++) {
		t = ivl_matrix_get(m,r,c);
		ivl_matrix_set(m,r,c, ivl_matrix_get(m,c,r) );
		ivl_matrix_set(m,c,r,t);
	    }
	}

	return m;
    } else {
	if( m->nrows != result->ncols )	{
	    printf("matrix transpose: input rows(%d) != output columns(%d)\n",m->nrows, result->ncols);
	    return NULL;
	}

	if( m->ncols != result->nrows )	{
	    printf("matrix transpose: input columns(%d) != output rows(%d)\n",m->nrows, result->ncols);
	    return NULL;
	}

	for( r = 0; r < m->nrows; r++) {
	    for( c = 0; c < m->ncols; c++) {
		ivl_matrix_set(result,c,r, ivl_matrix_get(m,r,c) );
	    }
	}

	return result;
    }

    return NULL;
}

/*! 
 * \brief Add 2 matrices
 *
 * Adds 2 matrices together
 *
 * result = a + b
 *
 * for inplace 
 *
 * a = a + b
 * 
 * \param a first matrix
 * \param b second matrix
 * \param result matrix to hold results (null or a for inplace)
 * 
 * \return pointer to result
 */
ivl_matrix *ivl_matrix_add(ivl_matrix *a, ivl_matrix *b, ivl_matrix *result)
{
    uint r,c;

    if(a->nrows != b->nrows || a->ncols != b->ncols)
	{
	    printf("matrix add: matrices must be the same size: a[%dx%d] != b[%dx%d]\n",
		   a->nrows,a->ncols,
		   b->nrows,b->ncols
		   );
	    return NULL;
	}

    if( result == NULL || result == a )
	{
	    /* do addition in place */
	    for(r=0; r < a->nrows; r++)
		for(c=0; c < a->ncols;c++)
		    {
			ivl_matrix_set(a,r,c, 
				       ivl_matrix_get(a,r,c) + ivl_matrix_get(b,r,c) 
				       );
		    }

	    return a;
	}
    else
	{
	    if(a->nrows != result->nrows || a->ncols != result->ncols)
		{
		    printf("matrix add: matrices must be the same size: a[%dx%d] != result[%dx%d]\n",
			   a->nrows,a->ncols,
			   result->nrows,result->ncols
			   );
		    return NULL;
		}

	    for(r=0; r < result->nrows; r++)
		{
		    for(c=0; c < result->ncols; c++)
			{
			    ivl_matrix_set(result,r,c, 
					   ivl_matrix_get(a,r,c) + ivl_matrix_get(b,r,c) 
					   );
			}
		}

	    return result;
	}

    return NULL;
}

/*! 
 * \brief Multiply 2 matrices together
 *
 * Multiply 2 matrices together
 *
 * [a] * [b] = result
 * 
 * \param a first matrix
 * \param b second matrix
 * \param result matrix to hold result
 * 
 * \return result matrix
 */
ivl_matrix *ivl_matrix_multiply(ivl_matrix *a, ivl_matrix *b,ivl_matrix *result)
{
    uint r, c, i;
    double v;

    if (a->ncols != b->nrows) 
	{
	    printf("Number of Columns in a(%d) does not equal number of rows in b(%d)\n"
		   ,a->ncols
		   ,b->nrows
		   );
	    return NULL;
  	}
    r = c = i = 0;
    ivl_matrix_zeros(result);

    for(c=0; c<result->ncols; c++)
    	for(r=0; r<result->nrows; r++)
	    for(i=0; i<a->ncols; i++)         // which == b->rows
		{
		    v = ivl_matrix_get(result,r,c);
		    v += ivl_matrix_get(a,r,i) * ivl_matrix_get(b,i,c);
		    ivl_matrix_set(result,r,c,v);
		}

    return result;
}


/*! 
 * \brief Add a scalar value to a matrix
 *
 * \param m pointer to matrix
 * \param scalar scalar value
 * \param result pointer to matrix to hold result(NULL or a to do math in place)
 * 
 * \return pointer to input matrix
 */
ivl_matrix *ivl_matrix_scalar_add(ivl_matrix *m, double scalar, ivl_matrix *result)
{	
    uint r,c;

    if( result == NULL || result == m )
	{
	    for(r = 0; r < m->nrows; r++)
		{
		    for(c = 0; c < m->ncols; c++)
			{
			    ivl_matrix_set(m,r,c, ivl_matrix_get(m,r,c) + scalar );
			}
		}

	    return m;
	}
    else
	{
	    if(m->nrows != result->nrows || m->ncols != result->ncols)
		{
		    printf("matrix scalar add: matrices must be the same size: m[%dx%d] != result[%dx%d]\n",
			   m->nrows,m->ncols,
			   result->nrows,result->ncols
			   );
		    return NULL;
		}

	    for(r = 0; r < m->nrows; r++)
		{
		    for(c = 0; c < m->ncols; c++)
			{
			    ivl_matrix_set(result,r,c, ivl_matrix_get(m,r,c) + scalar );
			}
		}

	    return result;
	}

    return NULL;
}

/*! 
 * \brief Multiply a matrix by a scalar value
 *
 * \param m pointer to matrix
 * \param scalar scalar value
 * \param result result matrix, if NULL will do in place
 * 
 * \return pointer to result matrix, input matrix if result is NULL
 */
ivl_matrix *ivl_matrix_scalar_mult( ivl_matrix *m, double scalar,
				    ivl_matrix *result )
{	
    uint r,c;

    if ( result == NULL || result == m ) {
      for ( r = 0; r < m->nrows; r++ ) {
	for ( c = 0; c < m->ncols; c++ ) {
	  ivl_matrix_set(m, r, c, ivl_matrix_get(m,r,c) * scalar );
	}
      }

      return m;
    } else {
      if ( m->nrows != result->nrows || m->ncols != result->ncols ) {
	printf("matrix scalar mult: matrices must be the same size: m[%dx%d] != result[%dx%d]\n",
	       m->nrows,m->ncols,
	       result->nrows,result->ncols
	       );
	return NULL;
      }

      for ( r = 0; r < m->nrows; r++ ) {
	for ( c = 0; c < m->ncols; c++ ) {
	  ivl_matrix_set(result, r, c, ivl_matrix_get(m,r,c) * scalar );
	}
      }

      return result;
    }

    return NULL;
}

/*! 
 * \brief y += alpha * x
 *
 * \param m pointer to matrix
 * \param scalar scalar value
 * \param result result matrix, if NULL will do in place
 * 
 * \return pointer to result matrix, input matrix if result is NULL
 */
ivl_matrix *ivl_matrix_daxpy(ivl_matrix *x, double scalar, ivl_matrix *y )
{	
    int i, size;
    double *yv = y->vector;
    double *xv = x->vector;

    if ( x->nrows != y->nrows || x->ncols != y->ncols ) {
	printf("matrix scalar mult: matrices must be the same size: m[%dx%d] != result[%dx%d]\n",
	       x->nrows, x->ncols,
	       y->nrows, y->ncols
	       );
	return NULL;
    }

    size = x->nrows * x->ncols;
    for ( i = 0; i < size; ++i ) {
	yv[i] += scalar * xv[i];
    }

    return y;
}


/*! 
 * \brief y = alpha*A*x + beta*y
 * \brief y = alpha*A'*x + beta*y
 *
 * \param m pointer to matrix
 * \param scalar scalar value
 * \param result result matrix, if NULL will do in place
 * 
 * \return pointer to result matrix, input matrix if result is NULL
 */
void ivl_matrix_dgemv( int /*bool*/ transA, double alpha, ivl_matrix *A,
		       double *x, double beta, double *y )
{	
    uint r,c;
    uint nrows, ncols;
    double accum = 0.0;

    nrows = A->nrows;
    ncols = A->ncols;

    if ( !transA ) {
	if ( beta != 0 ) {
	    for ( r = 0; r < nrows; r++ ) {
		accum = 0.0;
		for ( c = 0; c < ncols; c++ ) {
		    // accum += ivl_matrix_get(A,r,c) * x[c];
		    // accum += A->matrix[r][c] * x[c];
		    accum += A->vector[r*ncols + c] * x[c];
		}
		y[r] = alpha * accum + beta * y[r];
	    }
	} else {
	    for ( r = 0; r < nrows; r++ ) {
		accum = 0.0;
		for ( c = 0; c < ncols; c++ ) {
		    // accum += ivl_matrix_get(A,r,c) * x[c];
		    // accum += A->matrix[r][c] * x[c];
		    accum += A->vector[r*ncols + c] * x[c];
		}
		y[r] = alpha * accum;
	    }
	}
    } else {
	if ( beta != 0 ) {
	    for ( r = 0; r < nrows; r++ ) {
		accum = 0.0;
		for ( c = 0; c < ncols; c++ ) {
		    // accum += ivl_matrix_get(A,c,r) * x[c];
		    accum += A->vector[c*ncols + r] * x[c];
		}
		y[r] = alpha * accum + beta * y[r];
	    }
	} else {
	    for ( r = 0; r < nrows; r++ ) {
		accum = 0.0;
		for ( c = 0; c < ncols; c++ ) {
		    // accum += ivl_matrix_get(A,c,r) * x[c];
		    accum += A->vector[c*ncols + r] * x[c];
		}
		y[r] = alpha * accum;
	    }
	}
    }
}


/*! 
 * \brief C = alpha*A*B + beta*C
 * \brief A and B can be specified as transposes
 *
 * \return pointer to result matrix, input matrix if result is NULL
 */
void ivl_matrix_dgemm( int /*bool*/ transA, int /*bool*/ transB,
		       double alpha, ivl_matrix *A, ivl_matrix *B,
		       double beta, ivl_matrix *C )
{	
    int r, c, k, i;
    int nrows, ncols, kcols, krows;
    double accum = 0.0;

    nrows = A->nrows;
    kcols = A->ncols;
    if ( transB ) {
	ncols = B->nrows;
	krows = B->ncols;
    } else {
	ncols = B->ncols;
	krows = B->nrows;
    }
    double *av = A->vector;
    double *bv = B->vector;
    double *cv = C->vector;

    // printf("%d x %d\n", nrows, ncols);
    // printf("%d =?= %d\n", A->ncols, B->nrows);

    if ( !transA && !transB && beta == 0.0 ) {
	for ( r = 0; r < nrows; ++r ) {
	    for ( c = 0; c < ncols; ++c ) {
		accum = 0.0;
		for ( k = 0; k < kcols; ++k ) {
		    // printf("A[%d][%d] * B[%d][%d]\n", r, k, k, c);
		    accum += av[r*kcols + k] * bv[k*ncols+ c];
		}
		cv[r*ncols + c] = alpha * accum;
	    }
	}
    } else if (!transA && !transB && beta != 0.0 ) {
	for ( r = 0; r < nrows; ++r ) {
	    for ( c = 0; c < ncols; ++c ) {
		accum = 0.0;
		for ( k = 0; k < kcols; ++k ) {
		    // printf("A[%d][%d] * B[%d][%d]\n", r, k, k, c);
		    accum += av[r*kcols + k] * bv[k*ncols+ c];
		}
		i = r*ncols + c;
		cv[i] = alpha * accum + beta * cv[i];
	    }
	}
    } else if ( !transA && transB && beta == 0.0 ) {
	for ( r = 0; r < nrows; ++r ) {
	    for ( c = 0; c < ncols; ++c ) {
		accum = 0.0;
		for ( k = 0; k < kcols; ++k ) {
		    // printf("A[%d][%d] * B[%d][%d]\n", r, k, k, c);
		    accum += av[r*kcols + k] * bv[c*krows + k];
		}
		cv[r*ncols + c] = alpha * accum;
	    }
	}
    } else if (!transA && transB && beta != 0.0 ) {
	for ( r = 0; r < nrows; ++r ) {
	    for ( c = 0; c < ncols; ++c ) {
		accum = 0.0;
		for ( k = 0; k < kcols; ++k ) {
		    // printf("A[%d][%d] * B[%d][%d]\n", r, k, k, c);
		    accum += av[r*kcols + k] * bv[c*krows + k];
		}
		i = r*ncols + c;
		cv[i] = alpha * accum + beta * cv[i];
	    }
	}
    }
}


/*! 
 * \brief Print a matrix to stdout
 * 
 * \param m pointer to matrix
 * \param precision printf precision, -1 use default
 * \param sn true to use E instead of f for printf
 */
void ivl_matrix_print(ivl_matrix *m, int precision, int /*bool*/ sn)
{
    int r,c;
    int w;
    int maxwidth;
    char *format;

    if( precision == -1 )
	precision = 6;

    printf("-- %s, nrows=%d, ncols=%d --\n"
	   ,m->type == ivlMATRIX ? "matrix" : "matrix view"
	   ,m->nrows
	   ,m->ncols
	   );

    /* determine max width of all matrix values */
    maxwidth = 0;

    if( sn )
	format = "%.*E";
    else
	format = "%.*f";

    for(r = 0; r < m->nrows; r++)
	{
	    for(c = 0; c < m->ncols; c++)
		{
		    w = snprintf(NULL,0,format,precision, ivl_matrix_get(m,r,c));

		    if( w > maxwidth )
			maxwidth = w;
		}
	}
    maxwidth += 1;

    if( sn )
	format = "%*.*E";
    else
	format = "%*.*f";

    for(r = 0; r < m->nrows; r++)
	{
	    for(c = 0; c < m->ncols; c++)
		{
		    printf(format,maxwidth,precision, ivl_matrix_get(m,r,c) );
		}
	    printf("\n");
	}
}

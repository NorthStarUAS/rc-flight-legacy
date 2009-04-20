/*! 
 * \file mem.c
 * \author Bryan Newstrom
 * \date 2005-01-24
 *
 * \brief safe maloc, realloc and free
 */
#ifdef HAVE_CONFIG_H
#include "include/ugear_config.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ivl_mem.h"


/*! 
 * \brief allocate memory
 * Allocate memory or exit program on malloc error. The memory will be set
 * to zero. Also see malloc.
 * 
 * \param size number of byte to allocate
 * 
 * \return pointer to allocated memory
 */
void *ivl_malloc(size_t size)
{
    void *p = NULL;

    p = (void *)malloc(size);

    if ( p == NULL )
	printf("ivl_malloc() error: memory exhausted\n");

    memset( p, 0, size );

    return p;
}


/*! 
 * \brief free allocated memory
 * Free allocated memory.  Will not cause error if based a NULL value. Also see
 * free.
 * 
 * \param mem pointer to memory to free
 */
void ivl_free(void *mem)
{
    if ( mem != NULL )
	free( mem );
}



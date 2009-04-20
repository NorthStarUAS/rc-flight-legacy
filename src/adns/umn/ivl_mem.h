/*! 
 * \file mem.h
 * \author Bryan Newstrom
 * \date 2005-01-24
 *
 * \brief safe memory allocation functions
 */
#ifndef IVL_MEM_H
#define IVL_MEM_H


void *ivl_malloc(size_t size);
void ivl_free(void *mem);


#endif /* IVL_MEM_H */

/* rs-matrix-transpose.c --- in-place matrix transposition.

   Copyright (C) 2014 Ralph Schleicher

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in
        the documentation and/or other materials provided with the
        distribution.

      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.  */

/* Commentary:

   Compile with best optimization.  Otherwise, there is a big speed
   penalty.  For GCC, the optimization option -O3 is recommended.  */


#if HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#if HAVE_STDINT_H
#include <stdint.h>
#endif /* HAVE_STDINT_H */
#if HAVE_INTTYPES_H
#include <inttypes.h>
#endif /* HAVE_INTTYPES_H */

#ifndef SIZE_MAX
#define SIZE_MAX ((size_t) -1)
#endif

#include "rs-matrix.h"

/* Transposition permutation for mapping linear indices of matrix
   elements from the old (original) storage location KO to the new
   (transposed) storage location KN.

        kn = P(ko, m, n)

   The original matrix has M rows and N columns.  The inverse
   permutation is calculated via

        ko = P(kn, n, m).  */
#define P(k,m,n) (((k) % (n)) * (m) + (k) / (n))

/* Return the greatest common divisor of the two numbers A and B.  */
static size_t
gcd (size_t a, size_t b)
{
  size_t u[3], v[3], q, t[3];

  u[0] = 1;
  u[1] = 0;
  u[2] = a;

  v[0] = 0;
  v[1] = 1;
  v[2] = b;

  while (v[2] != 0)
    {
      q = u[2] / v[2];

      t[0] = u[0] - v[0] * q;
      t[1] = u[1] - v[1] * q;
      t[2] = u[2] - v[2] * q;

      memcpy (u, v, sizeof (v));
      memcpy (v, t, sizeof (t));
    }

  return u[2];
}

/* In-place matrix transposition.

   First argument A is the address of a memory block containing a
    two-dimensional array, that is a matrix.  The matrix elements
    have to be stored continuously in either row-major (like C) or
    column-major (like Fortran) order.
   Second argument M is the number of matrix rows.
   Third argument N is the number of matrix columns.
   Fourth argument S is the size of a matrix element.

   Return value is A on success, that means the elements of the
   matrix are transposed.  In case of an error, a null pointer is
   returned and 'errno' is set to describe the error.  */
void *
rs_matrix_transpose (void *a, size_t m, size_t n, size_t s)
{
  char temp[16], *t, *v;
  size_t mn;

  /* There is nothing to do when matrix A is either empty,
     a row vector, or a column vector.  */
  if (m <= 1 || n <= 1 || s == 0)
    return a;

  /* Total number of elements.  */
  if (m > SIZE_MAX / n)
    {
      errno = EINVAL;
      return NULL;
    }

  mn = m * n;

  /* Total array size.  */
  if (mn > SIZE_MAX / s)
    {
      errno = EINVAL;
      return NULL;
    }

  /* Check pointer arithmetic.  */
  v = (char *) a;

  t = v + mn * s;
  if (t < v)
    {
      errno = EINVAL;
      return NULL;
    }

  /* Allocate swap space.  */
  if (s > sizeof (temp))
    {
      t = calloc (1, s);
      if (t == NULL)
	return NULL;
    }
  else
    t = temp;

  /* Transpose matrix A.  */
  if (m == n)
    {
      /* Matrix A is square.  */
      char *d, *p, *q;
      size_t sn, i, j;

      /* Row increment.  */
      sn = s * n;

      /* First diagonal element.  */
      d = v;

      for (i = 1; i < m; ++i)
	{
	  p = d + sn;
	  q = d + s;

	  for (j = i; j < n; ++j)
	    {
	      memcpy (t, p, s);
	      memcpy (p, q, s);
	      memcpy (q, t, s);

	      p += sn;
	      q += s;
	    }

	  d += sn;
	  d += s;
	}
    }
  else
    {
      /* Matrix A is rectangular.  */
      size_t l, c, i, j, k;
      char *p, *q;

      /* Linear index of the last array element.  */
      l = mn - 1;

      /* Number of elements to be moved, that is
	 M * N - (1 + gcd (M - 1, N - 1)).  */
      c = l - gcd (m - 1, n - 1);

      /* Loop over all possible start indices for a permutation cycle.
	 The first and last array element will never be moved.

	 Mnemonic: J is the start index, K is the next index in the
	 permutation cycle, and I is the previous index.  */
      for (j = 1; c > 0 /* && j < l */; )
	{
	  /* Check if J is the start index of an unprocessed cycle.  Do
	     so from both directions, that means from start to end and
	     from end to start.  */
	  k = P (j, m, n);
	  if (k <= j)
	    /* This is either a 1-cycle (K = J) or an already processed
	       cycle (K < J).  */
	    goto next;

	  i = P (j, n, m);
	  if (i <= j)
	    /* Likewise.  */
	    goto next;

	  /* At this point K and I are greater than J, If K is equal
	     to I, then this is an unprocessed 2-cycle, i.e. a simple
	     swap of two array elements.  */
	  if (i == k)
	    {
	      p = v + j * s;
	      q = v + i * s;

	      memcpy (t, p, s);
	      memcpy (p, q, s);
	      memcpy (q, t, s);

	      c -= 2;

	      goto next;
	    }

	  /* Check the other indices of the permutation cycle.  K and I
	     either meet in the middle or at the other end.  */
	  do
	    {
	      k = P (k, m, n);
	      if (k < j)
		goto next;

	      i = P (i, n, m);
	      if (i < j)
		goto next;
	    }
	  while (k != i);

	  /* At this point J is the start index of an unprocessed
	     permutation cycle, i.e. the linear index of the first
	     and last element of the cycle.

	     Save last (first) element of the cycle.  */
	  memcpy (t, v + j * s, s);

	  /* Loop over the cycle in reverse order.  */
	  for (k = j; ; k = i)
	    {
	      i = P (k, n, m);
	      if (i == j)
		break;

	      /* Move previous element to location of next element.  */
	      memcpy (v + k * s, v + i * s, s); --c;
	    }

	  /* Move first element to the new location.  */
	  memcpy (v + k * s, t, s); --c;

	next:

	  ++j;
	}
    }

  if (t != temp)
    free (t);

  return a;
}

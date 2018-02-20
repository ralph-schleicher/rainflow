/* rs-rainflow.c --- rainflow cycle counting.

   Copyright (C) 2010 Ralph Schleicher

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

#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <float.h>
#include <math.h>

#ifdef _MSC_VER
/* Microsoft Visual C.  */
#if _MSC_VER >= 1600
#include <stdint.h>
#else /* _MSC_VER < 1600 */
typedef signed __int8 int8_t;
typedef unsigned __int8 uint8_t;
typedef signed __int16 int16_t;
typedef unsigned __int16 uint16_t;
typedef signed __int32 int32_t;
typedef unsigned __int32 uint32_t;
typedef signed __int64 int64_t;
typedef unsigned __int64 uint64_t;
#endif /* _MSC_VER < 1600 */
#else /* not _MSC_VER */
#ifdef __LCC__
#if CHAR_MAX == 127
typedef signed char int8_t;
typedef unsigned char uint8_t;
#else /* CHAR_MAX != 127 */
#error "Fix me"
#endif /* CHAR_MAX != 127 */
#if SHRT_MAX == 32767
typedef signed short int int16_t;
typedef unsigned short int uint16_t;
#else /* SHRT_MAX != 32767 */
#error "Fix me"
#endif /* SHRT_MAX != 32767 */
#if INT_MAX == 2147483647
typedef signed int int32_t;
typedef unsigned int uint32_t;
#else /* INT_MAX != 2147483647 */
#error "Fix me"
#endif /* INT_MAX != 2147483647 */
typedef signed __int64 int64_t;
typedef unsigned __int64 uint64_t;
#else /* not __LCC__  */
#include <stdint.h>
#endif /* not __LCC__  */
#endif /* not _MSC_VER */

#include "rs-rainflow.h"

#ifdef __GNUC__
#define static_inline static inline
#else /* not __GNUC__ */
#ifdef _MSC_VER
#define static_inline static __inline
#else /* not _MSC_VER */
#define static_inline static
#endif /* not _MSC_VER */
#endif /* not __GNUC__ */

#ifndef SIZE_MAX
#define SIZE_MAX ((size_t) -1)
#endif

#ifdef _WIN32
#define SIZEOF_PAGE 4096
#else /* not _WIN32 */
#include <unistd.h>
#define SIZEOF_PAGE (getpagesize ())
#endif /* not _WIN32 */

/* Rainflow cycle counting object data structure.  */
struct rs_rainflow
  {
    /* Latest cycle.

       First and second element represent the cycle and the third
       element is the cycle count (i.e. the number of half cycles).
       Optional fourth and fifth element are the signal labels.

       Address is only valid if CYCLE_LEN is greater than zero.  */
    double *cycle;

    /* Base address of the cycle counting sequence.  */
    double *cycle_base;

    /* Allocated number of cycles.  */
    size_t cycle_count;

    /* Number of cycles to be added iff the cycle counting sequence has
       to grow.  */
    size_t cycle_add;

    /* Zero-based index of the first unshifted cycle.  */
    size_t cycle_offs;

    /* Number of unshifted cycles.  The latest cycle can't be shifted
       while counting cycles.  */
    size_t cycle_len;

    /* Number of elements in a cycle (either three of five).  */
    size_t cycle_elem;

    /* Size of a cycle in byte.  */
    size_t cycle_size;

    /* Maximum number of cycles that can be allocated.  */
    size_t cycle_max;

    /* Memory allocation hints for the cycle counting sequence.  */
    size_t cycle_count_hint;
    size_t cycle_add_hint;

    /* Stack pointer.

       First element is the stack top.

       Address is only valid if STACK_LEN is greater than zero.  */
    double *stack;

    /* Base address of the signal value stack.  */
    double *stack_base;

    /* Allocated number of signal values.  */
    size_t stack_count;

    /* Number of signal values on the stack.  */
    size_t stack_len;

    /* Last local extremum in the signal history.

       First element is the signal value and optional second element is
       the corresponding signal label.

       Value is only valid if flag FIRST is non-zero.  */
    double last_extremum[2];

    /* Signal history pointer.

       Address is only valid if SIG_LEN is greater than zero.  */
    char *sig;

    /* Number of elements in the signal history.  */
    size_t sig_len;

    /* Size of an element in the signal history.  */
    size_t sig_incr;

    /* Function to read signal values.

       First argument is the signal history pointer.
       Second argument is the destination address.
       Third argument is the number of signal values to read.  */
    /* Fourth argument is the signal index of the first signal
       value.  */
    size_t (*sig_fun1) (void *, double *, size_t);
    size_t (*sig_fun2) (void *, double *, size_t, double);

    /* Buffer for signal values.

       First element is the last unprocessed signal value
       if flag LAST is non-zero.  */
    double *sig_buf;

    /* Allocated buffer size.  */
    size_t sig_count;

    /* Number of elements in a signal value (either one or two).  */
    size_t sig_elem;

    /* Size of a signal value in byte.  */
    size_t sig_size;

    /* Maximum number of signal values that can be allocated.  */
    size_t sig_max;

    /* Continuous signal index.

       Used as the built-in signal label.  */
    double sig_index;

    /* Function to be run when a cycle can be shifted.

       First argment is a user-supplied value.
       Second argument is the cycle.  */
    void (*shift_fun) (void *, double const *);

    /* First argument of the SHIFT_FUN function.  */
    void *shift_arg;

    /* Memory allocation functions.  */
    void *(*malloc_fun) (size_t);
    void *(*realloc_fun) (void *, size_t);
    void (*free_fun) (void *);

    /* Cycle representation, i.e. the meaning of the first and second
       element of a cycle.

       RS_RAINFLOW_AMPLITUDE_MEAN
            First element is the signal amplitude and the second element
            is the signal mean.

       RS_RAINFLOW_RANGE_MEAN
            First element is the signal range, i.e. two times the signal
            amplitude, and the second element is the signal mean.

       RS_RAINFLOW_FROM_TO
            First element is the ‘from’ signal value and the second
            element is the ‘to’ signal value, i.e. the extrema values
            of the cycle in chronological order.  */
    int style;

    /* Non-zero means that rainflow counting has started.  */
    int busy:2;

    /* Non-zero means that the first signal value is processed.  */
    int first:1;

    /* Non-zero means that the last signal value is unprocessed.  */
    int last:1;

    /* Non-zero means to label signal values.  */
    int label:1;

    /* Non-zero means to retain the sign of the signal amplitude
       or signal range.  */
    int sign:1;

    /* Non-zero means to merge contiguous cycles.  */
    int merge:1;
  };

/* Relocate latest cycle.  */
static_inline void
relocate_cycle (rs_rainflow_t *obj)
{
  obj->cycle = obj->cycle_base;
  if (obj->cycle != NULL)
    obj->cycle += (obj->cycle_offs + obj->cycle_len - 1) * obj->cycle_elem;
}

/* Relocate stack pointer.  */
static_inline void
relocate_stack (rs_rainflow_t *obj)
{
  obj->stack = obj->stack_base;
  if (obj->stack != NULL)
    obj->stack += (obj->stack_len - 1) * obj->sig_elem;
}

/* Processing steps.  */
enum
  {
    SETUP = 0,
    UPDATE,
    FINISH
  };

/* Wrapper functions for different array element data types.  */
#define def(S,T)							\
/* Read signal values.  */						\
static size_t								\
read_##S (T const *y, double *v, size_t c)				\
{									\
  size_t n;								\
									\
  for (n = c; n > 0; --n)						\
    *v++ = *y++;							\
									\
  return c;								\
}									\
									\
static size_t								\
read_##S##_null (T const *y, double *v, size_t c, double t)		\
{									\
  size_t n;								\
									\
  for (n = c; n > 0; --n)						\
    {									\
      *v++ = *y++;							\
      *v++ = t++;							\
    }									\
									\
  return c;								\
}

def (double, double);
def (float, float);
def (char, signed char);
def (uchar, unsigned char);
def (short, signed short int);
def (ushort, unsigned short int);
def (int, signed int);
def (uint, unsigned int);
def (long, signed long int);
def (ulong, unsigned long int);
def (int8, int8_t);
def (uint8, uint8_t);
def (int16, int16_t);
def (uint16, uint16_t);
def (int32, int32_t);
def (uint32, uint32_t);
def (int64, int64_t);
def (uint64, uint64_t);

/* Array element information.  */
struct elem
  {
    /* Data type identifier.  */
    int type;

    /* Size of an array element.  */
    size_t incr;

    /* Function to copy signal values, without and with labeling of
       signal values.  */
    void *fun1;
    void *fun2;
  };

static struct elem const elem_tab[RS_RAINFLOW_TYPES] =
  {
    {
      RS_RAINFLOW_TYPE_UNKNOWN,
      0,
      NULL,
      NULL,
    },
    {
      RS_RAINFLOW_TYPE_DOUBLE,
      sizeof (double),
      read_double,
      read_double_null,
    },
    {
      RS_RAINFLOW_TYPE_FLOAT,
      sizeof (float),
      read_float,
      read_float_null,
    },
    {
      RS_RAINFLOW_TYPE_CHAR,
      sizeof (signed char),
      read_char,
      read_char_null,
    },
    {
      RS_RAINFLOW_TYPE_UCHAR,
      sizeof (unsigned char),
      read_uchar,
      read_uchar_null,
    },
    {
      RS_RAINFLOW_TYPE_SHORT,
      sizeof (signed short int),
      read_short,
      read_short_null,
    },
    {
      RS_RAINFLOW_TYPE_USHORT,
      sizeof (unsigned short int),
      read_ushort,
      read_ushort_null,
    },
    {
      RS_RAINFLOW_TYPE_INT,
      sizeof (signed int),
      read_int,
      read_int_null,
    },
    {
      RS_RAINFLOW_TYPE_UINT,
      sizeof (unsigned int),
      read_uint,
      read_uint_null,
    },
    {
      RS_RAINFLOW_TYPE_LONG,
      sizeof (signed long int),
      read_long,
      read_long_null,
    },
    {
      RS_RAINFLOW_TYPE_ULONG,
      sizeof (unsigned long int),
      read_ulong,
      read_ulong_null,
    },
    {
      RS_RAINFLOW_TYPE_INT8_T,
      sizeof (int8_t),
      read_int8,
      read_int8_null,
    },
    {
      RS_RAINFLOW_TYPE_UINT8_T,
      sizeof (uint8_t),
      read_uint8,
      read_uint8_null,
    },
    {
      RS_RAINFLOW_TYPE_INT16_T,
      sizeof (int16_t),
      read_int16,
      read_int16_null,
    },
    {
      RS_RAINFLOW_TYPE_UINT16_T,
      sizeof (uint16_t),
      read_uint16,
      read_uint16_null,
    },
    {
      RS_RAINFLOW_TYPE_INT32_T,
      sizeof (int32_t),
      read_int32,
      read_int32_null,
    },
    {
      RS_RAINFLOW_TYPE_UINT32_T,
      sizeof (uint32_t),
      read_uint32,
      read_uint32_null,
    },
    {
      RS_RAINFLOW_TYPE_INT64_T,
      sizeof (int64_t),
      read_int64,
      read_int64_null,
    },
    {
      RS_RAINFLOW_TYPE_UINT64_T,
      sizeof (uint64_t),
      read_uint64,
      read_uint64_null,
    },
  };

/* Allocate a memory block.

   Argument SIZE is greater than zero.  */
static_inline void *
call_malloc (rs_rainflow_t *obj, size_t size)
{
  if (obj->malloc_fun == NULL)
    abort ();

  return obj->malloc_fun (size);
}

/* Resize a memory block.

   Argument ADDR is not a null pointer.
   Argument SIZE is greater than zero.  */
static_inline void *
call_realloc (rs_rainflow_t *obj, void *addr, size_t size)
{
  if (obj->realloc_fun == NULL)
    abort ();

  return obj->realloc_fun (addr, size);
}

/* Release a memory block.

   Argument ADDR is not a null pointer.  */
static_inline void
call_free (rs_rainflow_t *obj, void *addr)
{
  if (obj->free_fun != NULL)
    obj->free_fun (addr);
}

/* Set default values except for the memory allocation functions.
   The later are initialized once after object creation.  */
static void
init (rs_rainflow_t *obj)
{
  obj->cycle = NULL;
  obj->cycle_base = NULL;
  obj->cycle_count = 0;
  obj->cycle_add = 0;
  obj->cycle_offs = 0;
  obj->cycle_len = 0;
  obj->cycle_elem = 3;
  obj->cycle_size = obj->cycle_elem * sizeof (double);
  obj->cycle_max = SIZE_MAX / obj->cycle_size;

  obj->cycle_count_hint = 0;
  obj->cycle_add_hint = 0;

  obj->stack = NULL;
  obj->stack_base = NULL;
  obj->stack_count = 0;
  obj->stack_len = 0;

  obj->last_extremum[0] = 0.0;
  obj->last_extremum[1] = 0.0;

  obj->sig = NULL;
  obj->sig_len = 0;
  obj->sig_incr = sizeof (double);
  obj->sig_fun1 = (void *) read_double;
  obj->sig_fun2 = (void *) read_double_null;
  obj->sig_buf = NULL;
  obj->sig_count = 0;
  obj->sig_elem = 1;
  obj->sig_size = obj->sig_elem * sizeof (double);
  obj->sig_max = SIZE_MAX / obj->sig_size;
  obj->sig_index = 0;

  obj->shift_fun = NULL;
  obj->shift_arg = NULL;

  obj->style = RS_RAINFLOW_AMPLITUDE_MEAN;

  obj->busy = SETUP;
  obj->first = 0;
  obj->last = 0;
  obj->label = 0;
  obj->sign = 0;
  obj->merge = 1;
}

/* Reset default values.  */
static void
reset (rs_rainflow_t *obj)
{
  if (obj->cycle_base != NULL)
    call_free (obj, obj->cycle_base);

  if (obj->stack_base != NULL)
    call_free (obj, obj->stack_base);

  if (obj->sig_buf != NULL)
    call_free (obj, obj->sig_buf);

  init (obj);
}

/* Clear state variables.  */
static void
clear (rs_rainflow_t *obj)
{
  obj->cycle_offs = 0;
  obj->cycle_len = 0;
  relocate_cycle (obj);

  obj->stack_len = 0;
  relocate_stack (obj);

  obj->sig_len = 0;
  obj->sig_index = 0;

  obj->busy = SETUP;
  obj->first = 0;
  obj->last = 0;
}

/* Initialize cycle counting, i.e. allocate buffers.  */
static int
setup (rs_rainflow_t *obj, size_t sig_len)
{
  double *cycle_base;
  size_t cycle_count;
  size_t cycle_add;
  double *stack_base;
  size_t stack_count;
  double *sig_buf;
  size_t sig_count;
  size_t size;

  /* Fix number of elements in a cycle.  */
  obj->cycle_elem = (obj->label == 0 ? 3 : 5);
  obj->cycle_size = obj->cycle_elem * sizeof (double);
  obj->cycle_max = SIZE_MAX / obj->cycle_size;

  /* Fix number of elements in a signal value.  */
  obj->sig_elem = (obj->label == 0 ? 1 : 2);
  obj->sig_size = obj->sig_elem * sizeof (double);
  obj->sig_max = SIZE_MAX / obj->sig_size;

  /* Determine buffer size for the cycle counting sequence.  */
  if (obj->shift_fun != NULL)
    {
      /* The shift function consumes a cycle as soon as it is shifted.
	 That means we have to cache not more than one cycle and the
	 buffer never has to grow.  It also means that the shift
	 function can't be cleared during cycle counting.  */
      cycle_count = 1;
      cycle_add = 0;
    }
  else
    {
      /* Evaluate user hints.  */
      cycle_count = obj->cycle_count_hint;
      cycle_add = obj->cycle_add_hint;

      if (cycle_count == 0)
	cycle_count = (sig_len != 0 && sig_len != SIZE_MAX ? sig_len :
		       (cycle_add != 0 ? cycle_add :
			8192)); /* 192 kiB (without labeling) */

      if (cycle_count < 2)
	cycle_count = 2;
      if (cycle_count > obj->cycle_max)
	cycle_count = obj->cycle_max;

      if (cycle_add == 0)
	cycle_add = cycle_count;

      if (cycle_add < 1)
	cycle_add = 1;
      if (cycle_add > obj->cycle_max)
	cycle_add = obj->cycle_max;
    }

  /* Largest stack size encountered so far is 21.  */
  stack_count = 32;

  /* Determine buffer size for the signal values.
     Reserve 16 byte for memory manager overhead. */
  sig_count = (SIZEOF_PAGE - 16) / obj->sig_size;

  /* Allocate buffers.  */
  size = cycle_count * obj->cycle_size;
  cycle_base = (obj->cycle_base != NULL ?
		call_realloc (obj, obj->cycle_base, size) :
		call_malloc (obj, size));

  if (cycle_base != NULL)
    {
      obj->cycle_base = cycle_base;
      obj->cycle_count = cycle_count;
      obj->cycle_add = cycle_add;
    }

  size = stack_count * obj->sig_size;
  stack_base = (obj->stack_base != NULL ?
		call_realloc (obj, obj->stack_base, size) :
		call_malloc (obj, size));

  if (stack_base != NULL)
    {
      obj->stack_base = stack_base;
      obj->stack_count = stack_count;
    }

  size = sig_count * obj->sig_size;
  sig_buf = (obj->sig_buf != NULL ?
	     call_realloc (obj, obj->sig_buf, size) :
	     call_malloc (obj, size));

  if (sig_buf != NULL)
    {
      obj->sig_buf = sig_buf;
      obj->sig_count = sig_count;
    }

  /* Clear state variables.  */
  clear (obj);

  if (cycle_base == NULL || stack_base == NULL || sig_buf == NULL)
    {
      errno = ENOMEM;
      return -1;
    }

  return 0;
}

static_inline int
add_cycle (rs_rainflow_t *obj, double const *to, double const *from, int count)
{
  double cycle[2];
  double label[2];

  if (obj->style == RS_RAINFLOW_FROM_TO)
    {
      cycle[0] = *from;
      cycle[1] = *to;
    }
  else
    {
      double ampl, mean;

      /* Signal amplitude.

	 If the signal amplitude is too small, the original trough and
	 peak signal value can't be calculated exactly from the signal
	 amplitude and mean value.  */
      ampl = fabs (to[0] - from[0]) / 2.0;
      if (ampl < (DBL_EPSILON / 2.0))
	return 0;

      /* Mean value.  */
      mean = (to[0] + from[0]) / 2.0;

      cycle[0] = ampl;
      cycle[1] = mean;

      if (obj->style == RS_RAINFLOW_RANGE_MEAN)
	cycle[0] *= 2.0;

      if (obj->sign != 0 && to[0] < from[0])
	cycle[0] = - cycle[0];
    }

  /* Save signal labels in chronological order of the signal values.  */
  if (obj->label != 0)
    {
      memcpy (label + 0, from + 1, sizeof (double));
      memcpy (label + 1, to + 1, sizeof (double));
    }

  /* Check for repeating cycle.

     Cycles are equal if the signal amplitude and mean value are equal
     and if the signal labels are equal.  */
  if (obj->merge != 0
      && obj->cycle_len > 0
      && obj->cycle[0] == cycle[0]
      && obj->cycle[1] == cycle[1]
      && (obj->label == 0
	  || memcmp (obj->cycle + 3, label, 2 * sizeof (double)) == 0))
    {
      /* Yes, increment cycle count.  */
      obj->cycle[2] += count;
    }
  else
    {
      /* No, create a new cycle.  */
      if (obj->shift_fun != NULL)
	{
	  /* Shift latest cycle.  */
	  if (obj->cycle_len > 0)
	    obj->shift_fun (obj->shift_arg, obj->cycle);

	  /* Set number of cycles.  */
	  obj->cycle_len = 1;

	  /* Storage location of the new cycle.  */
	  obj->cycle = obj->cycle_base;
	}
      else
	{
	  /* Check for room.  */
	  if (obj->cycle_offs + obj->cycle_len == obj->cycle_count)
	    {
	      if (obj->cycle_offs > 0)
		{
		  /* Reuse shifted cycles.  */
		  memmove (obj->cycle_base,
			   obj->cycle_base + obj->cycle_offs * obj->cycle_elem,
			   obj->cycle_len * obj->cycle_size);

		  obj->cycle_offs = 0;
		}
	      else
		{
		  size_t c;
		  void *p;

		  /* Enlarge buffer.  */
		  if (obj->cycle_count == obj->cycle_max)
		    return RS_RAINFLOW_ERROR_CYCLE_OVERFLOW;

		  /* New length.  */
		  c = (obj->cycle_count <= obj->cycle_max - obj->cycle_add ?
		       obj->cycle_count + obj->cycle_add :
		       obj->cycle_max);

		  p = call_realloc (obj, obj->cycle_base, c * obj->cycle_size);
		  if (p == NULL)
		    return -1;

		  obj->cycle_base = p;
		  obj->cycle_count = c;
		}

	      relocate_cycle (obj);
	    }

	  /* Increase number of cycles.  */
	  ++obj->cycle_len;

	  /* Storage location of the new cycle.  */
	  obj->cycle += obj->cycle_elem;
	}

      /* Save cycle.  */
      obj->cycle[0] = cycle[0];
      obj->cycle[1] = cycle[1];
      obj->cycle[2] = count;

      if (obj->label != 0)
	memcpy (obj->cycle + 3, label, 2 * sizeof (double));
    }

  return 0;
}

/* Push a signal value on the stack.  */
#define push_stack(obj,y)						\
do									\
  {									\
    /* Check for room.  */						\
    if (obj->stack_len == obj->stack_count)				\
      {									\
	size_t c;							\
	void *p;							\
									\
	/* Enlarge buffer.  */						\
	if (obj->stack_count == obj->sig_max)				\
	  return RS_RAINFLOW_ERROR_STACK_OVERFLOW;			\
									\
	/* New length.  */						\
	c = (obj->stack_count <= obj->sig_max / 2 ?			\
	     obj->stack_count * 2 :					\
	     obj->sig_max);						\
									\
	p = call_realloc (obj, obj->stack_base, c * obj->sig_size);	\
	if (p == NULL)							\
	  return -1;							\
									\
	obj->stack_base = p;						\
	obj->stack_count = c;						\
	relocate_stack (obj);						\
      }									\
									\
    ++obj->stack_len;							\
    obj->stack += obj->sig_elem;					\
    memcpy (obj->stack, y, obj->sig_size);				\
  }									\
while (0)

/* Create a rainflow cycle counting object.  */
rs_rainflow_t *
rs_rainflow_new (void)
{
  rs_rainflow_t *obj;

  obj = malloc (sizeof (rs_rainflow_t));
  if (obj != NULL)
    {
      init (obj);

      obj->malloc_fun = malloc;
      obj->realloc_fun = realloc;
      obj->free_fun = free;
    }

  return obj;
}

/* Create a rainflow cycle counting object
   using an alternative memory manager.  */
rs_rainflow_t *
rs_rainflow_alloc (void *(*malloc_fun) (size_t), void *(*realloc_fun) (void *, size_t), void (*free_fun) (void *))
{
  rs_rainflow_t *obj;

  if (malloc_fun == NULL || realloc_fun == NULL)
    {
      errno = EINVAL;
      return NULL;
    }

  obj = malloc_fun (sizeof (rs_rainflow_t));
  if (obj != NULL)
    {
      init (obj);

      obj->malloc_fun = malloc_fun;
      obj->realloc_fun = realloc_fun;
      obj->free_fun = free_fun;
    }

  return obj;
}

/* Destroy a rainflow cycle counting object.  */
void
rs_rainflow_delete (rs_rainflow_t *obj)
{
  if (obj == NULL)
    return;

  reset (obj);

  /* Release myself.  */
  call_free (obj, obj);
}

/* Reset a rainflow cycle counting object.  */
int
rs_rainflow_reset (rs_rainflow_t *obj)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  reset (obj);

  return 0;
}

/* Clear a rainflow cycle counting object.  */
int
rs_rainflow_clear (rs_rainflow_t *obj)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  clear (obj);

  return 0;
}

/* Provide hints for the memory manager.

   First argument OBJ is a rainflow cycle counting object.
   Second argument LEN is the initial number of elements.
   Third argument ADD is the number of elements to be added
    iff a buffer has to grow.  */
int
rs_rainflow_set_length (rs_rainflow_t *obj, size_t len, size_t add)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  obj->cycle_count_hint = len;
  obj->cycle_add_hint = add;

  return 0;
}

/* Customize the array element type.

   Default is a double precision floating-point number.  */
int
rs_rainflow_set_signal_type (rs_rainflow_t *obj, int type)
{
  struct elem const *elem;

  if (obj == NULL || type < RS_RAINFLOW_TYPE_UNKNOWN || type >= RS_RAINFLOW_TYPES)
    {
      errno = EINVAL;
      return -1;
    }

  /* Resolve element type.  */
  elem = elem_tab + type;
  if (elem->type != type)
    abort ();

  obj->sig_incr = elem->incr;
  obj->sig_fun1 = elem->fun1;
  obj->sig_fun2 = elem->fun2;

  return 0;
}

/* Customize the signal history access function.  */
int
rs_rainflow_set_read_signals (rs_rainflow_t *obj, size_t (*fun) (void *, double *, size_t), size_t incr)
{
  if (obj == NULL || (fun == NULL && incr > 0))
    {
      errno = EINVAL;
      return -1;
    }

  obj->sig_incr = incr;
  obj->sig_fun1 = fun;
  obj->sig_fun2 = NULL;

  return 0;
}

/* Customize the cycle shift function.

   Default is to cache shifted cycles.  */
int
rs_rainflow_set_shift_cycle (rs_rainflow_t *obj, void (*fun) (void *, double const *), void *arg)
{
  if (obj == NULL || (fun == NULL && arg != NULL))
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  obj->shift_fun = fun;
  obj->shift_arg = arg;

  return 0;
}

/* Customize labeling of signal values.  */
int
rs_rainflow_set_signal_label (rs_rainflow_t *obj, int label)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  obj->label = (label != 0 ? 1 : 0);

  return 0;
}

int
rs_rainflow_set_signal_index (rs_rainflow_t *obj, double index)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  /* TODO: Check for loss of precision.  */
  if (fmod (index, 1.0) != 0.0)
    {
      errno = EDOM;
      return -1;
    }

  obj->sig_index = index;

  return 0;
}

/* Customize merging of similar cycles.  */
int
rs_rainflow_set_merge_cycles (rs_rainflow_t *obj, int merge)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  obj->merge = (merge != 0 ? 1 : 0);

  return 0;
}

/* Customize the cycle representation.  */
int
rs_rainflow_set_cycle_style (rs_rainflow_t *obj, int style)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  if (style < 0 || style >= RS_RAINFLOW_CYCLE_REPRESENTATIONS)
    {
      errno = EINVAL;
      return -1;
    }

  obj->style = style;

  return 0;
}

/* Customize signed cycles.  */
int
rs_rainflow_set_cycle_sign (rs_rainflow_t *obj, int flag)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy != SETUP)
    {
      errno = EBUSY;
      return -1;
    }

  obj->sign = (flag != 0 ? 1 : 0);

  return 0;
}

/* Perform rainflow cycle counting.  */
static_inline int
inner_loop (rs_rainflow_t *obj)
{
  double *st0, *st1, *st2;
  int retval;

  while (obj->stack_len > 3)
    {
      st0 = obj->stack;
      st1 = st0 - obj->sig_elem;
      st2 = st1 - obj->sig_elem;

      if (fabs (*st0 - *st1) < fabs (*st1 - *st2))
	break;

      /* Full cycle.  */
      retval = add_cycle (obj, st1, st2, 2);
      if (retval != 0)
	return retval;

      /* Drop ST1 and ST2.  */
      memcpy (st2, st0, obj->sig_size);

      obj->stack_len -= 2;
      obj->stack = st2;
    }

  while (obj->stack_len == 3)
    {
      st0 = obj->stack;
      st1 = st0 - obj->sig_elem;
      st2 = st1 - obj->sig_elem;

      if (fabs (*st0 - *st1) < fabs (*st1 - *st2))
	break;

      /* Half cycle.  */
      retval = add_cycle (obj, st1, st2, 1);
      if (retval != 0)
	return retval;

      /* Drop ST2.  */
      memcpy (st2, st1, obj->sig_size);
      memcpy (st1, st0, obj->sig_size);

      --obj->stack_len;
      obj->stack = st1;
    }

  return 0;
}

/* Outer loop, i.e. push local extrema on the stack.  */
static_inline int
outer_loop (rs_rainflow_t *obj)
{
  int retval;
  double *y0, *y1, *y2;
  double *y, *buffer;
  size_t c, n, count;

  /* Read signal values.  */
  y = obj->sig_buf;
  c = 0;

  buffer = obj->sig_buf;
  count = obj->sig_count;

  if (obj->last != 0)
    {
      obj->last = 0;

      buffer += obj->sig_elem;
      --count;

      ++c;
    }

  memset (buffer, 0, count * obj->sig_size);

  if (count > obj->sig_len)
    count = obj->sig_len;

  n = (obj->label == 0 || obj->sig_fun2 == NULL ?
       obj->sig_fun1 (obj->sig, buffer, count) :
       obj->sig_fun2 (obj->sig, buffer, count, obj->sig_index));
  if (n == 0 && count != 0)
    {
      /* At end of file.  */
      obj->sig_len = 0;
    }
  else if (obj->sig_len != SIZE_MAX)
    obj->sig_len -= n;

  if (obj->sig_incr != 0)
    obj->sig += n * obj->sig_incr;

  if (obj->label != 0)
    obj->sig_index += n;

  /* Process signal values.  */
  c += n;

  if (c == 0)
    return 0;

  if (obj->first == 0)
    {
      obj->first = 1;

      /* First signal value is a local extremum.  */
      memcpy (obj->last_extremum, y, obj->sig_size);

      y += obj->sig_elem;
      --c;

      /* Add signal value to the stack.  */
      push_stack (obj, obj->last_extremum);
    }

  if (c == 0)
    return 0;

  while (c > 1)
    {
      y0 = obj->last_extremum;
      y1 = y;
      y2 = y + obj->sig_elem;

      /* Drop Y1.  */
      y += obj->sig_elem;
      --c;

      /* Check for hold or intermediate value.  */
      if (fabs (*y1 - *y0) < DBL_EPSILON
	  || fabs (*y2 - *y1) < DBL_EPSILON
	  || (*y0 < *y1 && *y1 < *y2)
	  || (*y0 > *y1 && *y1 > *y2))
	continue;

      /* Found local extremum.  */
      memcpy (obj->last_extremum, y1, obj->sig_size);

      /* Add signal value to the stack.  */
      push_stack (obj, obj->last_extremum);

      /* Inner loop.  */
      retval = inner_loop (obj);
      if (retval != 0)
	return retval;
    }

  /* Save remaining value.  */
  memcpy (obj->sig_buf, y, obj->sig_size);
  obj->last = 1;

  return 0;
}

int
rs_rainflow (rs_rainflow_t *obj, void *sig, size_t sig_len, int finish)
{
  int retval;
  double *y0, *y1;
  double *st1, *st2;

  if (obj == NULL || (sig == NULL && sig_len > 0))
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy == SETUP)
    {
      retval = setup (obj, sig_len);
      if (retval != 0)
	return retval;

      obj->busy = UPDATE;
    }
  else if (obj->busy != UPDATE)
    {
      errno = EINVAL;
      return -1;
    }

  /* Relocate signal history.  */
  obj->sig = sig;
  obj->sig_len = sig_len;

  if (obj->sig_len > 0)
    {
      if (obj->sig_fun1 == NULL)
	{
	  errno = EINVAL;
	  return -1;
	}

      while (obj->sig_len > 0)
	{
	  retval = outer_loop (obj);
	  if (retval != 0)
	    return retval;
	}
    }

  if (finish == 0)
    return 0;

  if (obj->last != 0)
    {
      y0 = obj->last_extremum;
      y1 = obj->sig_buf;

      /* Check for hold.  */
      if (fabs (*y1 - *y0) >= DBL_EPSILON)
	{
	  /* Found local extremum.  */
	  memcpy (obj->last_extremum, y1, obj->sig_size);

	  /* Add signal value to the stack.  */
	  push_stack (obj, obj->last_extremum);

	  /* Inner loop.  */
	  retval = inner_loop (obj);
	  if (retval != 0)
	    return retval;
	}
    }

  /* Add remaining half cycles.

     ASTM E1049-85 says to add those cycles from bottom to top.  */
  obj->stack = obj->stack_base;

  while (obj->stack_len >= 2)
    {
      st2 = obj->stack;
      st1 = st2 + obj->sig_elem;

      /* Half cycle.  */
      retval = add_cycle (obj, st1, st2, 1);
      if (retval != 0)
	return retval;

      /* Drop ST2.  */
      --obj->stack_len;
      obj->stack = st1;
    }

  /* Reset cycle counting sequence.  */
  if (obj->shift_fun != NULL)
    {
      /* Shift latest cycle.  */
      if (obj->cycle_len > 0)
	obj->shift_fun (obj->shift_arg, obj->cycle);

      obj->cycle_offs = 0;
      obj->cycle_len = 0;
      relocate_cycle (obj);
    }

  /* Reset stack.  */
  obj->stack_len = 0;
  relocate_stack (obj);

  obj->busy = FINISH;
  obj->first = 0;
  obj->last = 0;

  return 0;
}

int
rs_rainflow_finish (rs_rainflow_t *obj)
{
  return rs_rainflow (obj, NULL, 0, RS_RAINFLOW_FINISH);
}

/* Return number of shiftable cycles.  */
static_inline size_t
shiftable_cycles (rs_rainflow_t *obj)
{
  return (obj->cycle_len > 0 && obj->busy == UPDATE ?
	  obj->cycle_len - 1 :
	  obj->cycle_len);
}

size_t
rs_rainflow_cycles (rs_rainflow_t *obj)
{
  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  return shiftable_cycles (obj);
}

/* Fetch the oldest cycles from the rainflow cycle counting sequence.  */
int
rs_rainflow_shift (rs_rainflow_t *obj, void *buffer, size_t count)
{
  double *cycle;

  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (shiftable_cycles (obj) < count)
    {
      errno = EAGAIN;
      return -1;
    }

  if (count > 0)
    {
      /* Location of the oldest cycle.  */
      cycle = obj->cycle_base + obj->cycle_offs * obj->cycle_elem;

      if (buffer != NULL)
	memcpy (buffer, cycle, count * obj->cycle_size);

      obj->cycle_offs += count;
      obj->cycle_len -= count;
    }

  return 0;
}

/* Return the cycle counting sequence.  */
void *
rs_rainflow_capture (rs_rainflow_t *obj)
{
  double *cycle;

  if (obj == NULL)
    {
      errno = EINVAL;
      return NULL;
    }

  if (obj->busy == UPDATE)
    {
      errno = EBUSY;
      return NULL;
    }

  if (obj->cycle_len == 0)
    return NULL;

  /* Move remaining cycles to the beginning of the buffer.  */
  if (obj->cycle_offs > 0)
    {
      memmove (obj->cycle_base,
	       obj->cycle_base + obj->cycle_offs * obj->cycle_elem,
	       obj->cycle_len * obj->cycle_size);

      obj->cycle_offs = 0;
      relocate_cycle (obj);
    }

  /* Truncate the buffer.  */
  if (obj->cycle_len < obj->cycle_count)
    {
      cycle = call_realloc (obj, obj->cycle_base, obj->cycle_len * obj->cycle_size);
      if (cycle != NULL)
	{
	  obj->cycle_base = cycle;
	  relocate_cycle (obj);
	}
    }

  /* Detach the buffer.  */
  cycle = obj->cycle_base;

  obj->cycle = NULL;
  obj->cycle_base = NULL;
  obj->cycle_count = 0;
  obj->cycle_offs = 0;
  obj->cycle_len = 0;

  return cycle;
}

/* Sort cycle counting sequence.  */
int
rs_rainflow_sort (rs_rainflow_t *obj, int (*compare) (void const *, void const *))
{
  double *cycle;
  size_t cycles;
  int retval;

  if (obj == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (obj->busy == UPDATE)
    {
      errno = EBUSY;
      return -1;
    }

  cycle = obj->cycle_base + obj->cycle_offs * obj->cycle_elem;
  cycles = obj->cycle_len;

  if (cycles > 1)
    {
      retval = rs_rainflow_sort_cycles (cycle, cycles, obj->cycle_size, compare);
      if (retval != 0)
	return retval;

      /* Unconditionally merge similar cycles.  */
      retval = rs_rainflow_merge_cycles (cycle, &cycles, obj->cycle_size, compare);
      if (retval != 0)
	return retval;

      obj->cycle_len = cycles;
      relocate_cycle (obj);
    }

  return 0;
}

/* Compare the number A against B.  If A is considered greater than B,
   the return value is a positive number.  If A is considered less than
   B, the return value is a negative number.  If the two numbers are
   equal, the return value is zero.  */
static_inline int
fcmp (double a, double b)
{
  return (a > b) - (a < b);
}

/* Comparison function for sorting cycles by signal amplitude and mean
   value in ascending order.  */
static_inline int
compare_cycles (double const *a, double const *b)
{
  int diff;

  diff = fcmp (a[0], b[0]);
  if (diff != 0)
    return diff;

  return fcmp (a[1], b[1]);
}

/* Comparison function for sorting cycles by signal labels in ascending
   order.  */
static_inline int
compare_labels (double const *a, double const *b)
{
  int diff;

  diff = memcmp (a + 3, b + 3, sizeof (double));
  if (diff != 0)
    return diff;

  return memcmp (a + 4, b + 4, sizeof (double));
}

/* Comparison function for sorting cycles in ascending order.  */
int
rs_rainflow_compare_ascending (void const *left, void const *right)
{
  return compare_cycles (left, right);
}

/* Comparison function for sorting cycles in descending order.  */
int
rs_rainflow_compare_descending (void const *left, void const *right)
{
  return compare_cycles (right, left);
}

/* Low-level sorting procedure.  */
int
rs_rainflow_sort_cycles (void *buffer, size_t count, size_t size, int (*compare) (void const *, void const *))
{
  if (buffer == NULL)
    {
      errno = EINVAL;
      return -1;
    }

  if (count > 1)
    {
      if (compare == NULL)
	compare = rs_rainflow_compare_descending;

      qsort (buffer, count, size, compare);
    }

  return 0;
}

/* Low-level merging procedure.  */
int
rs_rainflow_merge_cycles (void *buffer, size_t *count, size_t size, int (*compare) (void const *, void const *))
{
  double *head, *tail;
  size_t c, n, elem;

  if (buffer == NULL || count == NULL || (size % sizeof (double)) != 0)
    {
      errno = EINVAL;
      return -1;
    }

  elem = size / sizeof (double);

  c = *count;
  if (c > 1)
    {
      if (compare == NULL)
	compare = rs_rainflow_compare_descending;

      head = buffer;

      for (n = c - 1; n > 0; --n)
	{
	  tail = head + elem;

	  if (compare (head, tail) == 0)
	    {
	      /* Merge cycles.  */
	      head[2] += tail[2];

	      /* Drop TAIL cycle.  */
	      --c;

	      memmove (tail, tail + elem, (n - 1) * size);
	    }
	  else
	    {
	      /* Move forward.  */
	      head += elem;
	    }
	}

      *count = c;
    }

  return 0;
}

/*
 * local variables:
 * compile-command: "gcc -g -Wall -W -c rs-rainflow.c "
 * end:
 */

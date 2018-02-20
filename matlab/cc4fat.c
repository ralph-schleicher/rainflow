/* cc4fat.c --- cycle counting for fatigue analysis.

   Copyright (C) 2011 Ralph Schleicher

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

   This MEX Function has been tested with Matlab and Octave.
   Compile with

        >> mex cc4fat.c rs-rainflow.c rs-matrix-transpose.c  */

#include <stdlib.h>
#include <string.h>
#include <float.h>
#include <math.h>

#include "rs-rainflow.h"
#include "rs-matrix.h"

#include <mex.h>

/* Allocate cycle counting sequence in chunks not larger than 4 MiB.  */
#define SIZEOF_HUGE_PAGE (((size_t) 4) << 20)

/* Supported cycle counting methods.  */
enum
  {
    METHOD_RAINFLOW,
    METHOD_RESERVOIR,

    METHODS
  };

/* Work variables for the cycle counting call-back functions.  */
struct work
  {
    /* Pointer to signal values.  */
    void const *value;

    /* Pointer to signal labels.  */
    void const *label;

    /* Signal index.  */
    double index;
  };

typedef struct work work_t;

/* Supported Matlab data types for signal values and labels.  */
enum
  {
    TYPE_DOUBLE,
    TYPE_SINGLE,
    TYPE_INT8,
    TYPE_UINT8,
    TYPE_INT16,
    TYPE_UINT16,
    TYPE_INT32,
    TYPE_UINT32,
    TYPE_INT64,
    TYPE_UINT64,
    TYPE_LOGICAL,
    TYPE_CHAR,

    TYPES
  };

/* Data type properties.  */
struct type
  {
    /* Matlab data type (see above).  */
    int index;

    /* Corresponding Matlab class.  */
    mxClassID mx_class_id;

    /* Corresponding rs-rainflow library data type.  */
    int rs_rainflow_type;

    /* Size of an element.  */
    size_t size;

    /* Function to locate the index offset of the maximum
       signal value.  */
    size_t (*indmax) (void const *, size_t);

    /* Function to copy signal values without signal labels.  */
    size_t (*copy) (void *, void *, size_t);

    /* Function to copy signal values with implicit signal labels.  */
    size_t (*copy_implicit) (void *, void *, size_t);

    /* Function to copy signal values with explicit signal labels.  */
    size_t (*copy_explicit[TYPES]) (void *, void *, size_t);
  };

typedef struct type type_t;

/* Wrapper functions for different array element data types.  */
#define def1(S1,T1)							\
/* Locate the index offset of the maximum value.  */			\
static size_t								\
indmax_##S1 (T1 const *y, size_t n)					\
{									\
  size_t j, k;								\
									\
  k = 0;								\
									\
  for (j = 1; j < n; ++j)						\
    {									\
      if (y[j] > y[k])							\
	k = j;								\
    }									\
									\
  return k;								\
}									\
									\
/* Copy signal values without signal labels.  */			\
static size_t								\
copy_##S1 (work_t *work, double *v, size_t c)				\
{									\
  size_t n;								\
  T1 const *y;								\
									\
  y = work->value;							\
									\
  for (n = c; n > 0; --n)						\
    *v++ = *y++;							\
									\
  work->value = y;							\
									\
  return c;								\
}									\
									\
/* Copy signal values with implicit signal labels.  */			\
static size_t								\
copy_##S1##_null (work_t *work, double *v, size_t c)			\
{									\
  size_t n;								\
  T1 const *y;								\
  double t;								\
									\
  y = work->value;							\
  t = work->index;							\
									\
  for (n = c; n > 0; --n)						\
    {									\
      *v++ = *y++;							\
      *v++ = t++;							\
    }									\
									\
  work->value = y;							\
  work->index = t;							\
									\
  return c;								\
}

#define def2(S1,T1,S2,T2)						\
/* Copy signal values with explicit signal labels.  */			\
static size_t								\
copy_##S1##_##S2 (work_t *work, double *v, size_t c)			\
{									\
  size_t n;								\
  T1 const *y;								\
  T2 const *t;								\
									\
  y = work->value;							\
  t = work->label;							\
									\
  for (n = c; n > 0; --n)						\
    {									\
      *v++ = *y++;							\
      *v++ = *t++;							\
    }									\
									\
  work->value = y;							\
  work->label = t;							\
									\
  return c;								\
}

/* Generate all the functions.  */
#define def(S1,T1)							\
def1 (S1, T1)								\
def2 (S1, T1, double, double)						\
def2 (S1, T1, single, float)						\
def2 (S1, T1, int8, int8_T)						\
def2 (S1, T1, uint8, uint8_T)						\
def2 (S1, T1, int16, int16_T)						\
def2 (S1, T1, uint16, uint16_T)						\
def2 (S1, T1, int32, int32_T)						\
def2 (S1, T1, uint32, uint32_T)						\
def2 (S1, T1, int64, int64_T)						\
def2 (S1, T1, uint64, uint64_T)						\
def2 (S1, T1, logical, mxLogical)					\
def2 (S1, T1, char, mxChar)

def (double, double)
def (single, float)
def (int8, int8_T)
def (uint8, uint8_T)
def (int16, int16_T)
def (uint16, uint16_T)
def (int32, int32_T)
def (uint32, uint32_T)
def (int64, int64_T)
def (uint64, uint64_T)
def (logical, mxLogical)
def (char, mxChar)

/* Database of type properties.  */
static type_t const types[TYPES] =
  {
    {
      TYPE_DOUBLE,
      mxDOUBLE_CLASS,
      RS_RAINFLOW_TYPE_DOUBLE,
      sizeof (double),
      (void *) indmax_double,
      (void *) copy_double,
      (void *) copy_double_null,
      {
	(void *) copy_double_double,
	(void *) copy_double_single,
	(void *) copy_double_int8,
	(void *) copy_double_uint8,
	(void *) copy_double_int16,
	(void *) copy_double_uint16,
	(void *) copy_double_int32,
	(void *) copy_double_uint32,
	(void *) copy_double_int64,
	(void *) copy_double_uint64,
	(void *) copy_double_logical,
	(void *) copy_double_char,
      },
    },
    {
      TYPE_SINGLE,
      mxSINGLE_CLASS,
      RS_RAINFLOW_TYPE_FLOAT,
      sizeof (float),
      (void *) indmax_single,
      (void *) copy_single,
      (void *) copy_single_null,
      {
	(void *) copy_single_double,
	(void *) copy_single_single,
	(void *) copy_single_int8,
	(void *) copy_single_uint8,
	(void *) copy_single_int16,
	(void *) copy_single_uint16,
	(void *) copy_single_int32,
	(void *) copy_single_uint32,
	(void *) copy_single_int64,
	(void *) copy_single_uint64,
	(void *) copy_single_logical,
	(void *) copy_single_char,
      },
    },
    {
      TYPE_INT8,
      mxINT8_CLASS,
      RS_RAINFLOW_TYPE_INT8_T,
      sizeof (int8_T),
      (void *) indmax_int8,
      (void *) copy_int8,
      (void *) copy_int8_null,
      {
	(void *) copy_int8_double,
	(void *) copy_int8_single,
	(void *) copy_int8_int8,
	(void *) copy_int8_uint8,
	(void *) copy_int8_int16,
	(void *) copy_int8_uint16,
	(void *) copy_int8_int32,
	(void *) copy_int8_uint32,
	(void *) copy_int8_int64,
	(void *) copy_int8_uint64,
	(void *) copy_int8_logical,
	(void *) copy_int8_char,
      },
    },
    {
      TYPE_UINT8,
      mxUINT8_CLASS,
      RS_RAINFLOW_TYPE_UINT8_T,
      sizeof (uint8_T),
      (void *) indmax_uint8,
      (void *) copy_uint8,
      (void *) copy_uint8_null,
      {
	(void *) copy_uint8_double,
	(void *) copy_uint8_single,
	(void *) copy_uint8_int8,
	(void *) copy_uint8_uint8,
	(void *) copy_uint8_int16,
	(void *) copy_uint8_uint16,
	(void *) copy_uint8_int32,
	(void *) copy_uint8_uint32,
	(void *) copy_uint8_int64,
	(void *) copy_uint8_uint64,
	(void *) copy_uint8_logical,
	(void *) copy_uint8_char,
      },
    },
    {
      TYPE_INT16,
      mxINT16_CLASS,
      RS_RAINFLOW_TYPE_INT16_T,
      sizeof (int16_T),
      (void *) indmax_int16,
      (void *) copy_int16,
      (void *) copy_int16_null,
      {
	(void *) copy_int16_double,
	(void *) copy_int16_single,
	(void *) copy_int16_int8,
	(void *) copy_int16_uint8,
	(void *) copy_int16_int16,
	(void *) copy_int16_uint16,
	(void *) copy_int16_int32,
	(void *) copy_int16_uint32,
	(void *) copy_int16_int64,
	(void *) copy_int16_uint64,
	(void *) copy_int16_logical,
	(void *) copy_int16_char,
      },
    },
    {
      TYPE_UINT16,
      mxUINT16_CLASS,
      RS_RAINFLOW_TYPE_UINT16_T,
      sizeof (uint16_T),
      (void *) indmax_uint16,
      (void *) copy_uint16,
      (void *) copy_uint16_null,
      {
	(void *) copy_uint16_double,
	(void *) copy_uint16_single,
	(void *) copy_uint16_int8,
	(void *) copy_uint16_uint8,
	(void *) copy_uint16_int16,
	(void *) copy_uint16_uint16,
	(void *) copy_uint16_int32,
	(void *) copy_uint16_uint32,
	(void *) copy_uint16_int64,
	(void *) copy_uint16_uint64,
	(void *) copy_uint16_logical,
	(void *) copy_uint16_char,
      },
    },
    {
      TYPE_INT32,
      mxINT32_CLASS,
      RS_RAINFLOW_TYPE_INT32_T,
      sizeof (int32_T),
      (void *) indmax_int32,
      (void *) copy_int32,
      (void *) copy_int32_null,
      {
	(void *) copy_int32_double,
	(void *) copy_int32_single,
	(void *) copy_int32_int8,
	(void *) copy_int32_uint8,
	(void *) copy_int32_int16,
	(void *) copy_int32_uint16,
	(void *) copy_int32_int32,
	(void *) copy_int32_uint32,
	(void *) copy_int32_int64,
	(void *) copy_int32_uint64,
	(void *) copy_int32_logical,
	(void *) copy_int32_char,
      },
    },
    {
      TYPE_UINT32,
      mxUINT32_CLASS,
      RS_RAINFLOW_TYPE_UINT32_T,
      sizeof (uint32_T),
      (void *) indmax_uint32,
      (void *) copy_uint32,
      (void *) copy_uint32_null,
      {
	(void *) copy_uint32_double,
	(void *) copy_uint32_single,
	(void *) copy_uint32_int8,
	(void *) copy_uint32_uint8,
	(void *) copy_uint32_int16,
	(void *) copy_uint32_uint16,
	(void *) copy_uint32_int32,
	(void *) copy_uint32_uint32,
	(void *) copy_uint32_int64,
	(void *) copy_uint32_uint64,
	(void *) copy_uint32_logical,
	(void *) copy_uint32_char,
      },
    },
    {
      TYPE_INT64,
      mxINT64_CLASS,
      RS_RAINFLOW_TYPE_INT64_T,
      sizeof (int64_T),
      (void *) indmax_int64,
      (void *) copy_int64,
      (void *) copy_int64_null,
      {
	(void *) copy_int64_double,
	(void *) copy_int64_single,
	(void *) copy_int64_int8,
	(void *) copy_int64_uint8,
	(void *) copy_int64_int16,
	(void *) copy_int64_uint16,
	(void *) copy_int64_int32,
	(void *) copy_int64_uint32,
	(void *) copy_int64_int64,
	(void *) copy_int64_uint64,
	(void *) copy_int64_logical,
	(void *) copy_int64_char,
      },
    },
    {
      TYPE_UINT64,
      mxUINT64_CLASS,
      RS_RAINFLOW_TYPE_UINT64_T,
      sizeof (uint64_T),
      (void *) indmax_uint64,
      (void *) copy_uint64,
      (void *) copy_uint64_null,
      {
	(void *) copy_uint64_double,
	(void *) copy_uint64_single,
	(void *) copy_uint64_int8,
	(void *) copy_uint64_uint8,
	(void *) copy_uint64_int16,
	(void *) copy_uint64_uint16,
	(void *) copy_uint64_int32,
	(void *) copy_uint64_uint32,
	(void *) copy_uint64_int64,
	(void *) copy_uint64_uint64,
	(void *) copy_uint64_logical,
	(void *) copy_uint64_char,
      },
    },
    {
      TYPE_LOGICAL,
      mxLOGICAL_CLASS,
      RS_RAINFLOW_TYPE_UNKNOWN,
      sizeof (mxLogical),
      (void *) indmax_logical,
      (void *) copy_logical,
      (void *) copy_logical_null,
      {
	(void *) copy_logical_double,
	(void *) copy_logical_single,
	(void *) copy_logical_int8,
	(void *) copy_logical_uint8,
	(void *) copy_logical_int16,
	(void *) copy_logical_uint16,
	(void *) copy_logical_int32,
	(void *) copy_logical_uint32,
	(void *) copy_logical_int64,
	(void *) copy_logical_uint64,
	(void *) copy_logical_logical,
	(void *) copy_logical_char,
      },
    },
    {
      TYPE_CHAR,
      mxCHAR_CLASS,
      RS_RAINFLOW_TYPE_UNKNOWN,
      sizeof (mxChar),
      (void *) indmax_char,
      (void *) copy_char,
      (void *) copy_char_null,
      {
	(void *) copy_char_double,
	(void *) copy_char_single,
	(void *) copy_char_int8,
	(void *) copy_char_uint8,
	(void *) copy_char_int16,
	(void *) copy_char_uint16,
	(void *) copy_char_int32,
	(void *) copy_char_uint32,
	(void *) copy_char_int64,
	(void *) copy_char_uint64,
	(void *) copy_char_logical,
	(void *) copy_char_char,
      },
    },
  };

/* Return non-zero if the Matlab array A is a vector.
   An empty array is a vector, too.  */
static int
vectorp (mxArray const *a)
{
  mwSize const *s;
  mwSize k;
  int p;

  if (mxIsComplex (a) || mxIsSparse (a))
    return 0;

  /* Number of elements in each dimension.  */
  s = mxGetDimensions (a);

  /* Number of dimensions with more than one element.  */
  p = 0;

  for (k = mxGetNumberOfDimensions (a); k > 0; --k, ++s)
    {
      if (*s > 1)
	++p;
    }

  return p <= 1;
}

/* Return non-zero if the Matlab array A is a string.  */
static int
stringp (mxArray const *a)
{
  return (mxIsChar (a)
	  && mxGetNumberOfDimensions (a) == 2
	  && mxGetM (a) <= 1);
}

/* Return non-zero if the Matlab array A is an option.  */
static int
optionp (mxArray const *a)
{
  return (stringp (a)
	  && mxGetNumberOfElements (a) > 0
	  && mxGetScalar (a) == ((double) '-'));
}

/* Return the type for the Matlab array A.  */
static type_t const *
find_type (mxArray const *a)
{
  type_t const *type = types;
  mxClassID key;
  int k;

  /* Search by Matlab class.  */
  key = mxGetClassID (a);

  for (k = TYPES; k > 0; --k, ++type)
    {
      if (type->mx_class_id == key)
	return type;
    }

  return NULL;
}

/* Largest integral number that can be stored in a 'double' without
   loss of precision.  */
#ifdef __LCC__
#define flintmax(T) ((T)  9007199254740992LL)
#define flintmin(T) ((T) -9007199254740992LL)
#else /* not __LCC__ */
#if FLT_RADIX == 2
#define flintmax(T) (((T) 1) << DBL_MANT_DIG)
#define flintmin(T) (((T) 0) - flintmax (T))
#else /* FLT_RADIX != 2 */
#error "Fix me"
#endif /* FLT_RADIX != 2 */
#endif /* not __LCC__ */

static int
int64_out_of_range_p (int64_T const *v, size_t c)
{
  static int64_T const min = flintmin (int64_T);
  static int64_T const max = flintmax (int64_T);

  for (; c > 0; --c, ++v)
    {
      if (*v < min || *v > max)
	return 1;
    }

  return 0;
}

static int
uint64_out_of_range_p (uint64_T const *v, size_t c)
{
  static uint64_T const max = flintmax (uint64_T);

  for (; c > 0; --c, ++v)
    {
      if (*v > max)
	return 1;
    }

  return 0;
}

/* Comparison function for sorting cycles.  */
static int
compare_descending (void const *left, void const *right)
{
  double const *a = left;
  double const *b = right;

  int diff;

  /* Sort by signal amplitude and mean value in descending order.  */
  diff = (a[0] < b[0]) - (a[0] > b[0]);
  if (diff != 0)
    return diff;

  diff = (a[1] < b[1]) - (a[1] > b[1]);
  if (diff != 0)
    return diff;

  return 0;
}

static int
compare_descending_with_label (void const *left, void const *right)
{
  double const *a = left;
  double const *b = right;

  int diff;

  diff = compare_descending (left, right);
  if (diff != 0)
    return diff;

  /* Sort signal labels in ascending order.  */
  diff = memcmp (a + 3, b + 3, sizeof (double));
  if (diff != 0)
    return diff;

  diff = memcmp (a + 4, b + 4, sizeof (double));
  if (diff != 0)
    return diff;

  return 0;
}

static int
compare_time (void const *left, void const *right)
{
  double const *a = left;
  double const *b = right;

  int diff;

  /* Sort by start time and period in ascending order.  */
  diff = (a[3] > b[3]) - (a[3] < b[3]);
  if (diff != 0)
    return diff;

  diff = (a[4] > b[4]) - (a[4] < b[4]);
  if (diff != 0)
    return diff;

  return compare_descending (left, right);
}

/* Signal common errors.  */
#define enomem() \
do { mexErrMsgTxt ("Out of memory"); } while (0)

#define ebug() \
do { mexErrMsgTxt ("Should not happen"); } while (0)

/* Program entry point.  */
void
mexFunction (int val_count, mxArray *val_vec[], int arg_count, const mxArray *arg_vec[])
{
  rs_rainflow_t *obj;
  work_t work[1];
  void *copy;
  int method;
  int style;
  int sign;
  int label;
  int base;
  int merge;
  int sort;
  int full;
  int period;
  int first;
  int transp;

  mxArray const *y;
  type_t const *y_type;
  void const *y_buf;
  size_t y_len;

  mxArray const *t;
  type_t const *t_type;
  void const *t_buf;
  size_t t_len;

  mxArray *c;
  void *c_buf;
  size_t c_len, c_elem;
  size_t c_size;
  double *v;
  size_t k;

  /* Check number of arguments.  */
  if (arg_count < 1)
    mexErrMsgTxt ("Too few arguments");

  /* Check number of return values.  */
  if (val_count > 1)
    mexErrMsgTxt ("Too many return values");

  /* First argument is the signal history.  */
  y = arg_vec[0];

  --arg_count;
  ++arg_vec;

  y_type = find_type (y);
  if (y_type == NULL || ! vectorp (y))
    mexErrMsgTxt ("Signal history has to be a real vector of numbers");

  y_buf = mxGetData (y);
  y_len = mxGetNumberOfElements (y);

  if (y_type->index == TYPE_INT64 && int64_out_of_range_p (y_buf, y_len))
    mexErrMsgTxt ("Loss of precision in signal history due to type conversion from 'int64' to 'double'");

  if (y_type->index == TYPE_UINT64 && uint64_out_of_range_p (y_buf, y_len))
    mexErrMsgTxt ("Loss of precision in signal history due to type conversion from 'uint64' to 'double'");

  /* Optional second argument are the corresponding labels.  */
  t = NULL;

  if (arg_count > 0 && ! optionp (arg_vec[0]))
    {
      t = arg_vec[0];

      --arg_count;
      ++arg_vec;

      t_type = find_type (t);
      if (t_type == NULL || ! vectorp (t))
	mexErrMsgTxt ("Signal labels array has to be a real vector of numbers");

      t_buf = mxGetData (t);
      t_len = mxGetNumberOfElements (t);

      /* If argument T is empty, it has the same meaning as if it is
	 omitted.  Otherwise, argument T must have the same number of
	 elements as argument Y.  */
      if (t_len == 0)
	t = NULL;
      else
	{
	  if (t_len != y_len)
	    mexErrMsgTxt ("Wrong number of elements in signal labels array");

	  if (t_type->index == TYPE_INT64 && int64_out_of_range_p (t_buf, t_len))
	    mexErrMsgTxt ("Loss of precision in signal labels array due to type conversion from 'int64' to 'double'");

	  if (t_type->index == TYPE_UINT64 && uint64_out_of_range_p (t_buf, t_len))
	    mexErrMsgTxt ("Loss of precision in signal labels array due to type conversion from 'uint64' to 'double'");
	}
    }

  if (t == NULL)
    {
      t_type = types + TYPE_DOUBLE;
      t_buf = NULL;
      t_len = 0;
    }

  /* Remaining arguments are options.  */

  /* Cycle counting method.  */
  method = METHOD_RAINFLOW;
  /* Cycle representation.  */
  style = RS_RAINFLOW_AMPLITUDE_MEAN;
  /* Non-zero means to retain the sign of the signal amplitude
     or signal range.  */
  sign = 0;
  /* Non-zero means to label signal values.  */
  label = (t != NULL ? 1 : 0);
  /* Value of the first implicit signal label.  */
  base = 1;
  /* Non-zero means to merge similar consecutive cycles.  */
  merge = 1;
  /* Non-zero means to sort cycles by amplitude and mean value.  */
  sort = 0;
  /* Non-zero means to count full cycles.  */
  full = 1;
  /* Non-zero means to calculate start time and period.  */
  period = 0;
  /* Non-zero means to place the cycle count at the beginning
     of a cycle.  */
  first = 0;
  /* Non-zero means to return cycles as column vectors.  */
  transp = 0;

  while (arg_count > 0)
    {
      mxArray const *a;
      char *str;

      a = arg_vec[0];
      if (! stringp (a))
	mexErrMsgTxt ("Option has to be a string");

      str = mxArrayToString (a);
      if (str == NULL)
	enomem ();

      if (*str != '-')
	mexErrMsgTxt ("Invalid option");
      else if (strcmp (str, "-rainflow") == 0)
	method = METHOD_RAINFLOW;
      else if (strcmp (str, "-reservoir") == 0)
	method = METHOD_RESERVOIR;
      else if (strcmp (str, "-from-to") == 0)
	style = RS_RAINFLOW_FROM_TO;
      else if (strcmp (str, "-range-mean") == 0)
	style = RS_RAINFLOW_RANGE_MEAN;
      else if (strcmp (str, "-amplitude-mean") == 0)
	style = RS_RAINFLOW_AMPLITUDE_MEAN;
      else if (strcmp (str, "-sign") == 0)
	sign = 1;
      else if (strcmp (str, "-no-sign") == 0)
	sign = 0;
      else if (strcmp (str, "-label") == 0)
	label = 1;
      else if (strcmp (str, "-no-label") == 0)
	label = 0;
      else if (strcmp (str, "-zero") == 0)
	base = 0;
      else if (strcmp (str, "-no-zero") == 0)
	base = 1;
      else if (strcmp (str, "-one") == 0)
	base = 1;
      else if (strcmp (str, "-no-one") == 0)
	base = 0;
      else if (strcmp (str, "-merge") == 0)
	merge = 1;
      else if (strcmp (str, "-no-merge") == 0)
	merge = 0;
      else if (strcmp (str, "-sort") == 0)
	sort = 1;
      else if (strcmp (str, "-no-sort") == 0)
	sort = 0;
      else if (strcmp (str, "-full") == 0)
	full = 1;
      else if (strcmp (str, "-no-full") == 0)
	full = 0;
      else if (strcmp (str, "-half") == 0)
	full = 0;
      else if (strcmp (str, "-no-half") == 0)
	full = 1;
      else if (strcmp (str, "-time") == 0)
	period = 1;
      else if (strcmp (str, "-no-time") == 0)
	period = 0;
      else if (strcmp (str, "-first") == 0)
	first = 1;
      else if (strcmp (str, "-third") == 0)
	first = 0;
      else if (strcmp (str, "-transpose") == 0)
	transp = 1;
      else if (strcmp (str, "-no-transpose") == 0)
	transp = 0;
      else
	mexErrMsgTxt ("Unknown option");

      mxFree (str);

      --arg_count;
      ++arg_vec;
    }

  if (style == RS_RAINFLOW_FROM_TO)
    sort = 0;

  if (label == 0)
    {
      period = 0;

      /* Elements per cycle.  */
      c_elem = 3;

      /* Signal value copying function.  */
      copy = y_type->copy;
    }
  else
    {
      /* Elements per cycle.  */
      c_elem = 5;

      /* Signal value copying function.  */
      copy = (t == NULL ?
	      /* Implicit labeling.  */
	      y_type->copy_implicit :
	      /* Explicit labeling.  */
	      y_type->copy_explicit[t_type->index]);
    }

  /* Size of a cycle in byte.  */
  c_size = c_elem * sizeof (double);

  /* Initial length of the cycle counting sequence.
     Reserve 16 byte for memory manager overhead.  */
  c_len = (SIZEOF_HUGE_PAGE - 16) / c_size;
  if (c_len > y_len)
    c_len = y_len;

  /* Create rainflow cycle counting object.   */
  obj = rs_rainflow_alloc (mxMalloc, mxRealloc, mxFree);
  if (obj == NULL)
    enomem ();

  if (rs_rainflow_set_length (obj, c_len, 0) != 0)
    ebug ();

  if (rs_rainflow_set_read_signals (obj, copy, 0) != 0)
    ebug ();

  if (rs_rainflow_set_signal_label (obj, label) != 0)
    ebug ();

  if (rs_rainflow_set_merge_cycles (obj, merge) != 0)
    ebug ();

  if (rs_rainflow_set_cycle_style (obj, style) != 0)
    ebug ();

  if (rs_rainflow_set_cycle_sign (obj, sign) != 0)
    ebug ();

  /* Initialize the work variables.  */
  work->value = y_buf;
  work->label = t_buf;
  work->index = base;

  /* Count cycles.  */
  if (y_len < 2)
    {
      /* Nothing to do.  */
      (void) 0;
    }
  else if (method == METHOD_RAINFLOW)
    {
      /* TODO: Evaluate return value and issue an appropriate
	 error message.  However, a memory error is the most
	 likely failure.  */
      if (rs_rainflow (obj, work, y_len, RS_RAINFLOW_FINISH) != 0)
	enomem ();
    }
  else if (method == METHOD_RESERVOIR)
    {
      char const *y;
      char const *t;
      double i;

      /* Locate global maximum of the signal history.  */
      k = y_type->indmax (y_buf, y_len);

      /* Rearrange the cycle counting history so that it starts
	 and ends with the global maximum.  */
      y = work->value;
      t = work->label;
      i = work->index;

      work->value = y + k * y_type->size;
      work->label = t + k * t_type->size;
      work->index = i + k;

      if (rs_rainflow (obj, work, y_len - k, RS_RAINFLOW_CONTINUE) != 0)
	enomem ();

      work->value = y;
      work->label = t;
      work->index = i;

      if (rs_rainflow (obj, work, k + 1, RS_RAINFLOW_FINISH) != 0)
	enomem ();
    }
  else
    ebug ();

  /* Actual number of cycles.  */
  c_len = rs_rainflow_cycles (obj);

  /* Capture the cycle counting sequence.  */
  c_buf = rs_rainflow_capture (obj);
  if (c_buf == NULL && c_len > 0)
    ebug ();

  /* Delete rainflow cycle counting object.  */
  rs_rainflow_delete (obj);

  /* Calculate start time and period.

     Don't ask me for what this is actually good for.
     However, it's trivial.  Therefore, I do it.  */
  if (period != 0)
    {
      double t1, t2;

      v = c_buf;
      v += 3;

      for (k = c_len; k > 0; --k)
	{
	  /* Signal label, i.e. time, of the trough
	     and peak signal value.  */
	  t1 = v[0];
	  t2 = v[1];

	  /* Start time and period.  */
	  v[0] = (t1 > t2 ? t2 : t1);
	  v[1] = fabs (t2 - t1) * 2.0;

	  /* Next cycle.  */
	  v += c_elem;
	}
    }

  if (sort != 0)
    {
      /* Cycle comparison function.  */
      void *compare = (period != 0 ?
		       compare_time :
		       (label == 0 ?
			compare_descending :
			compare_descending_with_label));

      if (rs_rainflow_sort_cycles (c_buf, c_len, c_size, compare) != 0)
	ebug ();

      /* Sorting without merging does not make much sense.  */
      if (rs_rainflow_merge_cycles (c_buf, &c_len, c_size, compare) != 0)
	ebug ();
    }

  /* Change half cycles to full cycles.  */
  if (full != 0)
    {
      v = c_buf;
      v += 2;

      for (k = c_len; k > 0; --k)
	{
	  /* Number of full cycles.  */
	  *v /= 2.0;

	  /* Next cycle.  */
	  v += c_elem;
	}
    }

  /* Place the cycle count at the beginning of a cycle.  */
  if (first != 0)
    {
      v = c_buf;

      for (k = c_len; k > 0; --k)
	{
	  double tem;

	  tem = v[2];
	  v[2] = v[1];
	  v[1] = v[0];
	  v[0] = tem;

	  /* Next cycle.  */
	  v += c_elem;
	}
    }

  /* Return value is the cycle counting sequence.  */
  c = mxCreateNumericMatrix (0, 0, mxDOUBLE_CLASS, mxREAL);
  if (c == NULL)
    enomem ();

  val_vec[0] = c;

  v = mxGetData (c);
  if (v != NULL)
    mxFree (v);

  mxSetData (c, c_buf);

  if (transp == 0)
    {
      mxSetM (c, (mwSize) c_len);
      mxSetN (c, (mwSize) c_elem);

      /* Convert from C row-major storage layout to Fortran
	 column-major storage layout.  */
      rs_matrix_transpose (c_buf, c_len, c_elem, sizeof (double));
    }
  else
    {
      mxSetM (c, (mwSize) c_elem);
      mxSetN (c, (mwSize) c_len);
    }
}

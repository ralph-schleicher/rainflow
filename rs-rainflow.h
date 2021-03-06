/* rs-rainflow.h --- rainflow cycle counting.

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

#ifndef RS_RAINFLOW_H
#define RS_RAINFLOW_H

#include <stddef.h>

#ifdef __cplusplus
#define RS_RAINFLOW_BEGIN_DECL extern "C" {
#define RS_RAINFLOW_END_DECL }
#else /* not __cplusplus */
#define RS_RAINFLOW_BEGIN_DECL
#define RS_RAINFLOW_END_DECL
#endif /* not __cplusplus */

RS_RAINFLOW_BEGIN_DECL

/* Opaque rainflow cycle counting object.  */
typedef struct rs_rainflow rs_rainflow_t;

/* Cycle representations.  */
enum
  {
    RS_RAINFLOW_AMPLITUDE_MEAN = 0,
    RS_RAINFLOW_RANGE_MEAN = 1,
    RS_RAINFLOW_FROM_TO = 2,

    RS_RAINFLOW_CYCLE_REPRESENTATIONS
  };

/* Predefined element data types.  */
enum
  {
    RS_RAINFLOW_TYPE_UNKNOWN = 0,
    RS_RAINFLOW_TYPE_DOUBLE,
    RS_RAINFLOW_TYPE_FLOAT,
    RS_RAINFLOW_TYPE_CHAR,
    RS_RAINFLOW_TYPE_UCHAR,
    RS_RAINFLOW_TYPE_SHORT,
    RS_RAINFLOW_TYPE_USHORT,
    RS_RAINFLOW_TYPE_INT,
    RS_RAINFLOW_TYPE_UINT,
    RS_RAINFLOW_TYPE_LONG,
    RS_RAINFLOW_TYPE_ULONG,
    RS_RAINFLOW_TYPE_INT8_T,
    RS_RAINFLOW_TYPE_UINT8_T,
    RS_RAINFLOW_TYPE_INT16_T,
    RS_RAINFLOW_TYPE_UINT16_T,
    RS_RAINFLOW_TYPE_INT32_T,
    RS_RAINFLOW_TYPE_UINT32_T,
    RS_RAINFLOW_TYPE_INT64_T,
    RS_RAINFLOW_TYPE_UINT64_T,

    RS_RAINFLOW_TYPES
  };

/* Error conditions.  */
enum
  {
    RS_RAINFLOW_ERROR_STACK_OVERFLOW = 1,
    RS_RAINFLOW_ERROR_CYCLE_OVERFLOW
  };

/* Control flow.  */
enum
  {
    RS_RAINFLOW_CONTINUE = 0,
    RS_RAINFLOW_FINISH
  };

/* Instantiation.  */
extern rs_rainflow_t *rs_rainflow_new (void);
extern rs_rainflow_t *rs_rainflow_alloc (void *(*__malloc_fun) (size_t), void *(*__realloc_fun) (void *, size_t), void (*__free_fun) (void *));
extern void rs_rainflow_delete (rs_rainflow_t *__obj);
extern int rs_rainflow_reset (rs_rainflow_t *__obj);

/* Execution.  */
extern int rs_rainflow (rs_rainflow_t *__obj, void *__sig, size_t __sig_len, int __finish);
extern int rs_rainflow_finish (rs_rainflow_t *__obj);
extern size_t rs_rainflow_cycles (rs_rainflow_t *__obj);
extern int rs_rainflow_shift (rs_rainflow_t *__obj, double *__buffer, size_t __count);
extern double *rs_rainflow_capture (rs_rainflow_t *__obj);
extern int rs_rainflow_clear (rs_rainflow_t *__obj);

/* Customization.  */
extern int rs_rainflow_set_length (rs_rainflow_t *__obj, size_t __len, size_t __add);
extern int rs_rainflow_set_signal_type (rs_rainflow_t *__obj, int __type);
extern int rs_rainflow_set_read_signals (rs_rainflow_t *__obj, size_t (*__fun) (void *, double *, size_t), size_t __incr);
extern int rs_rainflow_set_shift_cycle (rs_rainflow_t *__obj, void (*__fun) (void *, double const *), void *__arg);
extern int rs_rainflow_set_signal_label (rs_rainflow_t *__obj, int __label);
extern int rs_rainflow_set_signal_index (rs_rainflow_t *__obj, double __index);
extern int rs_rainflow_set_merge_cycles (rs_rainflow_t *__obj, int __merge);
extern int rs_rainflow_set_cycle_style (rs_rainflow_t *__obj, int __style);
extern int rs_rainflow_set_cycle_sign (rs_rainflow_t *__obj, int __flag);

/* Sorting and merging.  */
extern int rs_rainflow_sort (rs_rainflow_t *__obj, int (*__compare) (void const *, void const *));
extern int rs_rainflow_sort_cycles (double *__buffer, size_t __count, size_t __size, int (*__compare) (void const *, void const *));
extern int rs_rainflow_merge_cycles (double *__buffer, size_t *__count, size_t __size, int (*__compare) (void const *, void const *));
extern int rs_rainflow_compare_ascending (void const *__left, void const *__right);
extern int rs_rainflow_compare_descending (void const *__left, void const *__right);

/* Rainflow matrix.  */
typedef struct rs_rainflow_matrix rs_rainflow_matrix_t;

extern rs_rainflow_matrix_t *rs_rainflow_matrix_new (void);
extern void rs_rainflow_matrix_delete (rs_rainflow_matrix_t *__obj);
extern int rs_rainflow_matrix_add (rs_rainflow_matrix_t *__obj, double const *__cycle);
extern int rs_rainflow_matrix_add3 (rs_rainflow_matrix_t *__obj, double __first, double __second, double __count);
extern double rs_rainflow_matrix_get (rs_rainflow_matrix_t *__obj, double const *__cycle);
extern double rs_rainflow_matrix_get2 (rs_rainflow_matrix_t *__obj, double __first, double __second);
extern int rs_rainflow_matrix_limits (rs_rainflow_matrix_t *__obj, int __dim, double *__min, double *__max);
extern size_t rs_rainflow_matrix_non_zero (rs_rainflow_matrix_t *__obj);
extern void rs_rainflow_matrix_map (rs_rainflow_matrix_t *__obj, void (*__fun) (void *, double const *), void *__arg);

extern double rs_rainflow_round_up (double __x, double __scale);
extern double rs_rainflow_round_down (double __x, double __scale);
extern double rs_rainflow_round_zero (double __x, double __scale);
extern double rs_rainflow_round_inf (double __x, double __scale);

extern void rs_rainflow_round_amplitude_mean (double *__cycle, double __scale);
extern void rs_rainflow_round_range_mean (double *__cycle, double __scale);
extern void rs_rainflow_round_from_to (double *__cycle, double __scale);

RS_RAINFLOW_END_DECL

#endif /* not RS_RAINFLOW_H */

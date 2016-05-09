/* ex-reservoir.c --- reservoir cycle counting.

   Copyright (C) 2015 Ralph Schleicher

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
#include <stdio.h>

#include "ex.h"

/* Reservoir cycle counting.  */
static void
reservoir (rs_rainflow_t *obj, double *sig_buf, size_t sig_len)
{
  size_t j, k;

  /* Locate global maximum of the signal history.  */
  k = 0;

  for (j = 1; j < sig_len; ++j)
    if (sig_buf[j] > sig_buf[k])
      k = j;

  /* Rearrange the cycle counting history so that it starts and ends
     with the global maximum.  */
  if (rs_rainflow (obj, sig_buf + k, sig_len - k, RS_RAINFLOW_CONTINUE) != 0)
    abort ();

  if (rs_rainflow (obj, sig_buf, k + 1, RS_RAINFLOW_FINISH) != 0)
    abort ();
}

int
main (int arg_count, char *arg_vec[])
{
  rs_rainflow_t *obj;
  double *sig;
  size_t sig_len;

  if (arg_count != 2)
    abort ();

  /* Get signal history.  */
  sig = read_binary_file (arg_vec[1], sizeof (double), &sig_len);
  if (sig == NULL)
    abort ();

  /* Create rainflow cycle counting object.  */
  obj = rs_rainflow_new ();
  if (obj == NULL)
    abort ();

  /* Perform reservoir cycle counting.  */
  reservoir (obj, sig, sig_len);

  /* Sort cycle counting sequence.  */
  if (rs_rainflow_sort (obj, rs_rainflow_compare_descending) != 0)
    abort ();

  /* Print cycle counting sequence.  */
  print_cycles (stdout, obj);

  /* Destroy rainflow cycle counting object.  */
  rs_rainflow_delete (obj);

  return 0;
}

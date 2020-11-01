/* ex-matrix1-round.c --- rainflow cycle counting.

   Copyright (C) 2018 Ralph Schleicher

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
#include <stdio.h>

#include "ex.h"

static void
add (rs_rainflow_matrix_t *mat, double const *cycle)
{
  double tem[3];

  memcpy (tem, cycle, 3 * sizeof (double));
  rs_rainflow_round_amplitude_mean (tem, 0.5);
  rs_rainflow_matrix_add (mat, tem);
}

int
main (int arg_count, char *arg_vec[])
{
  rs_rainflow_matrix_t *mat;
  rs_rainflow_t *obj;
  double *sig;
  size_t sig_len;

  if (arg_count != 2)
    abort ();

  /* Create rainflow matrix.  */
  mat = rs_rainflow_matrix_new ();
  if (mat == NULL)
    abort ();

  /* Get signal history.  */
  sig = read_binary_file (arg_vec[1], sizeof (double), &sig_len);
  if (sig == NULL)
    abort ();

  /* Create rainflow cycle counting object.  */
  obj = rs_rainflow_new ();
  if (obj == NULL)
    abort ();

  /* Add cycles to the rainflow matrix.  */
  if (rs_rainflow_set_shift_cycle (obj, (void *) add, mat) != 0)
    abort ();

  /* Perform rainflow cycle counting.  */
  if (rs_rainflow (obj, sig, sig_len, RS_RAINFLOW_FINISH) != 0)
    abort ();

  /* Destroy rainflow cycle counting object.  */
  rs_rainflow_delete (obj);

  /* Print rainflow matrix.  */
  rs_rainflow_matrix_map (mat, (void *) print_cycle, stdout);

  /* Destroy rainflow matrix.  */
  rs_rainflow_matrix_delete (mat);

  return 0;
}

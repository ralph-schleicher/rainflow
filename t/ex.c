/* ex.c --- utility functions for the rs-rainflow library examples.

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
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>

#include "ex.h"

size_t
read_from_stream (FILE *stream, double *buffer, size_t count)
{
  size_t n;

  for (n = 0; count > 0; --count, ++buffer, ++n)
    {
      if (fscanf (stream, "%lf", buffer) != 1)
	break;
    }

  return n;
}

void *
read_binary_file (char const *file_name, size_t size, size_t *_count)
{
  FILE *stream;
  struct stat st[1];
  void *buffer = NULL;
  size_t count;

  stream = fopen (file_name, "rb");
  if (stream == NULL)
    goto failure;

  if (fstat (fileno (stream), st) != 0)
    goto failure;

  if (st->st_size % size != 0)
    goto failure;

  count = st->st_size / size;
  if (_count != NULL)
    *_count = count;

  buffer = calloc (count, size);
  if (buffer == NULL)
    goto failure;

  if (fread (buffer, size, count, stream) != count)
    goto failure;

  if (fclose (stream) != 0)
    goto failure;

  return buffer;

 failure:

  if (buffer != NULL)
    free (buffer);

  if (stream != NULL)
    {
      int e = errno;
      fclose (stream);
      errno = e;
    }

  return NULL;
}

void
print_header (FILE *stream)
{
  /* Print file header.  */
  fprintf (stream, "SA;SM;N\n");
}

void
print_cycle (FILE *stream, double const *cycle)
{
  double ampl, mean, count;

  ampl = cycle[0];
  mean = cycle[1];
  count = cycle[2];

  /* Print cycle count as the number of full cycles.  */
  fprintf (stream, "%.5G;%.5G;%.5G\n", ampl, mean, count / 2.0);
}

void
print_cycles (FILE *stream, rs_rainflow_t *obj)
{
  double cycle[3];
  size_t n;

  for (n = rs_rainflow_cycles (obj); n > 0; --n)
    {
      /* Consume oldest cycle.  */
      rs_rainflow_shift (obj, cycle, 1);
      print_cycle (stream, cycle);
    }
}

void
print_cycles_fast (FILE *stream, rs_rainflow_t *obj)
{
  double buffer[24], *cycle;
  size_t c, n;

  n = rs_rainflow_cycles (obj);

  for (c = n / 8; c > 0; --c)
    {
      /* Fetch eight cycles.  */
      rs_rainflow_shift (obj, buffer, 8);

      /* Print them.  */
      cycle = buffer;

      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle); cycle += 3;
      print_cycle (stream, cycle);
    }

  c = n % 8;
  if (c > 0)
    {
      rs_rainflow_shift (obj, buffer, c);

      /* Print them.  */
      cycle = buffer;

      switch (c)
	{
	case 7: print_cycle (stream, cycle); cycle += 3;
	case 6: print_cycle (stream, cycle); cycle += 3;
	case 5: print_cycle (stream, cycle); cycle += 3;
	case 4: print_cycle (stream, cycle); cycle += 3;
	case 3: print_cycle (stream, cycle); cycle += 3;
	case 2: print_cycle (stream, cycle); cycle += 3;
	case 1: print_cycle (stream, cycle);
	}
    }
}

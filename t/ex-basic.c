#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "rs-rainflow.h"
#include "ex.c"

int
main (void)
{
  rs_rainflow_t *obj;
  size_t buf_len;
  double *buf;

  /* Maximum number of signal values to be read.  */
  buf_len = 1000;

  /* Signal value buffer.  */
  buf = calloc (buf_len, sizeof (double));
  if (buf == NULL)
    abort ();

  /* Create rainflow cycle counting object.  */
  obj = rs_rainflow_new ();
  if (obj == NULL)
    abort ();

  /* Process signal values.  */
  while (1)
    {
      size_t count;

      count = read_from_stream (stdin, buf, buf_len);
      if (count == 0)
	break;

      if (rs_rainflow (obj, buf, count, RS_RAINFLOW_CONTINUE) != 0)
	abort ();
    }

  if (rs_rainflow_finish (obj) != 0)
    abort ();

  /* Sort cycles in descending order.  */
  rs_rainflow_sort (obj, rs_rainflow_compare_descending);

  /* Print cycle counting sequence.  */
  print_cycles (stdout, obj);

  /* Destroy object.  */
  rs_rainflow_delete (obj);

  return 0;
}

/*
 * local variables:
 * compile-command: "gcc -g -Wall -W -pg -o ex-basic ex-basic.c rs-rainflow.c "
 * end:
 */

#include <stdlib.h>
#include "ex.h"

/* User-defined signal history access function.  */
static size_t
read1 (struct sig1 *sig, double *buffer, size_t count)
{
  size_t c;

  for (c = count; c > 0; --c, ++sig)
    {
      /* Copy signal value.  */
      *buffer++ = sig->value;
    }

  return count - c;
}

int
main (void)
{
  rs_rainflow_t *obj;

  /* Create rainflow cycle counting object.  */
  obj = rs_rainflow_new ();
  if (obj == NULL)
    abort ();

  /* Install a user-defined signal history access function
     and define the size of an array element of the signal
     history.  */
  rs_rainflow_set_read_signals (obj, (void *) read1, sizeof (struct sig1));

  /* Print cycle counting sequence to 'stdout'.  */
  rs_rainflow_set_shift_cycle (obj, (void *) sig1_print1, stdout);

  /* Perform rainflow cycle counting.  */
  if (rs_rainflow (obj, sig1, sig1_len, RS_RAINFLOW_FINISH) != 0)
    abort ();

  /* Destroy rainflow cycle counting object.  */
  rs_rainflow_delete (obj);

  return 0;
}

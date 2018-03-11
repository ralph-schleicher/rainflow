#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "ex.h"

/* User-defined signal history access function.  */
static size_t
read2 (struct sig1 *sig, double *buffer, size_t count)
{
  size_t c;

  for (c = count; c > 0; --c, ++sig)
    {
      /* Copy signal value.  */
      *buffer++ = sig->value;

      /* Copy signal label (argument BUFFER
	 is initialized with zeros).  */
      memcpy (buffer, &sig->label, sizeof (char *));

      ++buffer;
    }

  return count - c;
}

int
main (void)
{
  rs_rainflow_t *obj;

  /* Ensure that a pointer can be stored in a ‘double’.  */
  assert (sizeof (char *) <= sizeof (double));

  /* Create rainflow cycle counting object.  */
  obj = rs_rainflow_new ();
  if (obj == NULL)
    abort ();

  /* Enable signal labels.  */
  rs_rainflow_set_signal_label (obj, 1);

  /* Install a user-defined signal history access function
     and define the size of an array element of the signal
     history.  */
  rs_rainflow_set_read_signals (obj, (void *) read2, sizeof (struct sig1));

  /* Print cycle counting sequence to 'stdout'.  */
  rs_rainflow_set_shift_cycle (obj, (void *) sig1_print2, stdout);

  /* Perform rainflow cycle counting.  */
  if (rs_rainflow (obj, sig1, sig1_len, RS_RAINFLOW_FINISH) != 0)
    abort ();

  /* Destroy rainflow cycle counting object.  */
  rs_rainflow_delete (obj);

  return 0;
}

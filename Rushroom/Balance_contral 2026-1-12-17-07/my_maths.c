#include "my_maths.h"

/**
 * ªπŒ¥≤‚ ‘
 */
void location_pid(float p,float i,float d,float target,float now,float* controller,float* error,float* accumulated_error,float* pre_error)
{
    *pre_error = *error;
    *error = target - now;
    *accumulated_error += *error;

    *controller = p*(*error)+i*(*accumulated_error)-d*(*error-*pre_error);
}
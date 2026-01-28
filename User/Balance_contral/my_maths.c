#include "my_maths.h"

/**
 * »¹Î´²âÊÔ
 */
void location_pid(float p,float i,float d,float target,float now,float* controller,float* error,float* accumulated_error,float* pre_error,float I_MAX,float controller_MAX)
{
    *pre_error = *error;
    *error = target - now;
    *accumulated_error += *error;

    if (*accumulated_error > I_MAX)
    {
        *accumulated_error = I_MAX;
    }
    if (*accumulated_error < -I_MAX)
    {
        *accumulated_error = -I_MAX;
    }

    *controller = p*(*error)+i*(*accumulated_error)-d*(*error-*pre_error);
    if (*controller > controller_MAX)
    {
        *controller = controller_MAX;
    }
    if (*controller < -controller_MAX)
    {
        *controller = -controller_MAX;
    }
}

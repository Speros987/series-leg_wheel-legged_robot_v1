#ifndef  MATHS_H
#define MATHS_H

void location_pid(float p,float i,float d,float target,float now,float* controller,float* error,float* accumulated_error,float* pre_error,float I_MAX,float controller_MAX);

#endif

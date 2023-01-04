/*
 * PID.h
 *
 *  Created on: Oct 10, 2019
 *      Author: 16138
 */

#ifndef APPLICATION_PID_PID_H_
#define APPLICATION_PID_PID_H_

#include <stdbool.h>

typedef struct{
	// Public variables
	float KP;
	float KI;
	float KD;

	// Private parameters
	float minIpart;
	float maxIPart;
	float minDpart;
	float maxDPart;
	float minOut;
	float maxOut;

	// Private variables
	bool isFistCompute;
	float sp;
	float fb;
	float err;
	float preIPart;
	float preError;
	float P_Part;
	float I_Part;
	float D_Part;
	float U;
}pid_params_t;

void  pid_init(pid_params_t *pid_params, float KP, float KI, float KD, float minIpart,
	float maxIPart, float minDpart, float maxDPart,	float minOut, float maxOut);
float pid_compute(pid_params_t *pid_params, float sp, float fb, float dt);
void  pid_reset(pid_params_t *pid_params);

#endif /* APPLICATION_PID_PID_H_ */

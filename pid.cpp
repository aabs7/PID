/*
 * pid.cpp
 *
 * Created: 10/13/2017 6:23:02 PM
 *  Author: abheesh
 */ 

#define F_CPU 16000000UL

#include "pid.h"
#include <avr/io.h>
#include <util/delay.h>

pid::pid(void)
{
	kp = 0;
	ki = 0;
	kd = 0;
}


void pid::setTunings( double Kp, double Ki , double Kd)
{
	double sampleTimeInSec = ((double)sampleTime)/1000;
	kp = Kp;
	ki = Ki * sampleTimeInSec;
	kd = Kd / sampleTimeInSec;
}

void pid::setSampleTime(int newSampleTime)
{
	if (newSampleTime > 0)
	{
		double ratio = (double)newSampleTime / (double)sampleTime;
		ki *= ratio;
		kd /= ratio;
		sampleTime = (unsigned long)newSampleTime;
	}
	
}

void pid::setOutputLimits(double min, double max)
{
	if (min > max)	return;
	outMin = min;
	outMax = max;
	
	if (output > outMax)	 output = outMax;
	else if (output<outMin)	 output = outMin;
	
	if (iTerm > outMax)		 iTerm = outMax;
	else if(iTerm < outMin)	 iTerm = outMin;
}

void pid::computePid()
{
	_delay_ms(SampleTime);
	double error = setPoint - input;
	iTerm += (ki*error);
	
	if(iTerm>outMax)			iTerm = outMax;
	else if (iTerm < outMin)	iTerm = outMin;
	
	double dInput = (input - lastInput);
	output = kp * error + iTerm - kd * dInput;
	
	if(output>outMax)			output = outMax;
	else if (output<outMin)		output = outMin;
	
	lastInput = input;
	 
}


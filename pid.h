/*
 * pid.h
 *
 * Created: 10/13/2017 6:21:23 PM
 *  Author: abheesh
 */ 


#ifndef PID_H_
#define PID_H_

#define SampleTime 500

double input,output,setPoint;
double iTerm,lastInput;
double kp, ki , kd;
double outMin, outMax;
int sampleTime = SampleTime;	//0.5 sec sample time


class pid{
	public:
		double input,output,setPoint;
		double iTerm,lastInput;
		
		double outMin, outMax;
		int sampleTime = SampleTime;	//0.5 sec sample time
		
		pid(void);
		void computePid(void);
		void setTunings(double Kp, double Ki, double Kd);
		void setSampleTime(int newSampleTime);
		void setOutputLimits(double min, double max);
	private:
		double kp, ki , kd;
};





#endif /* PID_H_ */
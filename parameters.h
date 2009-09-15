///////////////////////////////////////////////////////////////////////
//////////////////////                         ////////////////////////
//////////////////////       parameters.h      ////////////////////////
//////////////////////                         ////////////////////////
///////////////////////////////////////////////////////////////////////
//#include "globals.h"
//
//		Parameters required for operation and tuning go here
//
//
//#define 	CPUCLK_PARAM   		8000000		//	The speed of the cpu currently in use

///////////////////////////////////////////////////////////////////////
//	Application Specific Parameters
///////////////////////////////////////////////////////////////////////
//
//	Balancing Bot 
//
//	dt_PARAM is how often the Kalman state is updated with
//	gyro rate measurements.  You will have to
//	change this value if you update at a different rate.
//	For example, the 50Hz is based upon the call frequency
//	of balance() from make_it_balance().
//	If xFrequency is changed in make_inBalance() then this
//	should be changed and vice versa.
//
//	40Hz Rate for debugging because of so many rprintfs 
//define		dt_PARAM		0.100		//	10 Hz rate
//#define		dt_PARAM		0.01		//	100 Hz rate
#define		dt_PARAM		0.02		//	50 Hz rate
#define 	my_dt_PARAM 		20		// 50 Hz rate

//	R may be used for tuning Kalman
//	R represents the measurement covariance noise.  In this case,
//	it is a 1x1 matrix that says that we expect 0.3 rad jitter
//	from the accelerometer.
//
//	Adjusting R changes the speed at which the filter converges.
//	It is an indication of trust in the measurement
#define		R_angle_PARAM		0.1			//	n rad jitter  / .3
//  #define R_angle 0.4 // 0.0034906555555555555556; // 0.3;

//	Q may be used for tuning Kalman
//	Q is a 2x2 matrix that represents the process covariance noise.
//	In this case, it indicates how much we trust the acceleromter
//	relative to the gyros.
//
// 	originally .001 and .003
#define		Q_angle_PARAM		0.001
#define		Q_gyro_PARAM		0.001


// These are measured values.  They will change with sensor 
// orientation, so be careful to change these when necessary.
//
// value at 0 degrees / sec - unimportant, as the Kalman filter corrects this
//#define 	GYRO_OFFSET 		613		
#define 	GYRO_OFFSET 		580
		
// value at 0 degrees  / 1 unit .4 degees  (move to +90 -90 / 2)
//#define 	ACCEL_OFFSET 		650	
//#define 	ACCEL_OFFSET 		532   // 540	
#define 	ACCEL_OFFSET		740   // 647, 733, 744

// gyro is 2 mv/(deg/sec), ADC is 2.5mv/tick: 2/2.5 ; degrees/sec = adc * .8 
// gyro is 2 mv/(deg/sec), ADC is 3.22mv/tick: 2/3.3  ; degrees/sec = adc * .62
// ADXRS401 gyro is 15mv/(deg/sec), ADC is 3.22mv/bit: 15/3.22  ; degrees/sec = adc * 4.65
// ADXRS401 gyro is 15mv/(deg/sec), ADC is 4.88mv/bit: 15/4.88  ; degrees/sec = adc * 3.07
#define 	GYRO_SCALE 		-4.65


// ADC bits per 90 degrees
#define 	ACCEL_SCALE 		0x0d8	
//#define 	ACCEL_SCALE 		106.0	


//	Neutral or Natural balance angle
//	You might change this by .1 at a time until tuned    
//#define	neutral_PARAM 	-7.7		// "angle of natural balance"  
#define		neutral_PARAM 	0		// "angle of natural balance"  

//	Motor Bias - left and right
#define		mb_PARAM 	5		//	PWM required to get the motor turning
#define         MAX_TORQUE	255

// 	PID adjustable gains
#define Kp_PARAM 		1000	    // balance loop P gain
#define Ki_PARAM 		80	    // balance loop I gain
#define Kd_PARAM 		0.1         // balance loop D gain
#define Kf_PARAM 		4	    // balance loop f gain

#define RAD_TO_DEGREE	(180 / 3.1415926535897932384626433832795)


//	Robot statistics. Can be used for calculating torque.
#define rob_weight_PARAM	0.630	 // Weight of robot measured in kilogram.
#define rob_lenght_PARAM 	0.22 	 // Height of robot measured in meter.
#define rob_wheel_PARAM 	0.021	 // Radius of Wheels measured in meter.
#define rob_motor_kg_PARAM	0.002548 // Torque of motors measured in kilogram meter.
#define gravity_PARAM		9.8 	 // m/s gravity. This should be 0.098 if mydt is 10. 


//  Analog Pins on Arduino
#define x_pin 		0
#define gyro_pin 	1 
#define gyroref_pin     2
#define pot_pin 	4
#define pot_pin2        5

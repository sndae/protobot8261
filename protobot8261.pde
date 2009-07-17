/*
*     Based on Trammell Hudsons work on the autopilot system.
 *     see http://www.rotomotion.com/downloads/tilt.c for original
 *     code. Although, this version is based on James Ronald's
 *     c implementation found at Google code; http://code.google.com/p/b3r1/.
 *     For excessive comments on Kalman look at the tilt.pde code.
 *     Tilt.pde is almost unchanged 
 *     Thanks also to Michael P. Thompson for his work on the 
 *     Kalman implementation in C, making it easier for us 
 *     non-mathematicians. 
 
 *     This code is created for a 3 DOF IMU board, measuring
 *     tilt and angular velocity fusing the sensors with the 
 *     Kalman filter to adhence for noise and predict it's 
 *     movement. 
 
 *     Simon Zimmermann, June 2009.
 */
#include <math.h>
#include <PushButton.h>
#include <CompactQik2s9v1.h>
#include <NewSoftSerial.h>
#include "parameters.h"

NewSoftSerial mySerial =  NewSoftSerial(rx_pin, tx_pin);
CompactQik2s9v1 motor = CompactQik2s9v1(&mySerial,rst_pin);
PushButton pushButton(2);

// ADC (Built in 10 bit 0-5V) Information.
// Each ADC unit equals: 5.0V / 1024 = 0.004882812 V = 4.8828125 mV
// Each ADC unit equals: 3.3V / 1024 = 0.003222656 V = 3.22265625 mV
//
// Rate Gyro (ADXRS401) Information:
// Rate Gyro measures rate at 15 mV/degree/second
// Each ADC Gyro unit equals: 4.8828125 / 15.0 = 0.325520833 degrees/sec
// Each ADC Gyro unit equals: 3.22265625 / 15.0 = 0.21484375 degrees/sec
//
// Acceleromter (ADXL203) Information:
// Accelerometer measures acceleration at 1000 mV/g
// Each ADC unit equals: 4.8828125 / 1000mv/G = 0.0048828125 g
// Each ADC unit equals: 3.22265625 / 1000mv/G = 0.003222656 g

// Balance
int torque;
int angle_cnt = 0;
int rate_cnt = 0;

double angle_drift = 0.0;
double angle_velocity = 0.0;
double angle_x = 0.0;
double angle_avg[10];
double rate_avg[50];     // rate average.

/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
 
double	angle = 0.0;
double	q_bias = 0.0;
double	rate = 0.0;

//	PID Konstants
double Kp = 25;				// balance loop P gain
double Ki = 5;				// balance loop I gain
double Kd = 1;    			// balance loop D gain

//double Ksteer;			// steering control gain
//double Ksteer2;			// steering control tilt sensitivity
//double Kspeed; 			// speed tracking gain
double neutral = neutral_PARAM;		// "angle of natural balance"

//double KpTurn;			// turn rate loop P gain
//double KiTurn;			// turn rate loop I gain
//double KdTurn;			// turn rate loop D gain

// Analog Input
double pot;			 	// analog input from Potentiometer

// Time
unsigned long last_read; 		// Last time we ran our script

// Init mode
void setup()
{
    pinMode(led1_pin, OUTPUT); 		// LED 1
    pinMode(led2_pin, OUTPUT);		// LED 2
    analogReference(EXTERNAL); 		// 3.3v External Ref on Arduino
    Serial.begin(38400);		// Serial at 38,400 Baud rate.
    mySerial.begin(38400);		// Serial at 38,400 Baud rate used for motor.
    motor.begin();			// Init Motor controller.
    motor.stopBothMotors();		// Stop.
    delay(200);    			// Wait 0.2 sec.
}

// Constant Loop for our bot. 
void loop() {
    if(pushButton.getState()){
        digitalWrite(led1_pin, HIGH);   // Light a LED
        digitalWrite(led2_pin, LOW);	// Lid a LED
        balance(); 			// Run balance script.
    }
    else{
        digitalWrite(led1_pin, LOW); 	// Lid a LED
        digitalWrite(led2_pin, HIGH);	// Light a LED
    }
}

/* BALANCE my robot, please */
void balance(void)
{
	long int g_bias = 0;
//	double x_offset = 532;		// offset value 2.56V * 1024 / 4.93V = 4254
	double q_m = 0.0;               // gyro in degrees per second
	double int_angle = 0.0;           
	double x = 0.0;                 // raw x value
	double tilt = 0.0;              // X * (pi/180.0). Will store x in degree's
        double last_angle, last_tilt = 0.0;
        

	/* as a 1st step, a reference measurement of the angular rate sensor is 
	 * done. This value is used as offset compensation */
	
	for (int i=1 ; i<=200; i++) // determine initial value for bias of gyro
	{
		g_bias = g_bias + analogRead(gyro_pin);
	}
	
	g_bias = g_bias / 200;


    while (pushButton.getState())
	{
		/* insure loop runs at specified Hz */
		while(millis() - last_read < my_dt_PARAM)
			;
		last_read = millis();
		
		// Get input from our Potentiometer used for debugging
		pot = map(analogRead(pot_pin), 0, 1023, 0, 25);
                
		// get rate gyro reading and convert to deg/sec
		// q_m = (GetADC(gyro_sensor) - g_bias) / -3.072;	// -3.07bits/deg/sec (neg. because forward is CCW)
                // q_m = (analogRead(gyro_pin) - q_bias) / GYRO_SCALE;      // 3.3v / 1024-bit / 0.15mV * 10
                //q_m = (analogRead(gyro_pin) - g_bias) * -0.325520833;	// 5v / each bit = 0.3255 /deg/sec 
                q_m = (analogRead(gyro_pin) - q_bias) * -0.21484375;      // 3.3v / 1024-bit / 0.15mV * 10
		state_update(q_m);                                	// Update Kalman filter
		
		// get Accelerometer reading and convert to units of gravity.  
                // x = (GetADC(accel_sensor) - x_offset) / 204.9;	// (205 bits/G)
                //x = (analogRead(x_pin) - ACCEL_OFFSET) * 0.0048828125;	// each bit = 0.00322/G
		x = (analogRead(x_pin) - ACCEL_OFFSET) * 0.003222656;	// each bit = 0.00322/G

		// x is measured in multiples of earth gravitation g
		// therefore x = sin (tilt) or tilt = arcsin(x)
		// for small angles in rad (not deg): arcsin(x)=x 
		// Calculation of deg from rad: 1 deg = 180/pi = 57.29577951
		tilt = 57.29577951 * (x);
		kalman_update(tilt);
		
		int_angle += angle * dt_PARAM;
                angle_velocity = (tilt - last_tilt)* 60 ;               
                last_tilt = tilt;
                
                angle_avg[angle_cnt] = tilt;
                angle_cnt++;
                if(angle_cnt > 9)
                    angle_cnt = 0;
                angle_x = getAverageDouble(angle_avg, 10);
		angle_drift = (angle - angle_x);

                // Rate average calculations               
                rate_avg[rate_cnt] = rate;
                rate_cnt++;
                if(rate_cnt > 19)
                    rate_cnt = 0;
                rate = getAverageDouble(rate_avg, 20);
		
                
                // Balance.  The most important line in the entire program.
		torque = ((Kp * ((angle_x /*- (-157.7 - q_bias)*/) + angle_drift)) + (rate * pot); // + (int_angle * Ki); 
               // torque = (int) (angle_velocity) + (rate * 3); //+ (int_angle * Ki);
		// change from current angle to something proportional to speed
		// should this be the abs val of the cur speed or just curr speed?
		//double steer_cmd = (1.0 / (1.0 + Ksteer2 * fabs(current_angle))) * (Ksteer * steer_knob);
		//double steer_cmd = 0.0;

		// Get current rate of turn
		//double current_turn = left_speed - right_speed; //<-- is this correct
		//double turn_accel = current_turn - prev_turn;
		//prev_turn = current_turn;

		// Closed-loop turn rate PID
		//double steer_cmd = KpTurn * (current_turn - steer_desired)
		//					+ KdTurn * turn_accel;
		//					//+ KiTurn * turn_integrated;

		// Possibly optional
		//turn_integrated += current_turn - steer_cmd;

		//	Differential steering
		//left_motor_torque	= balance_torque + steer_cmd; //+ cur_speed + steer_cmd;
		//right_motor_torque	= balance_torque - steer_cmd; //+ cur_speed - steer_cmd;

		// Limit extents of torque demand
		//left_motor_torque = flim(left_motor_torque, -MAX_TORQUE, MAX_TORQUE);
//		if (left_motor_torque < -MAX_TORQUE) left_motor_torque = -MAX_TORQUE;
//		if (left_motor_torque > MAX_TORQUE)  left_motor_torque =  MAX_TORQUE;

		//right_motor_torque = flim(right_motor_torque, -MAX_TORQUE, MAX_TORQUE);
//		if (right_motor_torque < -MAX_TORQUE) right_motor_torque = -MAX_TORQUE;
//		if (right_motor_torque > MAX_TORQUE)  right_motor_torque =  MAX_TORQUE;
                
		//torque = (int) ((angle -3.5) * Kp) + (rate * Kd);//  + (int_angle * Ki);

		// Set PWM values for both motors
                // See motor.pde for more comments on the theoretics behind torque. etc.
		driveMotors(torque);
		printData();		// Print data for debugging.
	}
    motor.stopBothMotors();			// Stop.
}

/* Print data from acc and gyro. 
 */
void printData()
{
    Serial.print(int(angle));
    Serial.print(",");
    Serial.print((angle - angle_drift)); /*(-157.7 - q_bias)) -*/
    Serial.print(",");
    Serial.print(int(rate));
    Serial.print(",");
    Serial.print(int(q_bias));
    Serial.print(",");
    Serial.println(pot);
}

/* Convert ADC to volt.
 @param ADC 10-bit value to convert
 @return volt of ADC.
 */
float getVolt(float adc)
{
    return (adc / 1024.0) * 3.3;
}

/* Get average of a floating point array */
double getAverageDouble(double avg[], int size)
{
    double sum = 0.0;
    for (int i = 0; i < size; i++)
        sum += avg[i];
    return sum / size;
}

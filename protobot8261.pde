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
#include "parameters.h"

PushButton pushButton(2);

// Motor pins stored in array. 
int motor_left[] = {5, 6};
int motor_right[] = {9, 10};

// Settings 
boolean debug = false;

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
int torque, p_term, d_term, f_term, i_term, old_angle, rate_cnt = 0;


// PID Konstants
int Kp = 1000;			        // balance loop P gain
int Ki = 80;				// balance loop I gain
double Kd = 0.1;                        // balance loop D gain
int Kf = 4;			        // balance loop f gain


/*
 * Our two states, the angle and the gyro bias.  As a byproduct of computing
 * the angle, we also have an unbiased angular rate available.   These are
 * read-only to the user of the module.
 */
double angle = 0.0;                    // after kalman filter result in degrees
double angle_inv = 0.0;                // inverted case of kalman res degrees
double q_bias = 0.0;
double rate = 0.0;                      // Angular velocity
int rate_inv = 0.0;                  // inverted case of rate
int rate_vel = 0.0;                  // product of several rate records.
int rate_avg[20];                    // rate average Array.
double q_m = 0.0;                       // Raw gyro in degrees per second.
double x = 0.0;                         // Raw X value.
double tilt = 0.0;                      // X * (pi/180.0). Will store X in degree's
//double x_offset = 532;		// offset value 2.56V * 1024 / 4.93V = 4254
long int g_bias = 0;


// Analog Input
int pot;			 	// analog input from Potentiometer
int pot2;                               // analog input from Potentiometer 


// Time
unsigned long last_read; 		// Last time we ran our script


// Init mode
void setup()
{
    pinMode(led1_pin, OUTPUT); 		// LED 1
    pinMode(led2_pin, OUTPUT);		// LED 2
    for(int i = 0; i < 2; i++){
        pinMode(motor_left[i], OUTPUT);
        pinMode(motor_right[i], OUTPUT);
    }
    analogReference(EXTERNAL); 		// 3.3v External Ref on Arduino
    if(debug)
        Serial.begin(115200);		// Serial at 115,200 Baud rate.
    delay(200);    			// Wait 0.2 sec.
}


// Loop for our bot. 
void loop() {
    if(pushButton.getState()){
        digitalWrite(led1_pin, HIGH);   // Light a LED
        digitalWrite(led2_pin, LOW);	// Lid a LED
        balance(); 			// Run balance script.
    }
    else{
        digitalWrite(led1_pin, LOW); 	// Lid a LED
        digitalWrite(led2_pin, HIGH);	// Light a LED
        motor_stop();
    }
}


/* BALANCE my robot, please */
void balance(void)
{
    /*
     * As a 1st step, a reference measurement of the angular rate sensor is 
     * done. This value is used as offset compensation 
     */

    for (int i=1 ; i<=200; i++)
    {
        g_bias = g_bias + analogRead(gyro_pin);
    }
    g_bias = g_bias / 200;

    /*
    * As a 2nd step, we run the game loop. This will continue until the user
     * press the off switch. The loop runs at a specific Hz which is set based
     * on hardware. It's limited by sensor refresh rate and analog to DC conversions. 
     */
    while (pushButton.getState())
    {
        /* insure loop runs at specified Hz */
        while(millis() - last_read < my_dt_PARAM)
            ;
        last_read = millis();

        // Get input from our Potentiometer used for debugging
        pot = map(analogRead(pot_pin), 0, 1023, 50, 1000);
        pot2 = map(analogRead(pot_pin2), 0, 1023, 670, 740);

        // get rate gyro reading and convert to deg/sec
        // q_m = (GetADC(gyro_sensor) - g_bias) / -3.072;	        // -3.07bits/deg/sec (neg. because forward is CCW)
        // q_m = (analogRead(gyro_pin) - q_bias) / GYRO_SCALE;          // 3.3v / 1024-bit / 0.15mV * 10
        // q_m = (analogRead(gyro_pin) - g_bias) * -0.325520833;	// 5v / each bit = 0.3255 /deg/sec 
        q_m = (analogRead(gyro_pin) - q_bias) * -0.21484375;            // 3.3v / 1024-bit / 0.15mV * 10
        state_update(q_m);                                	        // Update Kalman filter
        rate_inv = getInvertedDouble(rate);

        // Rate average calculations. Due to noise rate has to be avaraged out.
        rate_vel = getAverageRate(rate_inv, 15);

        // get Accelerometer reading and convert to units of gravity(G).  
        // x = (GetADC(accel_sensor) - x_offset) / 204.9;	        // (205 bits/G)
        // x = (analogRead(x_pin) - ACCEL_OFFSET) * 0.0048828125;	// each bit = 0.00322/G
        x = (analogRead(x_pin) - pot2) * 0.003222656;	                // each bit = 0.00322/G

        // x is measured in multiples of earth gravitation g
        // therefore x = sin (tilt) or tilt = arcsin(x)
        // for small angles in rad (not deg): arcsin(x)=x 
        // Calculation of deg from rad: 1 deg = 180/pi = 57.29577951
        tilt = 57.29577951 * (x);
        kalman_update(tilt);

        // PID. Calculate our motor gain or loss.
        p_term = tilt * Kp;
       // i_term = (i_term + (tilt * Ki)) * .9999;
        d_term = (tilt - old_angle) * Kd;
        f_term = (rate_inv * Kf) + (rate_vel * (Kf + 2));

        old_angle = tilt;

        // Balance.  The most important line in the entire program.
        torque = (int) p_term + d_term + f_term;

        /*overspeed? if(torque > 800 || torque < -800)
         torque *= 1.2;*/

        // Set PWM values for both motors
        // See motor.pde for more comments on the theoretics behind torque. etc.
        driveMotors(torque);

        // Print data for debugging.
        if(debug)
            printData();
    }
}

/* 
 * Print data from acc and gyro. 
 */
void printData()
{
    Serial.print(angle);
    Serial.print(",");
    Serial.print(tilt); 
    Serial.print(",");
    Serial.print(rate);
    Serial.print(",");
    Serial.print(q_bias);
    Serial.print(",");
    Serial.println(pot2);
}

/*
 *   Return the average for our rate sensor
 */
int getAverageRate(double _rate, int size){
    rate_avg[rate_cnt] = (int) _rate;
    rate_cnt++;
    if(rate_cnt == size)
        rate_cnt = 0;
    return getAverageDouble(rate_avg, size);
}

/* 
 * Return average of a floating point array 
 */
int getAverageDouble(int avg[], int size)
{
    int sum = 0.0;
    for (int i = 0; i < size; i++)
        sum += avg[i];
    return sum / size;
}
/* 
 * Return the inverted value of given imput 
 */
double getInvertedDouble(double value)
{
    return (value - (value * 2));
}

/* 
 * Convert ADC to volt.
 @param ADC 10-bit value to convert
 @return volt of ADC.
 */
double getVolt(double adc)
{
    return (adc / 1024.0) * 3.3;
}

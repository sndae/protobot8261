/*
  To get the Newton meter of motors we take motors kg meter value multiply 9.8^2 
 [0.002548 kg m * 9.8m/s^2 = 0.24471 N m] This is the Motors torque, its strenght.
 [(0.002548 kg m * 2) * 9.8m/s^2 = 0.48942 N m] (for two motors).
 Since we have wheels the we'll have to find the force at the lever arm (force at wheel)
 Formula: Force at wheels = torque / radius. 
 [0.24471 N m / 0.021 m = 11.6529 N]
 [0.48942 N m / 0.021 m = 23.3057 N] (for two motors)
 (note: Adding bigger wheels might be smart, bigger wheels will drive the robot faster, 
 thus lowering the overal torque) 
 To level the robot at a given angle we'll have to know the height of the robot
 so we can find the force we need to counter it. 
 Torque applied [Force at wheels * length * cos(degrees)] has to be bigger than
 the torque applied by gravity to overcome it [mass * gravity * length * sin(degrees)]
 
 Torque: Force * Radius
 
 Example I: if the angle of the robot is 10 degrees the torque applied by gravity is
 [0.650 kg * 9.8 m/s * 0.22 L * sin(10) = 0.243351 N m]
 1.17924
 Lets now calculate the torque we need to counter this.
 [11.6529 * 0.22 L * cos(10) = 2.52469 N]
 [23.3057 * 0.22 L * cos(10) = 5.04936 N] (for two motors)
 
 Example II: trying out some new formulas; cant make sence of earlier calc.
 0.650 Kg  * 9.8 m/s * 0.22 L * (radians(20 degree) * PI) = 0.569751 N m
 20 degree in radians is 0.3491. using the above formula gives me a proportional
 incline based on degree of robot. 
 */
int prop_speed = 1000;
#include "parameters.h"

void driveMotors(int pwm)
{
    if(pwm >= 0){
        pwm = constrain(pwm, 0, prop_speed);
	pwm = map(pwm, 0, prop_speed, 0, MAX_TORQUE);       
        /*motor.motor0Forward(pwm);
        motor.motor1Forward(pwm);*/
        drive_backward(pwm);
    }
    else{
        pwm = abs(pwm);
        pwm = constrain(pwm, 0, prop_speed);
	pwm = map(pwm, 0, prop_speed, 0, MAX_TORQUE);
       /* motor.motor0Reverse(pwm);
        motor.motor1Reverse(pwm);*/
        drive_forward(pwm);
    }
}

void motor_stop()
{
    digitalWrite(motor_left[0], LOW);
    digitalWrite(motor_left[1], LOW);

    digitalWrite(motor_right[0], LOW);
    digitalWrite(motor_right[1], LOW);
}

void drive_forward(int pwm)
{
    analogWrite(motor_left[0], pwm);
    analogWrite(motor_left[1], LOW);

    analogWrite(motor_right[0], pwm);
    analogWrite(motor_right[1], LOW);
}

void drive_backward(int pwm)
{
    analogWrite(motor_left[0], LOW);
    analogWrite(motor_left[1], pwm);

    analogWrite(motor_right[0], LOW);
    analogWrite(motor_right[1], pwm);
}

void turn_left()
{
    digitalWrite(motor_left[0], LOW);
    digitalWrite(motor_left[1], HIGH);

    digitalWrite(motor_right[0], HIGH);
    digitalWrite(motor_right[1], LOW);
}

void turn_right()
{
    digitalWrite(motor_left[0], HIGH);
    digitalWrite(motor_left[1], LOW);

    digitalWrite(motor_right[0], LOW);
    digitalWrite(motor_right[1], HIGH);
}

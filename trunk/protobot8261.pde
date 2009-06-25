#include <PushButton.h>
#include <math.h>

/******* DECLEAR PINS *********/
int led1 = 13;
int gyro_pin = 2;
int ref_pin = 4;
int y_pin = 0;
int x_pin = 1;

/******* Variables *********/
int gyro = 0; // gyro reading
int gyro_ref = 0; // 2.5v reference
int y = 0; // y acc reading
int x = 0; // x acc reading
int time = 0; // time
long int previous = 0; // last run
#define average_size 10
float y_average[average_size];
int average_count = 0; // keeps track of average count
float y_zero = 0; 
float y_zero_g = 0;
float x_average[average_size];
float x_degree = 0; // x acc in degree
float x_zero = 0; // x acc zero point calibrated at reset
float x_zero_g = 0; // x acc zero point in g calculated at reset
int g = 97; // ±1g voltage of 0.312V is (0.312V/3.3V) * 1024 = 97 
int nominal_g = 776; // Nominal G for y-axis based on the fact that
					 // zero g is supposed to be 2.5v ((2.5V/3.3V) * 1024 = 776)
int nominal_xg = 747;					 
 
PushButton pushButton(2);

void setup(){
    Serial.begin(9600);
    pinMode(led1, OUTPUT);
    
    analogReference(EXTERNAL);
    y_zero = calibrateY();
    x_zero = calibrateX();

    delay(200);
}

void loop(){
    if(pushButton.getState()){
        digitalWrite(led1, HIGH);
            
        gyro = analogRead(gyro_pin);
        gyro_ref = analogRead(ref_pin);
        y = analogRead(y_pin);
        x = analogRead(x_pin);
        
        if(millis() - previous >= time){
           previous = millis();
            processData();
        }
        
    }
    else{
         digitalWrite(led1, LOW);
    }
}

/* Process the data from our IMU board. Using global vars
 */
void processData()
{
    // theta = acos (accelerometer / gravity)
    // accelerometer = cos (theta) * gravity
    // pitch = asin (accelerometer / gravity)
    // (2.5V/5V) * 1024 = 512.

    ////// Volt Conversion ///////
    float x_volt = (x / 1024.0) * 3.3;
    /******** Accelerometer Calculations ********/
    float y_diff = y - nominal_g; // get the difference between y acc and y - zero point. 
    float y_g = (y_diff / g); 
    y_g -= y_zero_g; // Adjusting according to zero g for the axis. 
    y_g = constrain(y_g, -0.999999, 0.999999); // Error adjust 
    float y_radians = asin(y_g); // get radians based on g
    float y_degree = (180.0 / PI) * y_radians; // calculate degree
    
    float x_diff = x - nominal_g;
    float x_g = x_diff / g;
    x_g -= x_zero_g;
    x_g = constrain(x_g, -0.999999, 0.999999); // Error adjust 
    float x_radians = asin(x_g);
    x_degree = (180.0 / PI) * x_radians;

    ////// Gyro Calculations ///////
    // Volt /  10-bit * milliV / degree per second / radians to degree.
    float conversion =  (3.3 / 1024) * 1000 / 0.15; // 57.29578;
    float gyro_radians =  ((gyro - nominal_g) * conversion) ;
    
    ////// Create Average //////
    y_average[average_count] = y_degree;
    x_average[average_count] = x_degree;
    average_count++;
    if(average_count == average_size)
        average_count = 0;
    float yaverage = getAverage(y_average, average_size);
    float xaverage = getAverage(x_average, average_size);
    
    ////// Print Data /////////
    /// Y - Debug ///
    /*Serial.print("Y-average: ");
    Serial.print(yaverage);
    Serial.print(" Y-degree: ");
    Serial.print(y_degree);
    Serial.print(" Zero: ");
    Serial.print(y_zero);
    Serial.print(" Y-diff: ");
    Serial.print(y_diff);
    Serial.print(" Y-g: ");
    Serial.print(y_g);  
    Serial.print(" Y-rad: ");
    Serial.println(y_radians);*/
    
    /// X  - Debug ///
   /* Serial.print("x-average: ");
    Serial.print(xaverage);
    Serial.print(" x-degree: ");
    Serial.print(x_degree);
    Serial.print(" Zero: ");
    Serial.print(x_zero);
    Serial.print(" x-diff: ");
    Serial.print(x_diff);
    Serial.print(" x-g: ");
    Serial.print(x_g);  
    Serial.print(" x-rad: ");
    Serial.println(x_radians);*/
    
    /// PROCESSING COMMUNICATION ///
/*    Serial.print(y_degree);
    Serial.print(",");
    Serial.print(yaverage);
    Serial.print(",");
    Serial.print(x_degree);
    Serial.print(",");
    Serial.print(xaverage);
    Serial.print(",");*/
    Serial.println(x_volt);    
}

/* Calibration routine, ran at startup to get zero g. */
float calibrateY(){
    int total = 0;
    for(int i = 0; i < 10; i++){
        y = analogRead(y_pin);
        total += y;
        delay(2);
    }
    y = total / 10;
    float diff = y - nominal_g;
    y_zero_g = diff / g;
    float radian = asin(y_zero_g);
    return (180/PI)*radian;
}

/* Calibration routine, ran at startup to get zero g. */
float calibrateX(){
    int total = 0;
    for(int i = 0; i < 10; i++){
        x = analogRead(x_pin);
        total += x;
        delay(2);
    }
    x = total / 10;
    float diff = x - nominal_g;
    x_zero_g = diff / g;
    float radian = asin(x_zero_g);
    return (180/PI)*radian;
}

/* Return the average based on a array of numbers 
 @returns the average based on the numbers in the array
 */
float getAverage(float average_arr[], int size)
{
    float sum = 0;
    for(int i = 0; i < size; i++){
        sum += average_arr[i];
    }
    return (sum / size);
}

/* Convert ADC to volt.
 @param ADC 10-bit value to convert
 @return volt of ADC.
 */
float getVolt(float adc)
{
    return (adc / 1024.0) * 3.3;
}

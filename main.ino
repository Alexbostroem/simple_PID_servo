
#include <PID_v1.h>
#include <MegunoLink.h>
#include <Filter.h>

#include <Servo.h>
#define SERVO_PIN 10
#define Xaxis A0
#define Yaxis A1
#define Zaxis A2
#define sampleSize 25

long FilterWeight = 15;
ExponentialFilter<long> ADCFilter(FilterWeight, 0);

Servo servo;

double Setpoint, Input, Output;

PID myPID(&Input, &Output, &Setpoint,0.5,3,0, DIRECT);

long read_data(int axis) {
    long readings = 0;
    analogRead(axis);
    delay(1);
    for (int i = 0; i < sampleSize; i++)
    {
       readings += analogRead(axis);
    }

    return readings / sampleSize;
    
}

float process_data(long raw) {
    float value = ( ( ( (float)(raw * 3.3)/1024) - 1.64 ) / 0.330 );
    return value;
}

float calc_pitch(float x, float y, float z) {
    float pitch;
    pitch = 180 * atan (x/sqrt(y*y + z*z))/M_PI;
  
    return pitch;
} 


void setup() {

servo.attach(SERVO_PIN);
Serial.begin(9600);
analogReference(EXTERNAL);

float x = process_data(read_data(Xaxis));
float y = process_data(read_data(Yaxis));
float z = process_data(read_data(Zaxis));


Input = calc_pitch(x,y,z);

Setpoint = 90;

myPID.SetMode(AUTOMATIC);



}

void loop() {
float x = process_data(read_data(Xaxis));
float y = process_data(read_data(Yaxis));
float z = process_data(read_data(Zaxis));
ADCFilter.Filter(calc_pitch(x,y,z));
Input = (double)map(ADCFilter.Current(), -90,90,0,180);
myPID.Compute();
servo.write(90 - Output);

}
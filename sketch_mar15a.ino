#include <math.h>
#include "Rotary.h"
#include "PIDController.h" //TODO: DISCUSS POTENTIAL REMOVAL

#define _USE_MATH_DEFINES
#define PPR 546
#define MAX_ANG_V (55/9)*M_PI

//encoder params
long counter1 = 0;
long lastCounter1 = 0;
unsigned long lastTime1 = 0;

long counter2 = 0;
long lastCounter2 = 0;
unsigned long lastTime2 = 0;

long counter3 = 0;
long lastCounter3 = 0;
unsigned long lastTime3 = 0;

long counter4 = 0;
long lastCounter4 = 0;
unsigned long lastTime4 = 0;

unsigned long lastTime = 0;

Rotary enc4 = Rotary(10, 11);
Rotary enc3 = Rotary(12, 13);
Rotary enc2 = Rotary(A0, A1);
Rotary enc1 = Rotary(A2, A3);

//PWM Variables
float x = 127;//(cos(t)/1)*255;
byte pwmcounter = 0; // Naturally limited to 0-255

byte pwms[] = { 0, x, 0, x, 0, x, 0, x}; 
byte pwms_0[] = { 0, 0, 0, 0, 0, 0, 0, 0}; 

double t = 0;

//FUNCTIONS
float getSpeed(long currentCount, long &lastCount, unsigned long &lastTime) {
  unsigned long currentTime = micros();
  float deltaT = (currentTime - lastTime) / 1e6; // Convert to seconds

  if (deltaT == 0) return 0; // Avoid division by zero

  float speed = ((currentCount - lastCount) * 360.0) / (PPR * deltaT);
  
  lastTime = currentTime;
  lastCount = currentCount;

  return speed * (M_PI / 180); // Returns speed in radians per second
}

void setSpeed(float rads) {
  x = (rads/19.198621772)*(255);
}

void setup() {
  DDRD |= 0xfc; // Set D2-D7 as outputs
  DDRB |= 0x03; // Set D8-D9 as outputs
  pinMode(10, INPUT_PULLUP); // Motor 4 encoder A
  pinMode(11, INPUT_PULLUP); // Motor 4 encoder B
  pinMode(12, INPUT_PULLUP); // Motor 3 encoder A
  pinMode(13, INPUT_PULLUP); // Motor 3 encoder B
  pinMode(A0, INPUT_PULLUP); // Motor 2 encoder A
  pinMode(A1, INPUT_PULLUP); // Motor 2 encoder B
  pinMode(A2, INPUT_PULLUP); // Motor 1 encoder A
  pinMode(A3, INPUT_PULLUP); // Motor 1 encoder B
  Serial.begin(230400);
}

void loop() {
  t += 0.0005;
  float s = (cos(t)/1)*((33/9)*M_PI);
  setSpeed(s);
  
  // PWM writing logic
  byte selected_pwms[] = {x,0,x,0,x,0,x,0};
  if(x < 0){
    selected_pwms[0] = 0;
    selected_pwms[1]= abs(x);
    selected_pwms[2] = 0;
    selected_pwms[3] = abs(x);
    selected_pwms[4]= 0;
    selected_pwms[5]= abs(x);
    selected_pwms[6]= 0;
    selected_pwms[7]= abs(x);
  }

  PORTD ^= (-(pwmcounter >= selected_pwms[0]) ^ PORTD) & (1 << 2); // D2
  PORTD ^= (-(pwmcounter >= selected_pwms[1]) ^ PORTD) & (1 << 3); // D3
  PORTD ^= (-(pwmcounter >= selected_pwms[2]) ^ PORTD) & (1 << 4); // D4
  PORTD ^= (-(pwmcounter >= selected_pwms[3]) ^ PORTD) & (1 << 5); // D5
  PORTD ^= (-(pwmcounter >= selected_pwms[4]) ^ PORTD) & (1 << 6); // D6
  PORTD ^= (-(pwmcounter >= selected_pwms[5]) ^ PORTD) & (1 << 7); // D7

  PORTB ^= (-(pwmcounter >= selected_pwms[6]) ^ PORTB) & (1 << 0); // D8
  PORTB ^= (-(pwmcounter >= selected_pwms[7]) ^ PORTB) & (1 << 1); // D9

  pwmcounter += 1;
  
  // Encoder reading logic
  unsigned char result1 = enc1.process_n(&PINC, 2, 3);
  unsigned char result2 = enc2.process_n(&PINC, 0, 1);
  unsigned char result3 = enc3.process_n(&PINB, 4, 5);
  unsigned char result4 = enc4.process_n(&PINB, 2, 3);

  if (result1 == DIR_CW) counter1++;
    else if (result1 == DIR_CCW) counter1--;

  if (result2 == DIR_CW) counter2++;
    else if (result2 == DIR_CCW) counter2--;
  
  if (result3 == DIR_CW) counter3++; 
    else if (result3 == DIR_CCW) counter3--;
    
  if (result4 == DIR_CW) counter4++; 
    else if (result4 == DIR_CCW) counter4--;

  // speed calculation
  unsigned long currentTime = micros();
  float deltaT = (currentTime - lastTime) / 1000000.0; // Time in seconds

  if(deltaT >= 0.05){
    double w1 = getSpeed(counter1, lastCounter1, lastTime1);
    double w2 = getSpeed(counter2, lastCounter2, lastTime2);
    double w3 = getSpeed(counter3, lastCounter3, lastTime3);
    double w4 = getSpeed(counter4, lastCounter4, lastTime4);

    lastTime = currentTime;
    Serial.print(w1);
    Serial.print(" ");
    Serial.print(w2);
    Serial.print(" ");
    Serial.print(w3);
    Serial.print(" ");    
    Serial.println(w4);
  }

}

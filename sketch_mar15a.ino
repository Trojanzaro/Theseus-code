#include <math.h>
#include "Rotary.h"

//PWM Variables
byte pwm = 64;
byte pwmCounter = 0; // Naturally limited to 0-255

byte pwms[] = { 0, pwm, 0, pwm, 0, pwm, 0, pwm}; 
byte pwms_0[] = { 0, 0, 0, 0, 0, 0, 0, 0}; 

double t = 0;

Rotary enc1 = Rotary(11, 12);

void setup() {
  DDRD |= 0xfc; // Set D2-D7 as outputs
  DDRB |= 0x03; // Set D8-D9 as outputs
  
  DDRB &= ~(1 << PB3);  // Clear bit 3 of DDRB (D11 as input)
  DDRB &= ~(1 << PB4);  // Clear bit 4 of DDRB (D12 as input)

  // Optionally enable pull-up resistors for D11 and D12
  PORTB |= (1 << PB3);  // Enable pull-up resistor on D11 (PORTB3)
  PORTB |= (1 << PB4);  // Enable pull-up resistor on D12 (PORTB4)
  Serial.begin(230400);
}

void loop() {
  t += 0.0005;
  float x = (cos(t)/1)*255;

  // PWM writing logic
  byte selected_pwms[] = {0,x,0,x,0,x,0,x};
  if(x < 0){
    selected_pwms[0] = abs(x);
    selected_pwms[1]= 0;
    selected_pwms[2] = abs(x);
    selected_pwms[3] = 0;
    selected_pwms[4]= abs(x);
    selected_pwms[5]= 0;
    selected_pwms[6]= abs(x);
    selected_pwms[7]= 0;
  }

  PORTD ^= (-(pwmCounter >= selected_pwms[0]) ^ PORTD) & (1 << 2); // D2
  PORTD ^= (-(pwmCounter >= selected_pwms[1]) ^ PORTD) & (1 << 3); // D3
  PORTD ^= (-(pwmCounter >= selected_pwms[2]) ^ PORTD) & (1 << 4); // D4
  PORTD ^= (-(pwmCounter >= selected_pwms[3]) ^ PORTD) & (1 << 5); // D5
  PORTD ^= (-(pwmCounter >= selected_pwms[4]) ^ PORTD) & (1 << 6); // D6
  PORTD ^= (-(pwmCounter >= selected_pwms[5]) ^ PORTD) & (1 << 7); // D7

  PORTB ^= (-(pwmCounter >= selected_pwms[6]) ^ PORTB) & (1 << 0); // D8
  PORTB ^= (-(pwmCounter >= selected_pwms[7]) ^ PORTB) & (1 << 1); // D9

  pwmCounter += 1;
  
  // Encoder reading logic
  unsigned char result = enc1.process_n();
  if (result == DIR_CW) {
    counter++;
  } else if (result == DIR_CCW) {
    counter--;
  }
  Serial.println(counter);

}

//Encoder Variables
int counter = 0;
byte last_enca = 0;  // Last state of D11
byte last_encb = 0;  // Last state of D12


//PWM Variables
byte pwm = 64;
byte pwmCounter = 0; // Naturally limited to 0-255
byte pwms[] = { 0, pwm, 0, pwm, 0, pwm, 0, pwm};


void setup() {
  DDRD |= 0xfc; // Set D2-D7 as outputs
  DDRB |= 0x03; // Set D8-D9 as outputs
  
  DDRB &= ~(1 << PB3);  // Clear bit 3 of DDRB (D11 as input)
  DDRB &= ~(1 << PB4);  // Clear bit 4 of DDRB (D12 as input)

  // Optionally enable pull-up resistors for D11 and D12
  PORTB |= (1 << PB3);  // Enable pull-up resistor on D11 (PORTB3)
  PORTB |= (1 << PB4);  // Enable pull-up resistor on D12 (PORTB4)
  //Serial.begin(115200);
}

void loop() {
  // PWM writing logic
  PORTD ^= (-(pwmCounter >= pwms[0]) ^ PORTD) & (1 << 2); // D2
  PORTD ^= (-(pwmCounter >= pwms[1]) ^ PORTD) & (1 << 3); // D3
  PORTD ^= (-(pwmCounter >= pwms[2]) ^ PORTD) & (1 << 4); // D4
  PORTD ^= (-(pwmCounter >= pwms[3]) ^ PORTD) & (1 << 5); // D5
  PORTD ^= (-(pwmCounter >= pwms[4]) ^ PORTD) & (1 << 6); // D6
  PORTD ^= (-(pwmCounter >= pwms[5]) ^ PORTD) & (1 << 7); // D7

  PORTB ^= (-(pwmCounter >= pwms[6]) ^ PORTB) & (1 << 0); // D8
  PORTB ^= (-(pwmCounter >= pwms[7]) ^ PORTB) & (1 << 1); // D9

  pwmCounter += 1;
  
  // Encoder reading logic
  last_enca = (PINB & (1 << PB3)) != 0;
  last_encb = (PINB & (1 << PB4)) != 0; 
  
  byte enca = (PINB & (1 << PB3)) != 0;
  byte encb = (PINB & (1 << PB4)) != 0;

  // Compare the current and last states to determine direction
  if(enca) {
    if(!last_encb) {
      counter++; // if encoder A is on and B last state is 0 CLOCKWISE
    } else {
      return;
    }
  } else if(encb){
    if(!last_enca) {
      counter--; // if encoder B is on and A last state is 0 COUNTER-CLOCKWISE
    } else{
      return;
    }
  }
  
  //Serial.println(counter);
}

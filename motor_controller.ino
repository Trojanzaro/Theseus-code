void setup() {
  DDRD |= 0xfc; // Set D2-D7 as outputs
  DDRB |= 0x03; // Set D8-D9 as outputs
  Serial.begin(9600);
}

byte pwm = 64;
byte pwmCounter = 0; // Naturally limited to 0-255
byte pwms[] = { 0, 0, 0, 0, 0, 0, 0, 0 };

int steps = 0;

void loop() {
  PORTD ^= (-(pwmCounter >= pwms[0]) ^ PORTD) & (1 << 2); // D2
  PORTD ^= (-(pwmCounter >= pwms[1]) ^ PORTD) & (1 << 3); // D3
  PORTD ^= (-(pwmCounter >= pwms[2]) ^ PORTD) & (1 << 4); // D4
  PORTD ^= (-(pwmCounter >= pwms[3]) ^ PORTD) & (1 << 5); // D5
  PORTD ^= (-(pwmCounter >= pwms[4]) ^ PORTD) & (1 << 6); // D6
  PORTD ^= (-(pwmCounter >= pwms[5]) ^ PORTD) & (1 << 7); // D7

  PORTB ^= (-(pwmCounter >= pwms[6]) ^ PORTB) & (1 << 0); // D8
  PORTB ^= (-(pwmCounter >= pwms[7]) ^ PORTB) & (1 << 1); // D9

  pwmCounter += 1;

  if((PINB & 0b00000100) >> 2) {
    if((PINB & 0b00001000) >> 3) {
      steps++;
    }
  }
  if((PINB & 0b00001000) >> 3) {
    if(!(PINB & 0b00000100) >> 2) {
      steps--;
    }
  }
  Serial.print((PINB & 0b00000100) >> 2);
  Serial.print(" ");
  Serial.print((PINB & 0b00001000) >> 3);
  Serial.print(" ");
  Serial.println(steps);
}

//TODO
void enc_print() {
  if ((PORTD & 0b00000100) == 0){ 
    if  ((PORTD & 0b00001000) == 0){
      // steps++
    }
  }
  else {
  // stepps--
  }
}

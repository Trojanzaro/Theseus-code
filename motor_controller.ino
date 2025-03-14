void setup() {
  DDRD |= 0xfc; // Set D2-D7 as outputs
  DDRB |= 0x03; // Set D8-D9 as outputs
}

byte pwmCounter = 0; // Naturally limited to 0-255
byte pwms[] = { 10, 20, 40, 80, 10, 20, 40, 80 };

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

  //TODO:
}
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

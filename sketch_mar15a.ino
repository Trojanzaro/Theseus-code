int counter = 0;
byte last_enca = 0;  // Last state of D11
byte last_encb = 0;  // Last state of D12

void setup() {
  DDRB &= ~(1 << PB3);  // Clear bit 3 of DDRB (D11 as input)
  DDRB &= ~(1 << PB4);  // Clear bit 4 of DDRB (D12 as input)

  // Optionally enable pull-up resistors for D11 and D12
  PORTB |= (1 << PB3);  // Enable pull-up resistor on D11 (PORTB3)
  PORTB |= (1 << PB4);  // Enable pull-up resistor on D12 (PORTB4)
  Serial.begin(9600);
}

void loop() {
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
  
  Serial.println(counter);
}

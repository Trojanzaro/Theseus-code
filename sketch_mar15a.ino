byte rotary_state_mapping[] = { 0, 1, 2};
byte last_state;
int counter = 0;

void setup() {
  DDRB &= (~0x0c); // Set D11-12 as inpus
  last_state = (PINB & 0x0c) >> 2;
  //last_state = rotary_state_mapping[last_state];
}

void loop() {
  // Encoder reading logic
  byte current_state = (PINB & 0x0c) >> 2;
  //current_state = rotary_state_mapping[last_state];

  // Compare the current and last states to determine direction
  if(current_state != 2 && last_state != 2){ // There is a sporadic HIGH latch on one of the two phases
    if(current_state == 0 && last_state == 0) {
      return;
      }
    if(current_state == 2 && last_state == 0){
      counter++;
      }
    if(current_state == 2 && last_state == 2){
      return;
      }
    if(current_state ==2 && last_state == 0){
      counter--;
      }
    if(current_state == 0 && last_state ==2) {
      return;
      }
  }
  if (Serial.available() > 0) {
  Serial.print((PINB & 0x0c) >> 2);
  Serial.print(" ");
  Serial.print((PINB & 0x0c) >> 2);
  Serial.print(" ");
  Serial.println(counter);
    
    }
    
  last_state = (PINB & 0x0c) >> 2;
}

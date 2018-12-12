unsigned long timer[4];
byte last_channel[4];
int input[4];

void setup() {

PCICR |= (1 << PCIE0);
PCMSK0 |= (1 << PCINT0);  // UNO board pin8
PCMSK0 |= (1 << PCINT1);  // UNO board pin9

Serial.begin(115200); // Pour a bowl of Serial

}

void loop() {

  print();
  //delay(10);
  
}

ISR(PCINT0_vect){
  timer[0] = micros(); // make a time stamp when interrupt occur
  
  // Channel 1
  
  // This condition check last channel was LOW and new channel value was HIGH 
  // PINB is similar to digitalRead but a lot quicker
  if(last_channel[0] == 0 && PINB & B00000001){
    last_channel[0] = 1;
    timer[1] = timer[0];
  }
  else if(last_channel[0] == 1 && !(PINB & B00000001)){
    last_channel[0] = 0;
    // This line below will calculate the time between rising edge and falling edge of pulse
    input[0] = timer[0] - timer[1];
  }

  // Channel 2
    if(last_channel[1] == 0 && PINB & B00000010){
    last_channel[1] = 1;
    timer[2] = timer[0];
  }
  else if(last_channel[1] == 1 && !(PINB & B00000010)){
    last_channel[1] = 0;
    input[1] = timer[0] - timer[2];
  }

  
}
void print(){
  Serial.print(input[0]);
  Serial.print(",");
  Serial.println(input[1]);
}


#include "can.h";
#include "lights.h";
#include "helpers.h";

bool debug = false;

#define PIN_HORN       29
#define PIN_BLINKER_L  30
#define PIN_BRAKE      31
#define PIN_BLINKER_R  33
#define PIN_LAP_COUNT  34
#define PIN_THROTTLE   35

bool brakeOrAccPressed = false;


static volatile uint8_t buttons,buttons2;

static CAN_message_t txMsg,rxMsg;


void initPins(){
  pinMode(PIN_THROTTLE,INPUT);
  
  pinMode(PIN_BREAK,INPUT);
  
  pinMode(PIN_BLINKER_L,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BLINKER_L),leftBlinkerChanged_ISR,CHANGE);

  pinMode(PIN_BLINKER_R, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BLINKER_R),rightBlinkerChanged_ISR,CHANGE);

  pinMode(PIN_LAP_COUNT,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LAP_COUNT),resetLapTimeAndIncrementLapCount_ISR,CHANGE);

  pinMode(PIN_HORN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_HORN),hornChanged_ISR,CHANGE);

}





void setup() {
  
  initCAN();

  initValuesSWheel(txMsg);
  clockSpeed120Mhz(debug);
  delay(500);
  
  initPins();
  clockSpeed2Mhz(debug);
  

}

void loop() {
  txMsg.buf[0] = buttons2;
  txMsg.buff[1] = buttons;
  txMsg.buf[2] = readBreak(PIN_BRAKE,&buttons);
  txMsg.buf[3] = readThrottle(PIN_THROTTLE,&buttons);
  readCan(rxMsg);
  writeCan(txMsg);
  printCanToSerial(txMsg,debug);
  delay(50);
  dashCAN(txMsg);

}


//-------ISR------//

void resetLapTimeAndIncrementLapCount_ISR(){
  
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
  if(interrupt_time - last_interrupt_time > 10){
    int state = digitalRead(PIN_LAP_COUNT);

    if(state == HIGH){
      buttons &= ~lap;
      Serial.println(" LAP UNPRESSED ");

    } 
    if(state == LOW && interrupt_time - last_interrupt_time > 200){
      buttons |= lap;
      Serial.println(" LAP PRESSED ");
    }
  }
  last_interrupt_time = interrupt_time;
}


void leftBlinkerChanged_ISR(){
  int state = digitalRead(PIN_BLINKER_L);
  int leftBlinkerPressed;

  if(state == HIGH && leftBlinkerPressed == LOW){
    leftBlinkerPressed = false;
    buttons &= ~leftBlink;
    Serial.println(" BLINK L UNPRESSED ");
  }
  else {
    leftBlinkerPressed = true;
    buttons |= leftBlink;
    Serial.println(" BLINK L PRESSED ");
    
  }
}

void rightBlinkerChanged_ISR(){
  int state = digitalRead(PIN_BLINKER_R);
  int rightBlinkerPressed;

  if(state == HIGH && rightBlinkerPressed == LOW){
    rightBlinkerPressed = false;
    buttons &= ~rightBlink;
    Serial.println(" BLINK R UNPRESSED ");
  } 
  else {
    rightBlinkerPressed = true;
    buttons |= rightBlink;
    Serial.println(" BLINK R PRESSED ");
  }
}

void hornChanged_ISR(){
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();

  if(interrupt_time - last_interrupt_time > 10){
    int state = digitalRead(PIN_HORN);

    if(state == HIGH){
      buttons &= ~horn;
      Serial.println(" HORN UNPRESSED ");
    
    } else {
      buttons |= horn;
      Serial.println(" HORN PRESSED ");
    
    }
  }
  last_interrupt_time = interrupt_time;
}

void dashCAN(CAN_message_t& rxMsg){
  if(rxMsg.id == dashID){
    for(int i = 0; i < rxMsg.len(); i++){
      dashValues[i] = rxMsg.buf[i];
    }
  }
}

#include "lights.h"

static bool hazardLightRunning = false;
static bool blinkMode;


void turnOffStrip(Adafruit_NeoPixel& strip, const uint16_t& start, uint16_t& end) {
  if (end == 0){
    end = strip.numPixels();
  }

  for (uint16_t i = start; i < end; ++i){
    strip.setPixelColor(i,strip.Color(0,0,0,0));
  }

  strip.show();
  
}

void initLights(Adafruit_NeoPixel& frontLights, Adafruit_NeoPixel& backlights){
  frontLights.begin();
  backLights.begin();
  frontLights.setBrightness(BRIGHTNESS_FRONT);
  backLights.setBrightness(BRIGHTNESS_BACK);
  
}


void showLights(Adafruit_NeoPixel& frontLights, Adafruit_NeoPixel& backLights){
  turnOffStrip(frontLights, 0, NUM_FRONTLIGHTS);
  turnOffStrip(backLights, 0, NUM_BACKLIGHTS);

  //BACKLIGHTS
  backLights.setBrightness(BRIGHTNESS_BACK);
  frontLights.setBrightness(BRIGHTNESS_FRONT);
  for (int i = 288; i > 255; i--){
      backLights.setPixelColor(i, COLOR_BACKLIGHTS);
      backLights.show();
      delay(30);
    }
    int i = 0;
    int n = 0;
    int j = 0;
    while(i <= 128){
      backLights.setPixelColor(128 + i, COLOR_BACKLIGHTS);
      backLights.setPixelColor(128 - i, COLOR_BACKLIGHTS);
      i++;
      n++;
      if( 128 + i >= 177){
       backLights.setPixelColor(255 - j, COLOR_BACKLIGHTS);
       backLights.setPixelColor(0 + j, COLOR_BACKLIGHTS);
       j++; 
      }
      if( n >= 5){
        backLights.show();
        n = 0;
      }
    }

    // FRONTLIGHTS

  i = 0;
  n = 0;
  j = 0;
  int k = 0;

    while( i < 150){
      frontLights.setPixelColor(93 - i, COLOR_FRONTLIGHTS);
      frontLights.setPixelColor(94 + i, COLOR_FRONTLIGHTS);
      frontLights.setPixelColor(116 - i, COLOR_FRONTLIGHTS);
      frontLights.setPixelColor(117 + i, COLOR_FRONTLIGHTS);
      if(i >= 71){
        frontLights.setPixelColor(0 + j, COLOR_FRONTLIGHTS);
        j++;
      }
      if(i + 116 > 140){
        frontLights.setPixelColor(210 - k, COLOR_FRONTLIGHTS);
        frontLights.setPixelColor(0 + k, COLOR_FRONTLIGHTS);
        k++;
      }
      i++;
      n++;
      frontLights.show();
      
    }
}

void brakeLights(Adafruit_NeoPixel& backLights, bool brakeON){
  if(brakeON){
    backLights.setBrightness(BRIGHTNESS_BACK_BRAKE);
  }else{
    backLights.setBrightness(BRIGHTNESS_BACK);
  }
  backLights.show();
}



void blinkLights(Adafruit_NeoPixel& frontLights, Adafruit_NeoPixel& backLights, bool left, bool raceModeON){
  
  if(left){                                                                                                             // LEFT SIDE OF CAR
    for (int i = 1; i < NUM_BLINK; i++) {
      if(raceModeON){                                                                                                   // IF RACEMODE OR SHOWMODE
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_OFF);                                         // TURN OFF FRONTLIGHTS BEHIND BLINK
      }else{
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_FRONTLIGHTS);                                 // TURN ON FRONTLIGHTS BEHIND BLINK
      }
      frontLights.show();
    }
    turnOffStrip(backLights, BLINK_LEFT_START_BACKLIGHTS, BLINK_LEFT_END_BACKLIGHTS - 2);                               // TURN OFF BACKLIGHTS BEHIND BLINK
    for (int i = 1; i < NUM_BLINK; i++) {
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_BLINKLIGHTS);                                 // FILL BLINKLIGHTS GRADUALY
        backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_BLINKLIGHTS);
        frontLights.show();
        backLights.show();
    }
    delay(200);
  } 
    for (int i = 1; i < NUM_BLINK; i++) {
      if(raceModeON){
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_OFF);                                         // SET COLOR BACK TO ORIGINAL COLOR
        backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_OFF);
      }else{
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_FRONTLIGHTS);                                 
        backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_BACKLIGHTS);
      }   
  }
  if(!left){                                                                                                            // RIGHT SIDE OF CAR
    for (int i = 1; i < NUM_BLINK; i++) {
      if(raceModeON){                                                                                                   // IF RACEMODE OR SHOWMODE
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_OFF);                                        // TURN OFF FRONTLIGHTS BEHIND BLINK
      }else{
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_FRONTLIGHTS);                                // TURN ON FRONTLIGHTS BEHIND BLINK
      }
      frontLights.show();
    }
    turnOffStrip(backLights, BLINK_RIGHT_END_BACKLIGHTS , BLINK_RIGHT_START_BACKLIGHTS-2);                              // TURN OFF BACKLIGHTS BEHIND BLINK
    for (int i = 1; i < NUM_BLINK; i++) {
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_BLINKLIGHTS);                                // FILL BLINKLIGHTS GRADUALY
        backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_BLINKLIGHTS);
        frontLights.show();
        backLights.show();
    }
    delay(200);
  } 
  for (int i = 1; i < NUM_BLINK; i++) {
      if(raceModeON){
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_OFF);
        backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_OFF);                                          // SET COLOR BACK TO ORIGINAL COLOR
      } else {
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_FRONTLIGHTS);
        backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_BACKLIGHTS);
      
    }
  }
  
  frontLights.show();
  backLights.show();
}

bool hazardLights(Adafruit_NeoPixel& frontLights, Adafruit_NeoPixel& backLights, bool hazardLightsON, bool raceModeON){
  if(hazardLightsON == true && hazardLightRunning == false ){
    turnOffStrip(backLights, BLINK_LEFT_START_BACKLIGHTS , BLINK_LEFT_END_BACKLIGHTS);                                 // TURN OFF LIGHTS BEHIND BLINK
    turnOffStrip(backLights, BLINK_RIGHT_END_BACKLIGHTS, BLINK_RIGHT_START_BACKLIGHTS);   
    for (int i = 1; i < NUM_BLINK; i++) {
          frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_BLINKLIGHTS);                              // FILL BLINKLIGHTS GRADUALY
          backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_BLINKLIGHTS);
          frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_BLINKLIGHTS);
          backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_BLINKLIGHTS);
          frontLights.show();
          backLights.show();
    }
    delay(200);
    hazardLightRunning = true;
  }else if(hazardLightsON == false || hazardLightRunning == true){
  if(raceModeON){
      for (int i = 1; i < NUM_BLINK; i++) {
        frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_OFF);                                        // TURN OFF FRONTLIGHTS BEHIND BLINK
        frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_OFF);
        backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_OFF);
        backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_OFF);                                          // TURN OFF BACKLIGHTS BEHIND BLINK
      }
    }else{
      for (int i = 1; i < NUM_BLINK; i++) {
              frontLights.setPixelColor(BLINK_LEFT_START_FRONTLIGHTS - i + 3, COLOR_FRONTLIGHTS);                          // SET COLOR BACK TO ORIGINAL COLOR
              backLights.setPixelColor(BLINK_LEFT_START_BACKLIGHTS + i, COLOR_BACKLIGHTS);
              frontLights.setPixelColor(BLINK_RIGHT_START_FRONTLIGHTS + i - 3, COLOR_FRONTLIGHTS);
              backLights.setPixelColor(BLINK_RIGHT_START_BACKLIGHTS - i, COLOR_BACKLIGHTS);
              
      }
    }
        frontLights.show();
        backLights.show();
        hazardLightRunning = false;
    }
  return hazardLightRunning;
}


void raceLights(Adafruit_NeoPixel& frontLights, Adafruit_NeoPixel& backLights) {
  turnOffStrip(frontLights, 0, NUM_FRONTLIGHTS);
  turnOffStrip(backLights, 0,NUM_BACKLIGHTS);   
  frontLights.show();
  backLights.show();
}

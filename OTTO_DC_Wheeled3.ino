
//----------------------------------------------------------------
//-- Zowi basic firmware v2 adapted to Otto
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Otto has thanks to Zowi!
//-----------------------------------------------------------------
//-- OCTOBERARY 2017: modified for OTTO with MAX7219 MATRIX LED module
//-- allows OTTO to work with ZOWI APP
//-- written for two x yellow 48:1 DC motors , new Ultrasonic library has been used
//-- Jason Snow

#include <BatReader.h>
#include "MaxMatrix.h"
MaxMatrix ledmatrix=MaxMatrix(12,10,11, 1); //PIN  12=DIN, PIN 10=CS, PIN 11=CLK

//-- Library to manage external interruptions
#include <EnableInterrupt.h> 
//-- Otto Library

#include "OttoDC_sounds.h"
#include "OttoDC_mouths.h"
#include "OttoDC_gestures.h"
#include "BatReader.h"
#include <Ultrasonic.h>

Ultrasonic ultrasonic(8,9); // (Trig PIN,Echo PIN)
//#include <ST_HW_HC_SR04.h>

//ST_HW_HC_SR04 ultrasonicSensor(8, 7); // ST_HW_HC_SR04(TRIG, ECHO)
BatReader battery;
//---------------------------------------------------------
//-- First step: Configure the pins where the servos are attached
/*
          --------------- 
         |     O   O     |
         |---------------|
PWM 3==>||               || <== PWM 5
DIR 2==>| ------  ------  | <== DIR 4
        |                 |
*/

// buzzer is now connected to pin 13

///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////

//---------------------------------------------------------
//-- Otto has 5 modes:
//--    * MODE = 0: Otto is awaiting  
//--    * MODE = 1: Dancing mode!  
//--    * MODE = 2: Obstacle detector mode  
//--    * MODE = 3: Noise detector mode   
//--    * MODE = 4: OttoPAD or any Teleoperation mode (listening SerialPort). 
//---------------------------------------------------------
volatile int MODE=2; //State of Otto in the principal state machine. 



unsigned long previousMillis=0;
unsigned long int getMouthShape(int number);
unsigned long int getAnimShape(int anim, int index);

int randomDance=0;
int randomSteps=0;
int gesture = 0;
int gesture1 = 0;
bool obstacleDetected = false;
int newmouth = 0;
bool goingforward = false; // motor direction logic
bool goingreverse = false;
bool goingleft = false;
bool goingright = false;
int mDelay = 10; // motor delay before changing movement
int lSpeed = 150; // motor speed (higher is faster)
int rSpeed = 150; // motor speed (higher is faster)
#define mE1 6 // motor 1 speed pin
#define mE2 5 // motor 2 speed pin
#define mI1 2 // motor 1 direction pin
#define mI2 4 // motor 2 direction pin
int batteryLevel = 0;
int calibration = 0; // used if one motor is faster than the other due to mechanical differences
int melody[] = {
  note_C4, note_G3, note_G3, note_A3, note_G3, 0, note_B3,note_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};
///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup(){
  rSpeed = (rSpeed - calibration); // make calibration adjustment
  pinMode(mE1, OUTPUT);
  pinMode(mE2, OUTPUT);
  pinMode(mI1, OUTPUT);
  pinMode(mI2, OUTPUT);
  Serial.begin(19200);  
  
  newmouth = 0;
// set up Matrix display
  ledmatrix.init();
  ledmatrix.setIntensity(1);
  //Set a random seed
  randomSeed(analogRead(A6));

  //Otto wake up!
  delay(500);
  
  //Checking battery requestBattery();
  // BAT_MAX  4.2
  // BAT_MIN  3.25

  OttoLowBatteryAlarm(); // check battery level


 // Animation Uuuuuh - A little moment of initial surprise
 //-----
  for(int i=0; i<2; i++){
      for (int i=0;i<8;i++){
         
        putAnimationMouth(littleUuh,i);
        delay(150);
      }
  }
 //-----
   
    putMouth(smile);
    delay(200);
 

  previousMillis = millis();

// notes in the melody:

  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(13, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(13);
  }
  //OTTO animations, gestures, sounds, mouths
  //putAnimationMouth(littleUuh); //8 parts
  //putAnimationMouth(dreamMouth); //4 parts
  //putAnimationMouth(adivinawi); //6 parts
  //putAnimationMouth(wave); //10 parts
  //playGesture(OttoHappy);
  //playGesture(OttoSuperHappy);
  //playGesture(OttoSad);
  //playGesture(OttoSleeping);
  //playGesture(OttoFart);
  //playGesture(OttoConfused);
  //playGesture(OttoLove);
  //playGesture(OttoAngry);
  //playGesture(OttoFretful);
  //playGesture(OttoMagic);
  //playGesture(OttoWave);
  //playGesture(OttoVictory);
  //playGesture(OttoFail);
  //soundcmd(S_connection);
  //soundcmd(S_disconnection2);
  //soundcmd(S_buttonPushed);
  //soundcmd(S_mode1);
  //soundcmd(S_mode2);
  //soundcmd(S_mode3);
  //soundcmd(S_surprise);
  //soundcmd(S_OhOoh);
  //soundcmd(S_OhOoh2);  
  //soundcmd(S_cuddly);
  //soundcmd(S_sleeping);
  //soundcmd(S_happy);
  //soundcmd(S_superHappy);
  //soundcmd(S_happy_short);
  //soundcmd(S_sad);
  //soundcmd(S_fart1);
  //soundcmd(S_fart2);
  //soundcmd(S_fart3);
  //soundcmd(S_track_1);
  //putMouth(zero);
  //putMouth(one);
  //putMouth(two);
  //putMouth(three);
  //putMouth(four);
  //putMouth(five);
  //putMouth(six);
  //putMouth(seven);
  //putMouth(eight);
  //putMouth(nine);
  //putMouth(smile);
  //putMouth(happyOpen);
  //putMouth(happyClosed);
  //putMouth(heart);
  //putMouth(bigSurprise);
  //putMouth(smallSurprise);
  //putMouth(tongueOut);
  //putMouth(vamp1);
  //putMouth(vamp2);
  //putMouth(lineMouth);
  //putMouth(confused);
  //putMouth(diagonal);
  //putMouth(sad);
  //putMouth(sadOpen);
  //putMouth(sadClosed);
  //putMouth(okMouth);
  //putMouth(xMouth);
  //putMouth(interrogation);
  //putMouth(thunder);
  //putMouth(culito);
  //putMouth(angry);
  delay(1000);
}
 

///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {
 
    switch (MODE) {

      //-- MODE 2 - Obstacle detector mode
      //---------------------------------------------------------
      case 2:
        if(obstacleDetected){
          motorstop(); // stop motors
            soundcmd(S_OhOoh);
            delay(100);
              putMouth(bigSurprise);
              delay(500);
              putMouth(confused);
              delay(500);
              soundcmd(S_OhOoh2);
            //Otto drives back
            putMouth(xMouth);
             //delay(1000);
             motormoveBackward(); // moveBackwards
              delay(1500); 
              motorstop();
              delay(500);           
              putMouth(smile);
              delay(1500);
                motorturnLeft(); // turn small amount left
                delay(300);
                motorstop();
                obstacleDetector();
                delay(250);
                
           
            //If there are no obstacles, Otto is happy
            if(obstacleDetected==true){break;}           
            
                obstacleDetected=false;
                putMouth(happyOpen);
                soundcmd(S_happy_short);
                delay(500);
                
        }else
        {

 //select a random number between 1 and 5
      if (newmouth > 20)
      {
      gesture = random(1,5);
      // display the relevant gesture for the random number selected
      switch (gesture) {
      case 1: //H 1 
        putMouth(happyOpen);
        soundcmd(S_happy);
        putMouth(happyClosed);
        delay(500);
        putMouth(happyOpen);
        soundcmd(S_superHappy);
        putMouth(happyClosed);
        break;
      case 2: //H 2 
        putMouth(smile);
        soundcmd(S_cuddly);
        break;
      case 3: //H 3 
        putMouth(happyClosed);
        break;
      case 4: //H 4 
         putMouth(tongueOut);  
         soundcmd(13);  
        break;
      case 5: //H 5  
        putMouth(lineMouth);
        break;
        
      }
      newmouth = 0;
    }

            //Otto Drive straight
            motormoveForward(); // moveForward
            obstacleDetector(); // check for obstacles
            delay(100);
            obstacleDetector(); // check for obstacles
            delay(100);
            newmouth = (newmouth + 1);
            //OttoLowBatteryAlarm();

        }   

        break;


      default:
          MODE=2;
          break;
      

}  

}


///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////


//-- Function to read distance sensor & to actualize obstacleDetected variable
void obstacleDetector(){
  int distance = (ultrasonic.Ranging(CM));  
 Serial.println(distance);
        if(distance<29){ // if object detected closer than 29 then take action
          obstacleDetected = true;
        }else{
          obstacleDetected = false;
        }
}

//-- Functions with animatics
//--------------------------------------------------------

void OttoLowBatteryAlarm(){

     getBatteryLevel();
    //A7 = 880 = 4.2v = 100%
    //A7 = 758 = 3.7v = 50%
    Serial.println(batteryLevel);
    if(batteryLevel<45){
        
      {

          putMouth(thunder);
          bendTones (880, 2000, 1.04, 8, 3);  
          
          delay(30);

          bendTones (2000, 880, 1.02, 8, 3);  
          clearMouth();
          delay(500);
      } 
    }
}


void motormoveForward() {
// STOP motors before direction change to help protect motor drivers
if (goingforward == false) motorstop();
    // DIR motor
    digitalWrite(mI1, LOW);
    digitalWrite(mI2, HIGH);
    // PWM motor
    analogWrite(mE1, lSpeed);
    analogWrite(mE2, rSpeed);
    goingreverse = false;
    goingforward = true;
    goingleft = false;
    goingright = false; 
  }


/**
 * Move backward
 */
void motormoveBackward() {
// STOP motors before direction change to help protect motor drivers
if (goingreverse == false) motorstop();  
// PWM motor
    analogWrite(mE1, lSpeed);
    analogWrite(mE2, rSpeed);
// DIR motor
    digitalWrite(mI1, HIGH);
    digitalWrite(mI2, LOW);
    goingreverse = true;
    goingforward = false;
    goingleft = false;
    goingright = false; 
}

/**
 * Turn Left
 */
void motorturnLeft() {
// STOP motors before direction change to help protect motor drivers
if (goingleft == false) motorstop();   
    analogWrite(mE1, lSpeed);
    analogWrite(mE2, rSpeed);
    digitalWrite(mI1, LOW);
    digitalWrite(mI2, LOW);
    goingreverse = false;
    goingforward = false;
    goingleft = true;
    goingright = false; 

}

/**
 * Turn Right
 */
void motorturnRight() {
 // STOP motors before direction change to help protect motor drivers
if (goingright == false) motorstop();   
    analogWrite(mE1, lSpeed);
    analogWrite(mE2, rSpeed);

    digitalWrite(mI1, HIGH);
    digitalWrite(mI2, HIGH);
    goingreverse = false;
    goingforward = false;
    goingleft = false;
    goingright = true;  
}

/**
 * Stop Bot
 */
void motorstop() {
    digitalWrite(mE1, LOW);
    digitalWrite(mE2, LOW);
    digitalWrite(mI1, LOW);
    digitalWrite(mI2, LOW);
}
// OTTO SOUNDS
void tonenew (float noteFrequency, long noteDuration, int silentDuration) { 

// tone (10,261,500); 
// delay (500); 

if (silentDuration == 0) {silentDuration = 1;} 
tone (13, noteFrequency, noteDuration); 

delay (noteDuration); // milliseconds to microseconds 
delay (silentDuration); 
} 

void bendTones (float initFrequency, float finalFrequency, float prop, long noteDuration, int silentDuration) { 

if (silentDuration == 0) {silentDuration = 1;} 

if (initFrequency <finalFrequency) 
{ 
for (int i = initFrequency; i <finalFrequency; i = i * prop) { 
tonenew (i, noteDuration, silentDuration); 
} 

} 
else { 

for (int i = initFrequency; i> finalFrequency; i = i / prop) { 
tonenew (i, noteDuration, silentDuration); 
} 
} 
} 
void soundcmd (int cmd) 
{ 
switch (cmd) 
{ 
case 1: //connection
tonenew (note_E5, 50, 30); 
tonenew (note_E6, 55, 25); 
tonenew (note_A6, 60, 10); 
break; 
case 2: //disconnection
tonenew (note_E5, 50, 30); 
tonenew (note_A6, 55, 25); 
tonenew (note_E6, 50, 10); 
break; 
case 3: //buttonPushed
bendTones (note_E6, note_G6, 1.03, 20, 2); 
delay (30); 
bendTones (note_E6, note_D7, 1.04, 10, 2); 
break; 
case 4: //mode1
bendTones (note_E6, note_A6, 1.02, 30, 10); //1318.51 to 1760 
break; 
case 5: //mode2
bendTones (note_G6, note_D7, 1.03, 30, 10); //1567.98 to 2349.32 
break; 
case 6: //mode3
tonenew (note_E6, 50, 100); // D6 
tonenew (note_G6, 50, 80); // E6 
tonenew (note_D7, 300, 0); // G6 
break; 
case 7: //surprise
bendTones (800, 2150, 1.02, 10, 1); 
bendTones (2149, 800, 1.03, 7, 1); 
break; 
case 8: //OhOoh
bendTones (880, 2000, 1.04, 8, 3); // A5 = 880 
delay (200); 

for (int i = 880; i <2000; i = i * 1.04) { 
tonenew (note_B5, 5, 10); 
} 
break;
case 9: //OhOoh2
bendTones (1880, 3000, 1.03, 8, 3); 
delay (200); 

for (int i = 1880; i <3000; i = i * 1.03) { 
tonenew (note_C6, 10, 10); 
} 
break; 
case 10: //cuddly
bendTones (700, 900, 1.03, 16, 4); 
bendTones (899, 650, 1.01, 18, 7); 
break; 
case 11: //sleeping
bendTones (100, 500, 1.04, 10, 10); 
delay (500); 
bendTones (400, 100, 1.04, 10, 1); 
break; 
case 12: //happy
bendTones (1500, 2500, 1.05, 20, 8); 
bendTones (2499, 1500, 1.05, 25, 8); 
break; 
case 13: //superHappy
bendTones (2000, 6000, 1.05, 8, 3); 
delay (50); 
bendTones (5999, 2000, 1.05, 13, 2); 
break; 
case 14: //happy_short
bendTones (1500, 2000, 1.05, 15, 8); 
delay (100); 
bendTones (1900, 2500, 1.05, 10, 8); 
break; 
case 15: //sad
bendTones (880, 669, 1.02, 20, 200); 
break; 
case 16: //confused
bendTones (1000, 1700, 1.03, 8, 2); 
bendTones (1699, 500, 1.04, 8, 3); 
bendTones (1000, 1700, 1.05, 9, 10); 
break; 
case 17: //fart1
bendTones (1600, 3000, 1.02, 2, 15); 
break; 
case 18: //fart2
bendTones (2000, 6000, 1.02, 2, 20); 
break; 
case 19: //fart3
bendTones (1600, 4000, 1.02, 2, 20); 
bendTones (4000, 3000, 1.02, 2, 20); 
break; 
case 20: 
tonenew (note_A6,30,5); 
break; 
default: 

break; 
} 
} 
// OTTO MOUTHS
unsigned long int getMouthShape(int number){
unsigned long int types []={zero_code,one_code,two_code,three_code,four_code,five_code,six_code,seven_code,eight_code,
  nine_code,smile_code,happyOpen_code,happyClosed_code,heart_code,bigSurprise_code,smallSurprise_code,tongueOut_code,
  vamp1_code,vamp2_code,lineMouth_code,confused_code,diagonal_code,sad_code,sadOpen_code,sadClosed_code,
  okMouth_code, xMouth_code,interrogation_code,thunder_code,culito_code,angry_code};

  return types[number];
}

void putMouth(unsigned long int mouth){

 
    ledmatrix.writeFull(getMouthShape(mouth));
  
}


void clearMouth(){

  ledmatrix.clearMatrix();
}
//  OTTO ANIMATIONS
unsigned long int getAnimShape(int anim, int index){

  unsigned long int littleUuh_code[]={
     0b00000000000000001100001100000000,
     0b00000000000000000110000110000000,
     0b00000000000000000011000011000000,
     0b00000000000000000110000110000000,
     0b00000000000000001100001100000000,
     0b00000000000000011000011000000000,
     0b00000000000000110000110000000000,
     0b00000000000000011000011000000000  
  };

  unsigned long int dreamMouth_code[]={
     0b00000000000000000000110000110000,
     0b00000000000000010000101000010000,  
     0b00000000011000100100100100011000,
     0b00000000000000010000101000010000           
  };

  unsigned long int adivinawi_code[]={
     0b00100001000000000000000000100001,
     0b00010010100001000000100001010010,
     0b00001100010010100001010010001100,
     0b00000000001100010010001100000000,
     0b00000000000000001100000000000000,
     0b00000000000000000000000000000000
  };

  unsigned long int wave_code[]={
     0b00001100010010100001000000000000,
     0b00000110001001010000100000000000,
     0b00000011000100001000010000100000,
     0b00000001000010000100001000110000,
     0b00000000000001000010100100011000,
     0b00000000000000100001010010001100,
     0b00000000100000010000001001000110,
     0b00100000010000001000000100000011,
     0b00110000001000000100000010000001,
     0b00011000100100000010000001000000    
  };

  switch  (anim){

    case littleUuh:
        return littleUuh_code[index];
        break;
    case dreamMouth:
        return dreamMouth_code[index];
        break;
    case adivinawi:
        return adivinawi_code[index];
        break;
    case wave:
        return wave_code[index];
        break;    
  }   
}


void putAnimationMouth(unsigned long int aniMouth, int index){

      ledmatrix.writeFull(getAnimShape(aniMouth,index));
}
////////////////////////////////////////////////////////
void playGesture(int gesture1){
switch(gesture1){

    case OttoHappy: 
        tonenew(note_E5,50,30);
        putMouth(smile);
        soundcmd(S_happy_short);
        //swing(1,800,20); 
        soundcmd(S_happy_short);

        putMouth(happyOpen);
    break;


    case OttoSuperHappy:
        putMouth(happyOpen);
        soundcmd(S_happy);
        putMouth(happyClosed);
        //tiptoeSwing(1,500,20);
        putMouth(happyOpen);
        soundcmd(S_superHappy);
        putMouth(happyClosed);
        //tiptoeSwing(1,500,20); 

         
        putMouth(happyOpen);
    break;


    case OttoSad: 
        putMouth(sad);
        bendTones(880, 830, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(830, 790, 1.02, 20, 200);  
        putMouth(sadOpen);
        bendTones(790, 740, 1.02, 20, 200);
        putMouth(sadClosed);
        bendTones(740, 700, 1.02, 20, 200);
        putMouth(sadOpen);
        bendTones(700, 669, 1.02, 20, 200);
        putMouth(sad);
        delay(500);

 
        delay(300);
        putMouth(happyOpen);
    break;


    case OttoSleeping:

        for(int i=0; i<4;i++){
          putAnimationMouth(dreamMouth,0);
          bendTones (100, 200, 1.04, 10, 10);
          putAnimationMouth(dreamMouth,1);
          bendTones (200, 300, 1.04, 10, 10);  
          putAnimationMouth(dreamMouth,2);
          bendTones (300, 500, 1.04, 10, 10);   
          delay(500);
          putAnimationMouth(dreamMouth,1);
          bendTones (400, 250, 1.04, 10, 1); 
          putAnimationMouth(dreamMouth,0);
          bendTones (250, 100, 1.04, 10, 1); 
          delay(500);
        } 

        putMouth(lineMouth);
        soundcmd(S_cuddly);

   
        putMouth(happyOpen);
    break;


    case OttoFart:
        delay(300);     
        putMouth(lineMouth);
        soundcmd(S_fart1);  
        putMouth(tongueOut);
        delay(300);
        putMouth(lineMouth);
        soundcmd(S_fart2); 
        putMouth(tongueOut);
        delay(300);
        putMouth(lineMouth);
        soundcmd(S_fart3);
        putMouth(tongueOut);    
        delay(300);

      
        delay(500); 
        putMouth(happyOpen);
    break;


    case OttoConfused:
        putMouth(confused);
        soundcmd(S_confused);
        delay(500);

        
        putMouth(happyOpen);
    break;


    case OttoLove:
        putMouth(heart);
        soundcmd(S_cuddly);
        //crusaito(2,1500,15,1);

        
        soundcmd(S_happy_short);  
        putMouth(happyOpen);
    break;


    case OttoAngry: 
        putMouth(angry);

        tonenew(note_A5,100,30);
        bendTones(note_A5, note_D6, 1.02, 7, 4);
        bendTones(note_D6, note_G6, 1.02, 10, 1);
        bendTones(note_G6, note_A5, 1.02, 10, 1);
        delay(15);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(400);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);

         
        putMouth(happyOpen);
    break;

    case OttoFretful: 
        putMouth(angry);
        bendTones(note_A5, note_D6, 1.02, 20, 4);
        bendTones(note_A5, note_E5, 1.02, 20, 4);
        delay(500);
        putMouth(lineMouth);
        delay(500);
        putMouth(angry);
        delay(500);
  
        putMouth(happyOpen);
    break;


    case OttoMagic:

        //Initial note frecuency = 400
        //Final note frecuency = 1000
        
        // Reproduce the animation four times
        for(int i = 0; i<4; i++){ 

          int noteM = 400; 

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //400 -> 1000 
              noteM+=100;
            }

            clearMouth();
            bendTones(noteM-100, noteM+100, 1.04, 10, 10);  //900 -> 1100

            for(int index = 0; index<6; index++){
              putAnimationMouth(adivinawi,index);
              bendTones(noteM, noteM+100, 1.04, 10, 10);    //1000 -> 400 
              noteM-=100;
            }
        } 
 
        delay(300);
        putMouth(happyOpen);
    break;


    case OttoWave:
        
        // Reproduce the animation four times
        for(int i = 0; i<2; i++){ 

            int noteW = 500; 

            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW+100, 1.02, 10, 10); 
              noteW+=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
            for(int index = 0; index<10; index++){
              putAnimationMouth(wave,index);
              bendTones(noteW, noteW-100, 1.02, 10, 10); 
              noteW-=101;
            }
        }    

        clearMouth();
        delay(100);
        putMouth(happyOpen);
    break;

    case OttoVictory:
        
        putMouth(smallSurprise);
        for (int i = 0; i < 60; ++i){
                    tonenew(1600+i*20,15,1);
        }

        putMouth(bigSurprise);
        for (int i = 0; i < 60; ++i){
                    tonenew(2800+i*20,15,1);
        }

        putMouth(happyOpen);
        //SUPER HAPPY
        soundcmd(S_superHappy);
        putMouth(happyClosed);
        //-----
        
        clearMouth();
        putMouth(happyOpen);

    break;

    case OttoFail:

        putMouth(sadOpen);
        tonenew(900,200,1);
        putMouth(sadClosed);
        tonenew(600,200,1);
        putMouth(confused);
        tonenew(300,200,1);
        putMouth(xMouth);
        tonenew(150,2200,1);
        
        delay(600);
        clearMouth();
        putMouth(happyOpen);

    break;

  }
}    
void getBatteryLevel(){

  //The first read of the batery is often a wrong reading, so we will discard this value. 
    double batteryLevel = battery.readBatPercent();
    double batteryReadings = 0;
    int numReadings = 10;

    for(int i=0; i<numReadings; i++){
        batteryReadings += battery.readBatPercent();
        delay(1); // delay in between reads for stability
    }

    batteryLevel = batteryReadings / numReadings;
}




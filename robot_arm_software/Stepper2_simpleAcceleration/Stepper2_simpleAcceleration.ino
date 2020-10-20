//------------------------------------------------------------------------------
// 4 Axis CNC Demo v2 - supports Adafruit motor shield v2
// dan@marginallycelver.com 2013-08-30
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

#define VERSION              (2)  // firmware version
#define BAUD                 (115200)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (800)  // depends on your stepper motor.  most are 200.
#define MIN_STEP_DELAY       (50)
#define MAX_FEEDRATE         (1000000/MIN_STEP_DELAY)
#define MIN_FEEDRATE         (0.01)
#define NUM_AXIES            (4)

//Gearing ratios
#define STEPPER_RATIO_X     (2)   //Defines the ratios between 
#define STEPPER_RATIO_Y     (2) 
#define STEPPER_RATIO_Z     (2) 
//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include <Wire.h>
#include <AccelStepper.h>
#include "pins.h"


//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;
  long absdelta;
  int dir;
  long over;
} Axis;

char serialBuffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

float px, py, pz, pa, pb;  // location

// spaeds
float fr=0;  // human version
long step_delay;  // machine version

// settings
char mode_abs=1;  // absolute mode?

long line_number=0;

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
 * delay for the appropriate number of microseconds
 * @input ms how many milliseconds to wait
 */
void pause(long ms) {
  delay(ms/1000);
  delayMicroseconds(ms%1000);  // delayMicroseconds doesn't work for values > ~16k.
}


/**
 * Set the feedrate (speed motors will move)
 * @input nfr the new speed in steps/second
 */
void feedrate(float nfr) {
  if(fr==nfr) return;  // same as last time?  Quit now.

  if(nfr>MAX_FEEDRATE || nfr<MIN_FEEDRATE) {  // don't allow crazy feed rates
    Serial.print(F("New feedrate must be greater than "));
    Serial.print(MIN_FEEDRATE);
    Serial.print(F("steps/s and less than "));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s."));
    return;
  }
  step_delay = 1000000.0/nfr;
  fr=nfr;
}


/**
 * Set the logical position
 * @input npx new position x
 * @input npy new position y
 */

void release(){
  digitalWrite(xEnable, LOW);
  digitalWrite(yEnable, LOW);
  digitalWrite(zEnable, LOW);
 
  }

void enable(){
  digitalWrite(xEnable, HIGH);
  digitalWrite(yEnable, HIGH);
  digitalWrite(zEnable, HIGH);
}
void position(float npx,float npy,float npz,float npa,float npb){
  // here is a good place to add sanity tests
  px=npx;
  py=npy;
  pz=npz;
  pa=npa;
  pb=npb;
}

void updateCurrentPosition() {
  px = map(analogRead(xPot), 0, 1023, 0, 255);
  py = map(analogRead(yPot), 0, 1023, 0, 255);
  pz = map(analogRead(zPot), 0, 1023, 0, 255);

}


/**
 * Look for character /code/ in the serialBuffer and read the float that immediately follows it.
 * @return the value found.  If nothing is found, /val/ is returned.
 * @input code the character to look for.
 * @input val the return value if /code/ is not found.
 **/
float parseNumber(char code,float val) {
  char *ptr=serialBuffer;  // start at the beginning of serialBuffer
  while((long)ptr > 1 && (*ptr) && (long)ptr < (long)serialBuffer+sofar) {  // walk to the end
    if(*ptr==code) {  // if you find code on your walk,
      return atof(ptr+1);  // convert the digits that follow into a float and return it
    }
    ptr=strchr(ptr,' ')+1;  // take a step from here to the letter after the next space
  }
  return val;  // end reached, nothing found, return default val.
}


/**
 * write a string followed by a float to the serial line.  Convenient for debugging.
 * @input code the string.
 * @input val the float.
 */
void output(char *code, float val) {
  Serial.print(code);
  Serial.println(val);
}


/**
 * print the current position, feedrate, and absolute mode.
 */
void where() {
  output("X",px);
  output("Y",py);
  output("Z",pz);
  output("A",pa);
  output("B",pb);
  output("F",fr);
  Serial.println(mode_abs?"ABS":"REL");
} 


/**
 * display helpful information
 */
void help() {
  Serial.print(F("GcodeCNCDemo4AxisV2 "));
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [Z(steps)] [E(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [Z(steps)] [E(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)] [Z(steps)] [E(steps)]; - change logical position"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("All commands must end with a newline."));
}


/**
 * Read the serialBuffer and find any recognized commands.  One G or M command per line.
 */
void processCommand() {
  // blank lines
  if(serialBuffer[0]==';') return;
  
  long cmd;
  
  // is there a line number?
  cmd=parseNumber('N',-1);
  if(cmd!=-1 && serialBuffer[0]=='N') {  // line number must appear first on the line
    if(cmd != line_number) {
      // wrong line number error
      Serial.print(F("BADLINENUM "));
      Serial.println(line_number);
      return;
    }
  
    // is there a checksum?
    if(strchr(serialBuffer,'*')!=0) {
      // yes.  is it valid?
      char checksum=0;
      int c=0;
      while(serialBuffer[c]!='*') checksum ^= serialBuffer[c++];
      c++; // skip *
      int against = strtod(serialBuffer+c,NULL);
      if(checksum != against) {
        Serial.print(F("BADCHECKSUM "));
        Serial.println(line_number);
        return;
      } 
    } else {
      Serial.print(F("NOCHECKSUM "));
      Serial.println(line_number);
      return;
    }
    
    line_number++;
  }

  cmd = parseNumber('G',-1);
  switch(cmd) {
  case  0: 
  case  1: 
    feedrate(parseNumber('F',fr));
    parseNumber('X',(mode_abs?px:0)) + (mode_abs?0:px);
    parseNumber('Y',(mode_abs?py:0)) + (mode_abs?0:py);
    parseNumber('Z',(mode_abs?pz:0)) + (mode_abs?0:pz);
    parseNumber('A',(mode_abs?pa:0)) + (mode_abs?0:pa);
    parseNumber('B',(mode_abs?pb:0)) + (mode_abs?0:pb);
    
  case  4:  pause(parseNumber('P',0)*1000);  break;  // dwell
  case 90:  mode_abs=1;  break;  // absolute mode
  case 91:  mode_abs=0;  break;  // relative mode
  case 92:  // set logical position
    position( parseNumber('X',0),
              parseNumber('Y',0),
              parseNumber('Z',0),
              parseNumber('A',0),
              parseNumber('B',0));
    break;
  default:  break;
  }

  cmd = parseNumber('M',-1);
  switch(cmd) {
  case 18:  // disable motors
    break;
  case 100:  help();  break;
  case 114:  where();  break;
  default:  break;
  }
}


/**
 * prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
 */
void ready() {
  sofar=0;  // clear input serialBuffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
 * First thing this machine does on startup.  Runs only once.
 */
void setup() {
  Serial.begin(BAUD);  // open coms
  delay(4000);

  help();  // say hello
  position(0,0,0,0,0);  // set staring position
  feedrate(200);  // set default speed
  ready();
}


/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  // listen for serial commands
  while(Serial.available() > 0) {  // if something is available
    char c=Serial.read();  // get it
    Serial.print(c);  // repeat it back so I know you got the message
    if(sofar<MAX_BUF-1) serialBuffer[sofar++]=c;  // store it
    if(c=='\n') {
      // entire message received
      serialBuffer[sofar]=0;  // end the buffer so string functions work right
      Serial.print(F("\r\n"));  // echo a return character for humans
      processCommand();  // do something with the command
      ready();
    }
  }
}


/**
* This file is part of GcodeCNCDemo.
*
* GcodeCNCDemo is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* GcodeCNCDemo is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Foobar. If not, see <http://www.gnu.org/licenses/>.
*/

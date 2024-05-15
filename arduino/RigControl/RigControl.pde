#include <EEPROM.h>

#include <math.h>

#define TEMP_USED 0
#define VALVES_USED 1
#define PUMP_USED 1
#define LEDS_USED 1
#define RIGLIGHTS_USED 1
#define TIMERS_USED 1
#define LAMBDA_USED 0

float targetTemp = 0.0;
float heaterMax = 40.0;
int heatEnabled = 0;
float delayConst = 20.0;

float avg[2] = {
  512.0,512.0};  // running average Vin values
float temp[2];                 // current temperatures
float derivative = 0.0;        // running average of rate of change in temperature
int tempCalculated = 0;        // 1 if temp is up to date
float output = 0.0;            // current desired heater power level (0.0 - 1.0)
double lastTempUpdate = 0.0;   // Last time outputs were recomputed

float outputPWM   = 0.0;         // current PWM output

// Coefficients are used to compute temperature from voltage inputs V0 and V1:
//    [A0, B0, C0, A1, B1, C1]
//    R = (1023 / V0) - 1                   // compute resistance of thermistor
//                                       (note R2 does not show up in the voltage divider equation because it is included in A0)
//    1 / T0 = A0 + B0 * log(R) + C0 * log(R)^3    // compute temperature using Steinhart-Hart equation
//    
float coefficients[6] = {
  0.04, 0.03, -0.01, 0.04, 0.03, -0.01};

// Min and max values from output sensor
short outputMin, outputMax;

double lastTimeRestart = 0.0;
unsigned long lastTime = 0;
int c = 0;

// DIO Line definitions
// 0 means line is inactive
// ALL DIOS must be defined here, to avoid collisions
#define RIG_LIGHT1  11
#define RIG_LIGHT2  0
#define RIG_LIGHT3  0
#define LED_LIGHT1  10
#define LED_LIGHT2  0
#define LED_LIGHT3  0
#define VALVE1  3
#define VALVE2  5
#define VALVE3  6
#define VALVE4  0 // inactive
#define STATUS_LED 8
#define ON_BOARD_LED 13
#define TEMP_PWM_OUT 9

#define LOW 0
#define HIGH 1

// Bit setting on DIO lines 2,3 to select the device controlled:
#define RIG_LIGHT 1
#define LED_LIGHT 2
#define VALVES   3

// blink stuff for testing/verification of logic
int bldur = 20;

void blpattern(int n, int m) {
  nblink(n);
  delay(bldur*5);
  nblink(m);
}

void nblink(int n) {
  int i;
  for (i = 0; i < n; i++) {
    blink();
    delay(bldur*3);
  }
}

void blink() {
  digitalWrite(ON_BOARD_LED, HIGH);
  delay(bldur);
  digitalWrite(ON_BOARD_LED, LOW);
}

void setup() {
  Serial.begin(115200);

  //  Timer0 - Pins 6, 5;  Regs TCCR0A,B; Beware--used for millis() and delay()
  //  Timer1 - Pins 9, 10; Regs TCCR1A,B; 16-bit timer
  //  Timer2 - Pins 11, 3; Regs TCCR2A,B
  // 
  // Register   Arduino Chip    Pin
  // OC0A	6	12	PD6
  // OC0B	5	11	PD5
  // OC1A	9	15	PB1
  // OC1B	10	16	PB2
  // OC2A	11	17	PB3
  // OC2B	3	5	PD3
  // 
  // TCCRnB:
  //   [0-4]
  //   [5-7] - clock divide
  //           Timer0,1: 0x1=1, 0x2=8, 0x3=64, 0x4=256, 0x5=1024
  //           Timer2: 0x1=1, 0x2=8, 0x3=32, 0x4=64, 0x5=128, 0x6=256, 0x7=1024
  // 

  // Set PWM on ports 9 & 10 to run at 32kHz
  // With an RC low-pass filter (10uF, 100Ohm) we get good analog output to about 100Hz  (corner frequency is 1kHz)
  TCCR1B = TCCR1B & 0b11111000 | 0x1;

  // Set 10-bit PWM (only available on Timer1)
  TCCR1B &= ~(1 << WGM13);    // Timer B clear bit 4
  TCCR1B |=  (1 << WGM12);    // set bit 3

  TCCR1A |= (1 << WGM11);    //  Timer A set bit 1
  TCCR1A |= (1 << WGM10);    //  set bit 0  

    pinMode(STATUS_LED, OUTPUT);  // status LED
  if (TEMP_USED == 1) {
    blink();
    measureOutputCurve();
    blink();
    loadCoeffs();
  }
  Serial.println("Ready.");
}


void measureOutputCurve() {
  short vals[128];
  short minVal = 1024.;
  short maxVal = 0.;

  // Scan entire 1024-bit output range
  for( int i=0; i<64; i++ ) {  
    analogWrite(TEMP_PWM_OUT, i*16);
    delay(20);
    vals[i] = analogRead(2);
    minVal = min(minVal, vals[i]);
    maxVal = max(maxVal, vals[i]);
  }
  analogWrite(TEMP_PWM_OUT, 0);

  outputMin = minVal;
  outputMax = maxVal;
}


// Read float from serial port
float readFloat() {
  union { 
    float x; 
    char c[4]; 
  } 
  fl;
  for (int i=0; i<4; i++) {
    fl.c[i] = Serial.read();
  }
  return fl.x;
}

void printFloat(float v) {
  union { 
    float x; 
    char c[4]; 
  } 
  fl;
  fl.x = v;
  for (int i=0; i<4; i++) {
    Serial.write(fl.c[i]);
  }
}

// Store float to eeprom
void storeFloat(int pos, float v) {
  union { 
    float x; 
    char c[4]; 
  } 
  fl;
  fl.x = v;
  for (int i=0; i<4; i++) {
    EEPROM.write(pos+i, fl.c[i]);
  }  
}

// Read float from eeprom
float loadFloat(int pos) {
  union { 
    float x; 
    char c[4]; 
  } 
  fl;
  for (int i=0; i<4; i++) {
    fl.c[i] = EEPROM.read(pos+i);
  }  
  return fl.x;
}

// Read thermistor coefficients from EEPROM
void loadCoeffs() {
  for (int i=0; i<6; i++) {
    coefficients[i] = loadFloat(i*4);
  }
}

// Write thermistor coefficients to EEPROM
void storeCoeffs() {
  for (int i=0; i<6; i++) {
    storeFloat(i*4, coefficients[i]);
  }
}

float computeTemps() {
  if (tempCalculated == 0) {
    for (int i=0; i<2; i++) {
      int cc = 3 * i;
      float R = (1023. / avg[i]) - 1.0;
      float lnR = log(R);
      temp[i] = 1.0 / (coefficients[cc]  +  (coefficients[1+cc] * lnR)  +  (coefficients[2+cc] * lnR * lnR * lnR));

    }
    tempCalculated = 1;
  }
}

void device_select(int dev) {
  switch (dev) {
  case 0:
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    break;
  case 1:
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    break;
  case 2:
    digitalWrite(2, LOW);
    digitalWrite(3, HIGH);
    break;
  case 3:
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    break;
  default:
    break;
  }
}



void readCommand() {
  char ledno;
  char onoff;

  char cmd = Serial.read();

  switch(cmd) {
  case 'l': // turn an led on or off: "l10" turns led 1 off
    {
      ledno = Serial.read();
      onoff = Serial.read();
      if (LEDS_USED == 1) {
        device_select(LED_LIGHT);
        switch(ledno) {
        case '1':
          blpattern(1, 1);
          if (onoff =='1') {
            digitalWrite(LED_LIGHT1, HIGH);
          } 
          else {
            digitalWrite(LED_LIGHT1, LOW);
          }  
          break;
        case '2':
          blpattern(1, 2);
          if (onoff =='1') {
            digitalWrite(LED_LIGHT2, HIGH);
          } 
          else {
            digitalWrite(LED_LIGHT2, LOW);
          }  
          break;
        case '3':
          blpattern(1, 3);
          if (onoff =='1') {
            digitalWrite(LED_LIGHT3, HIGH);
          } 
          else {
            digitalWrite(LED_LIGHT3, LOW);
          }  
          break;

        default:
          break;
        }
        device_select(0);
        break;
      }
    }


  case 'L': // turn Rig lights on or off: "L10" turns rig light 1 (red) off
    {
      ledno = Serial.read();
      onoff = Serial.read();
      if (RIGLIGHTS_USED == 1) {
        device_select(RIG_LIGHT);
        switch(ledno) {
        case '1':
          blpattern(2, 1);
          if (onoff =='1') {
            digitalWrite(RIG_LIGHT1, HIGH);
          } 
          else {
            digitalWrite(RIG_LIGHT1, LOW);
          }  
          break;

        case '2':
          blpattern(2, 2);
          if (onoff =='1') {
            digitalWrite(RIG_LIGHT2, HIGH);
          } 
          else {
            digitalWrite(RIG_LIGHT2, LOW);
          }  
          break;
        case '3':
          blpattern(2, 3);
          if (onoff =='1') {
            digitalWrite(RIG_LIGHT3, HIGH);
          } 
          else {
            digitalWrite(RIG_LIGHT3, LOW);
          }  
          break;
        default:
          break;
        }
        device_select(0);
        break;
      }
    }

  case 'V': // Valves: "V10" turns valve 0 off. 
    {
      ledno = Serial.read();
      onoff = Serial.read();
      if (VALVES_USED == 1 ) {
        device_select(VALVES);
        switch(ledno) {
        case '1':
          blpattern(3, 1);
          if (onoff =='1') {
            digitalWrite(VALVE1, HIGH);
          } 
          else {
            digitalWrite(VALVE1, LOW);
          }  
          break;

        case '2':
          blpattern(3, 2);
          if (onoff =='1') {
            digitalWrite(VALVE2, HIGH);
          } 
          else {
            digitalWrite(VALVE2, LOW);
          }  
          break;
        case '3':
          blpattern(3, 3);
          if (onoff =='1') {
            digitalWrite(VALVE3, HIGH);
          } 
          else {
            digitalWrite(VALVE3, LOW);
          }  
          break;
        case '4':
          blpattern(3, 4);
          if (onoff =='1') {
            digitalWrite(VALVE4, HIGH);
          } 
          else {
            digitalWrite(VALVE4, LOW);
          }  
          break;
        default:
          break;
        }
        device_select(0);
        break;
      }
    }

  case  'c': 
    if (TEMP_USED == 1){  // set new thermistor coeffs.
      Serial.print("Coefficients: ");
      for (int i=0; i<6; i++) {
        Serial.print(coefficients[i]);
        Serial.print("->");
        coefficients[i] = readFloat();
        Serial.print(coefficients[i]);
        Serial.print(", ");
      }
      Serial.println("");
      storeCoeffs();  // write new values to EEPROM 
      break;
    }

  case  's': 
    if (TEMP_USED == 1) {  // set temperature target
      Serial.print("Set Temperature: ");
      targetTemp = readFloat();
      Serial.print(targetTemp);
      Serial.println(""); 
      break;  
    }

  case 'm': 
    if (TEMP_USED == 1){  // set max heater temperature
      Serial.print("Set Max Heater Temp: ");
      heaterMax = readFloat();
      Serial.print(heaterMax);
      Serial.println(""); 
      break;  
    }

  case 'e': 
    if (TEMP_USED == 1) {  // enable heater
      Serial.print("Enable heater: ");
      heatEnabled = (int) Serial.read();
      Serial.print(heatEnabled);
      Serial.println("");   
      break;
    }

  case 'd': 

    if (TEMP_USED == 1) {  // set delay
      Serial.print("Set Delay: ");
      delayConst = readFloat();
      Serial.print(delayConst);
      Serial.println(""); 
      break;  
    }

//#define TEMP_USED 0
//#define VALVES_USED 1
//#define PUMP_USED 0
//#define LEDS_USED 0
//#define RIGLIGHTS_USED 0
//#define TIMERS_USED 0
//#define LAMBDA_USED 0
  case 'q': // report status of flags, so that python can disable unused tabs in the control
  {
      Serial.print(TEMP_USED);
      Serial.print(",");
      Serial.print(VALVES_USED);
      Serial.print(",");
      Serial.print(PUMP_USED);
      Serial.print(",");
      Serial.print(LEDS_USED);
      Serial.print(",");
      Serial.print(RIGLIGHTS_USED);
      Serial.print(",");
      Serial.print(TIMERS_USED);
      Serial.print(",");
      Serial.print(LAMBDA_USED);
      Serial.println("");
      break;
  }
  case '?': 
    {  // report state
      if (TEMP_USED == 1) {
        computeTemps();
        Serial.print(temp[0]);
        Serial.print(",");
        Serial.print(temp[1]);
        Serial.print(",");
        Serial.print(avg[0]);
        Serial.print(",");
        Serial.print(avg[1]);
        Serial.print(",");
        Serial.print(targetTemp);
        Serial.print(",");
        Serial.print(heaterMax);
        Serial.print(",");
        Serial.print(delayConst);
        Serial.print(",");
        Serial.print(output);
        Serial.print(",");
        Serial.print(outputPWM);
        Serial.print(",");
        Serial.print(heatEnabled);
        Serial.println("");
        break;
      }
    }

  default: 
    {
      Serial.flush();
      Serial.print("Ignored command: <");
      Serial.print(cmd);
      Serial.println(">");
      break;
    } 
    //blink();
  } // end of switch

}


void loop() {
  //measure current time
  unsigned long now = millis();
  if (now < lastTime) {
    lastTimeRestart += 4294967.296;  // millis() counter resets ~every 50 days; keep track.
  }
  lastTime = now;
  double time = lastTimeRestart + (float)now * 0.001;

  if (TEMP_USED == 1) {
    // first service the temperature controller
    // Measure analog inputs, do exponential average
    for (int i=0; i<2; i++) {
      int x = analogRead(i);
      avg[i] = avg[i] * 0.98 + x * 0.02;
    } 
    tempCalculated = 0;
  }

  // Every 10th iteration, compute temperature and check for serial commands
  if (c > 10) {
    // measure time since last update
    double dt = time - lastTempUpdate;
    lastTempUpdate = time;

    // remember last temperature and compute new ones
    float lastTemp = temp[0];
    if (TEMP_USED == 1) {
      computeTemps();

      // Determine rate of change in temperature
      float diff = (temp[0] - lastTemp) / dt;
      derivative = derivative * 0.9 + diff * 0.1;

      // How far off from target temp are we?
      diff = targetTemp - temp[0];

      // Select new output value 
      output = output + (diff * 0.001 / delayConst) - (derivative * 0.1 / delayConst);
      output = max(0, min(output, 1.0));
    }

    // // process serial commands
    c = 0;
    // if (Serial.available() > 0) {
    //   readCommand();
    // }
  }

  // Given we want a certain output 0.0 to 1.0, find the optimal PWM needed to achieve this value
  float currentOutput = 1.0 - ((float)(analogRead(2) - outputMin) / (float)(outputMax - outputMin));
  float diff = output - currentOutput;
  outputPWM += diff * 5.;  
  outputPWM = max(0.0, min(outputPWM, 1023.));

  // Output to PWM port, with failsafe
  if( temp[1] > heaterMax | heatEnabled == 0 ) {
    analogWrite(TEMP_PWM_OUT, 0);
  }
  else {
    analogWrite(TEMP_PWM_OUT, (int)outputPWM);  
  }
  // wait a bit for the analog-to-digital converter 
  // to stabilize after the last reading:
  delay(1);
  c++;
  // process serial commands

    if (Serial.available() > 0) {
      readCommand();
    }
}





/*
 * GUI LIBRIS Code - Synchronized with 20251031_generaloutputcollect_libgen_original.ino
 * 
 * Key Synchronizations:
 * - Regulator count: Updated to 9 regulators (6 active: AS/BS/CS/DS/ES/FS, 3 reserved: GS/HS/IS)
 * - Data structure: Receives 4 formulations x 9 regulators (last 3 always 0)
 * - Regulator mapping: arr[formulation][0-5] = AS/BS/CS/DS/ES/FS, [6-8] = GS/HS/IS (unused)
 * - Formulation indexing: pressureFormulation (0-3) directly maps to received array rows
 * 
 * Differences from 20251031:
 * - Uses PWM control (OCR registers) instead of Serial communication with Alicat regulators
 * - Receives only 4 formulations (no priming rows that 20251031 has at indices 0-1)
 * - Uses Adafruit MotorShield instead of direct stepper control
 * - Uses servos for Z-axis instead of stepper motors
 * - Different timing constants (deposit uses 20000ms vs 20251031's collect_time 2000ms)
 */

#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <Servo.h>

//GUI variables
// Updated to match 20251031: 9 regulators per formulation
// Note: 20251031 has NUMBER_OF_FORMULATIONS+2 rows (2 prime + 4 formulations)
// This GUI code receives only the 4 formulations (no prime rows)
const int rows = 4;
const int cols = 9;  // Updated to match 20251031: 9 regulators (6 active + 3 reserved)
int rowIndex = 0;
int counter = 0;
float arr[rows][cols] = { 0 };

Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #1 (M1 and M2) and #2.
Adafruit_StepperMotor *xLongStepper = AFMS.getStepper(200, 2);
Adafruit_StepperMotor *yShortStepper = AFMS.getStepper(200, 1);

//Attach the two servos. They should behave the same.
Servo myServo1;
//Servo myServo2;

//Listing some useful conversions/constants here because I will need
//to remember them to write the code, lol.
const int rotationPerInch = 1;  //right now, I think we are using the same 4-start threaded screw for both directions
const int stepPerRot = 200;
const float degreesPerStep = 360 / stepPerRot;  //I think all 3 motors are the same, 200-step motors.
// const int wellSpacing=9; //millimeters. that's 0.354331 inches. great.
// const int xWellNumber = 2; //need to move over 2 spaces at a time
// const int yWellNumber = 4; // need to move up and down 4 spaces at a time.


const int buttonPin = 4;

const int servoDelay = 5;
const int stepperSpeed = 250;  //10 rpm, or 3.333inches per minute. hm.... can go up to 50rpm.
const float timePerRot = 60 / stepperSpeed;
//Time per rotation in seconds
const float timePerStep = timePerRot / stepPerRot;
const float timePerInterStep = 2 * timePerStep;

//change this if the servos are going the wrong way bc I forget
const int servoUp_mem = 15;  //joke's on me because I did not use this to make it easy to switch my code
const int servoUp_cas = 40;
const int servoDown = 180;
int current_position = 180;
const int stepSize = 2;  //for servos?

//PLEASE MAKE SURE the step size goes evenly into the servoUp-servoDown so that my = condition works. thanks

// pretty sure if red wire is in lower number thingy, FORWARDS will be clockwise when looking down the axle.
//and so with this, I think yeah, forwards IS up, in terms of what wells you're in.
const unsigned int yUpDir = BACKWARD;  //"Note that forward and backward directions are arbitrary and can be swapped by swapping wires"
const unsigned int yDownDir = FORWARD;
const unsigned int xDir1 = BACKWARD;
const unsigned int xDir2 = FORWARD;
//this assumes that increasing the value on the servos increases the position, but idk
// if that's the case.

// const int x_center_number = 25;//optimal (circle hole)
// const int y_center_number = 380;//optimal

// const int x_center_number = 25;//optimal (square hole)
// const int y_center_number = 400;//optimal

const int x_center_number = 27;   //optimal
const int y_center_number = 400;  //optimal (square hole (needle))

int xStepNumber = 145;
const int yStepNumber = 284;
const int quickstepDelay = 850;

int pos = 0;
int buttonPushCounter = -1;
int buttonState = 0;
int moveFlag = 0;

int x_flush_target = 0;
int y_flush_target = 0;

bool Finished_flag = false;

const int NUMBER_OF_FORMULATIONS = 4;
// float arr[NUMBER_OF_FORMULATIONS][6] = {
//   { 8, 12, 8.5, 12, 18, 12.75 },
//   { 10, 10, 8.5, 15, 15, 12.75 },
//   { 12, 8, 8.5, 18, 12, 12.75 },
//   { 13, 7, 8.5, 19.5, 10.5, 12.75 },
// };

const int filter_size = 5;

int E1_filter[filter_size] = { 0 };
int E2_filter[filter_size] = { 0 };
int E3_filter[filter_size] = { 0 };
int A1_filter[filter_size] = { 0 };
int A2_filter[filter_size] = { 0 };
int A3_filter[filter_size] = { 0 };

const int reg_array_size = 30;

int E1_reg_array[reg_array_size] = { 0 };
int E2_reg_array[reg_array_size] = { 0 };
int E3_reg_array[reg_array_size] = { 0 };
int A1_reg_array[reg_array_size] = { 0 };
int A2_reg_array[reg_array_size] = { 0 };
int A3_reg_array[reg_array_size] = { 0 };

int mean_filter_position = 0;

// Workflow variable: tracks current formulation index (0-3)
// Note: In 20251031 code, this is called formulation_counter and includes 2 priming rows
// This GUI code receives only the 4 formulations (no prime rows), so pressureFormulation
// directly maps to arr[pressureFormulation] where pressureFormulation = 0,1,2,3
int pressureFormulation = 0;

int E1 = 2;
int E2 = 3;

int E3 = 7;
int pin_A1 = 8;

int pin_A2 = 12;
int pin_A3 = 13;

int E1_pin = A15;
int E2_pin = A13;
int E3_pin = A11;
int A1_pin = A9;
int A2_pin = A7;
int A3_pin = A2;

int E1_reg_pin = A15;
int E2_reg_pin = A14;
int E3_reg_pin = A13;
int A1_reg_pin = A12;
int A2_reg_pin = A11;
int A3_reg_pin = A10;

int max_E1 = 400;
int max_E2 = 400;
int max_E3 = 400;
int max_A1 = 400;
int max_A2 = 800;
int max_A3 = 400;
int delta_smooth = 0;
int delta_abs = 5;
bool kill_switch = false;

int output3B = 2;
int output3C = 3;

int output4B = 7;
int output4C = 8;

int output1B = 12;
int output1C = 13;

bool equilibrate = true;

bool find_data = false;



//data collection

void printArray() {
  Serial.println("Received array:");
  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      Serial.print(arr[i][j], 2);
      if (j < cols - 1) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }
}

void MakeArray() {
  //Serial.println("aaaa");
  if (Serial1.available()) {
    // Serial.println("looking for data");
    static String receivedData = "";
    char receivedChar = Serial1.read();
    Serial.println(receivedChar);
    if (receivedChar == 'f') {  //check for completion flag from other code
      // Newline indicates end of row
      // Split receivedData into individual floats and store in the array
      int colIndex = 0;
      int startIndex = 0;
      for (int i = 0; i < receivedData.length(); i++) {
        if (receivedData[i] == ',') {
          arr[rowIndex][colIndex] = receivedData.substring(startIndex, i).toFloat();
          colIndex++;
          startIndex = i + 1;
        }
      } 
      // Last element in the row
      arr[rowIndex][colIndex] = receivedData.substring(startIndex).toFloat();
      rowIndex++;
      counter++;
      receivedData = "";  // Reset receivedData for the next row
      if (counter == rows) {
        // End of array
        printArray();
        counter = 0;
        find_data = true;
      }
    } else {
      receivedData += receivedChar;  // Accumulate received characters
    }
  }
}

void plateMoveFunc(int location) {
  if (current_position < location){
    for(int i = current_position; i <= location; i++){
      myServo1.write(i);
      //myServo2.write(i);
      delay(2);
    }
  }
  else{
    for(int i = current_position; i >= location; i--){
      myServo1.write(i);
      //myServo2.write(i);
      delay(2);
    }
  }
  delay(220);
  current_position = location;
}

void center() {
  for (int i = 0; i < (x_center_number - 1); i++) {
    xLongStepper->quickstep(xDir2);
    delayMicroseconds(quickstepDelay);
    delayMicroseconds(900);
  }
  delay(200);
  for (int i = 0; i < (y_center_number - 1); i++) {
    yShortStepper->quickstep(yDownDir);
    delayMicroseconds(quickstepDelay);
    delayMicroseconds(200);
  }
}

void moveX1() {
  plateMoveFunc(servoDown);
  for (int i = 0; i < xStepNumber - 1; i++) {
    xLongStepper->quickstep(xDir1);
    delayMicroseconds(quickstepDelay);
    delayMicroseconds(500);
  }
  if (equilibrate == true) {
    plateMoveFunc(servoUp_mem);
  } else {
    plateMoveFunc(servoUp_cas);
  }
  x_flush_target += xStepNumber;
}

void moveX2() {
  plateMoveFunc(servoDown);
  for (int i = 0; i < xStepNumber - 1; i++) {
    xLongStepper->quickstep(xDir2);
    delayMicroseconds(quickstepDelay);
    delayMicroseconds(500);
  }
  if (equilibrate == true) {
    plateMoveFunc(servoUp_mem);
  } else {
    plateMoveFunc(servoUp_cas);
  }
  x_flush_target -= xStepNumber;
}

void moveYup() {
  plateMoveFunc(servoDown);
  for (int i = 0; i < yStepNumber - 1; i++) {
    yShortStepper->quickstep(yUpDir);
    delayMicroseconds(quickstepDelay);
  }
  if (equilibrate == true) {
    plateMoveFunc(servoUp_mem);
  } else {
    plateMoveFunc(servoUp_cas);
  }
  y_flush_target += yStepNumber;
}

void moveYdown() {
  plateMoveFunc(servoDown);
  for (int i = 0; i < yStepNumber - 1; i++) {
    yShortStepper->quickstep(yDownDir);
    delayMicroseconds(quickstepDelay);
  }
  if (equilibrate == true) {
    plateMoveFunc(servoUp_mem);
  } else {
    plateMoveFunc(servoUp_cas);
  }
  y_flush_target -= yStepNumber;
}

void equilibrate_func() {
  // Updated to match 20251031 regulator mapping:
  // arr[pressureFormulation][0] = AS (E1), [1] = BS (E2), [2] = CS (E3)
  // arr[pressureFormulation][3] = DS (A1), [4] = ES (A2), [5] = FS (A3)
  // arr[pressureFormulation][6] = GS (reserved, 0), [7] = HS (reserved, 0), [8] = IS (reserved, 0)
  
  float max_E1 = (2*16383)/5+19;
  int E1_target_C = (arr[pressureFormulation][0] / 20.0) * max_E1-10;
  int E1_target_Reg = (arr[pressureFormulation][0])*(1.0/10.0)*(1023/2.56);
  
  OCR3B = E1_target_C;
  

  float max_E2 = (2*16383)/5+19;
  int E2_target_C = (arr[pressureFormulation][1] / 20.0) * max_E2-10;
  int E2_target_Reg = (arr[pressureFormulation][1])*(1.0/10.0)*(1023/2.56);
  
  OCR3C = E2_target_C;

  float max_E3 = (2*16383)/5+20;
  int E3_target_C = (arr[pressureFormulation][2] / 20.0) * max_E3-10;
  int E3_target_Reg = (arr[pressureFormulation][2])*(1.0/10.0)*(1023/2.56);
  
  OCR4B = E3_target_C;

  float max_A1 = (2*16383)/5+21;
  int A1_target_C = (arr[pressureFormulation][3] / 20.0) * max_A1-10;
  int A1_target_Reg = (arr[pressureFormulation][3])*(1.0/10.0)*(1023/2.56);
  
  OCR4C = A1_target_C;//new circuit (optimized)


  float max_A2 = (16383)/5+3;
  int A2_target_C = (arr[pressureFormulation][4] / 20.0) * max_A2;
  int A2_target_Reg = (arr[pressureFormulation][4])*(1.0/20.0)*(1023/2.56);

  OCR1B = A2_target_C;

  
  float max_A3 = (2*16383)/5+20;
  int A3_target_C = (arr[pressureFormulation][5] / 20.0) * max_A3-10;
  int A3_target_Reg = (arr[pressureFormulation][5])*(1.0/10.0)*(1023/2.56);

  OCR1C = A3_target_C;
  
  // Note: Regulators GS[6], HS[7], IS[8] are reserved and not used in this PWM-based system
  

  int R_index = 0;
  int counter = 0;
  int R_index_old = 3;
  int old_index = 0;


  int E1_smooth = 10;
  int E2_smooth = 10;
  int E3_smooth = 10;
  int A1_smooth = 10;
  int A2_smooth = 10;
  int A3_smooth = 10;
  delay(200);
  Serial.println("time to reach: ");
  Serial.println(millis());
  while (((((abs(E1_reg_array[R_index])-E1_target_Reg)>delta_abs) | (E1_smooth > delta_smooth) | ((abs(E2_reg_array[R_index])-E2_target_Reg)>delta_abs) | (E2_smooth > delta_smooth) | ((abs(E3_reg_array[R_index])-E3_target_Reg)>delta_abs) | (E3_smooth > delta_smooth) | ((abs(A1_reg_array[R_index])-A1_target_Reg)>delta_abs) |  (A1_smooth > delta_smooth) | ((abs(A2_reg_array[R_index])-A2_target_Reg)>delta_abs) | (A2_smooth > delta_smooth)) | ((abs(A3_reg_array[R_index])-A3_target_Reg)>delta_abs) | (A3_smooth > delta_smooth)) & (kill_switch == false)) {
    delay(50);
    E1_reg_array[R_index] = analogRead(E1_reg_pin);
    E2_reg_array[R_index] = analogRead(E2_reg_pin);
    E3_reg_array[R_index] = analogRead(E3_reg_pin);
    A1_reg_array[R_index] = analogRead(A1_reg_pin);
    A2_reg_array[R_index] = analogRead(A2_reg_pin);
    A3_reg_array[R_index] = analogRead(A3_reg_pin);
    counter = counter + 1;
    R_index = counter % reg_array_size;
    if (counter >= R_index_old) {
      old_index = (counter - R_index_old) % reg_array_size;
      E1_smooth = abs(E1_reg_array[R_index] - E1_reg_array[old_index]);
      E2_smooth = abs(E2_reg_array[R_index] - E2_reg_array[old_index]);
      E3_smooth = abs(E3_reg_array[R_index] - E3_reg_array[old_index]);
      A1_smooth = abs(A1_reg_array[R_index] - A1_reg_array[old_index]);
      A2_smooth = abs(A2_reg_array[R_index] - A2_reg_array[old_index]);
      A3_smooth = abs(A3_reg_array[R_index] - A3_reg_array[old_index]);
    }
    if ((((abs(E1_reg_array[R_index])-E1_target_Reg)>delta_abs) | (E1_smooth > delta_smooth) | ((abs(E2_reg_array[R_index])-E2_target_Reg)>delta_abs) | (E2_smooth > delta_smooth) | ((abs(E3_reg_array[R_index])-E3_target_Reg)>delta_abs) | (E3_smooth > delta_smooth) | ((abs(A1_reg_array[R_index])-A1_target_Reg)>delta_abs) |  (A1_smooth > delta_smooth) | ((abs(A2_reg_array[R_index])-A2_target_Reg)>delta_abs) | (A2_smooth > delta_smooth)) | ((abs(A3_reg_array[R_index])-A3_target_Reg)>delta_abs) | (A3_smooth > delta_smooth)){
      if(abs((E1_reg_array[R_index]-E1_target_Reg)) > delta_abs){
        OCR3B = OCR3B + (((float) (E1_reg_array[R_index]-E1_target_Reg))/(1023.0/(max_E1)))
      }
      if(abs((E2_reg_array[R_index]-E2_target_Reg)) > delta_abs){
        OCR3C = OCR3C + (((float) (E2_reg_array[R_index]-E2_target_Reg))/(1023.0/(max_E2)))
      }
      if(abs((E3_reg_array[R_index]-E3_target_Reg)) > delta_abs){
        OCR4B = OCR4B + (((float) (E3_reg_array[R_index]-E3_target_Reg))/(1023.0/(max_E3)))
      }
      if(abs((A1_reg_array[R_index]-A1_target_Reg)) > delta_abs){
        OCR4C = OCR4C + (((float) (A1_reg_array[R_index]-A1_target_Reg))/(1023.0/(max_A1)))
      }
      if(abs((A2_reg_array[R_index]-A2_target_Reg)) > delta_abs){
        OCR1B = OCR1B + (((float) (A2_reg_array[R_index]-A2_target_Reg))/(1023.0/(max_A2)))
      }
      if(abs((A3_reg_array[R_index]-A3_target_Reg)) > delta_abs){
        OCR1C = OCR1C + (((float) (A3_reg_array[R_index]-A3_target_Reg))/(1023.0/(max_A3)))
      }
    }
    if (Serial1.available()) {
      if (Serial1.read() == 't') {
        kill_switch = true;
        OCR3B = 0;
        OCR3C = 0;
        OCR4B = 0;
        OCR4C = 0;
        OCR1B = 0;
        OCR1C = 0;
      }
    }
  }
  Serial.println("time to finish: ");
  Serial.println(millis());
  unsigned long int start_time = millis();
  while (((millis() - start_time) < 500) & (kill_switch == false)) {
    if (Serial1.available()) {
      if (Serial1.read() == 't') {
        kill_switch = true;
        OCR3B = 0;
        OCR3C = 0;
        OCR4B = 0;
        OCR4C = 0;
        OCR1B = 0;
        OCR1C = 0;
      }
    }
  }
  pressureFormulation = pressureFormulation + 1;
}

void deposit() {
  // Note: 20251031 code uses collect_time = 2000ms
  // This GUI code uses 20000ms (10x longer) - may be intentional for different system
  unsigned long int start_deposit = millis();
  while (((millis() - start_deposit) < 20000) & (kill_switch == false)) {
    if (Serial1.available()) {
      if (Serial1.read() == 't') {
        kill_switch = true;
        OCR3B = 0;
        OCR3C = 0;
        OCR4B = 0;
        OCR4C = 0;
        OCR1B = 0;
        OCR1C = 0;
      }
    }
  }
  TCNT1 = 0;
  TCNT3 = 0;
  TCNT4 = 0;
}

void move_robot() {

  // buttonState = digitalRead(buttonPin);
  // if (moveFlag == HIGH & Finished_flag == false) {
  //   //Step 0: MOVE THE TRAY BACK UP, but leave steppers released
  //   //for last adjustments
  //   if (buttonPushCounter == 0) {
  //     center();
  //     moveFlag = LOW;
  //   }
  //   else {
      center();
      plateMoveFunc(servoUp_mem);
      if (kill_switch == false) {
        equilibrate_func();
        equilibrate = false;

        if (kill_switch == false) {
          moveX2();
          deposit();
          equilibrate = true;

          if (kill_switch == false) {
            moveX2();
            equilibrate_func();
            equilibrate = false;

            if (kill_switch == false) {
              moveX2();
              deposit();
              equilibrate = true;

              if (kill_switch == false) {
                moveX2();
                equilibrate_func();
                equilibrate = false;

                if (kill_switch == false) {
                  xStepNumber = xStepNumber - 6;
                  moveX2();
                  deposit();
                  equilibrate = true;

                  if (kill_switch == false) {
                    moveYdown();
                    equilibrate_func();
                    equilibrate = false;

                    if (kill_switch == false) {
                      xStepNumber = xStepNumber + 6;
                      moveX1();
                      deposit();
                      moveX1();
                    }
                  }
                }
              }
            }
          }
        }
      }
      OCR1B = 0;
      OCR1C = 0;
      OCR3B = 0;
      OCR3C = 0;
      OCR4B = 0;
      OCR4C = 0;
      Finished_flag = true;
      moveFlag = LOW;
      find_data = false;
  //  }
  // } else {
  //   //Serial.println("hiii");
  //     if (buttonState == HIGH & Finished_flag == false) {
  //       Serial.println(buttonPushCounter);
  //       buttonPushCounter += 1;
  //       Serial.println("Button press:");
  //       Serial.print(buttonPushCounter, '\n');
  //       moveFlag = HIGH;
  //     }
  // }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);  // Initialize serial communication with ESP32
  while (!Serial)
    ;
  if (!AFMS.begin()) {
    while (1)
      ;
  }

  xLongStepper->setSpeed(stepperSpeed);
  yShortStepper->setSpeed(stepperSpeed);
  xLongStepper->onestep(FORWARD, DOUBLE);
  yShortStepper->onestep(FORWARD, DOUBLE);

  //this is to initialize the PWM to 100% speed? Apparently

  // FOR TESTING ONLY !!!!:
  // moveX1();

  //UNCOMMENT LATER PROBABLY
  // xLongStepper->release();
  // yShortStepper->release();
  //I release the X and Y stepper motors

  myServo1.attach(9);
  //myServo2.attach(10);

  pinMode(buttonPin, INPUT);

  pinMode(22, INPUT);
  pinMode(23, OUTPUT);
  pinMode(24, INPUT);
  pinMode(25, OUTPUT);
  plateMoveFunc(servoDown);

  analogReference(INTERNAL2V56);


  pinMode(E1, OUTPUT);  // select Pin as ch-B
  pinMode(E2, OUTPUT);  // select Pin as ch-C

  pinMode(E3, OUTPUT);      // select Pin as ch-B
  pinMode(pin_A1, OUTPUT);  // select Pin as ch-C

  pinMode(pin_A2, OUTPUT);  // select Pin as ch-B
  pinMode(pin_A3, OUTPUT);  // select Pin as ch-C

  TCCR1A = 0b10101011;  // COM4B1 = 1, COM4C1 = 1, WGM41 = 1, WGM40 = 1
  TCCR1B = 0b00011001;  // WGM53 = 1, WGM52 = 1, CS50 = 1 (no prescaling)

  TCCR3A = 0b10101011;  // COM3B1 = 1, COM3C1 = 1, WGM31 = 1, WGM30 = 1
  TCCR3B = 0b00011001;  // WGM33 = 1, WGM32 = 1, CS30 = 1 (no prescaling)

  TCCR4A = 0b10101011;  // COM4B1 = 1, COM4C1 = 1, WGM41 = 1, WGM40 = 1
  TCCR4B = 0b00011001;  // WGM43 = 1, WGM42 = 1, CS40 = 1 (no prescaling)

  OCR1A = 16383;  // 16MHz/10kHz=1600   prescaler set to 1

  OCR3A = 16383;  // 16MHz/10kHz=1600   prescaler set to 1

  OCR4A = 16383;  // 16MHz/10kHz=1600   prescaler set to 1

  OCR3B = 0;
  OCR3C = 0;
  OCR4B = 0;
  OCR4C = 0;
  OCR1B = 0;
  OCR1C = 0;
  // put your setup code here, to run once:
}

void loop() {
  //Serial.println("in loop");
  if (find_data == false) {
    MakeArray();
  } else {
    move_robot();
  }
}

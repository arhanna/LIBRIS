#include <Wire.h>
#include <SoftwareSerial.h>


//_____________________________________Serial Communication Things________________________________________//
int ArdBaud = 19200;   // Baud rate for the serial monitor
int AlctBaud = 19200;  // Baud rate on the Alicat (see RS-232 settings), default is 19200

int TxPin = 51;  // Arduino Tx pin, connected to the Alicat Rx RS-232 pin (pin 3 on the 8 pin mini-DIN)
int RxPin = 50;  // Arduino Rx pin, connected to the Alicat Tx RS-232 pin (pin 5 on the 8 pin mini-DIN)

SoftwareSerial mySerial(RxPin, TxPin, 1);  // (Arduino Rx pin, Arduino Tx pin, Invert=true).

//____________________________________Pressure/Formulation Variables______________________________________//

const int NUMBER_OF_LIPIDS = 1;//number of unique lipids
const int NUMBER_OF_FORMULATIONS = 4; //number of formulations per lipid
const int NUMBER_OF_COLLECTIONS = 1;//number of times a given formulation or formulation set is/are collected
const int NUM_REGULATORS = 9; 
const int NUM_ACTIVE_REGULATORS = 6; // Set to 6 for testing with existing hardware, change to 9 when all regulators connected

// char arr[NUMBER_OF_FORMULATIONS+2][6][10] = {
//   {  "100", "30", "30", "100", "35" ,"35"}, // prime both sippers
//   {  "100", "100", "100", "55", "55" ,"55"}, // prime ethanol inputs
//   {"48.42",   "28.04", "45.88",  "68.47",   "39.64",  "64.87"},
//   {"53.52",   "22.93",   "45.88",   "75.68",   "32.43",   "64.87"}
// };

char arr[NUMBER_OF_FORMULATIONS+2][NUM_REGULATORS][10] = {
  {  "100", "60", "100", "60", "60" ,"60", "0", "0", "0"}, // prime both sippers
  {  "100", "100", "100", "45", "45" ,"45", "0", "0", "0"}, // prime ethanol inputs
  {"28.04" ,  "48.42" ,  "45.88"  , "39.64" ,  "68.47" ,  "64.87", "0", "0", "0"},
  {"33.13",   "43.33",   "45.88",   "46.85",   "61.26",   "64.87", "0", "0", "0"},
  {"43.33" ,  "33.13" ,  "45.88"  ,  "68.47" , "39.64" ,  "64.87", "0", "0", "0"},
  {"53.52" ,  "22.94" ,  "45.88"  ,  "75.68" , "32.43" ,  "64.87", "0", "0", "0"}
  };
  
  
// int change_pressures = 0;
// int change_pressures_and_prime = 1;

int flush_both_inputs = 0;
int flush_ethanol = 1;
int prime_both_inputs = 2;
int prime_ethanol = 3;
int change_pressures = 4;
//____________________________Variables for talking to the sippers__________________________________________//

const int communication_input_1 = 26;
const int communication_output_1 = 27;

const int communication_input_2 = 28;
const int communication_output_2 = 29;

bool finished_one_half_1 = false;
bool finished_one_half_2 = false;
int stable_count_1 = 0;
int stable_count_2 = 0;
const int stable_threshold = 20;
bool finished_bot_1 = false;
bool finished_bot_2 = false;

int move_both_sippers = 0;
int move_ethanol_sipper = 1;
int move_aequous_sipper = 2;

//_____________________________Variables for taking data from the Regs______________________________________//
// Generalized to 9 regulators with configurable prefixes, pins, and max PSI
const char *regulatorPrefixes[NUM_REGULATORS] = {"AS","BS","CS","DS","ES","FS","GS","HS","IS"};

// NOTE: Verify and update the last three analog pins to match wiring
int regulatorAnalogPins[NUM_REGULATORS] = { A11, A14, A12, A10, A15, A13, A3, A4, A5 };
float regulatorMaxPsi[NUM_REGULATORS]   = { 100, 100, 150, 100, 100, 100, 50, 50, 50 };

float pressure_tolerance = 7;

float current_pressures_floats[NUM_REGULATORS] = {0};
const char* current_pressures_strings[NUM_REGULATORS] = {"0","0","0","0","0","0","0","0","0"};

char PressureMsg[NUM_REGULATORS][50];

//_____________________________________Z Motor and stuff Things_____________________________________________//

const int DIR_PIN_z1 = 7;
const int STEP_PIN_z1 = 8;
const int DIR_PIN_z2 = 10;
const int STEP_PIN_z2 = 9;

const int z_steppers_up_mem = 80;  //(400 optimal for plate reader well) (200(?) for deep 96) CASSETTE HEIGHTS
const int z_steppers_up_cas = 80;
const int z_steppers_needles_just_above = 2;
const int z_steppers_down = 2;
bool move_down_flag = false;

//__________________________________________encoder stuff___________________________________________________//

#define ENCODER_A1 21  // Must be an interrupt-capable pin (2)
#define ENCODER_B1 20  // Must be an interrupt-capable pin (3)

volatile long encoderPositionx = 0;
volatile long global_target_encoderPositionx = 0;
volatile int lastEncodedx = 0;


#define ENCODER_A2 3  // Must be an interrupt-capable pin 2
#define ENCODER_B2 2  // Must be an interrupt-capable pin 3

volatile long encoderPositiony = 0;
volatile long global_target_encoderPositiony = 0;
volatile int lastEncodedy = 0;

//_______________________________________x/y stepper variables_____________________________________________//

int MS1_y = 4;
int MS2_y = 5;
int MS3_y = 6;

int MS1_x = 47;
int MS2_x = 46;
int MS3_x = 45;

const int DIR_PIN_y = 11;
const int STEP_PIN_y = 12;
const int DIR_PIN_x = 14;
const int STEP_PIN_x = 15;
const int x_encoder_counts_per_step = 2825; //2835 previously
const int y_encoder_counts_per_step = 2*x_encoder_counts_per_step;
int x_forward = 0;
int x_back = 1;
int x_direction;
int x_counter = 0;
int y_direction = 0;

int center_x = 1480; //2165, 1391
int center_y = 4700; //4761, 4650
int center_offset = 0;

const int rising_edge_delay = 30;
int x_delay = 1500;
int y_delay = 1500;
int z_delay = 1500;

//_____________________________________General workflow variables____________________________________________//

const int buttonPin = 13;
int buttonState = 0;
int moveFlag = 0;
int formulation_counter = -1;
int collection_counter = 0;
int current_formulation_collection_counter = 0;
int num_collection_events_on_this_plate = 0;
bool kill_switch = false;
bool equilibrate = true;
int collections_to_be_used = 0;

char buffer[100];  // Ensure the buffer is large enough
const char *prefixes_down[] = {"AS", "DS", "BS", "ES","CS","FS"};
const char *prefixes_up[] = {"BS", "ES","CS","FS","AS", "DS"};
size_t numPrefixes = 6;
int up = 1;
int down = 0;

int collections_per_plate = 0;

//_____________________________________Grid Pattern Configuration________________________________________//
const int GRID_COLS = 4;        // Number of columns in collection grid
const int GRID_ROWS = 2;        // Number of rows in collection grid
const int MAX_MOVEMENTS_PER_PLATE = 11;  // Maximum movements beyond the first waste well position
int num_movements = 0;

//_________________________________________collect/purge times_______________________________________________//

const long purge_time = 5000;
const long pressure_change_delay = 10;
// const long collect_time = 2500;
const long collect_time = 2000;
const long prime_variables_time = 5000;
const long prime_all_time = 1000;
const long stabilize_time = 2600;
const long flush_lines_time = 3000;

// const long purge_time = 1000;
// const long pressure_change_delay = 10;
// // const long collect_time = 2500;
// const long collect_time = 1000;
// const long prime_variables_time = 1000;
// const long prime_all_time = 1000;
// const long stabilize_time = 1000;
// const long flush_lines_time = 1000;
// // const long flush_lines_time = 5000;

//______________________________________________routines_____________________________________________________//


int previous_location = 0;
int location;
void plateMoveFunc(int end_location) {                       //move plate up and down
  if (end_location == 0){
    location = z_steppers_down;
  }
  else if (end_location == 1){
    location = z_steppers_up_mem;
  }
  else if(end_location == 2){
    location = z_steppers_up_cas;
  }
  else{
    location = z_steppers_needles_just_above;
  }
  if (location < previous_location){
    digitalWrite(DIR_PIN_z1,HIGH);
    digitalWrite(DIR_PIN_z2,HIGH);
    for (int z=0;z<abs(location-previous_location);z++){
      // digitalWrite(STEP_PIN_z1,HIGH);
      // digitalWrite(STEP_PIN_z2,HIGH);
      PORTH |= (1 << 5) | (1 << 6);
      delayMicroseconds(rising_edge_delay);
      // digitalWrite(STEP_PIN_z1,LOW);
      // digitalWrite(STEP_PIN_z2,LOW);
      PORTH &= ~((1 << 5) | (1 << 6));
      delayMicroseconds(z_delay);
    }
  }
  else{
    digitalWrite(DIR_PIN_z1,LOW);
    digitalWrite(DIR_PIN_z2,LOW);
    for (int z=0;z<abs(location-previous_location);z++){
      // digitalWrite(STEP_PIN_z1,HIGH);
      // digitalWrite(STEP_PIN_z2,HIGH);
      PORTH |= (1 << 5) | (1 << 6);
      delayMicroseconds(rising_edge_delay);
      // digitalWrite(STEP_PIN_z1,LOW);
      // digitalWrite(STEP_PIN_z2,LOW);
      PORTH &= ~((1 << 5) | (1 << 6));
      delayMicroseconds(z_delay);
    }
  }
  previous_location = location;
}

// volatile uint8_t  encodedx, sumx;
// const int lookupTable[16] = {0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0};
volatile uint8_t  A_x,B_x,lastEncodex,encodedx, sumx;
void updateEncoderx() {
  // Direct pin access for ENCODER_A1 (Pin 21) and ENCODER_B1 (Pin 20)
  // For Pin 21 (PORTF5) and Pin 20 (PORTF4)
  A_x = (PIND & (1 << PD0)) ? HIGH : LOW;  // Read pin D0 (PORTD0) directly
  B_x = (PIND & (1 << PD1)) ? HIGH : LOW;  // Read pin D0 (PORTD1) directly

  encodedx = (A_x << 1) | B_x;
  sumx = (lastEncodedx << 2) | encodedx;

  if (sumx == 0b0001 || sumx == 0b0111 || sumx == 0b1110 || sumx == 0b1000) {
      encoderPositionx++;
  } else if (sumx == 0b0010 || sumx == 0b1101 || sumx == 0b1011 || sumx == 0b0100) {
      encoderPositionx--;
  }
  lastEncodedx = encodedx;
}

// void updateEncoderx() {
//   // Direct pin access for ENCODER_A1 (Pin 21) and ENCODER_B1 (Pin 20)
//   sumx = (lastEncodedx << 2) | (PIND >> PD0 & 0b11);
//   encoderPositionx += lookupTable[sumx];
//   lastEncodedx = encodedx;
// }

volatile uint8_t  A_y,B_y,lastEncodey,encodedy, sumy;

void updateEncodery() {
  // Direct pin access for ENCODER_A2 (Pin 2) and ENCODER_B2 (Pin 3)
  // For Pin 2 (PORTD2) and Pin 3 (PORTD3)
  A_y = (PINE & (1 << PINE4)) ? HIGH : LOW;  // Read pin 2 (PORTE4) directly
  B_y = (PINE & (1 << PINE5)) ? HIGH : LOW;  // Read pin 3 (PORTE5) directly
  
  encodedy = (A_y << 1) | B_y;
  sumy = (lastEncodedy << 2) | encodedy;

  if (sumy == 0b0001 || sumy == 0b0111 || sumy == 0b1110 || sumy == 0b1000) {
      encoderPositiony++;
  } else if (sumy == 0b0010 || sumy == 0b1101 || sumy == 0b1011 || sumy == 0b0100) {
      encoderPositiony--;
  }
  lastEncodedy = encodedy;
}

// void updateEncodery() {
//   // Direct pin access for ENCODER_A2 (Pin 2) and ENCODER_B2 (Pin 3)
//   sumy = (lastEncodedy << 2) | (PINE >> PINE4 & 0b11);
//   encoderPositiony += lookupTable[sumy];
//   lastEncodedy = encodedy;
// }

int current_microstep_setting_x = 0;
float steps_per_rotation_x = 200;
float encoder_counts_per_step_x=20;

void change_x_step(int microstep_size){
  if(microstep_size==0){ //full step
    digitalWrite(MS1_x,LOW);
    digitalWrite(MS2_x,LOW);
    digitalWrite(MS3_x,LOW);
    x_delay = 1500;
    steps_per_rotation_x = 200;
    encoder_counts_per_step_x=20;
    current_microstep_setting_x = 1;
  }
  else if(microstep_size ==1){ //half step
    digitalWrite(MS1_x,HIGH);
    digitalWrite(MS2_x,LOW);
    digitalWrite(MS3_x,LOW);
    x_delay = 1500;
    steps_per_rotation_x = 400;
    encoder_counts_per_step_x=10;
    current_microstep_setting_x = 2;
  }
  else if(microstep_size ==2){ //quarter step
    digitalWrite(MS1_x,LOW);
    digitalWrite(MS2_x,HIGH);
    digitalWrite(MS3_x,LOW);
    x_delay = 1500;
    steps_per_rotation_x = 800;
    encoder_counts_per_step_x=5;
    current_microstep_setting_x = 3;
  }
}

void move_x_stepper(long number_of_encoder_counts, int dir){
  change_x_step(0);
  delay(10);
  // while(true){
  //   // Serial.println("x_position");
  //   Serial.println(encoderPositionx);
  // }
  // Serial.println("----------in------------");
  unsigned long previous_encoder_value;
  if (dir == 0){
    global_target_encoderPositionx = global_target_encoderPositionx - number_of_encoder_counts;
    while (abs(global_target_encoderPositionx - encoderPositionx)>13){
      int fudge_steps = (int)((((float)(encoderPositionx-global_target_encoderPositionx))/4000.0)*steps_per_rotation_x);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_x,LOW);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(x_delay);
          if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
            break;
          }
        }
      }
      else{
        digitalWrite(DIR_PIN_x,HIGH);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(x_delay);
          if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
            break;
          }
        }
      }
      if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
        break;
      }
      while((abs(global_target_encoderPositionx - encoderPositionx)<(encoder_counts_per_step_x*5))&&(current_microstep_setting_x<3)){
        change_x_step(current_microstep_setting_x);
      }
    }
  }
  else{
    global_target_encoderPositionx = global_target_encoderPositionx + number_of_encoder_counts;
    while (abs(global_target_encoderPositionx - encoderPositionx)>13){
      int fudge_steps = (int)((((float)(encoderPositionx-global_target_encoderPositionx))/4000.0)*steps_per_rotation_x);
      // Serial.println("x_position");
      // Serial.println(encoderPositionx);
      // if (abs(encoderPositionx-previous_encoder_value)<5){
      //   fudge_steps = 100;
      // }
      // previous_encoder_value = encoderPositionx;
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_x,LOW);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(x_delay);
          if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
            break;
          }
        }
      }
      else{
        digitalWrite(DIR_PIN_x,HIGH);
        for (int x=0;x<abs(fudge_steps);x++){
          digitalWrite(STEP_PIN_x,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_x,LOW);
          delayMicroseconds(x_delay);
          if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
            break;
          }
        }
      }
      if(abs(global_target_encoderPositionx - encoderPositionx)<=13){
        break;
      }
      while((abs(global_target_encoderPositionx - encoderPositionx)<(encoder_counts_per_step_x*5))&&(current_microstep_setting_x<3)){
        change_x_step(current_microstep_setting_x);
      }
    }
  }
}

int current_microstep_setting_y = 0;
float steps_per_rotation_y = 200;
float encoder_counts_per_step_y=20;

void change_y_step(int microstep_size){
  if(microstep_size==0){
    digitalWrite(MS1_y,LOW);
    digitalWrite(MS2_y,LOW);
    digitalWrite(MS3_y,LOW);
    y_delay = 1000;
    steps_per_rotation_y = 200;
    encoder_counts_per_step_y=20;
    current_microstep_setting_y = 1;
  }
  else if(microstep_size ==1){
    digitalWrite(MS1_y,HIGH);
    digitalWrite(MS2_y,LOW);
    digitalWrite(MS3_y,LOW);
    y_delay = 1000;
    steps_per_rotation_y = 400;
    encoder_counts_per_step_y=10;
    current_microstep_setting_y = 2;
  }
  else if(microstep_size ==2){
    digitalWrite(MS1_y,LOW);
    digitalWrite(MS2_y,HIGH);
    digitalWrite(MS3_y,LOW);
    y_delay = 1000;
    steps_per_rotation_y = 800;
    encoder_counts_per_step_y=5;
    current_microstep_setting_y = 3;
  }
}

void move_y_stepper(int number_of_encoder_counts, int dir){
  change_y_step(0);
  unsigned long previous_encoder_value;
  delay(10);
  if (dir == 0){
    global_target_encoderPositiony = global_target_encoderPositiony - number_of_encoder_counts;
    while (abs(global_target_encoderPositiony - encoderPositiony)>13){
      int fudge_steps = (int)((((float)(encoderPositiony-global_target_encoderPositiony))/4000.0)*steps_per_rotation_y);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_y,LOW);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(y_delay);
          if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
            break;
          }
        }
      }
      else{
        digitalWrite(DIR_PIN_y,HIGH);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(y_delay);
          if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
            break;
          }
        }
      }
      if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
        break;
      }
      while((abs(global_target_encoderPositiony - encoderPositiony)<(encoder_counts_per_step_y*5))&&(current_microstep_setting_y<3)){
        change_y_step(current_microstep_setting_y);
      }
    }
  }
  else{
    global_target_encoderPositiony = global_target_encoderPositiony + number_of_encoder_counts;
    while (abs(global_target_encoderPositiony- encoderPositiony)>13){
      int fudge_steps = (int)((((float)(encoderPositiony-global_target_encoderPositiony))/4000.0)*steps_per_rotation_y);
      if (fudge_steps < 0){
        digitalWrite(DIR_PIN_y,LOW);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(y_delay);
          if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
            break;
          }
        }
      }
      else{
        digitalWrite(DIR_PIN_y,HIGH);
        for (int y=0;y<abs(fudge_steps);y++){
          digitalWrite(STEP_PIN_y,HIGH);
          delayMicroseconds(rising_edge_delay);
          digitalWrite(STEP_PIN_y,LOW);
          delayMicroseconds(y_delay);
          if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
            break;
          }
        }
      }
      if(abs(global_target_encoderPositiony - encoderPositiony)<=13){
        break;
      }
      while((abs(global_target_encoderPositiony - encoderPositiony)<(encoder_counts_per_step_y*5))&&(current_microstep_setting_y<3)){
        change_y_step(current_microstep_setting_y);
      }
    }
  }
}

void formatPressureValue(char* buffer, const char* prefix, const char* value) {         //make pressures in a format that can be read
  buffer[0] = '\0';
  strcat(buffer, prefix);
  strcat(buffer, value);
  strcat(buffer, "\r");
}

void readCurrentPressures(float* outArray) {
  for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
    outArray[i] = analogRead(regulatorAnalogPins[i]) * (5.0 / 1023.0) * (regulatorMaxPsi[i]/5.0);
  }
}

void wait_for_equilibrium_all(const float* targets, const char* const* messages) {
  float current[NUM_REGULATORS];
  unsigned long entered_time = millis();
  while (!kill_switch) {
    readCurrentPressures(current);
    bool all_within = true;
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
      float tol = pressure_tolerance;
      if (i == 3) { // A1 had +1 tolerance in original code
        tol = pressure_tolerance + 1;
      }
      if (abs(targets[i] - current[i]) > tol) {
        all_within = false;
      }
    }
    if (all_within) {
      break;
    }
    if (digitalRead(buttonPin) == HIGH) {
      kill_switch = true;
      setPressures("0","0","0","0","0","0","0","0","0", 0);
      break;
    }
    if (abs(millis() - entered_time) > 1000) {
      for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
        float tol = pressure_tolerance;
        if (i == 3) tol = pressure_tolerance + 1;
        if (abs(targets[i] - current[i]) > tol) {
          formatPressureValue(PressureMsg[i], regulatorPrefixes[i], messages[i]);
          mySerial.print(PressureMsg[i]);
        }
      }
      entered_time = millis();
    }
  }
}
 
void setPressures(const char* V0,const char* V1,const char* V2,const char* V3,const char* V4,const char* V5,const char* V6,const char* V7,const char* V8, int sipper_dir){               //send pressures to the regulators (sipper_dir used to identify if the sipper is increasing (1) in pressure or decreasing to zero (0) or decreasing in pressure not to 0 (2))
  unsigned long starting_time = millis();
  const char* values[NUM_REGULATORS] = { V0,V1,V2,V3,V4,V5,V6,V7,V8 };
  float targets[NUM_REGULATORS];
  for (int i = 0; i < NUM_REGULATORS; i++) targets[i] = atof(values[i]);
  if (sipper_dir == 0){
    if (formulation_counter>(-1)){
      // gentle ramp down: 16 -> 8 -> 4 -> 2
      const char* ramp_vals[] = { "16","8","4","2" };
      float targets_ramp[NUM_REGULATORS];
      const char* msgs[NUM_REGULATORS];
      for (int r = 0; r < 4; r++) {
        for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
          formatPressureValue(PressureMsg[i], regulatorPrefixes[i], ramp_vals[r]);
          mySerial.print(PressureMsg[i]);
          targets_ramp[i] = atof(ramp_vals[r]);
          msgs[i] = ramp_vals[r];
        }
        wait_for_equilibrium_all(targets_ramp, msgs);
      }
    }   
    // final to zero
    float targets_zero[NUM_REGULATORS];
    const char* msgs_zero[NUM_REGULATORS];
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
      formatPressureValue(PressureMsg[i], regulatorPrefixes[i], "0");
      mySerial.print(PressureMsg[i]);
      targets_zero[i] = 0.0;
      msgs_zero[i] = "0";
    }
    wait_for_equilibrium_all(targets_zero, msgs_zero);
  }
  else if (sipper_dir == 1){
    // ramp up 2 -> 4 -> 8, then optionally 16 if any target > 16
    const char* ramp_up_vals[] = { "2","4","8" };
    float targets_ramp[NUM_REGULATORS];
    const char* msgs[NUM_REGULATORS];
    for (int r = 0; r < 3; r++) {
      for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
        formatPressureValue(PressureMsg[i], regulatorPrefixes[i], ramp_up_vals[r]);
        mySerial.print(PressureMsg[i]);
        targets_ramp[i] = atof(ramp_up_vals[r]);
        msgs[i] = ramp_up_vals[r];
      }
      wait_for_equilibrium_all(targets_ramp, msgs);
    }
    bool need16 = false;
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) if (targets[i] > 16.0) { need16 = true; break; }
    if (need16) {
      for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
        formatPressureValue(PressureMsg[i], regulatorPrefixes[i], "16");
        mySerial.print(PressureMsg[i]);
        targets_ramp[i] = 16.0;
        msgs[i] = "16";
      }
      wait_for_equilibrium_all(targets_ramp, msgs);
    }
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
      formatPressureValue(PressureMsg[i], regulatorPrefixes[i], values[i]);
      mySerial.print(PressureMsg[i]);
    }
    wait_for_equilibrium_all(targets, values);
  }
  else if (sipper_dir == 2){
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
      formatPressureValue(PressureMsg[i], regulatorPrefixes[i], values[i]);
      mySerial.print(PressureMsg[i]);
    }
    delay(50);
    wait_for_equilibrium_all(targets, values);
  }
  else{
    for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
      formatPressureValue(PressureMsg[i], regulatorPrefixes[i], values[i]);
      mySerial.print(PressureMsg[i]);
    }
    wait_for_equilibrium_all(targets, values);
  }
  for (int i = 0; i < NUM_ACTIVE_REGULATORS; i++) {
    current_pressures_floats[i] = targets[i];
    current_pressures_strings[i] = values[i];
  }
}

void waitForTime(unsigned long duration) {
  unsigned long start = millis();
  while (((millis() - start) < duration) && !kill_switch) {
    // current_pressure_E1 = analogRead(E1_reg_pin) * (5.0 / 1023.0)*(E1_max_PSI/5.0);
    // current_pressure_E2 = analogRead(E2_reg_pin) * (5.0 / 1023.0)*(E2_max_PSI/5.0);
    // current_pressure_E3 = analogRead(E3_reg_pin) * (5.0 / 1023.0)*(E3_max_PSI/5.0);
    // current_pressure_A1 = analogRead(A1_reg_pin) * (5.0 / 1023.0)*(A1_max_PSI/5.0);
    // current_pressure_A2 = analogRead(A2_reg_pin) * (5.0 / 1023.0)*(A2_max_PSI/5.0);
    // current_pressure_A3 = analogRead(A3_reg_pin) * (5.0 / 1023.0)*(A3_max_PSI/5.0);
    if (digitalRead(buttonPin) == HIGH) {
      kill_switch = true;
      setPressures("0", "0", "0", "0", "0", "0", "0", "0", "0", 0);
    }
  }
}

void equilibrize(int equilibration_type) {                       //function that makes the pressures set in waste wells
  if (equilibration_type == 0 || equilibration_type == 1) {
    int index = (equilibration_type == 0) ? 0 : 1;
    setPressures(arr[index][0], arr[index][1], arr[index][2], arr[index][3], arr[index][4], arr[index][5], arr[index][6], arr[index][7], arr[index][8], 1);
    waitForTime(purge_time);
  } 
  //SET FIRST TWO PRESSURES TO BE PRIME PRESSURES, THEN THIS WILL BE FORMULATION MAKING PRESSURES
  else if(equilibration_type==4){
    int second_index = 2 + formulation_counter;
    setPressures(arr[second_index][0], arr[second_index][1], arr[second_index][2], arr[second_index][3], arr[second_index][4], arr[second_index][5], arr[second_index][6], arr[second_index][7], arr[second_index][8], 3);
    waitForTime(stabilize_time);
  }
  else {
    int first_index = (equilibration_type == 2) ? 0 : 1; // ? is a conditional operator, if eq type is 2, then first array row (prime both inputs), if not, then second array row (prime ethanol) 
    int second_index = 2 + formulation_counter;
    setPressures(arr[first_index][0], arr[first_index][1], arr[first_index][2], arr[first_index][3], arr[first_index][4], arr[first_index][5], arr[first_index][6], arr[first_index][7], arr[first_index][8], 1);
    waitForTime(prime_variables_time);
    setPressures("80","80","80","80","80","80","80","80","80",2);
    waitForTime(prime_all_time); 
    setPressures(arr[second_index][0], arr[second_index][1], arr[second_index][2], arr[second_index][3], arr[second_index][4], arr[second_index][5], arr[second_index][6], arr[second_index][7], arr[second_index][8], 2);
    waitForTime(stabilize_time);
  }
}

void flush_lines(){                 //code used to prime lines at the beginning of operation
  setPressures(arr[2][0],arr[2][1],arr[2][2],arr[2][3],arr[2][4],arr[2][5],arr[2][6],arr[2][7],arr[2][8],1);
  waitForTime(flush_lines_time);
  setPressures("0", "0", "0", "0", "0", "0", "0", "0", "0", 0);
}

void controlSippers(bool &finished_one_half, bool &finished_bot, int &stable_count,
  int communication_input, int communication_output) {
  if (!finished_one_half) {
    if (digitalRead(communication_input) == LOW) {
      Serial.println("waiting to write low");
      Serial.println(digitalRead(communication_input));
      stable_count++;
      if (stable_count >= stable_threshold) {
        digitalWrite(communication_output, LOW);
        Serial.println("writing low");
        finished_one_half = true;
        stable_count = 0;
      }
    } else {
      stable_count = 0;  // reset if not stable
    }
  } 
  else {
    if (digitalRead(communication_input) == HIGH) {
      Serial.println("waiting to be done");
      Serial.println(digitalRead(communication_input));
      stable_count++;
      if (stable_count >= stable_threshold) {
        finished_bot = true;
        stable_count = 0;
        Serial.println("done");
      }
    } else {
      stable_count = 0;
    }
  }
}

// void controlSippers(bool &finished_one_third, bool &finished_two_third, bool &finished_bot, 
//   int communication_input, int communication_output) {
//   if (!finished_bot) {
//     if (!finished_one_third && (digitalRead(communication_input) == LOW)) {
//       digitalWrite(communication_output, LOW);
//       finished_one_third = true;
//     } 
//     else if (finished_one_third && !finished_two_third && (digitalRead(communication_input) == HIGH)) {
//       finished_two_third = true;
//     } 
//     else if (finished_two_third && (digitalRead(communication_input) == LOW)) {
//       finished_bot = true;
//     }
//   }
// }

void move_sippers(int sippers_to_move){
  setPressures("0", "0", "0", "0", "0", "0", "0", "0", "0", 2);
  finished_one_half_1 = finished_one_half_2 = false;
  finished_bot_1 = finished_bot_2 = false;

  if (sippers_to_move == 0) {
    digitalWrite(communication_output_1, HIGH);
    digitalWrite(communication_output_2, HIGH);
    while (!finished_bot_1 || !finished_bot_2) {
      controlSippers(finished_one_half_1, finished_bot_1, stable_count_1, communication_input_1, communication_output_1);
      controlSippers(finished_one_half_2, finished_bot_2, stable_count_2, communication_input_2, communication_output_2);
    }
  } 
  else if (sippers_to_move == 1) {
    digitalWrite(communication_output_1, HIGH);
    while (!finished_bot_1) {
      controlSippers(finished_one_half_1, finished_bot_1, stable_count_1, communication_input_1, communication_output_1);
    }
  } 
  else {
    digitalWrite(communication_output_2, HIGH);
    while (!finished_bot_2) {
      controlSippers(finished_one_half_2, finished_bot_2, stable_count_2, communication_input_2, communication_output_2);
    }
  }
}

// void move_sippers(int sippers_to_move){
//   setPressures("0", "0", "0", "0", "0", "0", 0);
//   digitalWrite(communication_output_1, HIGH);
//   digitalWrite(communication_output_2, HIGH);

//   finished_one_third_1 = finished_one_third_2 = false;
//   finished_two_third_1 = finished_two_third_2 = false;
//   finished_bot_1 = finished_bot_2 = false;

//   if (sippers_to_move == 0) {
//     while (!finished_bot_1 || !finished_bot_2) {
//       controlSippers(finished_one_third_1, finished_two_third_1, finished_bot_1, communication_input_1, communication_output_1);
//       controlSippers(finished_one_third_2, finished_two_third_2, finished_bot_2, communication_input_2, communication_output_2);
//     }
//   } 
//   else if (sippers_to_move == 1) {
//     // while (!finished_bot_1) {
//     Serial2.print("s");
//     while (true) {
//       if (Serial2.available()) {
//         char c = Serial2.read();
//         if (c == 'o') {
//           break; // exit loop once 'g' is received
//         }
//       }
//     }
//     // }
//   } 
//   else {
//     while (!finished_bot_2) {
//       controlSippers(finished_one_third_2, finished_two_third_2, finished_bot_2, communication_input_2, communication_output_2);
//     }
//   }
// }

void center(){
  move_x_stepper(center_x-center_offset, 0); // Move to start (88) //1600
  move_y_stepper(center_y-center_offset, 1); // Move to start //3440
}

void homeSteppers(){
  Serial.println("here in home steppers");
  Serial.println(global_target_encoderPositionx);
  Serial.println(global_target_encoderPositiony);
  if (global_target_encoderPositiony>(-100)){
    move_y_stepper(global_target_encoderPositiony+100, 0); // Move to start (88) //1600
  }
  else{
    move_y_stepper(abs(global_target_encoderPositiony)-100, 1); // Move to start (88) //1600
  }
  if (global_target_encoderPositionx>(-100)){
    move_x_stepper(global_target_encoderPositionx+100, 0); // Move to start (88) //1600
  }
  else{
    move_x_stepper(abs(global_target_encoderPositionx)-100, 1); // Move to start (88) //1600
  }
}

void receiveFormulationData() {
  if (Serial2.available()) {
    static String receivedData = "";
    static int rowIndex = 2;  // Start at index 2 (after prime rows)
    char receivedChar = Serial2.read();
    
    if (receivedChar == 'f') {  // End of row marker
      // Parse receivedData into individual floats and store in arr
      int colIndex = 0;
      int startIndex = 0;
      for (int i = 0; i < receivedData.length(); i++) {
        if (receivedData[i] == ',') {
          float value = receivedData.substring(startIndex, i).toFloat();
          // Convert float to string and store in arr
          dtostrf(value, 4, 2, arr[rowIndex][colIndex]);
          colIndex++;
          startIndex = i + 1;
        }
      }
      // Last element in the row
      float value = receivedData.substring(startIndex).toFloat();
      dtostrf(value, 4, 2, arr[rowIndex][colIndex]);
      
      rowIndex++;
      receivedData = "";  // Reset for next row
      
      if (rowIndex > (NUMBER_OF_FORMULATIONS + 1)) {
        // All 4 formulations received (indices 2-5)
        rowIndex = 2;  // Reset for next batch
        Serial.println("Formulation data received from ESP32");
      }
    } else if (receivedChar == 't') {
      // Termination signal
      kill_switch = true;
      setPressures("0","0","0","0","0","0","0","0","0", 0);
    } else {
      receivedData += receivedChar;  // Accumulate received characters
    }
  }
}

void setup() {
  Serial.begin(ArdBaud);     // Arduino Serial monitor interface
  Serial2.begin(115200);     // ESP32 communication (115200 baud to match ESP32)
  mySerial.begin(AlctBaud);  // Alicat Serial interface (make sure baud rate matches Alicat setting)
  while (!Serial);

  pinMode(ENCODER_A1, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), updateEncoderx, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), updateEncoderx, CHANGE);
  pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A2), updateEncodery, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B2), updateEncodery, CHANGE);

  pinMode(communication_input_1,INPUT);
  pinMode(communication_output_1,OUTPUT);
  pinMode(communication_input_2,INPUT);
  pinMode(communication_output_2,OUTPUT);
  digitalWrite(communication_output_1,LOW);
  digitalWrite(communication_output_2,LOW);
  setPressures("0","0","0","0","0","0","0","0","0",2); //kill pressures
  formulation_counter = 0;

  pinMode(buttonPin, INPUT); //turn on all pins
  pinMode(DIR_PIN_x,OUTPUT);
  pinMode(STEP_PIN_x,OUTPUT);
  pinMode(DIR_PIN_y,OUTPUT);
  pinMode(STEP_PIN_y,OUTPUT);
  pinMode(STEP_PIN_z1, OUTPUT);
  pinMode(DIR_PIN_z1, OUTPUT);
  pinMode(STEP_PIN_z2, OUTPUT);
  pinMode(DIR_PIN_z2, OUTPUT);
  pinMode(MS1_x,OUTPUT);
  pinMode(MS2_x,OUTPUT);
  pinMode(MS3_x,OUTPUT);
  change_x_step(0);
  pinMode(MS1_y,OUTPUT);
  pinMode(MS2_y,OUTPUT);
  pinMode(MS3_y,OUTPUT);
  change_y_step(0);
  digitalWrite(STEP_PIN_z1, HIGH);
  digitalWrite(STEP_PIN_z2, HIGH);
  //Serial.println("fuck_2");
  while(digitalRead(buttonPin)==LOW); //wait for button press
  encoderPositionx = 0;
  encoderPositiony = 0;
  Serial.println("in");
  
  // flush_lines(); // use this just to flush in place without moving anything
}

// Helper function to change formulation pressures (handles sipper movement if needed)
void changeFormulation() {
  if (formulation_counter == (NUMBER_OF_FORMULATIONS - 1)) {
    // Reset to first formulation and move sippers to change inputs
    formulation_counter = 0;
    move_sippers(move_both_sippers);
    equilibrize(flush_both_inputs);
    move_sippers(move_both_sippers);
    equilibrize(prime_both_inputs);
  } else {
    // Move to next formulation
    formulation_counter++;
    equilibrize(change_pressures);
  }
}

// Move to next physical position in the grid (only XY movement, no Z or pressure changes)
void moveToNextGridPosition() {
  if (num_movements >= MAX_MOVEMENTS_PER_PLATE) {
    return; // Don't move if we've hit the limit
  }
  
  // Original pattern: movements 1-4 (index 0-3): forward, movement 5: reverse & move y down
  // movements 6-10 (index 5-9): backward, movement 11: move y up
  if (num_movements == 5) {
    // End of first row: reverse direction and move to next row
    x_direction = x_back;
    move_y_stepper(y_encoder_counts_per_step, 0); // Move down
  } 
  else if (num_movements == 11) {
    // End of second row: move to next row (up in y)
    move_y_stepper(y_encoder_counts_per_step, 1); // Move up
  } 
  else {
    // Normal movement in current direction
    move_x_stepper(x_encoder_counts_per_step, x_direction);
  }
  
  num_movements++;
  Serial.print("Grid position #");
  Serial.println(num_movements);
}

// Handle a waste well sequence: move to position, set Z to membrane level, change formulation
void handleWasteWell() {
  moveToNextGridPosition();          // Move XY to next grid position
  plateMoveFunc(1);                  // Move Z to membrane level (waste well position)
  changeFormulation();                // Change to next formulation pressures
  plateMoveFunc(3);                   // Move Z to just above plate
}

// Handle a collection well sequence: move to position, collect sample
void handleCollectionWell() {
  moveToNextGridPosition();          // Move XY to next grid position
  plateMoveFunc(2);                   // Move Z to cassette level (collection position)
  waitForTime(collect_time);          // Collect for specified time
  plateMoveFunc(3);                   // Move Z to just above plate
  
  // Update counters
  collection_counter++;
  current_formulation_collection_counter++;
  num_collection_events_on_this_plate++;
}

void loop() {
  // Check for incoming formulation data from ESP32 GUI
  receiveFormulationData();
  
  const int TOTAL_COLLECTIONS = NUMBER_OF_LIPIDS * NUMBER_OF_FORMULATIONS * NUMBER_OF_COLLECTIONS;
  
  // Initialization sequence
  while(digitalRead(buttonPin) == LOW); // Wait for button press to start
  center();
  formulation_counter = 0;
  
  while(digitalRead(buttonPin) == LOW); // Wait for button press
  plateMoveFunc(1); // Move to membrane level (initial waste well position)
  while(digitalRead(buttonPin) == HIGH); // Wait for button release
  
  // Setup initial pressure settings for first waste well
  x_direction = x_forward;
  // flush_lines();
  // equilibrize(flush_both_inputs);
  // move_sippers(move_both_sippers);
  // equilibrize(prime_both_inputs);
  equilibrize(change_pressures);
  plateMoveFunc(3); // Move needles just above plate
  
  // Main collection loop: process all collections across all plates
  while(collection_counter < TOTAL_COLLECTIONS && !kill_switch) {
    // Calculate how many collections we can fit on this plate
    collections_per_plate = 4; // Can be changed to GRID_COLS * GRID_ROWS or other value
    int remaining_collections = TOTAL_COLLECTIONS - collection_counter;
    collections_to_be_used = (remaining_collections < collections_per_plate) ? 
                             remaining_collections : collections_per_plate;
    
    // Reset per-plate tracking variables
    num_collection_events_on_this_plate = 0;
    current_formulation_collection_counter = 0;
    num_movements = 0; // Reset for new plate (position 0 is already at waste well)
    x_direction = x_forward; // Reset direction for each plate
    
    // Loop through collection wells on this plate
    while(num_collection_events_on_this_plate < collections_to_be_used && 
          collection_counter < TOTAL_COLLECTIONS && 
          !kill_switch) {
      
      // Check for incoming formulation data from ESP32 GUI (non-blocking)
      receiveFormulationData();
      
      // Collect in a collection well
      handleCollectionWell();
      
      // Check if we've completed this plate or all collections
      if(num_collection_events_on_this_plate >= collections_to_be_used) break;
      if(collection_counter >= TOTAL_COLLECTIONS) break;
      
      // Check if we need to change formulation before next collection
      if(current_formulation_collection_counter >= NUMBER_OF_COLLECTIONS) {
        // Move to waste well and change formulation
        handleWasteWell();
        current_formulation_collection_counter = 0; // Reset counter for new formulation
        
        // Check exit conditions after waste well
        if(kill_switch) break;
        if(num_collection_events_on_this_plate >= collections_to_be_used) break;
      }
    }
    
    // Check for early exit conditions
    if(kill_switch || collection_counter >= TOTAL_COLLECTIONS) {
      break;
    }
    
    // Prepare for next plate: return home, wait for user, restart
    setPressures("0","0","0","0","0","0","0","0","0",0);
    plateMoveFunc(0); // Bring plate down to bottom
    homeSteppers();
    center_offset = 100; // Adjust center offset for new plate
    
    while(digitalRead(buttonPin) == LOW); // Wait for button press (plate change)
    center();
    
    while(digitalRead(buttonPin) == LOW); // Wait for button press (ready)
    plateMoveFunc(1); // Bring needles to membrane level (start at waste well)
    while(digitalRead(buttonPin) == HIGH); // Wait for button release
    
    // Change formulation if needed before starting new plate
    changeFormulation();
    plateMoveFunc(3); // Bring needles just above plate
  }
  
  // Cleanup: return to safe state
  formulation_counter = -1;
  setPressures("0","0","0","0","0","0","0","0","0",2);
  formulation_counter = 0;
  plateMoveFunc(0);
  homeSteppers();
  kill_switch = false;
  
  // End program - wait indefinitely
  while(true);
}
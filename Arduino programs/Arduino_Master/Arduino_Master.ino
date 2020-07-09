#include <string.h>
#include <SD.h>        // SD card
#include <DueTimer.h>  // Timer interrupt
#include "RTClib.h"    // real time clock
#include "HX711.h"     // weighting amplifier libirary
//#include <math.h>

/* Connection Circuit:
  Controller.SerialUSB <<======>> PC (data and/or debug info)
  Controller.Serial1   <<======>> Bpod.Serial1 (protocol and events data)
                                 || (trigger laser and maskLED)
                                 vv
  Controller.Serial2   <<======>> waveSurfer.Serial2 (powerWeight: 0-100%)
*/

/*********************************************************************************************************/
/***************************************** Head-Fixation related *****************************************/
/*********************************************************************************************************/

////////////// define finite states for head-fixation //////////////
#define CHECK_TRIG          1
#define DELAY_TO_FIX        2
#define HEAD_FIXATION       3
#define HEAD_FIXATION_DELAY 4
#define CHECK_RELEASE       5
#define ESCAPE_DELAY_1000   6
#define TIMEUP_DELAY_1000   7
#define STRUGGLE_DELAY_1000 8
uint8_t timer_state = CHECK_TRIG; // init state
float weight_tmp    = 0;
unsigned long last_state_time;
unsigned long last_weight_read_time;
////////////// define finite states used in timer interrupt routines //////////////


// define protocols for the main loop
#define P_FIXATION   10 // Fixation Protocol: learn to head-fixation up to 25 sec
#define P_SAMPLE     21 // Sample Protocol:   sample period 1.2 sec
#define P_DELAY      22 // Delay Protocol:    delay period 0.3 -> 1.3 sec
#define P_OPTOSTIM   23 // Optostim Protocol: start optogenetics (during sample or delay, 20% each)

bool protocol_sent = 0;

// port definition
const byte portMotorLR     = 2;  // analog write channel, Left-Right
const byte portMotorFB     = 3;  // analog write channel, Forward-Backward
const byte portMotorPole   = 4;  // analog write channel, Motorized Pole
const byte portSwitchL     = 22;
const byte portSwitchR     = 23;
const byte portWeight_DOUT = 7;  // weight stage data
const byte portWeight_CLK  = 6;  // weight stage clock
const byte switchPin       = 49; // ToggerSwitch pin to start/pause experiment
const byte resetPin        = 51; // PushBoutton pin to start a new mouse: press the button and reset the controller for a new mouse
const byte ledPin          = 13; // LED pin
const byte chipSelect      = 10; // Adafruit SD shields and modules: pin 10

typedef struct {
  int trig_counter                      = 0;    // Switch triggered counter
  int headfixation_counter              = 0;
  unsigned int fixation_duration        = 3000; // ms
  int last_advance_headfixation_counter = 0;
  byte Fixation_Outcome [20]            = {0};  // recent 20 headfixation performance: timeup:9 or escape:10, or struggle 12
  int struggle_thres_neg = -1;
  int struggle_thres_pos = 32;
  int last_advance_threshold_counter    = 0;
  int fixation_interval_max             = 30000;  // 30 sec
} Parameters_fixation;

Parameters_fixation F;
boolean para_F_changed = 0;

// Events_fixation:
typedef struct {
  int events_num                = 0;
  unsigned long events_time[20] = {};
  byte events_id[20]            = {};
  /* not used in this program
    // 1-LickL; 2-rewardL; 3-LickR; 4-rewardR;  5-motorFBadvance; 6-motorLRadvance
  */
  // 1-mark arduino restarted
  // 7-switchTriggered; 8-headfixation; 11-headfixation again immediately after release
  // 9-release1:timeup; 10-release2:escape; 12-release3:struggle
  int events_value[20]          = {0};
} Events_fixation;

Events_fixation Ev;

// weight stage related init
HX711 scale(portWeight_DOUT, portWeight_CLK);
float calibration_factor = -12400;
float weighting_info[40] = {0}; // weighting info in last 2 sec
int weight_counter       = 0;

boolean SwitchL_LastStatus;
boolean SwitchR_LastStatus;
boolean SwitchL_CurrentStatus;
boolean SwitchR_CurrentStatus;

int motor_retract_step               = 10; // 0-255
bool is_motor_advance                = 0;
unsigned int last_advance_reward_num = 0;
int lick_num_before_motor_retract1   = 15;
int lick_num_before_motor_retract2   = 25;
int lick_num_before_motor_retract3   = 35;

boolean headfixation_flag           = 0;
boolean last_headfixation_flag      = 0;
int trigger_num_before_fix          = 30;
int delay_after_fixation            = 0;
int fixation_advance_step           = 30;     // for every old:30 headfixations, advance the parameters
unsigned int fixation_interval_step = 2000;   // 2 sec
int fixation_interval_min           = 15000;  // 15 sec
int fixation_interval_thres1        = 3000;   // msec; >4 sec, max pressure...
bool is_max_pressure                = 0;
bool headfixation_again             = 0;

int regulatorVal_step    = 10;  // not used
int regulatorVal_min     = 150; // start with a small pressure
int regulatorVal_max     = 255; // max pressure
int regulatorVal_release = 30;  // slightly release to reduce retract distance and noise


/****************************************************************************************************/
/************************************* Finite State Matrix related **********************************/
/****************************************************************************************************/

// Event codes list.
const PROGMEM String EventNames[50] = {
  "Port1In", "Port1Out", "Port2In", "Port2Out", "Port3In", "Port3Out", "Port4In", "Port4Out", "Port5In", "Port5Out", "Port6In", "Port6Out", "Port7In", "Port7Out", "Port8In", "Port8Out",
  "BNC1High", "BNC1Low", "BNC2High", "BNC2Low",
  "Wire1High", "Wire1Low", "Wire2High", "Wire2Low", "Wire3High", "Wire3Low", "Wire4High", "Wire4Low",
  "SoftCode1", "SoftCode2", "SoftCode3", "SoftCode4", "SoftCode5", "SoftCode6", "SoftCode7", "SoftCode8", "SoftCode9", "SoftCode10",
  "UnUsed",
  "Tup",
  "GlobalTimer1_End", "GlobalTimer2_End", "GlobalTimer3_End", "GlobalTimer4_End", "GlobalTimer5_End",
  "GlobalCounter1_End", "GlobalCounter2_End", "GlobalCounter3_End", "GlobalCounter4_End", "GlobalCounter5_End"
};

// Output action name list.
const PROGMEM String OutputActionNames[17] = {
  "ValveState", "BNCState", "WireState",
  "Serial1Code", "SerialUSBCode", "SoftCode", "GlobalTimerTrig", "GlobalTimerCancel", "GlobalCounterReset",
  "PWM1", "PWM2", "PWM3", "PWM4", "PWM5", "PWM6", "PWM7", "PWM8"
};

// Meta action name list.
const PROGMEM String MetaActions[4] = {"Placeholder", "Valve", "LED", "LEDState"};

// Scalar for float timer.
const PROGMEM int TimerScaleFactor = 10000; // Bpod: 0.1 ms resolution

// Port and wire parameters for Bpod.
byte PortInputsEnabled[8] = {1, 1, 0, 0, 0, 0, 0, 0};
byte WireInputsEnabled[4] = {0, 0, 0, 0};

// Define output format.
typedef struct OutputActions {
  String OutputType;
  int Value;
} OutputAction;

// Define state transition format.
typedef struct StateChanges {
  String StateChangeTrigger;
  String StateChangeTarget;
} StateChange;

// Define state.
typedef struct States {
  String Name;
  float StateTimer;
  int nStateChangeConditions = 0;
  StateChange *StateChangeCondition;
  int nOutputs = 0;
  OutputAction *Output;
} State;

// Define state machine. Adapted from GenerateBlankStateMatrix.m
typedef struct StateMatrices {
  byte nStates = 0;
  // byte nStatesInManifest = 0;
  // String Manifest = {}; // State names in the order they were added by user
  String StateNames[128]                   = {"Placeholder"}; //State names in the order they were added
  byte InputMatrix[128][40]                = {};
  byte OutputMatrix[128][17]               = {};
  byte GlobalTimerMatrix[128][5]           = {};
  float GlobalTimers[5]                    = {};
  byte GlobalTimerSet[5]                   = {0, 0, 0, 0, 0}; //Changed to 1 when the timer is given a duration with SetGlobalTimer
  byte GlobalCounterMatrix[128][5]         = {};
  byte GlobalCounterEvents[5]              = {254, 254, 254, 254, 254}; //Default event of 254 is code for "no event attached".
  unsigned long GlobalCounterThresholds[5] = {0, 0, 0, 0, 0};
  int GlobalCounterSet[5]                  = {0, 0, 0, 0, 0}; //Changed to 1 when the counter event is identified and given a threshold with SetGlobalCounter
  float StateTimers[128]                   = {};
  byte StatesDefined[128]                  = {};              //Referenced states are set to 0. Defined states are set to 1. Both occur with AddState

} StateMatrix;

/* Bpod output ports FYI.
  byte PortPWMOutputLines[8] = {53, 8, 7, 6, 5, 4, 3, 2}; // "pwm1": gocue; "pwm2-8": digital
  byte WireDigitalOutputLines[4] = {43, 41, 39, 37};      // "WireState"
  byte ValveDigitalOutputLines[2] = {22, 23};             // "ValveState"
  byte BncOutputLines[2] = {24, 25};                      // "BNCState"
*/
OutputAction RewardOutput;
OutputAction LeftWaterOutput  = {"ValveState", 1};
OutputAction RightWaterOutput = {"ValveState", 2};

OutputAction CueOutput        = {"PWM1", 255};

OutputAction PoleOutput       = {"BNCState", 1};

OutputAction WaveSurferTrig    = {"PWM8", 255}; // Masking Flash LED trigger: DIO2 in Bpod
OutputAction OptogeneticTrig   = {"PWM7", 255}; // Optogenetics stim trigger: DIO3 in Bpod

String LeftLickAction;
String RightLickAction;


/****************************************************************************************************/
/***************************************** Trial related *****************************************/
/****************************************************************************************************/

#define    RECORD_TRIALS   100          // record recent 100 trials history in controller
const byte recent_trials = 30;          // Calculate performance for rencent 30 trials

// Defines ProtocolHistory struct.
typedef struct {
  byte Protocol        = 0;
  unsigned int nTrials = 0;
  byte performance     = 0;     // percentage: 0-100
} ProtocolHistoryInfo;

// Define  RewardFlag struct.
typedef struct {
  byte flag_R_water;
  byte flag_L_water;
  unsigned int past_trials;
} RewardFlag;

typedef struct {
  /////////////////////////////////////////////////
  unsigned int currentTrialNum = 0;          // current trial number
  int FB_motor_position        = 255;        // Lickport motor Forward/Backward
  byte FB_final_position       = 0;
  byte LR_motor_position       = 70;
  byte ProtocolType            = P_FIXATION;
  byte Autolearn               = 0;          // {"Either", "On", "antiBias", "random","off","fixed"}; 0-5
  byte TrialType               = 2;          // 0 right; 1 left; 2 either side
  int random_delay_duration    = 1001;       // ms
  byte Trial_Outcome           = 3;          // 0 error; 1 correct; 2 no response; 3 others
  /////////////////////////////////////////////////

  float SamplePeriod        = 1.20; //
  float DelayPeriod         = 0.30; // 0.3->1.3
  float TimeOut             = 0.50; // 0.5->4

  byte ProtocolHistoryIndex               = 0;  // index of ProtocolHistory
  ProtocolHistoryInfo ProtocolHistory[20] = {}; // [protocol#, n_trials on this protocol, performance]

  RewardFlag GaveFreeReward = {0, 0, 0}; // [flag_R_water  flag_L_water    past trials] 	// keeps track of the number of trials since the last reward

  // Protocol type, Trial type, OutcomeHistory for recent RECORD_TRIALS=100 trials
  byte ProtocolTypeHistory[RECORD_TRIALS] = {}; // 1-5
  byte TrialTypeHistory[RECORD_TRIALS]    = {}; // 0 right; 1 left;
  byte OutcomeHistory[RECORD_TRIALS]      = {}; // 0 error; 1 correct; 2 no response; 3 others

  byte struggle_enable = 1;
  unsigned int totoal_reward_num          = 1;
  byte retract_times = 0;
  /////////////////////////////////////////////////
} Parameters_behavior;

Parameters_behavior    S;

byte halfPos  = 65;
byte leftPos  = 30;
byte rightPos = 100;
byte finalPos = leftPos;


int LickPortMove         = 0;
const byte mov_step_size = 5;

float Reward_duration_behavior = 0.028; // sec
float AnswerPeriod_behavior    = 4.0;
byte MaxSame              = 3;
float NoTrialProb         = 0.5;
byte Min_correct_Right    = 1;
byte Max_incorrect_Right  = 3;
byte Min_correct_Left     = 1;
byte Max_incorrect_Left   = 3;
byte is_earlylick         = 2; // 0-no earlylick; 1-earlylick; 2-N/A 

float ConsumptionPeriod   = 0.75; //
float StopLickingPeriod   = 1.0; //


byte Perf100        = 0;
bool Timer3_running = 0;
int paused          = 0;
byte ledState       = LOW;
bool Receiving_data_from_Bpod  = 0;

unsigned long last_reward_time = 0;
int timed_drop_count = 0;

byte either_left_right = 2;

uint16_t nEvents;
unsigned long eventTimeStamps[1024] = {};
byte Events[1024]                   = {};
byte state_visited[1024] = {};
uint16_t nTransition;

byte trial_stim_index = 0; // 0:no_stim; 1:delay_stim; 2:sample_stim; etc.
byte weightByte = 100;

// others
RTC_PCF8523         rtc;
DateTime            now;
byte LowByte;
byte SecondByte;
byte ThirdByte;
byte ForthByte;


/****************************************************************************************************/
/********************************************** Setup() *********************************************/
/****************************************************************************************************/

void setup() {
  //delay(3000);

  SerialUSB.begin(115200);   // To PC for debug info and/or Data
  //while (!SerialUSB);

  Serial1.begin(115200);     // To Bpod
  while (Serial1.available()) {
    Serial1.read(); // clear serial1 dirty data
  }

  Serial2.begin(115200);     // To waveSurfer

  analogReadResolution(8);   // Max: 12
  analogWriteResolution(8);

  analogWrite(DAC0, regulatorVal_release);

  pinMode(ledPin, OUTPUT);
  digitalWriteDirect(ledPin, ledState);
  pinMode(switchPin, INPUT_PULLUP); // low if switch on; hight if switch off
  pinMode(resetPin, INPUT_PULLUP);  // low if button pressed; hight if boutton released

  pinMode(portSwitchL, INPUT_PULLUP);
  SwitchL_LastStatus = digitalReadDirect(portSwitchL);
  pinMode(portSwitchR, INPUT_PULLUP);
  SwitchR_LastStatus = digitalReadDirect(portSwitchR);

  pinMode(portMotorLR, OUTPUT);
  pinMode(portMotorFB, OUTPUT);
  pinMode(portMotorPole, OUTPUT);

  analogWrite(portMotorLR, 5);
  analogWrite(portMotorFB, 5);
  analogWrite(portMotorPole, 5);

  // Check if SD Card is working...
  if (!SD.begin(chipSelect)) {
    SerialUSB.println("E: SD Card failed, or not present");
    return; // don't do anything more:
  } else {
    SerialUSB.println("M: SD is working...");
  }

  // if(button_pressed) a new mouse! {write current (default) parameters S to SD card}
  // !!!not used in current version!!!
  if (digitalReadDirect(resetPin) == 0) { // Check if reset button is pressed
    SerialUSB.println("M: A new Mouse...");
    write_SD_para_F();
    write_SD_para_S();
  } else {                                // otherwise, read parameters from SD Card to override S
    read_SD_para_F();   // 4 ms
    read_SD_para_S();
  }

  //    // change parameters online...
  //    F.trig_counter  = 663; // Switch triggered counter
  //    F.headfixation_counter = 2037;
  //    F.fixation_duration = 30000; // ms
  //    F.last_advance_headfixation_counter = 0;
  //    for(int i = 0; i < 20; i++) {
  //    F.Fixation_Outcome [i] = 1;
  //    }
  //      F.struggle_thres_neg = -6;
  //      F.struggle_thres_pos = 36;
  //    F.last_advance_threshold_counter = 0;
  //    F.fixation_interval_max = 30000;
  //    write_SD_para_F();

  //  S.currentTrialNum         = 4074;       // current trial number
  //  S.FB_motor_position       = 100;        // Lickport motor Forward/Backward
  //  S.ProtocolType            = P_DELAY; // P_FIXATION P_SAMPLE P_DELAY P_OPTOSTIM
  //  S.Autolearn               = 2;          // {"Either", "On", "antiBias", "random","off","fixed"}; 0-5
  //  S.TrialType               = 2;
  //  S.random_delay_duration   = 1001;       // ms
  //  S.Trial_Outcome           = 3;          // 0 error; 1 correct; 2 no response; 3 others
  //  S.ProtocolHistoryIndex  = 13;          // 0:Fixation; 1-9:Sample; 10-18:Delay
  //  S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 500;
  //  S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 100;
  //  for (int i = RECORD_TRIALS - recent_trials; i < RECORD_TRIALS; i++) {
  //    S.OutcomeHistory[i] = 1;
  //  }
  //  S.struggle_enable = 0;
  //  S.totoal_reward_num = 1410;
  //  S.retract_times = 1;
  // S.SamplePeriod        = 1.20;
  //  S.DelayPeriod = 0.3;
  // S.TimeOut       = 3.0;
  //S.LR_motor_position = 70;
  //S.FB_final_position = 0;
  // write_SD_para_S();

  // Check if the RT Clock ready
  if (! rtc.begin()) {
    SerialUSB.println("E: Couldn't find RT Clock");
    return;
  }
  if (! rtc.initialized()) {
    SerialUSB.println("M: RTC is NOT running! Adjust Time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // print current time and date // 2 ms
  // printCurrentTime();

  // write an artificial event to mark the restart of Arduino board
  Ev.events_num = 0;
  Ev.events_id[Ev.events_num] = 1; // restart;
  now = rtc.now();
  Ev.events_time[Ev.events_num] = now.unixtime();
  Ev.events_value[Ev.events_num] = -1;
  Ev.events_num = 1;
  write_SD_event_fixation(); // write event to file
  Ev.events_num = 0;

  //  startMicros = micros();
  //  now = rtc.now();
  //  Ev.events_time[Ev.events_num] = now.unixtime(); // 950 us
  //  SerialUSB.println(micros()-startMicros);
  //
  //  startMicros = micros();
  //  Ev.events_time[Ev.events_num] = millis(); // 1us
  //  SerialUSB.println(micros()-startMicros);

  randomSeed(analogRead(0));

  analogWrite(DAC0, regulatorVal_min);
  delay(500);
  analogWrite(DAC0, regulatorVal_release);

  analogWrite(portMotorLR, S.LR_motor_position);
  delay(1000);
  analogWrite(portMotorFB, S.FB_motor_position);
  delay(1000);
  analogWrite(portMotorPole, finalPos);
  delay(1000);

  // initialize weighting scales
  scale.set_scale();
  scale.tare();  //Reset the scale to 0
  scale.set_scale(calibration_factor);

  // Handshake with Bpod
  int isHandshake = 0;
  while (!isHandshake) {
    Serial1.write('6'); // handshake with Bpod
    delay(100);
    while (!Serial1.available()) {
      SerialUSB.println("E: Trying to handshake with Bpod...");
      delay(5000);
      Serial1.write('6');
      delay(100);
    }
    if (Serial1.read() != '5') {
      // SerialUSB.println("E: handshake failed! Check.");
      while (Serial1.available()) {
        Serial1.read();
      }
    } else {
      SerialUSB.println("M: handshake successful.");
      ledState = HIGH;
      digitalWrite(ledPin, ledState);
      isHandshake = 1;
    }
  }

  Timer4.attachInterrupt(Incase_handler); // in case receiving Bpod data struck
  Timer4.setPeriod(5000000); // Runs  5 sec later to check if get struck
}



/****************************************************************************************************/
/********************************************** Loop() **********************************************/
/****************************************************************************************************/
void loop() {

  // Check if the toggle switch is ON
  if (digitalReadDirect(switchPin) == 0) { // if yes, run the state matrix

    if (paused == 1) {
      paused = 0;

      ledState = HIGH;
      digitalWriteDirect(ledPin, ledState);

      SerialUSB.println("M: Program RESUME!!!");

      // clear serial1 buffer
      while (Serial1.available()) {
        Serial1.read();
      }

      // in case SD card was removed and re-insert, need re-initilization
      SD.begin(chipSelect);

      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(100);
      valve_control(0); // close valve 1 and 2
    }

    if (protocol_sent == 0) {
      if (F.trig_counter <= trigger_num_before_fix || S.FB_motor_position > 0) {
        send_protocol_to_Bpod_and_Run(); //49 ms - 96 ms
      } else if (headfixation_flag == 1) { // need check if head is fixed
        send_protocol_to_Bpod_and_Run();
      }
    }

    // two-byte communication with PC for motor
    if (SerialUSB.available()) { // receiving data from PC
      byte CommandByte = SerialUSB.read();  // F for Motor F/B, L for Motor L/R
      byte   motorByte = SerialUSBReadByte();
      switch (CommandByte) {
        case 'F':  // Motor F/B; 0-255
          //S.FB_motor_position = motorByte;
          //write_SD_para_S();
          analogWrite(portMotorFB, motorByte);
          // delay(500);
          break;
        case 'L':  // Motor L/R; 0-255; 70 is center
          S.LR_motor_position = motorByte;
          write_SD_para_S();
          analogWrite(portMotorLR, motorByte);
          // delay(500);
          break;
        case 'P':  // Motor Pole; 0-255; 120 is anterior, 190 is posterior
          analogWrite(portMotorPole, motorByte);
          // delay(500);
          break;
        default: // never happen...
          while (SerialUSB.available()) {
            SerialUSB.read();
          }
          break;
      }
    }

    if (Serial1.available()) { // receiving data from Bpod (i.e., a trial is done)

      // Read data from Bpod, Write events to SD card, Update trial outcome
      Read_Data_from_Bpod(); // 5-100 ms (30% writ to SD card) depending how many data

      S.currentTrialNum++;

      UpdatePerformance(); // < 1 ms

      PrintResult2PC();

      // send_pars_S_to_PC();   // Send parameters S of this trial to PC; 1 ms
      write_SD_trial_info(); // log trial info to SD; 1 ms

      ////////////
      if (S.Trial_Outcome == 1) {
        last_reward_time = millis();  // record the last reward time
        timed_drop_count = 0;
      }
      ////////////

      //////////// for next trial ///////////
      autoChangeProtocol();    // Change protocol and parameters based on performance; < 1 ms
      autoAdjustLickportPosition(); // Adjust lickpost position if bias develops
      autoReward();            // Set Reward Flag if many wrongs in a row; < 1 ms
      trialSelection();        // determine S.TrialType; < 1 ms
      write_SD_para_S();       // write parameter S (updated after this trial) to SD card; 1 ms
      protocol_sent = 0;       // enable next trial
      //      if (F.fixation_duration > 10000) {
      //        delay(1000);             // additional inter-trial interval
      //      }
      //////////// for next trial ///////////
    }

    if (millis() - last_reward_time > 3 * 3600000) { // there is no reward in last 3 hours
      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(50);
      valve_control(0); // close valve 1 and 2
      last_reward_time = millis();
      SerialUSB.println("M: No Reward in Last 3 Hours.");

      timed_drop_count++;

      if (timed_drop_count >= 4) { // in last 12 hours no reward
        timed_drop_count = 0;
        // more free reward to fill the lickport tube
        valve_control(3); //  open valve 1 and 2
        delay(70);
        valve_control(0); // close valve 1 and 2
        last_reward_time = millis();
        SerialUSB.println("E: No Reward in Last 12 Hours.");

        if (F.fixation_duration <= 3000 && S.retract_times < 8 && S.FB_motor_position < 50) { // && F.trig_counter < 30
          // check if move the lickport closer to mouse to lure them in // todo: move back more frequent but retracting faster
          S.FB_motor_position = 50;
          analogWrite(portMotorFB, S.FB_motor_position);
          S.retract_times++;
          SerialUSB.print("E: Motor retracted back NO. ");
          SerialUSB.println(S.retract_times);
        }
      }
    }

    if (millis() - last_weight_read_time > 50) { // every 50 ms
      // read and restore weighting data
      weight_tmp = scale.get_units(); // 165 us
      last_weight_read_time = millis();
      if (weight_tmp > 10 && weight_tmp < 40 && headfixation_flag == 0) { // avoid record many zeros
        for (int i = 0; i < 39; i++) {
          weighting_info[i] = weighting_info[i + 1];
        }
        weighting_info[39] = weight_tmp;
        weight_counter++;
      }
      if (weight_counter > 29) {
        /*
                // writ to SD card (optional) // 1.5 ms
                File dataFile = SD.open("weight.txt", O_CREAT | O_APPEND | O_WRITE);
                if (dataFile) {
                  for (int i = 40 - weight_counter; i < 40; i++) {
                    dataFile.println(weighting_info[i]);
                  }
                } else {
                  SerialUSB.println("M: error opening weight.txt");
                }
                dataFile.close();
        */

        // writ to PC // 0.3 ms
        SerialUSB.write('W');
        SerialUSBWriteShort(weight_counter);
        SerialUSB.write((byte*)&weighting_info[40 - weight_counter], sizeof(float)*weight_counter); // weighting_info = &weighting_info[0]; start from lower byte
        SerialUSB.println();
        weight_counter = 0;
      }
    }

    SwitchL_CurrentStatus   = digitalReadDirect(portSwitchL);
    SwitchR_CurrentStatus   = digitalReadDirect(portSwitchR);

    // State Machine for the Head-fixation
    switch_fixation_state();

    SwitchL_LastStatus = SwitchL_CurrentStatus;
    SwitchR_LastStatus = SwitchR_CurrentStatus;

    // detection of release event => free water to lure next fixation
    if(headfixation_flag == 0 && last_headfixation_flag == 1 && F.fixation_duration < 8000){
      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(40);
      valve_control(0); // close valve 1 and 2
    }
    last_headfixation_flag = headfixation_flag;

    // write parameter F (updated after this trial) to SD card.
    if (para_F_changed == 1) {
      write_SD_para_F();
      para_F_changed = 0;
    }
    // current fixation event info
    if (Ev.events_num > 0) {
      //SerialUSB.println(Ev.events_num);
      write_SD_event_fixation();
      Ev.events_num = 0;
    }

  }
  else
  { // if the toggle switch is off
    if (paused == 0) {
      paused = 1; // execute only one time
      Timer4.stop();
      // stop head-fixation
      if (headfixation_flag == 1) { // release head-fixation
        analogWrite(DAC0, regulatorVal_release);
        headfixation_flag = 0;
        Ev.events_id[Ev.events_num] = 10; // headfixation release2: escape
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = millis() - last_state_time;
        Ev.events_num ++;
        write_SD_event_fixation();
        Ev.events_num = 0;
        timer_state = CHECK_TRIG;
      }
      // stop the trial
      if (protocol_sent == 1) {
        protocol_sent = 0;
        Serial1.write('X'); // stop Bpod
        delay(200);
        while (Serial1.available()) {
          Serial1.read(); // clear imput buffer
        }
      }
      // retract pole (not necessary with new Bpod firmware)
      Serial1.write('O');
      Serial1.write('B');
      Serial1.write(byte(0));

      SerialUSB.println("M: Program PAUSED!!!");
      printCurrentTime(); // print current time and date
    }
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWriteDirect(ledPin, ledState);
    delay(500);
  }

}// end of loop()




/****************************************************************************************************/
/********************************************** Functions *******************************************/
/****************************************************************************************************/

int send_protocol_to_Bpod_and_Run() {
  StateMatrix sma;                  // predefine the state matrix (sent later to Bpod)
  switch (S.ProtocolType) {
    // PROTOCOL 1: teaching animal to headfixation, which are universal to all behaviour tasks
    case P_FIXATION: { // Lick and Rectract until Switch Triggered, then increasing headfixation duration gradually

        //        // random pole out 100%
        //        if (random(100) < 0) {
        //          PoleOutput_left  = {"BNCState", 0};
        //          PoleOutput_right = {"BNCState", 0};
        //        } else {
        //          PoleOutput_left  = {"BNCState", 1};
        //          PoleOutput_right = {"BNCState", 2};
        //        }

        // Determine trial-specific state matrix fields
        switch (S.TrialType) {
          case 0: // right
            LeftLickAction  = "AnswerPeriod";
            RightLickAction = "RewardR";
            break;
          case 1: // left
            LeftLickAction  = "RewardL";
            RightLickAction = "AnswerPeriod";
            break;
          case 2: // either
            LeftLickAction  = "RewardL";
            RightLickAction = "RewardR";
            break;
        }

        MovePole(S.TrialType); // 0.75 sec

        StateChange TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //SamplePeriod
        StateChange SamplePeriod_Cond[1]       = {{"Tup", "DelayPeriod"}};
        StateChange DelayPeriod_Cond[1]        = {{"Tup", "ResponseCue"}};
        StateChange ResponseCue_Cond[1]        = {{"Tup", "AnswerPeriod"}};
        StateChange AnswerPeriod_Cond[3]       = {{"Port1In", LeftLickAction}, {"Port2In", RightLickAction}, {"Tup", "NoResponse"}};
        StateChange Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
        StateChange RewardL_Cond[1]            = {{"Tup", "RewardConsumption"}};
        StateChange RewardR_Cond[1]            = {{"Tup", "RewardConsumption"}};
        StateChange TrialEnd_Cond[1]           = {{"Tup", "exit"}};

        OutputAction Sample_Pole_Output[1]     = {PoleOutput};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction RewardL_Output[1]         = {LeftWaterOutput};
        OutputAction RewardR_Output[1]         = {RightWaterOutput};
        OutputAction NoOutput[0]               = {};

        State states[10] = {};
        S.SamplePeriod = 1.0;
        S.DelayPeriod = float(random(200, 500)) / 1000.0; // 0.2-0.5 sec random delay
        states[0]  = CreateState("TrigTrialStart",    0.01,                     1, TrigTrialStart_Cond,  0, NoOutput);
        states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           1, SamplePeriod_Cond,    1, Sample_Pole_Output);
        states[2]  = CreateState("DelayPeriod",       S.DelayPeriod,            1, DelayPeriod_Cond,     0, NoOutput);
        states[3]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
        states[4]  = CreateState("AnswerPeriod",      3600,                     3, AnswerPeriod_Cond,    0, NoOutput);
        states[5]  = CreateState("RewardL",           Reward_duration_behavior, 1, RewardL_Cond,         1, RewardL_Output);
        states[6]  = CreateState("RewardR",           Reward_duration_behavior, 1, RewardR_Cond,         1, RewardR_Output);
        states[7]  = CreateState("RewardConsumption", ConsumptionPeriod,        1, Tup_EndTrial_Cond,    0, NoOutput); //0.75
        states[8]  = CreateState("NoResponse",        0.002,                    1, Tup_EndTrial_Cond,    0, NoOutput);
        states[9]  = CreateState("TrialEnd",          0.01,                     1, TrialEnd_Cond,        0, NoOutput);

        // Predefine State sequence.
        for (int i = 0; i < 10; i++) {
          AddBlankState(&sma, states[i].Name);
        }

        // Add a state to state machine.
        for (int i = 0; i < 10; i++) {
          AddState(&sma, &states[i]);
        }
      }
      break;

    case P_SAMPLE: { // behavioral protocol 1
        //        if (S.ProtocolHistoryIndex == 1) { // no pole out
        //          PoleOutput_left  = {"BNCState", 0};
        //          PoleOutput_right = {"BNCState", 0};
        //        } else {
        //          PoleOutput_left  = {"BNCState", 1};
        //          PoleOutput_right = {"BNCState", 2};
        //        }
        switch (S.TrialType) {
          case 1: // left
            LeftLickAction  = "Reward";
            RightLickAction = "TimeOut";
            RewardOutput    = LeftWaterOutput;
            break;
          case 0: // right
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            break;
        }

        MovePole(S.TrialType); // 0.75 sec

        // Determine if give free water reward
        String ActionAfterDelay;
        if (S.TrialType == 0 && S.GaveFreeReward.flag_R_water == 1) {
          ActionAfterDelay = "GiveRightDrop";
        } else if (S.TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) {
          ActionAfterDelay = "GiveLeftDrop";
        } else {
          ActionAfterDelay = "ResponseCue";
        }

        String ActionAfterTimeOut;
        if (S.ProtocolHistoryIndex < 3) { // 1 or 2
          ActionAfterTimeOut = "AnswerPeriod"; // w/o TimeOut, continue until correct
        } else {
          ActionAfterTimeOut = "TrialEnd";  // w/ timeout, abort trial if wrong    StopLicking
        }

        StateChange SamplePeriod_Cond[1]       = {{"Tup", "DelayPeriod"}};
        StateChange TimeOut_Cond[1]            = {{"Tup", ActionAfterTimeOut}};
        StateChange TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //SamplePeriod
        StateChange DelayPeriod_Cond[1]        = {{"Tup", ActionAfterDelay}};
        StateChange ResponseCue_Cond[1]        = {{"Tup", "AnswerPeriod"}};
        StateChange GiveFreeDrop_Cond[1]       = {{"Tup", "ResponseCue"}};
        StateChange AnswerPeriod_Cond[3]       = {{"Port1In", LeftLickAction}, {"Port2In", RightLickAction}, {"Tup", "NoResponse"}};
        StateChange Reward_Cond[1]             = {{"Tup", "RewardConsumption"}};
        StateChange Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
        StateChange TrialEnd_Cond[1]           = {{"Tup", "exit"}};

        OutputAction Sample_Pole_Output[1]     = {PoleOutput};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction GiveRightDrop_Output[1]   = {RightWaterOutput};
        OutputAction GiveLeftDrop_Output[1]    = {LeftWaterOutput};
        OutputAction Reward_Output[1]          = {RewardOutput};
        OutputAction NoOutput[0]               = {};

        State states[12] = {};
        S.DelayPeriod = float(random(300, 600)) / 1000.0; // 0.3-0.6 sec random delay
        states[0]  = CreateState("TrigTrialStart",    0.01,                     1, TrigTrialStart_Cond,  0, NoOutput);
        states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           1, SamplePeriod_Cond,    1, Sample_Pole_Output);
        states[2]  = CreateState("DelayPeriod",       S.DelayPeriod,            1, DelayPeriod_Cond,     0, NoOutput);
        states[3]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
        states[4]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
        states[5]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
        states[6]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    0, NoOutput);
        states[7]  = CreateState("Reward",            Reward_duration_behavior, 1, Reward_Cond,          1, Reward_Output);
        states[8]  = CreateState("RewardConsumption", ConsumptionPeriod,        1, Tup_EndTrial_Cond,    0, NoOutput);
        states[9]  = CreateState("NoResponse",        0.002,                    1, Tup_EndTrial_Cond,    0, NoOutput);
        states[10] = CreateState("TimeOut",           S.TimeOut,                1, TimeOut_Cond,         0, NoOutput);
        states[11] = CreateState("TrialEnd",          0.01,                     1, TrialEnd_Cond,        0, NoOutput);

        // Predefine State sequence.
        for (int i = 0; i < 12; i++) {
          AddBlankState(&sma, states[i].Name);
        }

        // Add a state to state machine.
        for (int i = 0; i < 12; i++) {
          AddState(&sma, &states[i]);
        }
      }
      break;

    case P_DELAY: { // behavioural protocol 2
        switch (S.TrialType) {
          case 1:
            LeftLickAction  = "Reward";
            RightLickAction = "TimeOut";
            RewardOutput    = LeftWaterOutput;
            break;
          case 0:
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            break;
        }

        MovePole(S.TrialType); // 1 sec

        // Determine if give free water reward
        String ActionAfterDelay;
        if (S.TrialType == 0 && S.GaveFreeReward.flag_R_water == 1) {
          ActionAfterDelay = "GiveRightDrop";
        } else if (S.TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) {
          ActionAfterDelay = "GiveLeftDrop";
        } else {
          ActionAfterDelay = "ResponseCue";
        }

        StateChange TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //
        StateChange SamplePeriod_Cond[3]       = {{"Port1In", "EarlyLickSample"}, {"Port2In", "EarlyLickSample"}, {"Tup", "DelayPeriod"}};
        StateChange Sample_noforce_Cond[1]     = {{"Tup", "DelayPeriod"}};
        StateChange EarlyLickSample_Cond[1]    = {{"Tup", "SamplePeriod"}};
        StateChange DelayPeriod_Cond[3]        = {{"Port1In", "EarlyLickDelay"}, {"Port2In", "EarlyLickDelay"}, {"Tup", ActionAfterDelay}};
        StateChange EarlyLickDelay_Cond[1]     = {{"Tup", "DelayPeriod"}};
        StateChange ResponseCue_Cond[1]        = {{"Tup", "AnswerPeriod"}};
        StateChange GiveFreeDrop_Cond[1]       = {{"Tup", "ResponseCue"}};
        StateChange AnswerPeriod_Cond[3]       = {{"Port1In", LeftLickAction}, {"Port2In", RightLickAction}, {"Tup", "NoResponse"}};
        StateChange Reward_Cond[1]             = {{"Tup", "RewardConsumption"}};
        StateChange Tup_StopLicking_Cond[1]    = {{"Tup", "StopLicking"}};
        StateChange Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
        StateChange StopLicking_Cond[3]        = {{"Port1In", "StopLickingReturn"}, {"Port2In", "StopLickingReturn"}, {"Tup", "TrialEnd"}}; //
        StateChange TrialEnd_Cond[1]           = {{"Tup", "exit"}};

        OutputAction TrigTrialStart_Output[1]  = {WaveSurferTrig};
        OutputAction Sample_Pole_Output[1]     = {PoleOutput};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction GiveRightDrop_Output[1]   = {RightWaterOutput};
        OutputAction GiveLeftDrop_Output[1]    = {LeftWaterOutput};
        OutputAction Reward_Output[1]          = {RewardOutput};
        OutputAction NoOutput[0]               = {};

        State states[16] = {};
        if (S.ProtocolHistoryIndex == 18) { // start mask flshing and force sample
          states[0]  = CreateState("TrigTrialStart",    0.01,                     1, TrigTrialStart_Cond,  1, TrigTrialStart_Output);
          states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           3, SamplePeriod_Cond,    1, Sample_Pole_Output);
          states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
        } else {
          states[0]  = CreateState("TrigTrialStart",    0.01,                     1, TrigTrialStart_Cond,  0, NoOutput);
          states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           1, Sample_noforce_Cond,  1, Sample_Pole_Output);
          states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
        }
        states[3]  = CreateState("DelayPeriod",       S.DelayPeriod,            3, DelayPeriod_Cond,     0, NoOutput);
        states[4]  = CreateState("EarlyLickDelay",    0.05,                     1, EarlyLickDelay_Cond,  0, NoOutput);
        states[5]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
        states[6]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
        states[7]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
        states[8]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    0, NoOutput);
        states[9]  = CreateState("Reward",            Reward_duration_behavior, 1, Reward_Cond,          1, Reward_Output);
        states[10] = CreateState("RewardConsumption", ConsumptionPeriod,        1, Tup_StopLicking_Cond, 0, NoOutput);
        states[11] = CreateState("NoResponse",        0.002,                    1, Tup_StopLicking_Cond, 0, NoOutput);
        states[12] = CreateState("TimeOut",           S.TimeOut,                1, Tup_StopLicking_Cond, 0, NoOutput);
        states[13] = CreateState("StopLicking",       StopLickingPeriod,        3, StopLicking_Cond,     0, NoOutput);
        states[14] = CreateState("StopLickingReturn", 0.01,                     1, Tup_StopLicking_Cond, 0, NoOutput);
        states[15] = CreateState("TrialEnd",          0.01,                     1, TrialEnd_Cond,        0, NoOutput);

        // Predefine State sequence.
        for (int i = 0; i < 16; i++) {
          AddBlankState(&sma, states[i].Name);
        }

        // Add a state to state machine.
        for (int i = 0; i < 16; i++) {
          AddState(&sma, &states[i]);
        }
      }
      break;

    case P_OPTOSTIM: { // behavioural protocol 3
        switch (S.TrialType) {
          case 1:
            LeftLickAction  = "Reward";
            RightLickAction = "TimeOut";
            RewardOutput    = LeftWaterOutput;
            break;
          case 0:
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            break;
        }

        MovePole(S.TrialType); // 2 sec

        /* //no auto reward during optostim
            //otherwise the response optostim will be problematic!
          // Determine if give free water reward
          String ActionAfterCue;
          if (S.TrialType == 0 && S.GaveFreeReward.flag_R_water == 1) {
          ActionAfterCue = "GiveRightDrop";
          } else if (S.TrialType == 1 && S.GaveFreeReward.flag_L_water == 1) {
          ActionAfterCue = "GiveLeftDrop";
          } else {
          ActionAfterCue = "AnswerPeriod";
          }
        */
        String ActionAfterCue;
        ActionAfterCue = "AnswerPeriod";

        StateChange TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //SamplePeriod
        StateChange SamplePeriod_Cond[3]       = {{"Port1In", "EarlyLickSample"}, {"Port2In", "EarlyLickSample"}, {"Tup", "DelayPeriod"}};
        StateChange EarlyLickSample_Cond[1]    = {{"Tup", "SamplePeriod"}};
        StateChange DelayPeriod_Cond[3]        = {{"Port1In", "EarlyLickDelay"}, {"Port2In", "EarlyLickDelay"}, {"Tup", "ResponseCue"}};
        StateChange EarlyLickDelay_Cond[1]     = {{"Tup", "DelayPeriod"}};
        StateChange ResponseCue_Cond[1]        = {{"Tup", ActionAfterCue}};
        StateChange GiveFreeDrop_Cond[1]       = {{"Tup", "AnswerPeriod"}};
        StateChange AnswerPeriod_Cond[3]       = {{"Port1In", LeftLickAction}, {"Port2In", RightLickAction}, {"Tup", "NoResponse"}};
        StateChange Reward_Cond[1]             = {{"Tup", "RewardConsumption"}};
        StateChange Tup_StopLicking_Cond[1]    = {{"Tup", "StopLicking"}};
        StateChange Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
        StateChange StopLicking_Cond[3]        = {{"Port1In", "StopLickingReturn"}, {"Port2In", "StopLickingReturn"}, {"Tup", "TrialEnd"}}; //
        StateChange TrialEnd_Cond[1]           = {{"Tup", "exit"}};

        OutputAction TrigTrialStart_Output[1]  = {WaveSurferTrig};
        OutputAction Sample_Pole_Output[1]     = {PoleOutput};
        OutputAction Sample_Pole_Stim_Output[2] = {PoleOutput, OptogeneticTrig};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction ResponseCueStim_Output[2] = {CueOutput, OptogeneticTrig};
        OutputAction GiveRightDrop_Output[1]   = {RightWaterOutput};
        OutputAction GiveLeftDrop_Output[1]    = {LeftWaterOutput};
        OutputAction Reward_Output[1]          = {RewardOutput};
        OutputAction NoOutput[0]               = {};
        OutputAction OptoStim_Output[1]        = {OptogeneticTrig}; // new to Stim protocol

        State states[16] = {};
        states[0]  = CreateState("TrigTrialStart",    0.1,                      1, TrigTrialStart_Cond,  1, TrigTrialStart_Output);

        if (random(100) < 15) {  // 10% trials to stim
          int randomNum = random(100);
//          if (randomNum < 200) {
//            weightByte = 10;  // percent
//          } else if (randomNum < 400) {
//            weightByte = 30;  // percent
//          } else if (randomNum < 600) {
//            weightByte = 50;  // percent
//          } else if (randomNum < 800) {
//            weightByte = 70;  // percent
//          } else {
//            weightByte = 100; // percent
//          }
          if (randomNum < 33) {
            weightByte = 10;  // percent
          } else if (randomNum < 67){
            weightByte = 50; // percent
          } else {
            weightByte = 100; // percent
          }
          
          byte laserByte = 2; // 1-Thorlab; 2-UltraLaser
          
          Serial2.write(weightByte);
          Serial2.write(laserByte);

          randomNum = random(300);
          if (randomNum < 151) { // 1/3 sample
            states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           3, SamplePeriod_Cond,    2, Sample_Pole_Stim_Output);
            states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 2, Sample_Pole_Stim_Output);
            states[3]  = CreateState("DelayPeriod",       S.DelayPeriod,            3, DelayPeriod_Cond,     0, NoOutput);
            states[4]  = CreateState("EarlyLickDelay",    0.05,                     1, EarlyLickDelay_Cond,  0, NoOutput);
            states[5]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
            states[6]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
            states[7]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
            states[8]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    0, NoOutput);
            // mark this trial as stim trial...sample
            trial_stim_index = weightByte + 1; // 11, 31, 51, 71, 101
          } else if (randomNum < 301) {// 1/3 delay
            states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           3, SamplePeriod_Cond,    1, Sample_Pole_Output);
            states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
            states[3]  = CreateState("DelayPeriod",       S.DelayPeriod,            3, DelayPeriod_Cond,     1, OptoStim_Output);
            states[4]  = CreateState("EarlyLickDelay",    0.05,                     1, EarlyLickDelay_Cond,  1, OptoStim_Output);
            states[5]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
            states[6]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
            states[7]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
            states[8]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    0, NoOutput);
            // mark this trial as stim trial...delay
            trial_stim_index = weightByte + 2; // 12, 32, 52, 72, 102
          } else {// 1/3 response
            states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           3, SamplePeriod_Cond,    1, Sample_Pole_Output);
            states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
            states[3]  = CreateState("DelayPeriod",       S.DelayPeriod,            3, DelayPeriod_Cond,     0, NoOutput);
            states[4]  = CreateState("EarlyLickDelay",    0.05,                     1, EarlyLickDelay_Cond,  0, NoOutput);
            states[5]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     2, ResponseCueStim_Output);
            states[6]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
            states[7]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
            states[8]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    1, OptoStim_Output);
            // mark this trial as stim trial...response
            trial_stim_index = weightByte + 3; // 13, 33, 53, 73, 103
          }
        }
        else { // Control trial
          states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           3, SamplePeriod_Cond,    1, Sample_Pole_Output);
          states[2]  = CreateState("EarlyLickSample",   0.05,                     1, EarlyLickSample_Cond, 1, Sample_Pole_Output);
          states[3]  = CreateState("DelayPeriod",       S.DelayPeriod,            3, DelayPeriod_Cond,     0, NoOutput);
          states[4]  = CreateState("EarlyLickDelay",    0.05,                     1, EarlyLickDelay_Cond,  0, NoOutput);
          states[5]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
          states[6]  = CreateState("GiveRightDrop",     Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveRightDrop_Output);
          states[7]  = CreateState("GiveLeftDrop",      Reward_duration_behavior, 1, GiveFreeDrop_Cond,    1, GiveLeftDrop_Output);
          states[8]  = CreateState("AnswerPeriod",      AnswerPeriod_behavior,    3, AnswerPeriod_Cond,    0, NoOutput);
          // mark this trial as NON-stim (Control) trial
          trial_stim_index = 0;
        }
        states[9]  = CreateState("Reward",            Reward_duration_behavior, 1, Reward_Cond,          1, Reward_Output);
        states[10] = CreateState("RewardConsumption", ConsumptionPeriod,        1, Tup_StopLicking_Cond, 0, NoOutput);
        states[11] = CreateState("NoResponse",        0.002,                    1, Tup_StopLicking_Cond, 0, NoOutput);
        states[12] = CreateState("TimeOut",           S.TimeOut,                1, Tup_StopLicking_Cond, 0, NoOutput);
        states[13] = CreateState("StopLicking",       StopLickingPeriod,        3, StopLicking_Cond,     0, NoOutput);
        states[14] = CreateState("StopLickingReturn", 0.01,                     1, Tup_StopLicking_Cond, 0, NoOutput);
        states[15] = CreateState("TrialEnd",          0.05,                     1, TrialEnd_Cond,        0, NoOutput);

        // Predefine State sequence.
        for (int i = 0; i < 16; i++) {
          AddBlankState(&sma, states[i].Name);
        }

        // Add a state to state machine.
        for (int i = 0; i < 16; i++) {
          AddState(&sma, &states[i]);
        }
      }
      break;

    default:
      break;
  }// end for Switch(protocol)

  // Send the StateMatrix to Bpod
  SendStateMatrix(&sma);

  // Send 'R' to Bpod to run the state machine
  RunStateMatrix();
  protocol_sent = 1;
  SerialUSB.println("M: Bpod Running...");

} // end for send_protocol_to_Bpod_and_Run()


void MovePole(byte trial_type) {
  // halfPos, leftPos, rightPos, finalPos
  switch (trial_type) {
    case 0: // right
      finalPos = rightPos;
      break;
    case 1: // left
      finalPos = leftPos;
      break;
    case 2: // either
      if (random(100) < 50) {
        finalPos = rightPos;
      } else {
        finalPos = leftPos;
      }
      break;
  }
  analogWrite(portMotorPole, halfPos);
  delay(750);
  analogWrite(portMotorPole, finalPos);
  delay(750);
}


int Read_Data_from_Bpod() {
  Timer4.start(); // in case of get struck in receiving data
  Receiving_data_from_Bpod = 1;
  byte opCode = Serial1.read();
  if (opCode == 1) {
    // Receive time stamps.
    //unsigned long trialStartTimeStamp = Serial1ReadLong();
    //unsigned long matrixStartTimeStamp = Serial1ReadLong();
    nEvents = Serial1ReadShort();
    //SerialUSB.println(nEvents);
    if (nEvents > 1024) {
      nEvents = 1024;
    }
    //unsigned long eventTimeStamps[1024] = {};
    //byte Events[1024] = {};
    for (int i = 0; i < nEvents; i++) {
      Events[i] = Serial1ReadByte();
      eventTimeStamps[i] = Serial1ReadLong();
    }
    //SerialUSB.println("Receive TimeStamps fini...");
    // Receiving state visited in this trial
    //byte state_visited[1024] = {};
    nTransition = Serial1ReadShort();
    //SerialUSB.println(nTransition);
    if (nTransition > 1024) {
      nTransition = 1024;
    }
    for (int i = 0; i < nTransition; i++) {
      state_visited[i] = Serial1ReadByte();
    }
  } else { // error reading Bpod data...
    //SerialUSB.println(opCode);
    delay(1000);
    while (Serial1.available()) {
      Serial1.read(); // clear serial1 dirty data
    }
    nEvents = 0; nTransition = 0;
    SerialUSB.println("E: Receiving Bpod Data Error...");
  }

  if (Serial1.available()) {
    delay(1000);
    while (Serial1.available()) {
      Serial1.read(); // clear serial1 dirty data
    }
  }

  Receiving_data_from_Bpod = 0;
  Timer4.stop(); // in case of get struck in receiving data

  /*
    // And send to PC. fast < 10 ms
    //unsigned long startMillis = millis();
    SerialUSB.write('E'); // Event and timestamp
    SerialUSBWriteShort(nEvents);
    for (int i = 0; i < nEvents; i++) {
    SerialUSB.write(Events[i]);
    SerialUSBWriteLong(eventTimeStamps[i]);//
    }
    //SerialUSB.println(millis() - startMillis);
  */
  // write event to SD card
  File dataFile = SD.open("eventsT.txt", O_CREAT | O_APPEND | O_WRITE);
  if (dataFile) {
    dataFile.print(S.currentTrialNum + 1);
    dataFile.print(" ");
    dataFile.print(nEvents);
    for (int i = 0; i < nEvents; i++) {
      dataFile.print(" ");
      dataFile.print(Events[i]);
      dataFile.print(" ");
      dataFile.print(eventTimeStamps[i]);//
    }
    dataFile.println();
  } else {
    SerialUSB.println("M: error opening eventsT.txt");
  }
  dataFile.close();

  // Update the outcome of current trial
  S.Trial_Outcome = 3;// 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
  switch (S.ProtocolType) {
    case P_FIXATION:
      if (nTransition == 0) {
        S.Trial_Outcome = 3;
      } else {
        S.Trial_Outcome = 0; // No Response-0
        for (int i = 0; i < nTransition; i++) {
          if (state_visited[i] == 5 || state_visited[i] == 6) { // Reward-1
            S.Trial_Outcome = 1;
            if (state_visited[i] == 5) {
              either_left_right = 1; // left
            } else {
              either_left_right = 0; // right
            }
            break;
          }
        }
      }
      break;
    case P_SAMPLE:
      if (S.ProtocolHistoryIndex < 3) { // 1 or 2, no timeout
        for (int i = 0; i < nTransition; i++) {
          if (state_visited[i] == 7 || state_visited[i] == 9) { // Reward || No Response
            if (state_visited[i] == 7) {
              S.Trial_Outcome = 1;
            } else {
              S.Trial_Outcome = 0;
            }
            break;
          }
        }
      } else {
        for (int i = 0; i < nTransition; i++) {
          if (state_visited[i] == 7 || state_visited[i] == 9 || state_visited[i] == 10) { // Reward || No Response || Time Out (error)
            if (state_visited[i] == 7) {
              S.Trial_Outcome = 1;
            } else if (state_visited[i] == 9) {
              S.Trial_Outcome = 0;
            } else {
              S.Trial_Outcome = 2;
            }
            break;
          }
        }
      }
      break;

    case P_DELAY:
      for (int i = 0; i < nTransition; i++) {
        if (state_visited[i] == 9 || state_visited[i] == 11 || state_visited[i] == 12) { // Reward || No Response || Time Out (error)
          if (state_visited[i] == 9) {
            S.Trial_Outcome = 1;
          } else if (state_visited[i] == 11) {
            S.Trial_Outcome = 0;
          } else {
            S.Trial_Outcome = 2;
          }
          break;
        }
      }
      is_earlylick = 0;
      for (int i = 0; i < nTransition; i++) {
        if (state_visited[i] == 2 || state_visited[i] == 4) { // earlylick sample || delay
          is_earlylick = 1;
          break;
        }
      }
      break;

    case P_OPTOSTIM: // the same as P_DELAY
      for (int i = 0; i < nTransition; i++) {
        if (state_visited[i] == 9 || state_visited[i] == 11 || state_visited[i] == 12) { // Reward || No Response || Time Out (error)
          if (state_visited[i] == 9) {
            S.Trial_Outcome = 1;
          } else if (state_visited[i] == 11) {
            S.Trial_Outcome = 0;
          } else {
            S.Trial_Outcome = 2;
          }
          break;
        }
      }
      is_earlylick = 0;
      for (int i = 0; i < nTransition; i++) {
        if (state_visited[i] == 2 || state_visited[i] == 4) { // earlylick sample || delay
          is_earlylick = 1;
          break;
        }
      }
      break;
  }
}

void PrintResult2PC() {
  SerialUSB.print("Trial No.:");
  SerialUSB.print(S.currentTrialNum);
  SerialUSB.print("; Perf100:");
  SerialUSB.print(Perf100);
  SerialUSB.print("%; R:");
  SerialUSB.print(S.ProtocolHistoryIndex);
  SerialUSB.print(" - ");
  SerialUSB.print(S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials);
  SerialUSB.print(" - ");
  SerialUSB.print(S.ProtocolHistory[S.ProtocolHistoryIndex].performance);
  SerialUSB.print("%; early:");
  SerialUSB.println(is_earlylick);

  SerialUSB.print("Protocol:");
  switch (S.ProtocolType) {
    case P_FIXATION:
      SerialUSB.print("P_FIXATION");
      break;
    case P_SAMPLE:
      SerialUSB.print("P_SAMPLE");
      break;
    case P_DELAY:
      SerialUSB.print("P_DELAY");
      break;
    case P_OPTOSTIM:
      SerialUSB.print("P_OPTOSTIM");
      break;
  }
  SerialUSB.print("; TrialType:");
  if (S.TrialType == 0) {
    SerialUSB.print("Right");
  } else if (S.TrialType == 1) {
    SerialUSB.print("Left");
  } else {
    SerialUSB.print("Either-");
    if (either_left_right == 1) {
      SerialUSB.print("Left");
    } else if (either_left_right == 0) {
      SerialUSB.print("Right");
    }
  }
  SerialUSB.print("; Outcome:");
  SerialUSB.print(S.Trial_Outcome);
  SerialUSB.print("; Reward No.:");
  SerialUSB.println(S.totoal_reward_num);
  //SerialUSB.print("Perf30      : ");
  //SerialUSB.print(S.ProtocolHistory[S.ProtocolHistoryIndex].performance);
  //SerialUSB.println("%");

  if (F.trig_counter <= trigger_num_before_fix || S.FB_motor_position > 0) {
    SerialUSB.print("M: Motor Pos: ");
    SerialUSB.print(S.FB_motor_position);
    SerialUSB.print("; Trig No.: ");
    SerialUSB.println(F.trig_counter);
  }
}

int UpdatePerformance() {
  if (S.Trial_Outcome == 1) {
    S.totoal_reward_num++;
  }

  // do FIFO
  for (int i = 0; i < RECORD_TRIALS - 1; i++) {
    S.ProtocolTypeHistory[i] = S.ProtocolTypeHistory[i + 1];
    S.TrialTypeHistory[i]    = S.TrialTypeHistory[i + 1]; // 0 right; 1 left;
    S.OutcomeHistory[i]      = S.OutcomeHistory[i + 1];         // 0 error; 1 correct; 2 no response; 3 others
  }
  // Keep record current trial info in the last position (RECORD_TRIALS-1) of the matrix
  S.ProtocolTypeHistory[RECORD_TRIALS - 1] = S.ProtocolType;
  S.OutcomeHistory[RECORD_TRIALS - 1]      = S.Trial_Outcome;
  if (S.TrialType == 2) {
    S.TrialTypeHistory[RECORD_TRIALS - 1]  = either_left_right;
  } else {
    S.TrialTypeHistory[RECORD_TRIALS - 1]  = S.TrialType;
  }

  byte Outcomes_sum = 0;
  for (int i = RECORD_TRIALS - recent_trials; i < RECORD_TRIALS; i++) {
    if (S.OutcomeHistory[i] == 1) {
      Outcomes_sum++;
    }
  }
  S.ProtocolHistory[S.ProtocolHistoryIndex].performance = round((float)Outcomes_sum / recent_trials * 100.0);
  S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials++;

  Perf100 = 0;
  for (int i = 0; i < RECORD_TRIALS; i++) {
    if (S.OutcomeHistory[i] == 1) {
      Perf100++;
    }
  }
}


/*
   Arguments:
      Parameters *S

   Returns:
      0 if successful.
      -1 if failed.

   Effects:
      This function will Update Protocol and training parameters (S.Autolearn, S.random_delay_duration) based on past performance.
*/
int autoChangeProtocol() {
  switch (S.ProtocolHistoryIndex) {
    case 0:
      // determine TrialType
      if (S.FB_motor_position < 50 && S.FB_motor_position > 0) {
        S.Autolearn = 0; // eigher side will be rewarded
      } else {
        S.Autolearn = 1; // 'ON': 3 left, 3 right
      }

      //      if (S.FB_motor_position > 50) { // start from autolearn ON
      //        S.Autolearn = 1;
      //      }
      //      if (S.Autolearn == 1 && S.FB_motor_position < 10 && F.fixation_duration <= 10000) {
      //        S.Autolearn = 0; // change from 'ON' to 'either'
      //      }
      //      if (S.Autolearn == 0 && F.fixation_duration > 10000) {
      //        S.Autolearn = 1; // change from 'either' to 'ON'
      //      }

      /*       // determine DelayPeriod
            switch (S.random_delay_duration) {
              case 1001:
                if (F.fixation_duration > 10000) {
                  S.random_delay_duration = 2001;
                }
                break;
              case 2001:
                if (F.fixation_duration > 13000) {
                  S.random_delay_duration = 3001;
                }
                break;
              case 3001:
                if (F.fixation_duration > 20000) {
                  S.random_delay_duration = 4001;
                }
                break;
              case 4001:
                if (F.fixation_duration > 22000) {
                  S.random_delay_duration = 5001;
                }
                break;
              default:
                S.random_delay_duration = 3001;
                break;
            } */

      // determine if advance motor
      if (S.FB_motor_position > 0) {
        if (S.FB_motor_position > 150) {
          if (S.totoal_reward_num != last_advance_reward_num && S.totoal_reward_num % lick_num_before_motor_retract1 == 0) {
            is_motor_advance = 1;
            last_advance_reward_num = S.totoal_reward_num;
          }
        } else if (S.FB_motor_position > 70) {
          if (S.totoal_reward_num != last_advance_reward_num && S.totoal_reward_num % lick_num_before_motor_retract2 == 0) {
            is_motor_advance = 1;
            last_advance_reward_num = S.totoal_reward_num;
          }
        } else {
          if (S.retract_times == 0) {
            if (S.totoal_reward_num != last_advance_reward_num && S.totoal_reward_num % lick_num_before_motor_retract3 == 0) {
              is_motor_advance = 1;
              last_advance_reward_num = S.totoal_reward_num;
            }
          }
          else {
            if (S.totoal_reward_num != last_advance_reward_num && S.totoal_reward_num % 5 == 0) { // (10 / S.retract_times + 2)
              is_motor_advance = 1;
              last_advance_reward_num = S.totoal_reward_num;
            }
          }
        }

        if (is_motor_advance == 1) {
          is_motor_advance = 0;

          if (S.FB_motor_position < motor_retract_step) {
            S.FB_motor_position = 0;
            analogWrite(portMotorFB, S.FB_motor_position);
          } else {
            for (int j = 0; j < motor_retract_step; j++) {
              S.FB_motor_position = S.FB_motor_position - 1;
              analogWrite(portMotorFB, S.FB_motor_position);
              delay(100);
            }
          }
          //          Ev.events_id[Ev.events_num] = 5; // motor advance
          //          Ev.events_time[Ev.events_num] = millis();
          //          Ev.events_value[Ev.events_num] = S.FB_motor_position;
          //          Ev.events_num ++;
          SerialUSB.print("M: Motor advanced to ");
          SerialUSB.println(S.FB_motor_position);
        }
      }

      if (F.fixation_duration > 25000) { // 25 sec
        S.ProtocolType         = P_SAMPLE;

        S.SamplePeriod        = 1.20;
        S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 1.00;
        S.Autolearn           = 1;    // autoLearn ON

        S.ProtocolHistoryIndex = 1;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
      }
      break;

    case 1: // P_SAMPLE-1: autoLearn ON, SamplePeriod = 0.75, w/o error trial, w/o pole out (removed)
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100) { // if  ran > 100 trials
        S.ProtocolHistoryIndex = 3;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.20;
        S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 1.00;
        S.Autolearn           = 1;    // autoLearn ON
      }
      break;

    /*     case 2: // P_SAMPLE-2: autoLearn ON, SamplePeriod = 0.75, w/o error trial
          if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100) { // if  ran > 100 trials
            S.ProtocolHistoryIndex = 3;
            S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
            S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
            S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
            S.SamplePeriod        = 0.75;
            S.TimeOut             = 0.50;
            S.Autolearn           = 1;    // autoLearn ON
          }
          break; */

    case 3: // P_SAMPLE-3: autoLearn ON, SamplePeriod = 0.75, w/ error trial (timeout = 0.5 sec)
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 200) {
        S.ProtocolHistoryIndex = 4;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.20;
        S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 2.00;
        S.Autolearn = 2; // antiBias
      }
      break;

    case 4: // P_SAMPLE-4: autoLearn Antibias, SamplePeriod = 0.75, w/ timeout = 0.5 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 75) {
        S.ProtocolHistoryIndex = 13;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.ProtocolType        = P_DELAY;
        S.SamplePeriod        = 1.20;  // in sec
        S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 3.0;
        S.Autolearn           = 2;     // antiBias
      }
      break;

    /* case 5: // P_SAMPLE-5: autoLearn Antibias, SamplePeriod = 1.0, w/ timeout = 0.5 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 6;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.2;
        S.TimeOut             = 0.50;
        S.Autolearn           = 2; // antiBias
      }
      break;

      case 6: // P_SAMPLE-6: autoLearn Antibias, SamplePeriod = 1.2, w/ timeout = 0.5 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 7;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.2;
        S.TimeOut             = 1.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

      case 7: // P_SAMPLE-7: autoLearn Antibias, SamplePeriod = 1.2, w/ timeout = 1.0 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 8;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.2;
        S.TimeOut             = 1.5;
        S.Autolearn           = 2; // antiBias
      }
      break;

      case 8: // P_SAMPLE-8: autoLearn Antibias, SamplePeriod = 1.2, w/ timeout = 2.0 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 9;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 1.2;
        S.TimeOut             = 2.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

      case 9: // P_SAMPLE-9: autoLearn Antibias, SamplePeriod = 1.2, w/ timeout = 4.0 sec
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 10;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.ProtocolType         = P_DELAY;
        S.SamplePeriod         = 0.75;  // in sec
        S.DelayPeriod          = 0.1;   // in sec
        S.TimeOut              = 2.0;
        S.Autolearn            = 2; // antiBias

        //S.struggle_enable      = 0;
        F.fixation_duration = 30000;
        para_F_changed = 1;
      }
      break;

      case 10: // P_DELAY-10: SamplePeriod = 0.75, DelayPeriod = 0.1
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 200 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 11;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod        = 0.75;  // in sec
        S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 2.0;
        S.Autolearn           = 2;     // antiBias
      }
      break;

      case 11: // P_DELAY-11: SamplePeriod = 0.75, DelayPeriod = 0.3
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 300 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 12;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod = 1.0;  // in sec
        S.DelayPeriod  = 0.3;   // in sec
        S.TimeOut             = 2.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

      case 12: // P_DELAY-12: SamplePeriod = 1.0, DelayPeriod = 0.3
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 13;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod  = 0.3;   // in sec
        S.TimeOut             = 2.0;
        S.Autolearn           = 2; // antiBias
      }
      break; */

    case 13: // P_DELAY-13: SamplePeriod = 1.2, DelayPeriod = 0.3
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 14;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod  = 0.5;   // in sec
        S.TimeOut             = 3.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

    case 14: // P_DELAY-14: SamplePeriod = 1.2, DelayPeriod = 0.5
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 15;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod  = 0.8;   // in sec
        S.TimeOut             = 3.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

    case 15: // P_DELAY-15: SamplePeriod = 1.2, DelayPeriod = 0.8
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 16;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod  = 1.0;   // in sec
        S.TimeOut             = 3.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

    case 16: // P_DELAY-16: SamplePeriod = 1.2, DelayPeriod = 1.0
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 17;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod  = 1.3;   // in sec
        S.TimeOut             = 3.0;
        S.Autolearn           = 2; // antiBias
      }
      break;

    case 17: // P_DELAY-17: SamplePeriod = 1.2, DelayPeriod = 1.3
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 18;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod   = 1.3;   // in sec
        S.TimeOut             = 4.0;
        S.Autolearn           = 2; // antiBias
        F.fixation_interval_max = 60000;
        para_F_changed = 1;
      }
      break;

    case 18: // P_DELAY-18: SamplePeriod = 1.2, DelayPeriod = 1.3, TimeOut = 6.0
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 500 && Perf100 > 70) {
        S.ProtocolHistoryIndex = 19;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_OPTOSTIM;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.ProtocolType  = P_OPTOSTIM;
        S.SamplePeriod  = 1.2;   // in sec
        S.DelayPeriod   = 1.3;   // in sec
        S.TimeOut       = 4.0;
        S.Autolearn     = 2; // antiBias
      }
      break;
    case 19: // P_OPTOSTIM-19:
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 3000 && Perf100 > 100) {
        // ...
      }
      break;

    default:
      break;
  }
}

void autoAdjustLickportPosition() {
  if (S.ProtocolHistoryIndex > 2 && S.ProtocolHistoryIndex <= 19) { //
    LickPortMove++;
    if (LickPortMove > 50) {
      byte recent1 = 50; // last 50 trials are biased 60% vs. 40%
      byte correct_R_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 0, RECORD_TRIALS - recent1, RECORD_TRIALS);
      byte correct_L_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 1, RECORD_TRIALS - recent1, RECORD_TRIALS);
      byte trial_R_num = compare_array_sum(S.TrialTypeHistory, 0, RECORD_TRIALS - recent1, RECORD_TRIALS);
      byte trial_L_num = compare_array_sum(S.TrialTypeHistory, 1, RECORD_TRIALS - recent1, RECORD_TRIALS);
      float Perf_R;
      float Perf_L;
      if (trial_R_num > 0) {
        Perf_R = (float)correct_R_history / (float)trial_R_num;
      } else {
        Perf_R = 1;
      }
      if (trial_L_num > 0) {
        Perf_L = (float)correct_L_history / (float)trial_L_num;
      } else {
        Perf_L = 1;
      }
      boolean condition1 = abs(Perf_R - Perf_L) > 0.3;

      byte recent2 = 20; // or, 90% vs. 10% in the last 20 trials
      correct_R_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 0, RECORD_TRIALS - recent2, RECORD_TRIALS);
      correct_L_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 1, RECORD_TRIALS - recent2, RECORD_TRIALS);
      trial_R_num = compare_array_sum(S.TrialTypeHistory, 0, RECORD_TRIALS - recent2, RECORD_TRIALS);
      trial_L_num = compare_array_sum(S.TrialTypeHistory, 1, RECORD_TRIALS - recent2, RECORD_TRIALS);
      byte ignore_rate = compare_array_sum(S.OutcomeHistory, 2, RECORD_TRIALS - recent2, RECORD_TRIALS);
      if (trial_R_num > 0) {
        Perf_R = (float)correct_R_history / (float)trial_R_num;
      } else {
        Perf_R = 1;
      }
      if (trial_L_num > 0) {
        Perf_L = (float)correct_L_history / (float)trial_L_num;
      } else {
        Perf_L = 1;
      }
      boolean condition2 = (abs(Perf_R - Perf_L) >= 0.8) | (Perf_R<0.2 & trial_R_num>15) | (Perf_L<0.2 & trial_L_num>15);
      condition2 = condition2 & (ignore_rate < 10);

      if (condition1 | condition2) { // move lickprot
        LickPortMove = 0;
        SerialUSB.print("E: bias to ");
        if (Perf_R > Perf_L) { // right port too close
          SerialUSB.print("right: ");
          SerialUSB.print(S.LR_motor_position);
          if (S.LR_motor_position <= 90) {
            S.LR_motor_position = S.LR_motor_position + mov_step_size;
          }
        } else if (Perf_R < Perf_L) {               // left port too close
          SerialUSB.print("left: ");
          SerialUSB.print(S.LR_motor_position);
          if (S.LR_motor_position >= 50) {
            S.LR_motor_position = S.LR_motor_position - mov_step_size;
          }
        }
        SerialUSB.print(" -> ");
        SerialUSB.println(S.LR_motor_position);

        //        Ev.events_id[Ev.events_num] = 6; // left/right motor move
        //        Ev.events_time[Ev.events_num] = millis();
        //        Ev.events_value[Ev.events_num] = S.LR_motor_position;
        //        Ev.events_num ++;

        analogWrite(portMotorLR, S.LR_motor_position);
      }
    }
  }
}

int autoReward() {
  if (S.ProtocolType > P_FIXATION) {
    // Give reward more frequently in the begining to encourage licking
    // 1) last <error_trials> trials of a particular type are incorrect;
    // 2) and, S.GaveFreeReward(:,3)>3

    S.GaveFreeReward.flag_R_water = 0;
    S.GaveFreeReward.flag_L_water = 0;
    S.GaveFreeReward.past_trials ++;
	
	byte error_trials = 3; // consecutive 4 errors in a paticular trial type

    // 2) and, S.GaveFreeReward(:,3)>3
    if (S.GaveFreeReward.past_trials > 3) { // && compare_array_sum(S.ProtocolTypeHistory, S.ProtocolType, 90, RECORD_TRIALS) == 10
      // 1) last <error_trials> trials of a particular type are incorrect;
      byte n_RSideList = 0;
      byte n_LSideList = 0;
      for (int i = 0; i < error_trials; i++) {
        if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 0 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1) {
          n_RSideList++;
        } else if (S.TrialTypeHistory[RECORD_TRIALS - error_trials + i] == 1 && S.OutcomeHistory[RECORD_TRIALS - error_trials + i] != 1) {
          n_LSideList++;
        }
      }
      if (n_RSideList = error_trials) {
        S.GaveFreeReward.flag_R_water = 1;
        S.GaveFreeReward.flag_L_water = 0;
        S.GaveFreeReward.past_trials = 0;
      } else if (n_LSideList == error_trials) {
        S.GaveFreeReward.flag_R_water = 0;
        S.GaveFreeReward.flag_L_water = 1;
        S.GaveFreeReward.past_trials = 0;
      }
    }
  }
}

int trialSelection() {
  int switch_indicator = 0;
  switch (S.Autolearn) {
    case 0: // either side
      S.TrialType = 2;
      break;

    case 1: // Autolearn     'On'
      // switch trial types only when reaching MaxSame CORRECT trials
      for (int i = RECORD_TRIALS - 1; i >= 0; i--) {
        if (S.TrialTypeHistory[i] == S.TrialTypeHistory[RECORD_TRIALS - 1]) {
          if (S.OutcomeHistory[i] == 1) {
            switch_indicator++;
          }
        } else {
          break;
        }
      }
      if (switch_indicator >= MaxSame) {
        if (S.TrialTypeHistory[RECORD_TRIALS - 1] == 0) {
          S.TrialType = 1;
        } else {
          S.TrialType = 0;
        }
      } else {
        S.TrialType = S.TrialTypeHistory[RECORD_TRIALS - 1];
      }
      break;

    case 2: { // Autolearn     'antiBias'
        float newNoTrialProb;
        if (S.currentTrialNum > recent_trials) {
          byte correct_R_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 0, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
          byte correct_L_history = compare_array_sum(S.OutcomeHistory, 1, S.TrialTypeHistory, 1, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
          byte incorrect_R_history = compare_array_sum(S.OutcomeHistory, 2, S.TrialTypeHistory, 0, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
          byte incorrect_L_history = compare_array_sum(S.OutcomeHistory, 2, S.TrialTypeHistory, 1, RECORD_TRIALS - recent_trials, RECORD_TRIALS);
          if ((correct_R_history + correct_L_history) != 0) {
            float percent_R_corr = (float)correct_R_history / (float)(correct_R_history + correct_L_history);
            if ((incorrect_L_history + incorrect_R_history) != 0) {
              float percent_L_incorr = (float)incorrect_L_history / (float)(incorrect_L_history + incorrect_R_history);
              newNoTrialProb = (percent_R_corr + percent_L_incorr) * 0.5;
            } else {
              newNoTrialProb = NoTrialProb;
            }
          } else {
            newNoTrialProb = NoTrialProb;
          }
        } else {
          newNoTrialProb = NoTrialProb;
        }

        //////////////////////////////////////
        for (int i = RECORD_TRIALS - 1; i >= RECORD_TRIALS - MaxSame; i--) {
          if (S.TrialTypeHistory[i] == S.TrialTypeHistory[RECORD_TRIALS - 1]) {
            switch_indicator++;
          } else {
            break;
          }
        }
        if (switch_indicator == MaxSame) {
          if (S.TrialTypeHistory[RECORD_TRIALS - 1] == 0) {
            S.TrialType = 1;
          } else {
            S.TrialType = 0;
          }
        } else { // Haven't reached MaxSame limits yet, choose at random:
          //randomSeed(analogRead(0));
          if (random(100) <= (newNoTrialProb * 100)) {
            S.TrialType = 1;
          } else {
            S.TrialType = 0;
          }
        }
        /////////////////////////////////////
        byte temp_left = 0;
        byte temp_left_correct = 0;
        byte left_num = max(Max_incorrect_Left + Min_correct_Left, 5);
        // in the last 'left_num' left trials, if there is 'Min_correct_Left' correct trials
        for (int i = RECORD_TRIALS - 1; i >= 0; i--) {
          if (S.TrialTypeHistory[i] == 1) { // left trial
            temp_left++;
            if (temp_left > left_num) {
              break;
            }
            if (S.OutcomeHistory[i] == 1) { // correct
              temp_left_correct++;
            }
          }
        }
        if (temp_left_correct < Min_correct_Left) {
          S.TrialType = 1; // keep left
        }

        byte temp_right = 0;
        byte temp_right_correct = 0;
        byte right_num = max(Max_incorrect_Right + Min_correct_Right, 5);
        // in the last 'right_num' right trials, if there is no 'Min_correct_Right' correct trials, then keep right
        for (int i = RECORD_TRIALS - 1; i >= 0; i--) {
          if (S.TrialTypeHistory[i] == 0) { // right trial
            temp_right++;
            if (temp_right > right_num) {
              break;
            }
            if (S.OutcomeHistory[i] == 1) { // correct
              temp_right_correct++;
            }
          }
        }
        if (temp_right_correct < Min_correct_Right) {
          S.TrialType = 0; // keep right
        }
        /////////////////////////////////////
      }
      break;

    case 3: // random
      S.TrialType = random(2);
      break;

    case 4: { // Autolearn     'Off'
        // switch trial types only when reaching MaxSame trials (WRONG or CORRECT)
        for (int i = RECORD_TRIALS - 1; i >= RECORD_TRIALS - MaxSame; i--) {
          if (S.TrialTypeHistory[i] == S.TrialTypeHistory[RECORD_TRIALS - 1]) {
            switch_indicator++;
          } else {
            break;
          }
        }
        if (switch_indicator == MaxSame) {
          if (S.TrialTypeHistory[RECORD_TRIALS - 1] == 0) {
            S.TrialType = 1;
          } else {
            S.TrialType = 0;
          }
        } else { // Haven't reached MaxSame limits yet, choose at random:
          //randomSeed(analogRead(0));
          if (random(100) <= (NoTrialProb * 100)) {
            S.TrialType = 1;
          } else {
            S.TrialType = 0;
          }
        }
      }
      break;

    case 5: { // Autolearn     'fixed'
        byte sides_sum;
        sides_sum = (byte)compare_array_sum(S.TrialTypeHistory, 0, RECORD_TRIALS - MaxSame, RECORD_TRIALS); //114 charcode for 'r', 108 for 'l'
        if (sides_sum == MaxSame) {
          S.TrialType = 1;
        } else if (sides_sum == 0) {
          S.TrialType = 0;
        } else {
          S.TrialType = S.TrialTypeHistory[RECORD_TRIALS - 1];
        }
      }
      break;

    default:
      break;
  }
}

void switch_fixation_state() {
  switch (timer_state) {
    case CHECK_TRIG:
      if (SwitchL_CurrentStatus == 0 && SwitchR_CurrentStatus == 0 && (SwitchL_LastStatus == 1 || SwitchR_LastStatus == 1 )) {
        // switch triggered
        F.trig_counter++;
        para_F_changed = 1;

        Ev.events_id[Ev.events_num] = 7; // switch_Triggered;
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = -1;
        Ev.events_num ++;

        SerialUSB.print("M: Switch triggered NO. ");
        SerialUSB.println(F.trig_counter);
        if (F.trig_counter > trigger_num_before_fix && S.FB_motor_position == 0) {
          timer_state = DELAY_TO_FIX;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("DELAY_TO_FIX");
          last_state_time = millis();
        }
      }
      break;

    case DELAY_TO_FIX: // delay a little bit to let the headbar move to the position (not used)
      if (millis() - last_state_time > 0) {
        timer_state = HEAD_FIXATION;
        analogWrite(DAC0, regulatorVal_min);
        //SerialUSB.print("Headfixation state changed to: ");
        //SerialUSB.println("HEAD_FIXATION");
      }
      break;

    case HEAD_FIXATION: {
        analogWrite(DAC0, regulatorVal_min);
        is_max_pressure = 0;

        //          for (int i = 0; i < 39; i++) {
        //            weighting_info[i] = weighting_info[i + 1];
        //          }


        F.headfixation_counter ++;
        para_F_changed = 1;

        if (headfixation_again == 1) {
          headfixation_again = 0;
          Ev.events_id[Ev.events_num] = 11; //  headfixation Again;
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = regulatorVal_min;
          Ev.events_num ++;

          //            weighting_info[39] = 2000;
          //            weight_counter++;

          SerialUSB.print("M: Head fixation (again) NO. ");
          SerialUSB.print(F.headfixation_counter);
          SerialUSB.print(", interval: ");
          SerialUSB.println(F.fixation_duration);

          headfixation_flag = 1;
          last_state_time = millis();
          timer_state = CHECK_RELEASE;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("CHECK_RELEASE");
        } else {
          Ev.events_id[Ev.events_num] = 8; //  headfixation;
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = regulatorVal_min;
          Ev.events_num ++;

          //            weighting_info[39] = 1000;
          //            weight_counter++;

          SerialUSB.print("M: Head fixation NO. ");
          SerialUSB.print(F.headfixation_counter);
          SerialUSB.print(", interval: ");
          SerialUSB.println(F.fixation_duration);
          if (F.fixation_duration > 10000) { // behavioral protocol: S.ProtocolHistoryIndex > 1
            last_state_time = millis();
            timer_state = HEAD_FIXATION_DELAY;
            //SerialUSB.print("Headfixation state changed to: ");
            //SerialUSB.println("HEAD_FIXATION_DELAY");
          } else {
            headfixation_flag = 1;
            last_state_time = millis();
            timer_state = CHECK_RELEASE;
            //SerialUSB.print("Headfixation state changed to: ");
            //SerialUSB.println("HEAD_FIXATION_DELAY");
          }
        }
      }
      break;

    case HEAD_FIXATION_DELAY:
      if (millis() - last_state_time > 1000) {
        if (SwitchL_CurrentStatus == 0 && SwitchR_CurrentStatus == 0) { // not escaped
          headfixation_flag = 1;
        }
        last_state_time = millis() - 1000;
        timer_state = CHECK_RELEASE;
        //SerialUSB.print("Headfixation state changed to: ");
        //SerialUSB.println("CHECK_RELEASE");
      }
      break;

    case CHECK_RELEASE: {
        if (F.fixation_duration > 10000 && is_max_pressure == 0 &&  (millis() - last_state_time) > fixation_interval_thres1) {
          analogWrite(DAC0, regulatorVal_max);
          is_max_pressure = 1;
        }
        if (S.struggle_enable > 0 && ((weight_tmp < F.struggle_thres_neg && weight_tmp > -30) || (weight_tmp > F.struggle_thres_pos && weight_tmp < 60))) {
          // struggle detected
          analogWrite(DAC0, regulatorVal_release);
          headfixation_flag = 0;

          //            for (int i = 0; i < 39; i++) {
          //              weighting_info[i] = weighting_info[i + 1];
          //            }
          //            weighting_info[39] = -2000;
          //            weight_counter++;


          Ev.events_id[Ev.events_num] = 12; // headfixation release3: struggle
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = (millis() - last_state_time);
          Ev.events_num ++;

          SerialUSB.print("M: Released - struggle!, interval: ");
          SerialUSB.print(Ev.events_value[Ev.events_num - 1] / 1000);
          SerialUSB.print("; ");
          SerialUSB.print(weight_tmp);
          SerialUSB.println(" g, DETECTED.");
          timer_state = STRUGGLE_DELAY_1000;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("STRUGGLE_DELAY_1000");
          // stop the trial
//          if (S.ProtocolHistoryIndex > 0) {
//            if (protocol_sent == 1) {
//              protocol_sent = 0;
//              Serial1.write('X'); // stop Bpod
//            }
//          }
          last_state_time = millis();
        } else if (SwitchL_CurrentStatus == 1 || SwitchR_CurrentStatus == 1) {
          // escape
          analogWrite(DAC0, regulatorVal_release);
          headfixation_flag = 0;

          //            for (int i = 0; i < 39; i++) {
          //              weighting_info[i] = weighting_info[i + 1];
          //            }
          //            weighting_info[39] = -1500;
          //            weight_counter++;

          Ev.events_id[Ev.events_num] = 10; // headfixation release2: escape
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = millis() - last_state_time;
          Ev.events_num ++;

          if (Ev.events_value[Ev.events_num - 1] > 1500) {
            for (int i = 0; i < 19; i++) {
              F.Fixation_Outcome[i] = F.Fixation_Outcome[i + 1];
            }
            F.Fixation_Outcome[19] = 10;
            para_F_changed = 1;
          }

          SerialUSB.print("M: Released - escape!, interval: ");
          SerialUSB.print(Ev.events_value[Ev.events_num - 1] / 1000);
          SerialUSB.print("; ");
          SerialUSB.print("Performance: ");
          SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
          SerialUSB.println("/20");
          timer_state = ESCAPE_DELAY_1000;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("ESCAPE_DELAY_1000");
          // stop the trial
//          if (S.ProtocolHistoryIndex > 0) {
//            if (protocol_sent == 1) {
//              protocol_sent = 0;
//              Serial1.write('X'); // stop Bpod
//            }
//          }
          last_state_time = millis();
        } else if (millis() - last_state_time > F.fixation_duration) {
          // timeup
          if (S.ProtocolHistoryIndex < 1 || protocol_sent == 0 || (millis() - last_state_time > F.fixation_duration + 5000)) {
            analogWrite(DAC0, regulatorVal_release);
            headfixation_flag = 0;

            //              for (int i = 0; i < 39; i++) {
            //                weighting_info[i] = weighting_info[i + 1];
            //              }
            //              weighting_info[39] = -1000;
            //              weight_counter++;

            Ev.events_id[Ev.events_num] = 9; // headfixation release1: timeup
            Ev.events_time[Ev.events_num] = millis();
            Ev.events_value[Ev.events_num] = millis() - last_state_time;
            Ev.events_num ++;

            for (int i = 0; i < 19; i++) {
              F.Fixation_Outcome[i] = F.Fixation_Outcome[i + 1];
            }
            F.Fixation_Outcome[19] = 9;
            para_F_changed = 1;

            SerialUSB.print("M: Released - timeup! ");
            SerialUSB.print("Performance: ");
            SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
            SerialUSB.println("/20");
            timer_state = TIMEUP_DELAY_1000;
            //SerialUSB.print("Headfixation state changed to: ");
            //SerialUSB.println("TIMEUP_DELAY_1000");
            last_state_time = millis();
          }
        }
      }
      break;

    case STRUGGLE_DELAY_1000:
      if (millis() - last_state_time > 1000) {
        // Check if there is too much struggle; if so, increase the threshold
        if (compare_array_sum(F.Fixation_Outcome, 12, 0, 20) > 8 && F.headfixation_counter - F.last_advance_threshold_counter >= fixation_advance_step) {
          // struggle rate >= 50% -> increase threshold
          F.struggle_thres_neg -= 2;
          if (F.struggle_thres_neg < -18) {
            F.struggle_thres_neg = -18;
          }
          F.struggle_thres_pos += 2;
          if (F.struggle_thres_pos > 48) {
            F.struggle_thres_pos = 48;
          }
          F.last_advance_threshold_counter = F.headfixation_counter;
          para_F_changed = 1;
        } else if (compare_array_sum(F.Fixation_Outcome, 12, 0, 20) < 2 && F.headfixation_counter - F.last_advance_threshold_counter >= fixation_advance_step && S.ProtocolHistoryIndex < 12) {
          // if there is no struggle in last 20 trials -> decrease threshold
          F.struggle_thres_neg += 2;
          if (F.struggle_thres_neg > -2) {
            F.struggle_thres_neg = -2;
          }
          F.struggle_thres_pos -= 2;
          if (F.struggle_thres_pos < 32) {
            F.struggle_thres_pos = 32;
          }
          F.last_advance_threshold_counter = F.headfixation_counter;
          para_F_changed = 1;
        }

        if (SwitchL_CurrentStatus == 0 && SwitchR_CurrentStatus == 0) {
          timer_state = HEAD_FIXATION;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("HEAD_FIXATION");
        } else {
          for (int i = 0; i < 19; i++) {
            F.Fixation_Outcome[i] = F.Fixation_Outcome[i + 1];
          }
          F.Fixation_Outcome[19] = 12;
          para_F_changed = 1;

          timer_state = CHECK_TRIG;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("CHECK_TRIG");
        }

        SerialUSB.print("M: Performance: ");
        SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
        SerialUSB.println("/20");
      }
      break;

    case ESCAPE_DELAY_1000: // delay a little bit to go back to CHECK_TRIG
      if (millis() - last_state_time > 1000) {
        //          // if timeup percentage < 25%; decreasing fixation duration
        //          if (F.fixation_duration > fixation_interval_min && compare_array_sum(F.Fixation_Outcome, 9, 0, 20) < 5 && F.headfixation_counter - F.last_advance_headfixation_counter >= fixation_advance_step) {
        //            para_F_changed = 1;
        //
        //            F.last_advance_headfixation_counter = F.headfixation_counter;
        //
        //            F.fixation_duration = F.fixation_duration - fixation_interval_step;
        //            if (F.fixation_duration < fixation_interval_min) {
        //              F.fixation_duration = fixation_interval_min; // min head fixation time 15 sec
        //            }
        //          }

        // Transit state
        timer_state = CHECK_TRIG;
        //SerialUSB.print("Headfixation state changed to: ");
        //SerialUSB.println("CHECK_TRIG");
      }
      break;

    case TIMEUP_DELAY_1000: // delay some time to see if the switch is still pressed, if so, head fixation again!
      if (millis() - last_state_time > 1000) {
        // determine if advance parameter (performance based)
        if (F.fixation_duration < F.fixation_interval_max && compare_array_sum(F.Fixation_Outcome, 9, 0, 20) >= 12 && F.headfixation_counter - F.last_advance_headfixation_counter >= fixation_advance_step) {
          // performance > 60%; increasing fixation duration
          para_F_changed = 1;

          F.last_advance_headfixation_counter = F.headfixation_counter;

          F.fixation_duration = F.fixation_duration + fixation_interval_step;
          if (F.fixation_duration > F.fixation_interval_max) {
            F.fixation_duration = F.fixation_interval_max; // max head fixation time 30 sec
          }
        }

        if (SwitchL_CurrentStatus == 0 && SwitchR_CurrentStatus == 0) {
          headfixation_again = 1;
          timer_state = HEAD_FIXATION;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("HEAD_FIXATION");
        } else {
          timer_state = CHECK_TRIG;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("CHECK_TRIG");
        }
      }
      break;

    default:
      break;
  }
}



/**************************************************************************************************************/
/********************************************** SD related Functions *******************************************/
/**************************************************************************************************************/
int write_SD_para_F() {
  File dataFile = SD.open("paraF.txt", O_WRITE);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.write((byte*)&F.trig_counter, sizeof(int));
    dataFile.write((byte*)&F.fixation_duration, sizeof(unsigned int));
    dataFile.write((byte*)&F.last_advance_headfixation_counter, sizeof(int));
    for (int i = 0; i < 20; i++) {
      dataFile.write(F.Fixation_Outcome[i]);
    }
    dataFile.write((byte*)&F.struggle_thres_neg, sizeof(int));
    dataFile.write((byte*)&F.struggle_thres_pos, sizeof(int));
    dataFile.write((byte*)&F.last_advance_threshold_counter, sizeof(int));
    dataFile.write((byte*)&F.headfixation_counter, sizeof(int));
    dataFile.write((byte*)&F.fixation_interval_max, sizeof(int));
  } else {
    SerialUSB.println("E: error opening paraF.txt");
    return -1;
  }
  dataFile.close();
  return 0;
}

int read_SD_para_F() {
  //noInterrupts();
  File dataFile = SD.open("paraF.txt", O_READ);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.read((byte*)&F.trig_counter, sizeof(int));
    dataFile.read((byte*)&F.fixation_duration, sizeof(unsigned int));
    dataFile.read((byte*)&F.last_advance_headfixation_counter, sizeof(int));
    for (int i = 0; i < 20; i++) {
      F.Fixation_Outcome[i] = dataFile.read();
    }
    dataFile.read((byte*)&F.struggle_thres_neg, sizeof(int));
    dataFile.read((byte*)&F.struggle_thres_pos, sizeof(int));
    dataFile.read((byte*)&F.last_advance_threshold_counter, sizeof(int));
    dataFile.read((byte*)&F.headfixation_counter, sizeof(int));
    dataFile.read((byte*)&F.fixation_interval_max, sizeof(int));
  } else {
    SerialUSB.println("E: error opening paraF.txt");
    return -1;
  }
  dataFile.close();
  //interrupts();
  return 0;
}

int write_SD_para_S() {
  //noInterrupts();
  File dataFile = SD.open("paraS.txt", O_WRITE);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.write((byte*)&S.currentTrialNum, sizeof(unsigned int));
    dataFile.write((byte*)&S.FB_motor_position, sizeof(int));
    dataFile.write(S.ProtocolType);
    dataFile.write(S.Autolearn);
    dataFile.write(S.TrialType);
    dataFile.write((byte*)&S.random_delay_duration, sizeof(int));
    dataFile.write(S.Trial_Outcome);

    dataFile.write((byte*)&S.SamplePeriod, sizeof(float));
    dataFile.write((byte*)&S.DelayPeriod, sizeof(float));
    dataFile.write((byte*)&S.TimeOut, sizeof(float));
    dataFile.write(S.ProtocolHistoryIndex);
    for (int i = 0; i < 20; i++) {
      dataFile.write(S.ProtocolHistory[i].Protocol);
      dataFile.write((byte*)&S.ProtocolHistory[i].nTrials, sizeof(unsigned int));
      dataFile.write(S.ProtocolHistory[i].performance);
    }

    dataFile.write(S.GaveFreeReward.flag_R_water);
    dataFile.write(S.GaveFreeReward.flag_L_water);
    dataFile.write((byte*)&S.GaveFreeReward.past_trials, sizeof(unsigned int));

    for (int i = 0; i < RECORD_TRIALS; i++) {
      dataFile.write(S.ProtocolTypeHistory[i]);
      dataFile.write(S.TrialTypeHistory[i]);
      dataFile.write(S.OutcomeHistory[i]);
    }

    dataFile.write(S.struggle_enable);
    dataFile.write((byte*)&S.totoal_reward_num, sizeof(unsigned int));
    dataFile.write(S.retract_times);
    dataFile.write(S.LR_motor_position);
    dataFile.write(S.FB_final_position);
  } else {
    SerialUSB.println("E: error opening paraS.txt");
    return -1;
  }
  dataFile.close();
  //interrupts();
  return 0;
}

int read_SD_para_S() {
  //noInterrupts();
  File dataFile = SD.open("paraS.txt", O_READ);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.read((byte*)&S.currentTrialNum, sizeof(unsigned int));
    dataFile.read((byte*)&S.FB_motor_position, sizeof(int));
    S.ProtocolType = dataFile.read();
    S.Autolearn = dataFile.read();
    S.TrialType = dataFile.read();
    dataFile.read((byte*)&S.random_delay_duration, sizeof(int));
    S.Trial_Outcome = dataFile.read();

    dataFile.read((byte*)&S.SamplePeriod, sizeof(float));
    dataFile.read((byte*)&S.DelayPeriod, sizeof(float));
    dataFile.read((byte*)&S.TimeOut, sizeof(float));
    S.ProtocolHistoryIndex = dataFile.read();
    for (int i = 0; i < 20; i++) {
      S.ProtocolHistory[i].Protocol = dataFile.read();
      dataFile.read((byte*)&S.ProtocolHistory[i].nTrials, sizeof(unsigned int));
      S.ProtocolHistory[i].performance = dataFile.read();
    }

    S.GaveFreeReward.flag_R_water = dataFile.read();
    S.GaveFreeReward.flag_L_water = dataFile.read();
    dataFile.read((byte*)&S.GaveFreeReward.past_trials, sizeof(unsigned int));

    for (int i = 0; i < RECORD_TRIALS; i++) {
      S.ProtocolTypeHistory[i] = dataFile.read();
      S.TrialTypeHistory[i] = dataFile.read();
      S.OutcomeHistory[i] = dataFile.read();
    }

    S.struggle_enable = dataFile.read();
    dataFile.read((byte*)&S.totoal_reward_num, sizeof(unsigned int));
    S.retract_times = dataFile.read();
    S.LR_motor_position = dataFile.read();
    S.FB_final_position = dataFile.read();
  } else {
    SerialUSB.println("E: error opening paraS.txt");
    return -1;
  }
  dataFile.close();
  //interrupts();
  return 0;
}

// Write Fixation Events to SD card
int write_SD_event_fixation() {
  File dataFile = SD.open("events.txt", O_CREAT | O_APPEND | O_WRITE);
  if (dataFile) {
    for (int i = 0; i < Ev.events_num; i++) {
      dataFile.print(Ev.events_time[i]);
      dataFile.print(" ");
      dataFile.print(Ev.events_id[i]);
      dataFile.print(" ");
      dataFile.println(Ev.events_value[i]);
    }
  } else {
    SerialUSB.println("E: error opening events.txt");
    return -1;
  }
  dataFile.close();
  return 0;
}

// Write Trial Events to SD card
int write_SD_trial_info() {
  //noInterrupts();
  File dataFile = SD.open("trials.txt", O_CREAT | O_APPEND | O_WRITE);
  if (dataFile) {
    now = rtc.now();
    dataFile.print(now.unixtime());
    dataFile.print(" ");
    dataFile.print(S.currentTrialNum);
    dataFile.print(" ");
    dataFile.print(S.ProtocolType);
    dataFile.print(" ");
    dataFile.print(S.ProtocolHistoryIndex);
    dataFile.print(" ");
    dataFile.print(S.TrialType);
    dataFile.print(" ");
    if (S.TrialType == 2 && S.Trial_Outcome == 1) {
      if (either_left_right == 1) {
        dataFile.print(4);// left: 4
      } else {
        dataFile.print(5);// right: 5
      }
    } else {
      dataFile.print(S.Trial_Outcome);
    }
    // print optostim info
    dataFile.print(" ");
    dataFile.print(trial_stim_index);

    // print F/B and L/R motor position todo... FB_motor_position
    dataFile.print(" ");
    dataFile.print(S.FB_motor_position);
    dataFile.print(" ");
    dataFile.print(S.LR_motor_position);

    for (int i = 0; i < nTransition; i++) {
      dataFile.print(" ");
      dataFile.print(state_visited[i]);
    }

    dataFile.println();
  } else {
    SerialUSB.println("E: error opening trials.txt");
    return -1;
  }
  dataFile.close();
  //interrupts();
  return 0;
}

int send_pars_S_to_PC() {
  //noInterrupts();
  SerialUSB.write('S'); // Event and timestamp
  SerialUSB.write((byte*)&S.currentTrialNum, sizeof(unsigned int));
  SerialUSB.write((byte*)&S.FB_motor_position, sizeof(int));
  SerialUSB.write(S.ProtocolType);
  SerialUSB.write(S.Autolearn);
  SerialUSB.write(S.TrialType);
  SerialUSB.write((byte*)&S.random_delay_duration, sizeof(int));
  SerialUSB.write(S.Trial_Outcome);

  SerialUSB.write((byte*)&S.SamplePeriod, sizeof(float));
  SerialUSB.write((byte*)&S.DelayPeriod, sizeof(float));
  SerialUSB.write((byte*)&S.TimeOut, sizeof(float));
  SerialUSB.write(S.ProtocolHistoryIndex);
  /*
    for (int i = 0; i < 20; i++) {
    SerialUSB.write(S.ProtocolHistory[i].Protocol);
    SerialUSB.write((byte*)&S.ProtocolHistory[i].nTrials, sizeof(unsigned int));
    SerialUSB.write(S.ProtocolHistory[i].performance);
    }
  */
  SerialUSB.write(S.GaveFreeReward.flag_R_water);
  SerialUSB.write(S.GaveFreeReward.flag_L_water);
  SerialUSB.write((byte*)&S.GaveFreeReward.past_trials, sizeof(unsigned int));

  /* not necessary for PC
    for (int i = 0; i < RECORD_TRIALS; i++) {
  	SerialUSB.write(S.ProtocolTypeHistory[i]);
  	SerialUSB.write(S.TrialTypeHistory[i]);
  	SerialUSB.write(S.OutcomeHistory[i]);
    }
  */

  //SerialUSB.write(S.struggle_enable);
  //SerialUSB.write((byte*)&S.totoal_reward_num, sizeof(unsigned int));
  //SerialUSB.write(S.retract_times);

  //interrupts();
  return 0;
}

/*
   Arguments:
      str_array: The string array pointer that points to the search space.
      array_length: The length of the search array.
      target: The target string

   Returns:
      It will return the index of the target string in the array.
      It will return -1 if the target is not found.

   Effects:
      This function will not mutate anything. Only query action is applied.
*/
int find_idx(const String * str_array, int array_length, String target) {
  for (int i = 0; i < array_length; i++) {
    if (target.compareTo(str_array[i]) == 0) {
      return i;
    }
  }
  return -1;
}

/*
   Arguments:
      sma: A pointer to the state machine.
      statename: The name of the referred state.

   Returns:
      0 if successful.
      -1 if failed.

   Effects:
      This function will add a blank state to the state machine.
      If the state is exit, this function will do nothing.
*/
int AddBlankState(StateMatrix * sma, String statename) {

  if (statename.compareTo("exit") == 0) {
    return 0;
  }
  sma->StateNames[sma->nStates] = statename;
  sma->StatesDefined[sma->nStates] = 0;
  sma->nStates++;
  return 0;
}

/*
   Arguments:
      Name: The name of the state.
      TimeOut: A float variable indicating the state timeout.
      nChangeConditions: The number of state transitions defined.
      StateChangeConditions: The pointer to the state transitions.
      nOutputActions: The number of output actions.
      Outputs: The pointer to the output actions.

   Returns:
      A constructed state.

   Effects:
      This function will construct a state struct defined by the input arguments.
*/
State CreateState(String Name, float TimeOut,
                  int nChangeConditions, StateChange * StateChangeConditions,
                  int nOutputActions, OutputAction * Outputs) {
  //Serial.println(Name);
  State newState;
  newState.Name = Name;
  newState.StateTimer = TimeOut;
  newState.nStateChangeConditions = nChangeConditions;
  newState.StateChangeCondition = StateChangeConditions;
  newState.nOutputs = nOutputActions;
  newState.Output = Outputs;
  return newState;
}

/*
   Arguments:
        sma: pointer to a state machine.
        state: pointer to a state to be added.

   Returns:
        0 if success.
        -1 if failure.

   Effects:
        Add a state to state machine.
        (A port from AddState.m)
*/
int AddState(StateMatrix * sma, State * state) {
  //Serial.println(state->Name);
  // Check whether the new state has already been referenced. Add new blank state to matrix.
  int CurrentState;
  int referred = 0;
  for (int i = 0; i < sma->nStates; i++) {
    // Exit if state is already defined.
    if (sma->StateNames[i].compareTo(state->Name) == 0 && sma->StatesDefined[i] == 1) {
      return -1;
    }

    // Check if the state is already referred.
    if (sma->StateNames[i].compareTo(state->Name) == 0 && sma->StatesDefined[i] == 0) {
      referred = 1;
      CurrentState = i;
    }
  }

  if (referred == 0) {
    if (sma->StateNames[0].compareTo("Placeholder") == 0) {
      CurrentState = 0;
    } else {
      CurrentState = sma->nStates;
    }
    // Increment states count.{
    sma->nStates++;
    sma->StateNames[CurrentState] = state->Name;
  }

  sma->StatesDefined[CurrentState] = 1;

  int OrigNStatesChangeConditions = state->nStateChangeConditions;


  // Make sure all the states in "StateChangeConditions" exist, and if not, create them as undefined states.
  for (int i = 0; i < OrigNStatesChangeConditions; i++) {
    int flag = 0;
    for (int j = 0; j < sma->nStates; j++) {
      if (state->StateChangeCondition[i].StateChangeTarget.compareTo(sma->StateNames[j]) == 0) {
        flag = 1;
      }
    }
    if (flag == 0) {
      AddBlankState(sma, state->StateChangeCondition[i].StateChangeTarget);
    }

    for (int i = 0; i < 40; i++) {
      sma->InputMatrix[CurrentState][i] = CurrentState;
    }
    for (int i = 0; i < 5; i++) {
      sma->GlobalTimerMatrix[CurrentState][i] = CurrentState;
      sma->GlobalCounterMatrix[CurrentState][i] = CurrentState;
    }

    // Add state transitions.
    for (int i = 0; i < state->nStateChangeConditions; i++) {
      int CandidateEventCode = find_idx(EventNames, 50, state->StateChangeCondition[i].StateChangeTrigger);
      if (CandidateEventCode < 0) {
        sma->nStates--;
        return -1;
      }
      String TargetState = state->StateChangeCondition[i].StateChangeTarget;
      int TargetStateNumber = 0;
      if (TargetState.compareTo("exit") == 0) {
        TargetStateNumber = sma->nStates; //;
      } else {
        TargetStateNumber = find_idx(sma->StateNames, sma->nStates, TargetState);
      }

      if (CandidateEventCode > 39) {
        String CandidateEventName = state->StateChangeCondition[i].StateChangeTrigger;
        if (CandidateEventName.length() > 4) {
          if (CandidateEventName.endsWith("_End")) {
            if (CandidateEventCode < 45) {
              int GlobalTimerNumber = CandidateEventName.substring(CandidateEventName.length() - 5, CandidateEventName.length() - 4).toInt();
              if (GlobalTimerNumber < 1 || GlobalTimerNumber > 5) {
                sma->nStates--;
                return -1;
              }
              sma->GlobalTimerMatrix[CurrentState][GlobalTimerNumber] = TargetStateNumber;
            } else {
              int GlobalCounterNumber = CandidateEventName.substring(CandidateEventName.length() - 5, CandidateEventName.length() - 4).toInt();
              if (GlobalCounterNumber < 1 || GlobalCounterNumber > 5) {
                sma->nStates--;
                return -1;
              }
              sma->GlobalTimerMatrix[CurrentState][GlobalCounterNumber] = TargetStateNumber;
            }
          } else {
            sma->nStates--;
            return -1;
          }
        }
      } else {
        sma->InputMatrix[CurrentState][CandidateEventCode] = TargetStateNumber;
      }
    }


    // Add output actions.
    for (int i = 0; i < 17; i++) {
      sma->OutputMatrix[CurrentState][i] = 0;
    }
    for (int i = 0; i < state->nOutputs; i++) {
      int MetaAction = find_idx(MetaActions, 4, state->Output[i].OutputType);
      if (MetaAction >= 0) {
        int Value = state->Output[i].Value;
        switch (MetaAction) {
          case 1: { // VALVE
              Value = pow(2, Value - 1);
              sma->OutputMatrix[CurrentState][1] = Value;
            }
            break;
          case 2: { // LED
              sma->OutputMatrix[CurrentState][9 + Value] = 255;
              break;
            }
          case 3: // LED STATE
            break;
        }
      } else {
        int TargetEventCode = find_idx(OutputActionNames, 17, state->Output[i].OutputType);
        if (TargetEventCode >= 0) {
          int Value = state->Output[i].Value;
          sma->OutputMatrix[CurrentState][TargetEventCode] = Value;
        } else {
          sma->nStates--;
          return -1;
        }
      }
    }

    // Add self timer.
    sma->StateTimers[CurrentState] = state->StateTimer;
  }

  // Return 0 if success.
  return 0;
}


/**
   Arguments:
      sma - Pointer to the state machine

   Returns:
      0 if successful.
      -1 if failed.

   Effect:
      This function will add the exit code to the state machine.
      After that, it will send the state machine to Bpod in bytes.
*/
int SendStateMatrix(StateMatrix * sma) {
  // clear serial
  if (Serial1.available()) {
    delay(1000);
    while (Serial1.available()) {
      Serial1.read(); // clear serial1 dirty data
    }
  }

  //SerialUSB.println("Start Sending.");
  byte stateNum = sma->nStates;
  byte output[2048];
  int index = 0;

  output[index++] = 'P';

  output[index++] = stateNum;

  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 40; j++) {
      // output[index++] = (sma->InputMatrix[i][j] == 255) ? stateNum : sma->InputMatrix[i][j];
      output[index++] = sma->InputMatrix[i][j];
    }
  }

  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 17; j++) {
      output[index++] = sma->OutputMatrix[i][j];
    }
  }

  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 5; j++) {
      output[index++] = sma->GlobalTimerMatrix[i][j];
    }
  }

  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 5; j++) {
      output[index++] = sma->GlobalCounterMatrix[i][j];
    }
  }

  for (int i = 0; i < 5; i++) {
    output[index++] = sma->GlobalCounterEvents[i];
  }

  for (int i = 0; i < 8; i++) {
    output[index++] = PortInputsEnabled[i];
  }

  for (int i = 0; i < 4; i++) {
    output[index++] = WireInputsEnabled[i];
  }

  for (int i = 0; i < stateNum; i++) {
    unsigned long ConvertedTimer = sma->StateTimers[i] * TimerScaleFactor;
    ForthByte = (ConvertedTimer & 0xff000000UL) >> 24;
    ThirdByte = (ConvertedTimer & 0x00ff0000UL) >> 16;
    SecondByte = (ConvertedTimer & 0x0000ff00UL) >> 8;
    LowByte = (ConvertedTimer & 0x000000ffUL);
    output[index++] = LowByte;
    output[index++] = SecondByte;
    output[index++] = ThirdByte;
    output[index++] = ForthByte;
  }

  for (int i = 0; i < 5; i++) {
    unsigned long ConvertedTimer = sma->GlobalTimers[i] * TimerScaleFactor;
    ForthByte = (ConvertedTimer & 0xff000000UL) >> 24;
    ThirdByte = (ConvertedTimer & 0x00ff0000UL) >> 16;
    SecondByte = (ConvertedTimer & 0x0000ff00UL) >> 8;
    LowByte = (ConvertedTimer & 0x000000ffUL);
    output[index++] = LowByte;
    output[index++] = SecondByte;
    output[index++] = ThirdByte;
    output[index++] = ForthByte;
  }

  for (int i = 0; i < 5; i++) {
    ForthByte = (sma->GlobalCounterThresholds[i] & 0xff000000UL) >> 24;
    ThirdByte = (sma->GlobalCounterThresholds[i] & 0x00ff0000UL) >> 16;
    SecondByte = (sma->GlobalCounterThresholds[i] & 0x0000ff00UL) >> 8;
    LowByte = (sma->GlobalCounterThresholds[i] & 0x000000ffUL);
    output[index++] = LowByte;
    output[index++] = SecondByte;
    output[index++] = ThirdByte;
    output[index++] = ForthByte;
  }

  //SerialUSB.println(index); // 1195 for Protocol 5
  //SerialUSB.println(millis());
  for (int i = 0; i < index; i++) {
    //while (Serial1.availableForWrite() == 0) {
    //SerialUSB.println("Buffer maxed.");
    //}
    Serial1.write(output[i]);
    //Serial1.flush();
    //delayMicroseconds(10);
  }
  //SerialUSB.println(millis());

  //SerialUSB.println("Done Sending.");
  byte returnVal = Serial1ReadByte();
  if (returnVal != 1) {
    // SerialUSB.print(returnVal);
    SerialUSB.println("E: SendStateMachine failed.");
    return -1;
  }
  return 0;
}

/*
   Arguments:
      N/A

   Returns:
      0 if successful.
      -1 if failed.

   Effects:
      This function will simply request the Bpod to run the loaded state machine.
      Meanwhile, it will read the Bpod event logs.
*/
int RunStateMatrix() {
  // Sending indicator 'R'
  Serial1.write('R');
  byte returnVal = Serial1ReadByte();
  if ( returnVal != 1) {
    // SerialUSB.print(returnVal);
    SerialUSB.println("E: Failed attempt to run state matrix (!= 1)");
    return -1;
  }
  return 0;
}



void digitalWriteDirect(int pin, boolean val) {
  if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

byte digitalReadDirect(int pin) {
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

int sum_array(byte a [], int array_length) {
  int res = 0;
  for (int i = 0; i < array_length; i++) {
    res = res + a[i];
  }
  return res;
}
int compare_array_sum(byte array1 [], byte oprant1, byte array2 [], byte oprant2, int start_ind, int end_ind) {
  int num = 0;
  for (int i = start_ind; i < end_ind; i++) {
    if (array1[i] == oprant1 && array2[i] == oprant2) {
      num++;
    }
  }
  return num;
}
int compare_array_sum(byte array1 [], byte oprant1, int start_ind, int end_ind) {
  int num = 0;
  for (int i = start_ind; i < end_ind; i++) {
    if (array1[i] == oprant1) {
      num++;
    }
  }
  return num;
}

void valve_control(byte on_off) {
  Serial1.write('O');
  Serial1.write('V');
  Serial1.write(on_off); // 0: both off; 1:left open; 2:right open; 3: both open
}

// This function will read in an unsigned long number from Bpod.
unsigned long Serial1ReadLong() {
  while (Serial1.available() == 0) {}
  LowByte = Serial1.read();
  while (Serial1.available() == 0) {}
  SecondByte = Serial1.read();
  while (Serial1.available() == 0) {}
  ThirdByte = Serial1.read();
  while (Serial1.available() == 0) {}
  ForthByte = Serial1.read();
  unsigned long LongInt =  (unsigned long)(((unsigned long)ForthByte << 24) | ((unsigned long)ThirdByte << 16) | ((unsigned long)SecondByte << 8) | ((unsigned long)LowByte));
  return LongInt;
}

// This function will read in an unsigned short number from Bpod.
uint16_t Serial1ReadShort() {
  while (Serial1.available() == 0) {}
  LowByte = Serial1.read();
  while (Serial1.available() == 0) {}
  SecondByte = Serial1.read();
  uint16_t ShortInt =  (uint16_t)(((uint16_t)SecondByte << 8) | ((uint16_t)LowByte));
  return ShortInt;
}

// This function will read in a byte from Bpod.
byte Serial1ReadByte() {
  while (Serial1.available() == 0) {}
  LowByte = Serial1.read();
  return LowByte;
}

/*
  Serial.write( (uint8_t *) &x, sizeof( x ) );
  int x = Serial.read() | ( Serial.read() << 8 );
*/

byte SerialUSBReadByte() {
  while (SerialUSB.available() == 0) {}
  LowByte = SerialUSB.read();
  return LowByte;
}

// write an unsigned long number to PC
void SerialUSBWriteLong(unsigned long num) {
  SerialUSB.write((byte)num);
  SerialUSB.write((byte)(num >> 8));
  SerialUSB.write((byte)(num >> 16));
  SerialUSB.write((byte)(num >> 24));
  //delayMicroseconds(100);
}

// write an unsigned short number to PC
void SerialUSBWriteShort(word num) {
  SerialUSB.write((byte)num);
  SerialUSB.write((byte)(num >> 8));
  //delayMicroseconds(100);
}

void Incase_handler() {
  Timer4.stop();
  if (Receiving_data_from_Bpod == 1) { // get struck in receiving Bpod data
    SerialUSB.println("E: Get struck in receiving Bpod data.");
    // Send 'K' to Bpod to have Bpod send back many bytes to get controller out of struck
    Serial1.write('K');
  }
}

void printCurrentTime() {
  now = rtc.now();
  SerialUSB.print("M: ");
  SerialUSB.print(now.year(), DEC);
  SerialUSB.print('/');
  SerialUSB.print(now.month(), DEC);
  SerialUSB.print('/');
  SerialUSB.print(now.day(), DEC);
  SerialUSB.print(" ");
  SerialUSB.print(now.hour(), DEC);
  SerialUSB.print(':');
  SerialUSB.print(now.minute(), DEC);
  SerialUSB.print(':');
  SerialUSB.print(now.second(), DEC);
  SerialUSB.println();
}


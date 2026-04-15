//combine Master,task and wave board. JL 09/16/2025
/*
  Auditory/Tactile discrimination including:
  1. initial sample type can be set in sd card: sampleType 0 for Tactile, 1 for Auditory.
  2. SamplePeriod duration can be set in SD card
  3. min/max/fininal delay-period-duration and step can be set in SD card
  4. optostim stimTrail percentage can be set in SD card (stimTrial_pct)
  5. all the aim conditions in sub-protocol 19 can be set in SD card (fixblock_trialnum_aim, contingency_trial_aim, Perf100_aim, Earlylick100_aim, response_trial_aim)	
  6. In sub-protocol 19, EITHER start reversing sample type randomly, OR after meet certain condition, start reversing sample type or contingency block by block,
     accoring to parameters "reverseType" and "is_random":
     reverseType 0 for reversing sampleType, 1 for reversing contingency; is_random 0 for block by block reversing, 1 for randomly reversing sampleType only.
*/

#include <string.h>
#include <SD.h>        // SD card
#include <DueTimer.h>  // Timer interrupt
#include "RTClib.h"    // real time clock
#include "HX711.h"     // weighting amplifier libirary

#include "pwm_lib.h" // pwm lib for mask flashing   JL 11/19/2024
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#include <SPI.h>

/* Connection Circuit:
  Controller.SerialUSB <<======>> PC (data and/or debug info)
*/

//*********************************************************************************************************/
//***************************************** Head-Fixation related *****************************************/
//*********************************************************************************************************/

////////////// define finite states for head-fixation //////////////
#define CHECK_TRIG          1
#define DELAY_TO_FIX        2
#define HEAD_FIXATION       3
#define HEAD_FIXATION_DELAY 4
#define CHECK_RELEASE       5
#define ESCAPE_DELAY_1000   6
#define TIMEUP_DELAY_1000   7
#define STRUGGLE_DELAY_1000 8
#define CHECK_RELEASE0      9
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

bool protocol_sent_run = 0;

// port definition
const byte portMotorLR     = 2;  // analog write channel, Left-Right
const byte portMotorFB     = 3;  // analog write channel, Forward-Backward
const byte portMotorPole   = 4;  // analog write channel, Motorized Pole
const byte portSD          = 5;  // SD card detection: low if inserted; high if removed
const byte portSwitchL     = 22;
const byte portSwitchR     = 23;
const byte portWeight_DOUT = 7;  // weight stage data
const byte portWeight_CLK  = 6;  // weight stage clock
const byte switchPin       = 49; // ToggerSwitch pin to start/pause experiment
const byte ledPin          = 13; // LED pin
const byte chipSelect      = 10; // Adafruit SD shields and modules: pin 10


using namespace arduino_due::pwm_lib;
// for go cue sound
#define PWM_PERIOD_PIN_53 27000  //3.7khz    
#define PWM_DUTY_PIN_53   13500  // 50% duty  

//for auditory cue sound
#define PWM_PERIOD_PIN_53_H 10000 //10khz    for high freq start cue sound  
#define PWM_DUTY_PIN_53_H  5000 

#define PWM_PERIOD_PIN_53_L 50000 //2khz    for low freq start cue sound
#define PWM_DUTY_PIN_53_L  25000 
            
// for masking led             
#define PWM_PERIOD_PIN_44 10000000 // 100 ms, 10 Hz      
#define PWM_DUTY_PIN_44   100000  //  1% 
             
pwm<pwm_pin::PWMH2_PB14> pwm_pin53;
pwm<pwm_pin::PWMH5_PC19> pwm_pin44;

Adafruit_MCP4725 dac;
volatile uint16_t sample_ind = 0;
const uint16_t thorlab_flat_sine[50]  = {4079, 4031, 3951, 3842, 3704, 3540, 3353, 3145, 2919, 2680,
                                         2431, 2176, 1919, 1664, 1415, 1176, 950, 742, 555, 391, 253,
                                         144, 64, 16, 0, 16, 64, 144, 253, 391, 555, 742, 950, 1176,
                                         1415, 1664, 1919, 2176, 2431, 2680, 2919, 3145, 3353, 3540,
                                         3704, 3842, 3951, 4031, 4079, 4095
                                        };

const uint16_t thorlab_ramp_sine[200] = {4058, 3990, 3892, 3765, 3611, 3434, 3235, 3019, 2788, 2546,
                                         2297, 2045, 1794, 1547, 1309, 1082, 870, 676, 502, 352, 227,
                                         128, 57, 14, 0, 14, 56, 124, 217, 332, 469, 624, 794, 976,
                                         1167, 1364, 1564, 1763, 1957, 2144, 2321, 2484, 2632, 2761,
                                         2871, 2958, 3023, 3063, 3080, 3071, 3039, 2983, 2904, 2804,
                                         2685, 2549, 2397, 2233, 2058, 1876, 1690, 1501, 1314, 1131,
                                         955, 788, 632, 490, 363, 254, 163, 92, 41, 10, 0, 10, 40, 88,
                                         153, 235, 330, 438, 556, 682, 814, 948, 1084, 1219, 1349, 1474,
                                         1591, 1698, 1794, 1876, 1945, 1998, 2035, 2056, 2060, 2048,
                                         2019, 1975, 1916, 1844, 1759, 1664, 1559, 1447, 1328, 1206,
                                         1082, 957, 835, 715, 601, 494, 394, 304, 225, 156, 100, 56,
                                         25, 6, 0, 6, 23, 52, 90, 137, 191, 252, 318, 388, 460, 532,
                                         604, 675, 742, 804, 861, 912, 955, 991, 1019, 1037, 1047,
                                         1048, 1040, 1024, 999, 967, 929, 884, 833, 779, 721, 660,
                                         598, 536, 474, 413, 355, 299, 248, 200, 157, 119, 86, 59, 37,
                                         20, 9, 2, 0, 2, 7, 16, 27, 39, 53, 67, 81, 94, 106, 116, 125,
                                         131, 134, 134, 131, 126, 117, 106, 93, 77, 59, 40, 20, 0
                                        };

const uint16_t ultra_flat_sine[50]  = {4081, 4041, 3974, 3882, 3767, 3629, 3471, 3297, 3107, 2907,
                                       2697, 2483, 2267, 2053, 1844, 1643, 1454, 1279, 1121, 984, 868, 776,
                                       709, 669, 655, 669, 709, 776, 868, 984, 1121, 1279, 1454, 1643, 1844,
                                       2053, 2267, 2483, 2697, 2907, 3107, 3297, 3471, 3629, 3767, 3882,
                                       3974, 4041, 4081, 4095
                                      };

const uint16_t ultra_ramp_sine[200] = {4064, 4007, 3924, 3818, 3689, 3540, 3373, 3191, 2997, 2794,
                                       2585, 2373, 2162, 1955, 1754, 1564, 1386, 1223, 1077, 951, 846,
                                       763, 703, 667, 655, 667, 702, 759, 837, 934, 1049, 1179, 1322,
                                       1475, 1636, 1801, 1969, 2136, 2299, 2456, 2605, 2742, 2866, 2975,
                                       3066, 3140, 3194, 3228, 3242, 3235, 3208, 3161, 3095, 3011, 2911,
                                       2796, 2669, 2531, 2384, 2231, 2075, 1916, 1759, 1606, 1457, 1317,
                                       1186, 1067, 961, 869, 792, 732, 690, 664, 655, 664, 688, 729, 784,
                                       852, 933, 1023, 1122, 1228, 1339, 1452, 1566, 1679, 1789, 1893,
                                       1992, 2082, 2162, 2231, 2289, 2333, 2364, 2382, 2385, 2375, 2351,
                                       2314, 2265, 2204, 2133, 2053, 1965, 1870, 1771, 1668, 1564, 1459,
                                       1356, 1256, 1160, 1070, 987, 911, 844, 787, 739, 702, 676, 660, 655,
                                       660, 675, 699, 731, 770, 816, 867, 923, 981, 1041, 1102, 1163, 1222,
                                       1278, 1331, 1379, 1421, 1458, 1488, 1511, 1527, 1535, 1535, 1529,
                                       1515, 1495, 1468, 1435, 1397, 1355, 1309, 1261, 1210, 1158, 1105,
                                       1053, 1002, 953, 907, 863, 823, 787, 755, 727, 704, 686, 672, 662,
                                       657, 655, 657, 661, 668, 678, 688, 699, 711, 723, 734, 744, 753, 760,
                                       765, 768, 768, 766, 761, 754, 744, 733, 720, 705, 689, 672, 655
                                      };

byte trial_stim_index = 0; // 0:no_stim; 1:delay_stim; 2:sample_stim; etc.
byte weightByte = 10;
byte laserByte = 2; // 1-using thorlab laser; 2-using Ultra laser
float powerWeight = 1.0;
int stimTrial_pct = 20;

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
  // 1-mark arduino restarted
  // 7-switchTriggered; 8-headfixation; 11-headfixation again immediately after release
  // 9-release1:timeup; 10-release2:escape; 12-release3:struggle;
  // 13-release4:switch off;
  // 14-free reward to lure next fixation;
  // 15-arduino switch on && SD card inserted;
  // 16-arduino switch off or SD card not inserted;
  // 20-user updating parameters from GUI: portMotorFB;
  // 21-user updating parameters from GUI: portMotorLR; 
  // 22-user updating parameters from GUI: portMotorFB_final;   
  
  int events_value[20]          = {0};
} Events_fixation;

Events_fixation Ev;

// weight stage related init
HX711 scale(portWeight_DOUT, portWeight_CLK);
float calibration_factor = -12300; // by default -12300.
float weighting_info[40] = {0}; // weighting info in last 2 sec
int weight_counter       = 0;
long weight_offset    = 0;
float knownWeight = 20;  //g

boolean SwitchL_LastStatus;
boolean SwitchR_LastStatus;
boolean SwitchL_CurrentStatus;
boolean SwitchR_CurrentStatus;

int motor_retract_step               = 10; // 0-255
bool is_motor_advance                = 0;
unsigned int last_advance_reward_num = 0;
int lick_num_before_motor_retract1   = 10;
int lick_num_before_motor_retract2   = 15;
int lick_num_before_motor_retract3   = 20;

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
int regulatorVal_max     = 210;//255; // max pressure
int regulatorVal_release = 10; //30;  // slightly release to reduce retract distance and noise

int polePos;
int poleLastPos;
int portMotorPos = A0;
bool isMotorOK = 1; 
int failureCont = 0;
int failureMax = 5;
int lastFailure = 0;

//****************************************************************************************************/
//************************************* Finite State Matrix related **********************************/
//****************************************************************************************************/

// Event codes list.
const PROGMEM String EventNames[40] = {
  "Port1In", "Port1Out", "Port2In", "Port2Out", "Port3In", "Port3Out", "Port4In", "Port4Out", "Port5In", "Port5Out", "Port6In", "Port6Out", "Port7In", "Port7Out", "Port8In", "Port8Out",
  "BNC1High", "BNC1Low", "BNC2High", "BNC2Low",
  "Wire1High", "Wire1Low", "Wire2High", "Wire2Low", "Wire3High", "Wire3Low", "Wire4High", "Wire4Low",
  "SoftCode1", "SoftCode2", "SoftCode3", "SoftCode4", "SoftCode5", "SoftCode6", "SoftCode7", "SoftCode8", "SoftCode9", "SoftCode10",
  "UnUsed",
  "Tup"   
};

// Output action name list.
const PROGMEM String OutputActionNames[17] = {
  "ValveState", "BNCState", "WireState",
  "Serial1Code", "SerialUSBCode", "SoftCode", "GlobalTimerTrig", "GlobalTimerCancel", "GlobalCounterReset",
  "PWM1", "PWM2", "PWM3", "PWM4", "PWM5", "PWM6", "PWM7", "PWM8"
};

// Scalar for float timer.
const PROGMEM int TimerScaleFactor = 1000;  //1s=1000ms   

// Port and wire parameters for Bpod.
byte PortInputsEnabled[8] = {1, 1, 0, 0, 0, 0, 0, 0};

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
  String StateNames[128]                   = {"Placeholder"}; //State names in the order they were added
  byte InputMatrix[128][40]                = {};
  byte OutputMatrix[128][17]               = {};
  float StateTimers[128]                   = {};
  byte StatesDefined[128]                  = {};              //Referenced states are set to 0. Defined states are set to 1. Both occur with AddState
} StateMatrix;


OutputAction RewardOutput;
OutputAction LeftWaterOutput  = {"ValveState", 1};
OutputAction RightWaterOutput = {"ValveState", 2};

OutputAction CueOutput        = {"PWM1", 255};

OutputAction PoleOutput       = {"BNCState", 1};

OutputAction WaveSurferTrig    = {"PWM2", 255}; // Masking Flash LED trigger
OutputAction OptogeneticTrig   = {"PWM3", 255}; // Optogenetics stim trigger

OutputAction SampleCueOutput;
OutputAction SampleCueOutputH   = {"PWM4", 255};  
OutputAction SampleCueOutputL   = {"PWM5", 255};  

String LeftLickAction;
String RightLickAction;


//****************************************************************************************************/
//***************************************** Trial related *****************************************/
//****************************************************************************************************/

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
  int FB_motor_position        = 0;        // Lickport motor Forward/Backward
  byte FB_final_position       = 30;
  byte LR_motor_position       = 70;
  byte ProtocolType            = P_FIXATION;
  byte Autolearn               = 0;          // {"Either", "On", "antiBias", "random","off","fixed"}; 0-5
  byte TrialType               = 2;          // 0 right; 1 left; 2 either side
  int random_delay_duration    = 1001;       // ms
  byte Trial_Outcome           = 3;          // 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
  /////////////////////////////////////////////////

  float SamplePeriod        = 1.30; //
  float DelayPeriod         = 0.30; // 0.3->1.3
  float TimeOut             = 0.50; // 0.5->4

  byte ProtocolHistoryIndex               = 0;  // index of ProtocolHistory
  ProtocolHistoryInfo ProtocolHistory[30] = {}; // [protocol#, n_trials on this protocol, performance]

  RewardFlag GaveFreeReward = {0, 0, 0}; // [flag_R_water  flag_L_water    past trials] 	// keeps track of the number of trials since the last reward

  // Protocol type, Trial type, OutcomeHistory for recent RECORD_TRIALS=100 trials
  byte ProtocolTypeHistory[RECORD_TRIALS] = {}; // 1-5
  byte TrialTypeHistory[RECORD_TRIALS]    = {}; // 0 right; 1 left;
  byte OutcomeHistory[RECORD_TRIALS]      = {}; // // 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
  byte EarlylickHistory[RECORD_TRIALS]    = {}; // // 0-no earlylick; 1-earlylick; 2-N/
  
  byte struggle_enable = 1;
  unsigned int totoal_reward_num          = 1;
  byte retract_times = 0;

  float reward_left  = 0.03;
  float reward_right = 0.03;
  
  byte anteriorPos  = 30;
  byte posteriorPos = 100;
  /////////////////////////////////////////////////
} Parameters_behavior;

Parameters_behavior    S;

byte halfPos  = (S.anteriorPos + S.posteriorPos) / 2;
byte finalPos = S.anteriorPos;


int LickPortMove         = 0;
const byte mov_step_size = 5;

float Reward_duration_behavior = 0.03; // sec
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
int paused          = 0;
byte ledState       = 0; //LOW;
bool Receiving_data_from_Bpod  = 0;
bool sdcard = 0;

byte Earlylick100	= 0;
byte response_trial = 100;

unsigned long last_reward_time = 0;
int timed_drop_count = 0;

byte either_left_right = 2;

uint16_t nEvents;
unsigned long eventTimeStamps[1024] = {};
byte Events[1024]                   = {};
byte state_visited[1024] = {};
uint16_t nTransition;

byte contingency = 0;
unsigned int contingency_trial = 0;
unsigned int fixblock_trialnum_cont = 0;

int sampleType = 0;  //0 for Tactile; 1 for Auditory.
int fixblock_trialnum_aim = 3000; //trial number for switching sample type.

int contingency_trial_aim=3000;
byte Perf100_aim=85;
byte Earlylick100_aim=50;
byte response_trial_aim=80;
byte meetaim_cont=0;

int reverseType = 0;  // 0 reversing sampleType, 1 reversing contingency
int is_random = 0;    // 1 randomizing sampleType

float minDelay = 0.3;  //s
float maxDelay = 1.3;
int delayStep = 5;
int stepIndex = 1;

// others
RTC_PCF8523         rtc;
DateTime            now;
byte LowByte;
byte SecondByte;
byte ThirdByte;
byte ForthByte;
String buffer_tmp;
unsigned long unix_time_24;

bool isStruck = 0;

unsigned long bpod_time = 0;  	//StateMatrix running startpoint
int usbWflag = 0;               //0:message/warning needs to write to USB serial port.

////////Task board related. JL 11/19/2024
//Hardware mapping:        //
byte PortDigitalInputLines[8] = {28, 30, 32, 34, 36, 38, 40, 42};   //28,30  is currently used
byte PortPWMOutputLines[2] = {53, 44}; // pwm1: gocue; pwm2:masking flush
byte ValveDigitalOutputLines[2] = {26,27};
byte BncOutputLines[2] = {11, 12};  //11 is currently used

//Initialize system state vars: 
//byte PortPWMOutputState[8] = {0}; // State of all 8 output lines (As 8-bit PWM value from 0-255 representing duty cycle. PWM cycles at 1KHZ)
boolean PortInputLineValue[8] = {0}; // Direct reads of digital values of IR beams
boolean PortInputLineOverride[8] = {0}; // set to 1 if user created a virtual input, to prevent hardware reads until user returns it to low
boolean PortInputLineLastKnownStatus[8] = {0}; // Last known status of IR beams
boolean MatrixFinished = false; // Has the system exited the matrix (final state)?
boolean MeaningfulStateTimer = false; // Does this state's timer get us to another state when it expires?
int CurrentState = 1; // What state is the state machine currently in? (State 0 is the final state)
int NewStateIdx = 1;
int stateNum = 0;
byte CurrentEvent[10] = {0}; // What event code just happened and needs to be handled. Up to 10 can be acquired per 30us loop.
byte nCurrentEvents = 0; // Index of current event
byte InputStateMatrix[128][40] = {0}; // Matrix containing all of Bpod's inputs and corresponding state transitions
byte OutputStateMatrix[128][17] = {0}; // Matrix containing all of Bpod's output actions for each Input state
int MaxTimestamps  = 1024; // Maximum number of timestamps (to check when to start event-dropping)
int MaxTransitions = 1024;
unsigned long StateTimers[128] = {0}; // Timers for each state
unsigned long MatrixStartTime = 0; // Trial Start Time
unsigned long StateStartTime = 0; // Session Start Time
unsigned long CurrentTime = 0; // Current time (units = timer cycles since start; used to control state transitions)
unsigned long TimeFromStart = 0;
//byte RunningStateMatrix = 0; // 1 if state matrix is running
////End 11/19/2024

//watchdog
int watchdogTime = 10000;  //10s

void watchdogSetup(void)
{
// do what you want here
}


//****************************************************************************************************/
//********************************************** Setup() *********************************************/
//****************************************************************************************************/

void setup() {
  isStruck = 0;
  isMotorOK = 1;
  usbWflag=0;
	
  watchdogEnable(watchdogTime);
  
  SerialUSB.begin(115200);   // To PC for debug info and/or Data

  Timer4.attachInterrupt(timer_updateDAC); //optostim
  Timer4.setPeriod(500); // every 0.5 ms to update DAC value

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  dac.begin(0x62);
  dac.setVoltage(0, false);

////Task board related JL 11/19/2024
	for (int x = 0; x < 8; x++) {
		pinMode(PortDigitalInputLines[x], INPUT_PULLUP);
	}
	for(int x = 0; x < 2; x++) {
		pinMode(PortPWMOutputLines[x], OUTPUT);
		analogWrite(PortPWMOutputLines[x], 0);
	}	
	for (int x = 0; x < 2; x++) {
		pinMode(BncOutputLines[x], OUTPUT);
		digitalWrite(BncOutputLines[x], LOW);
	}
	for (int x = 0; x < 2; x++) {
		pinMode(ValveDigitalOutputLines[x],OUTPUT);
		digitalWrite(ValveDigitalOutputLines[x], LOW);
	}

	Timer3.attachInterrupt(handler);
	Timer3.setPeriod(100); // Runs every 100us
////End 11/19/2024

  analogReadResolution(8);   // Max: 12
  analogWriteResolution(8);

  analogWrite(DAC0, regulatorVal_release);

  pinMode(ledPin, OUTPUT);
  analogWrite(ledPin, ledState);
  
  pinMode(switchPin, INPUT_PULLUP); // low if switch on; hight if switch off

  pinMode(portSwitchL, INPUT_PULLUP);
  SwitchL_LastStatus = digitalRead(portSwitchL);
  pinMode(portSwitchR, INPUT_PULLUP);
  SwitchR_LastStatus = digitalRead(portSwitchR);
 
  pinMode(portMotorLR, OUTPUT);
  pinMode(portMotorFB, OUTPUT);
  pinMode(portMotorPole, OUTPUT);
  
  analogWrite(portMotorLR, 5);
  analogWrite(portMotorFB, 5);
  analogWrite(portMotorPole, 5);

  pinMode(portSD, INPUT_PULLUP);

  // Check if SD Card is working...
  if (digitalRead(portSD) == 1) {
	sdcard = 0;
    SerialUSB.println("E: SD Card not inserted...0");
  } else {
	sdcard = 1;
    SD.begin(chipSelect);
	read_SD_para_F();   // 4 ms
    read_SD_para_S();
    SerialUSB.println("M: SD card detected!");
  }

  // Check if the RT Clock ready
  if (! rtc.begin()) {
    SerialUSB.println("E: Couldn't find RT Clock");
  }
  if (! rtc.initialized()) { // never happen
    SerialUSB.println("M: RTC is NOT running! Adjust Time!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // write an artificial event to mark the restart of Arduino board
  Ev.events_num = 0;
  Ev.events_id[Ev.events_num] = 1; // restart;
  now = rtc.now();
  Ev.events_time[Ev.events_num] = now.unixtime();
  Ev.events_value[Ev.events_num] = -1;
  Ev.events_num = 1;
  write_SD_event_fixation(); // write event to file
  Ev.events_num = 0;

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
  scale.set_offset(weight_offset);
  scale.set_scale(calibration_factor);

  if (digitalRead(switchPin) == 0 && digitalRead(portSD) == 0) {
    paused = 0;
  } else {
    paused = 1;
  }
  
}


//****************************************************************************************************/
//********************************************** Loop() **********************************************/
//****************************************************************************************************/
void loop() {
	
	watchdogReset();
	
	if (digitalRead(portSD) == 0){
		if (sdcard ==0){
			sdcard =1;
			SD.begin(chipSelect);
			read_SD_para_F();
			read_SD_para_S();
		}
	}else{
		if (sdcard ==1){
			sdcard=0;
			if (SerialUSB.dtr() && SerialUSB_connected()) {
				SerialUSB.println("E: SD Card not inserted...1");
			}
		}
	}

  // if (Rocker_switch is ON && SD card inserted), run the state matrix
  if (digitalRead(switchPin) == 0 && digitalRead(portSD) == 0 && isMotorOK==1){

    if (paused == 1) {
      paused = 0;
	  
	  Ev.events_id[Ev.events_num] = 15; //switch on
      Ev.events_time[Ev.events_num] = millis();
      Ev.events_value[Ev.events_num] = -1;
      Ev.events_num ++;

      ledState = 255; //HIGH;
	  analogWrite(ledPin, ledState);
      if (SerialUSB.dtr() && SerialUSB_connected()) {
        SerialUSB.println("M: Program RESUME!!!");
      }

      // in case SD card was removed and re-insert, need re-initilization
      read_SD_para_F();
      read_SD_para_S();
	  
      analogWrite(portMotorLR, S.LR_motor_position);
      analogWrite(portMotorFB, S.FB_motor_position);

      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(100);
      valve_control(0); // close valve 1 and 2
	  
	  scale.set_offset(weight_offset);
	  scale.set_scale(calibration_factor);
	  
	  timer_state = CHECK_TRIG;
    }

    if (protocol_sent_run == 0) {	
      if (F.trig_counter <= trigger_num_before_fix || S.FB_motor_position > S.FB_final_position) {
        send_protocol_to_Bpod_and_Run(); //49 ms - 96 ms
      } else if (headfixation_flag == 1) { // need check if head is fixed
        send_protocol_to_Bpod_and_Run();
      }
    }

	if (MatrixFinished == true) {//a trial is done, Write events to SD card, Update trial outcome
	    MatrixFinished = false;
	    valve_control(0); // Reset valves
		stopMaskingflash();
		stopGocue();	
		digitalWrite(BncOutputLines[0], LOW); // Reset BNC outputs
		ledState = 0;
		analogWrite(ledPin, ledState); //OFF
		
        write_SD_eventST_updateOutcome(); 

		S.currentTrialNum++;

		UpdatePerformance(); // < 1 ms

		PrintResult2PC();

		write_SD_trial_info(); // log trial info to SD; 1 ms

		if (S.Trial_Outcome == 1) {
			last_reward_time = millis();  // record the last reward time
			timed_drop_count = 0;
		}
    

      //////////// for next trial ///////////
		autoChangeProtocol();    // Change protocol and parameters based on performance; < 1 ms
		autoAdjustLickportPosition(); // Adjust lickpost position if bias develops
		autoReward();            // Set Reward Flag if many wrongs in a row; < 1 ms
		trialSelection();        // determine S.TrialType; < 1 ms
		write_SD_para_S();       // write parameter S (updated after this trial) to SD card; 1 ms
		protocol_sent_run = 0;       // enable next trial
      //////////// for next trial ///////////
	}

    if (millis() - last_reward_time > 3 * 3600000) { // there is no reward in last 3 hours
      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(50);
      valve_control(0); // close valve 1 and 2
      last_reward_time = millis();
      if (SerialUSB.dtr() && SerialUSB_connected()) {
        SerialUSB.println("M: No Reward in Last 3 Hours.");
      }

      timed_drop_count++;

      if (timed_drop_count >= 4) { // in last 12 hours no reward
        timed_drop_count = 0;
        // more free reward to fill the lickport tube
        valve_control(3); //  open valve 1 and 2
        delay(70);
        valve_control(0); // close valve 1 and 2
        last_reward_time = millis();
        if (SerialUSB.dtr() && SerialUSB_connected()) {
          SerialUSB.println("E: No Reward in Last 12 Hours.");
        }

        if (F.fixation_duration <= 3000 && S.retract_times < 8 && S.FB_motor_position < 50 + S.FB_final_position) { // && F.trig_counter < 30
          // check if move the lickport closer to mouse to lure them in // todo: move back more frequent but retracting faster
          S.FB_motor_position = 50 + S.FB_final_position;
          analogWrite(portMotorFB, S.FB_motor_position);
          S.retract_times++;
          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.print("E: Motor retracted back NO. ");
            SerialUSB.println(S.retract_times);
          }
        }
      }
    }


    if (millis() - last_weight_read_time > 50) { // every 50 ms
      // read and restore weighting data
      weight_tmp = scale.get_units(); // 165 us
      last_weight_read_time = millis();
      if (weight_tmp > 10 && weight_tmp < 40 && headfixation_flag == 0) { // avoid record many zeros
		weighting_info[weight_counter] = weight_tmp;
        weight_counter++;
      }
      if (weight_counter >= 40) { // have 2-sec data in the array

        // writ to SD card // 2.5 ms?
        File dataFile = SD.open("weight.txt", O_CREAT | O_APPEND | O_WRITE);
        if (dataFile) {
          now = rtc.now();
          dataFile.print(now.unixtime());
          for (int i = 0; i < 40; i++) {
            dataFile.print(" ");
            dataFile.print(weighting_info[i]);
          }
          dataFile.println();
        } else {
          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.println("M: error opening weight.txt");
          }
        }
        dataFile.close();

        // also writ to PC // 0.3 ms
        if (SerialUSB.dtr() && SerialUSB_connected()) {
          SerialUSB.write('W');
          SerialUSBWriteShort(weight_counter);
          SerialUSB.write((byte*)&weighting_info[40 - weight_counter], sizeof(float)*weight_counter); // weighting_info = &weighting_info[0]; start from lower byte
          SerialUSB.println();
        }

        weight_counter = 0;
      }
    }


    SwitchL_CurrentStatus   = digitalRead(portSwitchL);
    SwitchR_CurrentStatus   = digitalRead(portSwitchR);
	
    // State Machine for the Head-fixation
    switch_fixation_state();

    SwitchL_LastStatus = SwitchL_CurrentStatus;
    SwitchR_LastStatus = SwitchR_CurrentStatus;
	
    // detection of release event => free water to lure next fixation
    if (headfixation_flag == 0 && last_headfixation_flag == 1 && F.fixation_duration < 15000) {
      // free reward to fill the lickport tube
      valve_control(3); //  open valve 1 and 2
      delay(40);
      valve_control(0); // close valve 1 and 2
	  
	  Ev.events_id[Ev.events_num] = 14; //  free reward to lure next fixation
      Ev.events_time[Ev.events_num] = millis();
      Ev.events_value[Ev.events_num] = 40;
      Ev.events_num ++;
    }
    last_headfixation_flag = headfixation_flag;

    // write parameter F (updated after this trial) to SD card.
    if (para_F_changed == 1) {
      write_SD_para_F();
      para_F_changed = 0;
    }
    // current fixation event info
    if (Ev.events_num > 0) {
      write_SD_event_fixation();
      Ev.events_num = 0;
    }

  } else  { // if the toggle switch is off
    if (paused == 0) {
		paused = 1; // execute only one time
		
		valve_control(0); // Reset valves
		stopMaskingflash();
		stopGocue();	
		digitalWrite(BncOutputLines[0], LOW);
		
      // stop head-fixation
      if (headfixation_flag == 1) { // release head-fixation
        analogWrite(DAC0, regulatorVal_release);
        headfixation_flag = 0;
        Ev.events_id[Ev.events_num] = 13; //10; // headfixation release2: switch off release
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = millis() - last_state_time;
        Ev.events_num ++;
		if (sdcard ==1){
			write_SD_event_fixation();
		}
        Ev.events_num = 0;
        timer_state = CHECK_TRIG;
      }
	  
	  Ev.events_id[Ev.events_num] = 16; //switch off
      Ev.events_time[Ev.events_num] = millis();
      Ev.events_value[Ev.events_num] = -1;
      Ev.events_num ++;
	  	  
      // stop the trial
      if (protocol_sent_run == 1) {
        protocol_sent_run = 0;
		ledState = 0; 
		analogWrite(ledPin, ledState); //OFF
		Timer3.stop();      // stop timer
      }
	  
      if (SerialUSB.dtr() && SerialUSB_connected()) {
        SerialUSB.println("M: Program PAUSED!!!");
      }
      printCurrentTime(); // print current time and date
    }
    if (ledState == 0) {
      ledState = 255; //ON
    } else {
      ledState = 0; //OFF;
    }
	analogWrite(ledPin, ledState);
    delay(500);
  }

  // two-byte communication with PC
  if (SerialUSB.available()) { // receiving data from PC
    byte CommandByte = SerialUSB.read();
    byte dataByte;
    String buffer_tmp;
    // F for Motor FB,
    // L for Motor LR,
    // P for Motor for pole
    // f for final motor FB position
    // R for reward size and free reward
    // C for control panel info retrival
    // T for tare weighting stage
    switch (CommandByte) {
      case 'F':  // Motor F/B; 0-255
        dataByte = SerialUSBReadByte();
        S.FB_motor_position = dataByte;
        if (F.trig_counter > trigger_num_before_fix) {
          S.FB_final_position = S.FB_motor_position;
        }
        write_SD_para_S();
        analogWrite(portMotorFB, dataByte);
        // user change event
        Ev.events_id[Ev.events_num] = 20; // user change event: portMotorFB
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = S.FB_motor_position;
        Ev.events_num ++;
        break;
      case 'L':  // Motor L/R; 0-255; 70 is center
        dataByte = SerialUSBReadByte();
        S.LR_motor_position = dataByte;
        write_SD_para_S();
        analogWrite(portMotorLR, dataByte);
        // user change event
        Ev.events_id[Ev.events_num] = 21; // user change event: portMotorLR
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = S.LR_motor_position;
        Ev.events_num ++;
        break;
      case 'P':  // Motor Pole; 0-255; 30 is anterior, 100 is posterior
        dataByte = SerialUSBReadByte();
        analogWrite(portMotorPole, dataByte);
        delay(1000);
		analogReadResolution(12);
		polePos = analogRead(portMotorPos);
		if (SerialUSB.dtr() && SerialUSB_connected()) {
			SerialUSB.print("M: pole Position: ");
			SerialUSB.println(polePos);
		}
        break;
      case 'f': // update motor F/B final position
        dataByte = SerialUSBReadByte();
        S.FB_final_position = dataByte;

        if (F.trig_counter > trigger_num_before_fix) {
          S.FB_motor_position = S.FB_final_position;
          analogWrite(portMotorFB, S.FB_motor_position);
        }

        write_SD_para_S();

        // user change event
        Ev.events_id[Ev.events_num] = 22; // user change event: portMotorFB_final
        Ev.events_time[Ev.events_num] = millis();
        Ev.events_value[Ev.events_num] = S.FB_final_position;
        Ev.events_num ++;

        break;
      case 'R': // updating reward value
        buffer_tmp = SerialUSB.readStringUntil('\n');
        S.reward_left = buffer_tmp.toFloat();
        buffer_tmp = SerialUSB.readStringUntil('\n');
        S.reward_right = buffer_tmp.toFloat();
        write_SD_para_S();

        valve_control(1); //  open valve 1
        delay(uint(S.reward_left * 1000));
        valve_control(0); // close valve 1
        valve_control(2); //  open valve 2
        delay(uint(S.reward_right * 1000));
        valve_control(0); // close valve 2
        break;
      case 'C':
        SerialUSB.print("C");
        SerialUSB.print(S.FB_motor_position);
        SerialUSB.print(";");
        SerialUSB.print(S.LR_motor_position);
        SerialUSB.print(";");
        SerialUSB.print(finalPos); // pole motor position
        SerialUSB.print(";");
        SerialUSB.print(S.FB_final_position);
        SerialUSB.print(";");
        SerialUSB.print(S.reward_left);
        SerialUSB.print(";");
        SerialUSB.print(S.reward_right);
        SerialUSB.print(";");
        SerialUSB.print(scale.get_units()); // weight
        SerialUSB.print(";");
        SerialUSB.print(F.struggle_thres_neg);
        SerialUSB.print(";");
        SerialUSB.print(F.struggle_thres_pos);
        SerialUSB.print(";");
		SerialUSB.print(S.anteriorPos);
        SerialUSB.print(";");
        SerialUSB.print(S.posteriorPos);
        SerialUSB.println(";");
        break;
      case 'T': //tare the scale to 0
        scale.set_scale();
        scale.tare();  //Reset the scale to 0
		weight_offset = scale.get_offset();
		write_SD_para_F();
        break;
	  case 'G': //get the scale calibration_factor
		buffer_tmp = SerialUSB.readStringUntil('\n');
		knownWeight = buffer_tmp.toFloat();
		calibration_factor = (scale.read_average()-weight_offset)/knownWeight;
        scale.set_scale(calibration_factor);
		write_SD_para_F();
        break;
      case 'S': //set Struggle threshold			
        buffer_tmp = SerialUSB.readStringUntil('\n');
        F.struggle_thres_neg = buffer_tmp.toInt();
        buffer_tmp = SerialUSB.readStringUntil('\n');
        F.struggle_thres_pos = buffer_tmp.toInt();
        write_SD_para_F();
        break;
	  case 'E': //set Pole left/right position
        buffer_tmp = SerialUSB.readStringUntil('\n');
        S.anteriorPos = buffer_tmp.toInt();
        buffer_tmp = SerialUSB.readStringUntil('\n');
        S.posteriorPos = buffer_tmp.toInt();
        write_SD_para_S();
        break;
      case 'A': //
        send_PC_trials_24hr();
        break;
      default: // never happen...
        while (SerialUSB.available()) {
          SerialUSB.read();
        }
        break;
    }
  }

}// end of loop()




//****************************************************************************************************/
//********************************************** Functions *******************************************/
//****************************************************************************************************/

int send_protocol_to_Bpod_and_Run() {
  StateMatrix sma;                  // predefine the state matrix (sent later to Bpod)
  
  switch (S.ProtocolType) {
    // PROTOCOL 1: teaching animal to headfixation, which are universal to all behaviour tasks
    case P_FIXATION: { // Lick and Rectract until Switch Triggered, then increasing headfixation duration gradually

        // Determine trial-specific state matrix fields
        switch (S.TrialType) {
          case 0: // right
            LeftLickAction  = "AnswerPeriod";
            RightLickAction = "RewardR";
			SampleCueOutput = SampleCueOutputH;
            break;
          case 1: // left
            LeftLickAction  = "RewardL";
            RightLickAction = "AnswerPeriod";
			SampleCueOutput = SampleCueOutputL;
            break;
          case 2: // either
            LeftLickAction  = "RewardL";
            RightLickAction = "RewardR";
			if (random(100) < 50) {
				SampleCueOutput = SampleCueOutputH;
			} else {
				SampleCueOutput = SampleCueOutputL;
			}
            break;
        }

        if  (sampleType ==0) {//Tactile		
			MovePole(S.TrialType); // 2 sec
			SampleCueOutput = PoleOutput;			
        }

        StateChange TrigTrialStart_Cond[1]     = {{"Tup", "SamplePeriod"}}; //SamplePeriod
        StateChange SamplePeriod_Cond[1]       = {{"Tup", "DelayPeriod"}};
        StateChange DelayPeriod_Cond[1]        = {{"Tup", "ResponseCue"}};
        StateChange ResponseCue_Cond[1]        = {{"Tup", "AnswerPeriod"}};
        StateChange AnswerPeriod_Cond[3]       = {{"Port1In", LeftLickAction}, {"Port2In", RightLickAction}, {"Tup", "NoResponse"}};
        StateChange Tup_EndTrial_Cond[1]       = {{"Tup", "TrialEnd"}};
        StateChange RewardL_Cond[1]            = {{"Tup", "RewardConsumption"}};
        StateChange RewardR_Cond[1]            = {{"Tup", "RewardConsumption"}};
        StateChange TrialEnd_Cond[1]           = {{"Tup", "exit"}};

        OutputAction Sample_Pole_Output[1]     = {SampleCueOutput};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction RewardL_Output[1]         = {LeftWaterOutput};
        OutputAction RewardR_Output[1]         = {RightWaterOutput};
        OutputAction NoOutput[0]               = {};

        State states[10] = {};
        //S.SamplePeriod = 1.0;
        S.DelayPeriod = float(random(200, 500)) / 1000.0; // 0.2-0.5 sec random delay
        states[0]  = CreateState("TrigTrialStart",    0.01,                     1, TrigTrialStart_Cond,  0, NoOutput);
        states[1]  = CreateState("SamplePeriod",      S.SamplePeriod,           1, SamplePeriod_Cond,    1, Sample_Pole_Output);
        states[2]  = CreateState("DelayPeriod",       S.DelayPeriod,            1, DelayPeriod_Cond,     0, NoOutput);
        states[3]  = CreateState("ResponseCue",       0.1,                      1, ResponseCue_Cond,     1, ResponseCue_Output);
        states[4]  = CreateState("AnswerPeriod",      3600,                     3, AnswerPeriod_Cond,    0, NoOutput);
		states[5]  = CreateState("RewardL",           S.reward_left,            1, RewardL_Cond,         1, RewardL_Output);
        states[6]  = CreateState("RewardR",           S.reward_right,           1, RewardR_Cond,         1, RewardR_Output);
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
        switch (S.TrialType) {
          case 1: // left
            LeftLickAction  = "Reward";
            RightLickAction = "TimeOut";
            RewardOutput    = LeftWaterOutput;
            Reward_duration_behavior = S.reward_left;
			SampleCueOutput = SampleCueOutputL;
            break;
          case 0: // right
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            Reward_duration_behavior = S.reward_right;
			SampleCueOutput = SampleCueOutputH;
            break;
        }

        if  (sampleType ==0) {//Tactile		
			MovePole(S.TrialType); // 2 sec
			SampleCueOutput = PoleOutput;			
        }

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
        if (S.ProtocolHistoryIndex == 1) { 
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

        OutputAction Sample_Pole_Output[1]     = {SampleCueOutput};
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
            Reward_duration_behavior = S.reward_left;
			SampleCueOutput = SampleCueOutputL;
            break;
          case 0:
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            Reward_duration_behavior = S.reward_right;
			SampleCueOutput = SampleCueOutputH;
            break;
        }

        if  (sampleType ==0) {//Tactile		
			MovePole(S.TrialType); // 2 sec
			SampleCueOutput = PoleOutput;			
        }

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
        OutputAction Sample_Pole_Output[1]     = {SampleCueOutput};
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
            Reward_duration_behavior = S.reward_left;
			if (contingency == 0) {
				SampleCueOutput = SampleCueOutputL;
			} else {
				SampleCueOutput = SampleCueOutputH;
			}
            break;
          case 0:
            LeftLickAction  = "TimeOut";
            RightLickAction = "Reward";
            RewardOutput    = RightWaterOutput;
            Reward_duration_behavior = S.reward_right;
			if (contingency == 0) {
				SampleCueOutput = SampleCueOutputH;
			} else {
				SampleCueOutput = SampleCueOutputL;
			}
            break;
        }

        if  (sampleType ==0) {//Tactile		
			MovePole(S.TrialType); // 2 sec
			SampleCueOutput = PoleOutput;			
        }

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
        OutputAction Sample_Pole_Output[1]     = {SampleCueOutput};
        OutputAction Sample_Pole_Stim_Output[2] = {SampleCueOutput, OptogeneticTrig};
        OutputAction ResponseCue_Output[1]     = {CueOutput};
        OutputAction ResponseCueStim_Output[2] = {CueOutput, OptogeneticTrig};
        OutputAction GiveRightDrop_Output[1]   = {RightWaterOutput};
        OutputAction GiveLeftDrop_Output[1]    = {LeftWaterOutput};
        OutputAction Reward_Output[1]          = {RewardOutput};
        OutputAction NoOutput[0]               = {};
        OutputAction OptoStim_Output[1]        = {OptogeneticTrig}; // new to Stim protocol
		
		    
		State states[16] = {};
        states[0]  = CreateState("TrigTrialStart",    0.1,                      1, TrigTrialStart_Cond,  1, TrigTrialStart_Output);

		powerWeight = 0;        
		laserByte = 2; // 1-Thorlab; 2-UltraLaser		
        if (random(100) < stimTrial_pct) {  //    20% trials to stim
          int randomNum = random(300);
          if (randomNum < 100) {
            weightByte = 10;  // percent
          } else if (randomNum < 200) {
            weightByte = 50; // percent
          } else {
            weightByte = 100; // percent
          }
		  powerWeight = (float)weightByte / 100;

          randomNum = random(300);   
          if (randomNum < 100) { // 1/3 sample
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
          } else if (randomNum < 200) {// 1/3 delay
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
  protocol_sent_run = 1;
  
  bpod_time = millis();
  
  if (SerialUSB.dtr() && SerialUSB_connected()) {
    SerialUSB.println("M: Bpod Running...");
  }

} // end for send_protocol_to_Bpod_and_Run()


void MovePole(byte trial_type) {
  poleLastPos = analogRead(portMotorPos);
  // halfPos, anteriorPos, posteriorPos, finalPos
  switch (trial_type) {
    case 0: // right
      if (contingency == 0) {
        finalPos = S.posteriorPos;
      } else {
        finalPos = S.anteriorPos;
      }
      break;
    case 1: // left
      if (contingency == 0) {
        finalPos = S.anteriorPos;
      } else {
        finalPos = S.posteriorPos;
      }
      break;
    case 2: // either
      if (random(100) < 50) {
        finalPos = S.posteriorPos;
      } else {
        finalPos = S.anteriorPos;
      }
      break;
  }
  halfPos = (S.anteriorPos + S.posteriorPos) / 2;
  
  analogWrite(portMotorPole, halfPos);
  delay(1000);
  
  analogReadResolution(12);
  polePos = analogRead(portMotorPos);
  
  analogWrite(portMotorPole, finalPos);
  delay(1000);
  
  if (SerialUSB.dtr() && SerialUSB_connected()) {
	SerialUSB.print("M: pole half/final Position: ");
	SerialUSB.print(halfPos);
	SerialUSB.print("/");
	SerialUSB.println(finalPos);
	
	SerialUSB.print("M: pole motor last/curr Position: ");
	SerialUSB.print(poleLastPos);
	SerialUSB.print("/");
	SerialUSB.println(polePos);
  }
	
  if (abs(polePos-poleLastPos) < 50) {
	lastFailure=1;
  }else{	
	lastFailure=0;
  }
	
  if (lastFailure ==1) {
	failureCont++;
  }else{
	failureCont = 0;
  }
			
  if (failureCont >= failureMax) {
	failureCont = 0;
	isMotorOK = 0;
	lastFailure = 0;
	write_SD_para_S();
	if (SerialUSB.dtr() && SerialUSB_connected()) {
		SerialUSB.println("Q: pole motor has problem ");
	}
  }		
}


int write_SD_eventST_updateOutcome() {
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
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("M: error opening eventsT.txt");
    }
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
      if (S.ProtocolHistoryIndex == 1) { // 1, no timeout
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
  if (SerialUSB.dtr() && SerialUSB_connected()) {
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

    if (F.trig_counter <= trigger_num_before_fix || S.FB_motor_position > S.FB_final_position) {
      SerialUSB.print("M: Motor Pos: ");
      SerialUSB.print(S.FB_motor_position);
      SerialUSB.print("; Trig No.: ");
      SerialUSB.println(F.trig_counter);
    }
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
    S.OutcomeHistory[i]      = S.OutcomeHistory[i + 1];         // 'Others'-3 || Reward-1 || No Response-0 || Time Out (error)-2
	S.EarlylickHistory[i]    = S.EarlylickHistory[i + 1];       // Early lick-1 || No Early lick-0
  }
  // Keep record current trial info in the last position (RECORD_TRIALS-1) of the matrix
  S.ProtocolTypeHistory[RECORD_TRIALS - 1] = S.ProtocolType;
  S.OutcomeHistory[RECORD_TRIALS - 1]      = S.Trial_Outcome;
  S.EarlylickHistory[RECORD_TRIALS - 1]    = is_earlylick;
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
  Earlylick100 = 0;
  response_trial = 100;
  for (int i = 0; i < RECORD_TRIALS; i++) {
    if (S.OutcomeHistory[i] == 0) {
      response_trial = response_trial - 1;
    } else {
      if (S.OutcomeHistory[i] == 1) {
        Perf100++;
      }
	  if (S.EarlylickHistory[i] == 1) {
        Earlylick100++;
      }
    }
  }
  Perf100 = round(100 * (float)Perf100 / response_trial);
  Earlylick100 = round(100 * (float)Earlylick100 / response_trial);
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
  // case 0:  // P_FIXATION
   if (S.ProtocolHistoryIndex == 0){
      // determine TrialType
      if (S.FB_motor_position < 60 + S.FB_final_position && S.FB_motor_position > S.FB_final_position) {
        S.Autolearn = 0; // eigher side will be rewarded
      } else {
        S.Autolearn = 1; // 'ON': 3 left, 3 right
      }

      // determine if advance motor
      if (S.FB_motor_position > S.FB_final_position) {
        if (S.FB_motor_position > 150 + S.FB_final_position) {
          if (S.totoal_reward_num != last_advance_reward_num && S.totoal_reward_num % lick_num_before_motor_retract1 == 0) {
            is_motor_advance = 1;
            last_advance_reward_num = S.totoal_reward_num;
          }
        } else if (S.FB_motor_position > 70 + S.FB_final_position) {
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

          if (S.FB_motor_position - S.FB_final_position < motor_retract_step) {
            S.FB_motor_position = S.FB_final_position;
            analogWrite(portMotorFB, S.FB_motor_position);
          } else {
            for (int j = 0; j < motor_retract_step; j++) {
              S.FB_motor_position = S.FB_motor_position - 1;
              analogWrite(portMotorFB, S.FB_motor_position);
              delay(100);
            }
          }
          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.print("M: Motor advanced to ");
            SerialUSB.print(S.FB_motor_position);
            SerialUSB.print(", final position: ");
            SerialUSB.println(S.FB_final_position);
          }
        }
      }

      if (F.fixation_duration > 25000) { // 25 sec
        S.ProtocolType         = P_SAMPLE;
        //S.SamplePeriod        = 1.30;
        //S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 1.00;
        S.Autolearn           = 1;    // autoLearn ON

        S.ProtocolHistoryIndex = 1;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
      }
      return 0;
   }

    //case 1: // P_SAMPLE-1
	if (S.ProtocolHistoryIndex == 1){
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100) { // if  ran > 100 trials
        S.ProtocolHistoryIndex = 2;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        //S.SamplePeriod        = 1.30;
        //S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 1.00;
        S.Autolearn           = 1;    // autoLearn ON
      }
      return 0;
    }

    //case 2: // P_SAMPLE-2
	if (S.ProtocolHistoryIndex == 2){
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 200) {
        S.ProtocolHistoryIndex = 3;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_SAMPLE;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        //S.SamplePeriod        = 1.30;
        //S.DelayPeriod         = 0.3;   // in sec
        S.TimeOut             = 3.00;
        S.Autolearn = 2; // antiBias
      }
     return 0;
	}

   //case 3: // P_SAMPLE-3
	if (S.ProtocolHistoryIndex == 3){
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 75) {
        S.ProtocolHistoryIndex = 4;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.ProtocolType        = P_DELAY;
        //S.SamplePeriod        = 1.30;  // in sec
        S.DelayPeriod         = minDelay;   // in sec
        S.TimeOut             = 4.0;
        S.Autolearn           = 2;     // antiBias
      }
      return 0;
	}

    //case 4-16: // P_DELAY-4-16:with DelayPeriod gradually increase from minDelay to maxDelay	
	if (S.ProtocolHistoryIndex >= 4 && S.ProtocolHistoryIndex < 17) { 
	  if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
		stepIndex++;
		S.ProtocolHistoryIndex ++;
		S.DelayPeriod  = minDelay+stepIndex*(maxDelay-minDelay)/delayStep;   // in sec
		if (stepIndex >= delayStep){
			S.ProtocolHistoryIndex= 17;
		} else{
			if (S.ProtocolHistoryIndex >16){
				S.ProtocolHistoryIndex = 16;
			}
		}
		S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        //S.SamplePeriod  = 1.3;   // in sec       
        S.TimeOut             = 3.0;
        S.Autolearn           = 2; // antiBias				
      }
      return 0;
	}

    //case 17: // P_DELAY-17: DelayPeriod = maxDelay
	if (S.ProtocolHistoryIndex == 17){
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 100 && S.ProtocolHistory[S.ProtocolHistoryIndex].performance > 70) {
        S.ProtocolHistoryIndex = 18;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_DELAY;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        //S.SamplePeriod  = 1.3;   // in sec
        //S.DelayPeriod   = 1.3;   // in sec
        S.TimeOut             = 4.0;
        S.Autolearn           = 2; // antiBias
        F.fixation_interval_max = 60000;
        para_F_changed = 1;
      }
      return 0;
	}

    //case 18: 
	if (S.ProtocolHistoryIndex == 18){
      if (S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials > 500 && Perf100 > 70 && Earlylick100 < 50) {
        S.ProtocolHistoryIndex = 19;
        S.ProtocolHistory[S.ProtocolHistoryIndex].Protocol = P_OPTOSTIM;
        S.ProtocolHistory[S.ProtocolHistoryIndex].nTrials = 0;
        S.ProtocolHistory[S.ProtocolHistoryIndex].performance = 0;
        S.ProtocolType  = P_OPTOSTIM;
        //S.SamplePeriod  = 1.3;   // in sec
        //S.DelayPeriod   = 1.3;   // in sec
        S.TimeOut       = 4.0;
        S.Autolearn     = 2; // antiBias
		meetaim_cont   = 0;
      }
      return 0;
	}
	
    //case 19: // P_OPTOSTIM-19
	if (S.ProtocolHistoryIndex == 19){
	  contingency_trial = contingency_trial + 1;
	  if (S.Trial_Outcome > 0){
		fixblock_trialnum_cont = fixblock_trialnum_cont + 1;
	  }
	  
	  if (SerialUSB.dtr() && SerialUSB_connected()) {
		SerialUSB.print("M: contingency: ");
		SerialUSB.print(contingency);
		SerialUSB.print("; contingency_trial: ");
		SerialUSB.print(contingency_trial);
		SerialUSB.print("; sampleType: ");
		SerialUSB.print(sampleType);
		SerialUSB.print("; fixblock_trialnum_cont: ");
		SerialUSB.println(fixblock_trialnum_cont);
	  }
	  
	  if (is_random == 1) {
		if (random(100) < 50) {
			sampleType = 1; // auditory
		} else {
			sampleType = 0; // tactile
		}
		if (SerialUSB.dtr() && SerialUSB_connected()) {
			SerialUSB.print("M: sampleType changed to: ");
			SerialUSB.println(sampleType);
		}		  
	  }else{
		if (meetaim_cont < 2) {
			if (contingency_trial > contingency_trial_aim && Perf100 > Perf100_aim && Earlylick100 < Earlylick100_aim && response_trial > response_trial_aim) {
				contingency_trial = 0;
				meetaim_cont = meetaim_cont +1;
				fixblock_trialnum_cont = 0;
		
				if (reverseType == 1) {
					if (contingency == 1) {
						contingency == 0;
					} else {
						contingency == 1;
					}
					if (SerialUSB.dtr() && SerialUSB_connected()) {
						SerialUSB.print("M: contingency changed to: ");
						SerialUSB.println(sampleType);
					}					
				}else{
					if (sampleType == 1) {
						sampleType = 0;
					} else {
						sampleType = 1;
					}	
					if (SerialUSB.dtr() && SerialUSB_connected()) {
						SerialUSB.print("M: sampleType changed to: ");
						SerialUSB.println(sampleType);
					}
				}
			}
		} else {
			if (fixblock_trialnum_cont >= fixblock_trialnum_aim && Perf100 > Perf100_aim && Earlylick100 < Earlylick100_aim && response_trial > response_trial_aim ) {
				fixblock_trialnum_cont = 0;
			
				if (reverseType == 1) {
					if (contingency == 1) {
						contingency == 0;
					} else {
						contingency == 1;
					}
					if (SerialUSB.dtr() && SerialUSB_connected()) {
						SerialUSB.print("M: contingency changed to: ");
						SerialUSB.println(sampleType);
					}					
				}else{
					if (sampleType == 1) {
						sampleType = 0;
					} else {
						sampleType = 1;
					}	
					if (SerialUSB.dtr() && SerialUSB_connected()) {
						SerialUSB.print("M: sampleType changed to: ");
						SerialUSB.println(sampleType);
					}
				}
			}
		}
	  }
      return 0;
	}
}

void autoAdjustLickportPosition() {
  if (S.ProtocolHistoryIndex > 2 && S.ProtocolHistoryIndex <= 19) { //
    LickPortMove++;
    if (LickPortMove > 50) {
      byte recent1 = 50; // last 50 trials are biased 30%
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

      byte recent2 = 20; // in the last 20 trials, bias >= 80%
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
        String str_tmp;
        byte motor_tmp;
        if (Perf_R > Perf_L) { // right port too close
          str_tmp = "right: ";
          motor_tmp = S.LR_motor_position;
          if (S.LR_motor_position <= 90) {
            S.LR_motor_position = S.LR_motor_position + mov_step_size;
          }
        } else if (Perf_R < Perf_L) {               // left port too close
          str_tmp = "left: ";
          motor_tmp = S.LR_motor_position;
          if (S.LR_motor_position >= 50) {
            S.LR_motor_position = S.LR_motor_position - mov_step_size;
          }
        }
        if (SerialUSB.dtr() && SerialUSB_connected()) {
          SerialUSB.print("E: bias to ");
          SerialUSB.print(str_tmp);
          SerialUSB.print(motor_tmp);
          SerialUSB.print(" -> ");
          SerialUSB.println(S.LR_motor_position);
        }

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
      if (n_RSideList == error_trials) {
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
        if (SerialUSB.dtr() && SerialUSB_connected()) {
		  SerialUSB.print("M: Switch status: ");
		  SerialUSB.print(SwitchL_CurrentStatus);
		  SerialUSB.print(SwitchR_CurrentStatus);
		  SerialUSB.print(SwitchL_LastStatus);
		  SerialUSB.print(SwitchR_LastStatus);
		  SerialUSB.print("M: Switch triggered NO. ");
          SerialUSB.println(F.trig_counter);
        }
        if (F.trig_counter > trigger_num_before_fix && S.FB_motor_position <= S.FB_final_position) {
          timer_state = DELAY_TO_FIX;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("DELAY_TO_FIX");
          last_state_time = millis();
        }else{
			timer_state = CHECK_RELEASE0;
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

        F.headfixation_counter ++;
        para_F_changed = 1;

        if (headfixation_again == 1) {
          headfixation_again = 0;
          Ev.events_id[Ev.events_num] = 11; //  headfixation Again;
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = regulatorVal_min;
          Ev.events_num ++;
          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.print("M: Head fixation (again) NO. ");
            SerialUSB.print(F.headfixation_counter);
            SerialUSB.print(", interval: ");
            SerialUSB.println(F.fixation_duration);
          }

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

          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.print("M: Head fixation NO. ");
            SerialUSB.print(F.headfixation_counter);
            SerialUSB.print(", interval: ");
            SerialUSB.println(F.fixation_duration);
          }
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

          Ev.events_id[Ev.events_num] = 12; // headfixation release3: struggle
          Ev.events_time[Ev.events_num] = millis();
          Ev.events_value[Ev.events_num] = (millis() - last_state_time);
          Ev.events_num ++;
          if (SerialUSB.dtr() && SerialUSB_connected()) {
            SerialUSB.print("M: Released - struggle!, interval: ");
            SerialUSB.print(Ev.events_value[Ev.events_num - 1] / 1000);
            SerialUSB.print("; ");
            SerialUSB.print(weight_tmp);
            SerialUSB.println(" g, DETECTED.");
          }
          timer_state = STRUGGLE_DELAY_1000;
          last_state_time = millis();
        } else if (SwitchL_CurrentStatus == 1 || SwitchR_CurrentStatus == 1) {
          // escape
          analogWrite(DAC0, regulatorVal_release);
          headfixation_flag = 0;

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
          if (SerialUSB.dtr() && SerialUSB_connected()) {   
            SerialUSB.print("M: Released - escape!, interval: ");
            SerialUSB.print(Ev.events_value[Ev.events_num - 1] / 1000);
            SerialUSB.print("; ");
            SerialUSB.print("Performance: ");
            SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
            SerialUSB.println("/20");
          }
          timer_state = ESCAPE_DELAY_1000;
          //SerialUSB.print("Headfixation state changed to: ");
          //SerialUSB.println("ESCAPE_DELAY_1000");
          last_state_time = millis();
        } else if (millis() - last_state_time > F.fixation_duration) {
          // timeup
          if (S.ProtocolHistoryIndex < 1 || protocol_sent_run == 0 || (millis() - last_state_time > F.fixation_duration + 5000)) {
            analogWrite(DAC0, regulatorVal_release);
            headfixation_flag = 0;

            Ev.events_id[Ev.events_num] = 9; // headfixation release1: timeup
            Ev.events_time[Ev.events_num] = millis();
            Ev.events_value[Ev.events_num] = millis() - last_state_time;
            Ev.events_num ++;

            for (int i = 0; i < 19; i++) {
              F.Fixation_Outcome[i] = F.Fixation_Outcome[i + 1];
            }
            F.Fixation_Outcome[19] = 9;
            para_F_changed = 1;
            if (SerialUSB.dtr() && SerialUSB_connected()) {
              SerialUSB.print("M: Released - timeup! ");
              SerialUSB.print("Performance: ");
              SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
              SerialUSB.println("/20");
            }
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
        if (SerialUSB.dtr() && SerialUSB_connected()) {
          SerialUSB.print("M: Performance: ");
          SerialUSB.print(compare_array_sum(F.Fixation_Outcome, 9, 0, 20));
          SerialUSB.println("/20");
        }
      }
      break;

    case ESCAPE_DELAY_1000: // delay a little bit to go back to CHECK_TRIG
      if (millis() - last_state_time > 1000) {
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
	  
	case CHECK_RELEASE0:
		if (SwitchL_CurrentStatus == 1 && SwitchR_CurrentStatus == 1 && (SwitchL_LastStatus == 0 || SwitchR_LastStatus == 0 )){
			timer_state = CHECK_TRIG;
		}
		break;	
    default:
      break;
  }
}



//**************************************************************************************************************/
//********************************************** SD related Functions *******************************************/
//**************************************************************************************************************/
int write_SD_para_F() {
  File dataFile = SD.open("paraF.txt", O_WRITE);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.print("trig_counter = ");
    dataFile.println(F.trig_counter);
    dataFile.print("fixation_duration = ");
    dataFile.println(F.fixation_duration);
    dataFile.print("last_advance_headfixation_counter = ");
    dataFile.println(F.last_advance_headfixation_counter);
    dataFile.print("Fixation_Outcome = ");
    for (int i = 0; i < 20; i++) {
      dataFile.print(F.Fixation_Outcome[i]);
      dataFile.print("; ");
    }
    dataFile.println();
    dataFile.print("struggle_thres_neg = ");
    dataFile.println(F.struggle_thres_neg);
    dataFile.print("struggle_thres_pos = ");
    dataFile.println(F.struggle_thres_pos);
    dataFile.print("last_advance_threshold_counter = ");
    dataFile.println(F.last_advance_threshold_counter);
    dataFile.print("headfixation_counter = ");
    dataFile.println(F.headfixation_counter);
    dataFile.print("fixation_interval_max = ");
    dataFile.println(F.fixation_interval_max);
	
	dataFile.print("weight_offset = ");
    dataFile.println(weight_offset);
	
	dataFile.print("calibration_factor = ");
    dataFile.println(calibration_factor);
  } else {
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening paraF.txt for write");
    }
    return -1;
  }
  dataFile.close();
  return 0;
}

int read_SD_para_F() {
  File dataFile = SD.open("paraF.txt", O_READ);
  if (dataFile) {
    dataFile.seek(0);
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.trig_counter = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.fixation_duration = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.last_advance_headfixation_counter = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 20; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      F.Fixation_Outcome[i] = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.struggle_thres_neg = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.struggle_thres_pos = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.last_advance_threshold_counter = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.headfixation_counter = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    F.fixation_interval_max = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    weight_offset = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    calibration_factor = buffer_tmp.toFloat();
  } else {
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening paraF.txt for read");
    }
    return -1;
  }
  dataFile.close();
  return 0;
}

int write_SD_para_S() {
  File dataFile = SD.open("paraS.txt", O_WRITE);
  if (dataFile) {
    dataFile.seek(0);
    dataFile.print("currentTrialNum = ");
    dataFile.println(S.currentTrialNum);
    dataFile.print("FB_motor_position = ");
    dataFile.println(S.FB_motor_position);
    dataFile.print("ProtocolType = ");
    dataFile.println(S.ProtocolType);
    dataFile.print("Autolearn = ");
    dataFile.println(S.Autolearn);
    dataFile.print("TrialType = ");
    dataFile.println(S.TrialType);
    dataFile.print("random_delay_duration = ");
    dataFile.println(S.random_delay_duration);
    dataFile.print("Trial_Outcome = ");
    dataFile.println(S.Trial_Outcome);

    dataFile.print("SamplePeriod = ");
    dataFile.println(S.SamplePeriod);
    dataFile.print("DelayPeriod = ");
    dataFile.println(S.DelayPeriod);
    dataFile.print("TimeOut = ");
    dataFile.println(S.TimeOut);
    dataFile.print("ProtocolHistoryIndex = ");
    dataFile.println(S.ProtocolHistoryIndex);

    dataFile.print("ProtocolHistory.Protocol = ");
    for (int i = 0; i < 30; i++) {
      dataFile.print(S.ProtocolHistory[i].Protocol);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("ProtocolHistory.nTrials = ");
    for (int i = 0; i < 30; i++) {
      dataFile.print(S.ProtocolHistory[i].nTrials);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("ProtocolHistory.performance = ");
    for (int i = 0; i < 30; i++) {
      dataFile.print(S.ProtocolHistory[i].performance);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("GaveFreeReward.flag_R_water = ");
    dataFile.println(S.GaveFreeReward.flag_R_water);
    dataFile.print("GaveFreeReward.flag_L_water = ");
    dataFile.println(S.GaveFreeReward.flag_L_water);
    dataFile.print("GaveFreeReward.past_trials = ");
    dataFile.println(S.GaveFreeReward.past_trials);

    dataFile.print("ProtocolTypeHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++) {
      dataFile.print(S.ProtocolTypeHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("TrialTypeHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++) {
      dataFile.print(S.TrialTypeHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("OutcomeHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++) {
      dataFile.print(S.OutcomeHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();
	
	dataFile.print("EarlyLickHistory = ");
    for (int i = 0; i < RECORD_TRIALS; i++) {
      dataFile.print(S.EarlylickHistory[i]);
      dataFile.print("; ");
    }
    dataFile.println();

    dataFile.print("struggle_enable = ");
    dataFile.println(S.struggle_enable);
    dataFile.print("totoal_reward_num = ");
    dataFile.println(S.totoal_reward_num);
    dataFile.print("retract_times = ");
    dataFile.println(S.retract_times);
    dataFile.print("LR_motor_position = ");
    dataFile.println(S.LR_motor_position);
    dataFile.print("FB_final_position = ");
    dataFile.println(S.FB_final_position);

    dataFile.print("reward_left = ");
    dataFile.println(S.reward_left);
    dataFile.print("reward_right = ");
    dataFile.println(S.reward_right);
	
	dataFile.print("contingency = ");
    dataFile.println(contingency);
    dataFile.print("contingency_trial = ");
    dataFile.println(contingency_trial);
	
	dataFile.print("pole_anterior_position = ");
    dataFile.println(S.anteriorPos);
    dataFile.print("pole_posterior_position = ");
    dataFile.println(S.posteriorPos);
	
	dataFile.print("fixblock_trialnum_cont = ");
    dataFile.println(fixblock_trialnum_cont);
	
	dataFile.print("sampleType = ");
    dataFile.println(sampleType);
	
	dataFile.print("fixblock_trialnum_aim = ");
    dataFile.println(fixblock_trialnum_aim);
	
	dataFile.print("contingency_trial_aim = ");
    dataFile.println(contingency_trial_aim);
	
	dataFile.print("Perf100_aim = ");
    dataFile.println(Perf100_aim);
	
	dataFile.print("Earlylick100_aim = ");
    dataFile.println(Earlylick100_aim);
	
	dataFile.print("response_trial_aim = ");
    dataFile.println(response_trial_aim);
	
	dataFile.print("meetaim_cont = ");
    dataFile.println(meetaim_cont);

	dataFile.print("minDelay = ");
    dataFile.println(minDelay);
	dataFile.print("maxDelay = ");
    dataFile.println(maxDelay);
	dataFile.print("delayStep = ");
    dataFile.println(delayStep);
	dataFile.print("stepIndex = ");
    dataFile.println(stepIndex);

	dataFile.print("stimTrial_pct = ");
    dataFile.println(stimTrial_pct);	
	
	dataFile.print("is_motor_ok = ");
    dataFile.println(isMotorOK);
	
	dataFile.print("reverseType = ");
    dataFile.println(reverseType);
	
	dataFile.print("is_random = ");
    dataFile.println(is_random);
  } else {
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening paraS.txt for write");
    }
    return -1;
  }
  dataFile.close();
  return 0;
}

int read_SD_para_S() {
  File dataFile = SD.open("paraS.txt", O_READ);
  if (dataFile) {
    dataFile.seek(0);
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.currentTrialNum = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.FB_motor_position = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.ProtocolType = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.Autolearn = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.TrialType = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.random_delay_duration = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.Trial_Outcome = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.SamplePeriod = buffer_tmp.toFloat();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.DelayPeriod = buffer_tmp.toFloat();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.TimeOut = buffer_tmp.toFloat();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.ProtocolHistoryIndex = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 30; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.ProtocolHistory[i].Protocol = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 30; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.ProtocolHistory[i].nTrials = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < 30; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.ProtocolHistory[i].performance = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');


    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.flag_R_water = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.flag_L_water = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.GaveFreeReward.past_trials = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.ProtocolTypeHistory[i] = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.TrialTypeHistory[i] = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.OutcomeHistory[i] = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');
	
	buffer_tmp = dataFile.readStringUntil('=');
    for (int i = 0; i < RECORD_TRIALS; i++) {
      buffer_tmp = dataFile.readStringUntil(';');
      S.EarlylickHistory[i] = buffer_tmp.toInt();
    }
    buffer_tmp = dataFile.readStringUntil('\n');

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.struggle_enable = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.totoal_reward_num = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.retract_times = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.LR_motor_position = buffer_tmp.toInt();
    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.FB_final_position = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.reward_left = buffer_tmp.toFloat();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.reward_right = buffer_tmp.toFloat();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    contingency = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    contingency_trial = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.anteriorPos  = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    S.posteriorPos = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    fixblock_trialnum_cont = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    sampleType = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    fixblock_trialnum_aim = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    contingency_trial_aim = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    Perf100_aim = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    Earlylick100_aim = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    response_trial_aim = buffer_tmp.toInt();

    buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    meetaim_cont = buffer_tmp.toInt(); 

	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    minDelay = buffer_tmp.toFloat();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    maxDelay = buffer_tmp.toFloat();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    delayStep = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    stepIndex = buffer_tmp.toInt();	
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    stimTrial_pct = buffer_tmp.toInt();	
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    isMotorOK = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    reverseType = buffer_tmp.toInt();
	
	buffer_tmp = dataFile.readStringUntil('=');
    buffer_tmp = dataFile.readStringUntil('\n');
    is_random = buffer_tmp.toInt();
  } else {
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening paraS.txt for read");
    }
    return -1;
  }
  dataFile.close();
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
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening events.txt");
    }
    return -1;
  }
  dataFile.close();
  return 0;
}

// Write Trial Events to SD card
int write_SD_trial_info() {
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

    // print F/B and L/R motor position
    dataFile.print(" ");
    dataFile.print(S.FB_motor_position);
    dataFile.print(" ");
    dataFile.print(S.LR_motor_position);
	
	dataFile.print(" ");
    dataFile.print(contingency);
    dataFile.print(" ");
    dataFile.print(contingency_trial);
	
	dataFile.print(" ");
    dataFile.print(sampleType);	
	dataFile.print(" ");
    dataFile.print(fixblock_trialnum_cont);

    for (int i = 0; i < nTransition; i++) {
      dataFile.print(" ");
      dataFile.print(state_visited[i]);
    }

    dataFile.println();
  } else {
    if (SerialUSB.dtr() && SerialUSB_connected()) {
      SerialUSB.println("E: error opening trials.txt");
    }
    return -1;
  }
  dataFile.close();
  return 0;
}

int send_PC_trials_24hr() {
  unsigned long time_stamp;
  unsigned int trial_num;
  byte trial_type;
  byte trial_outcome;
  File dataFile = SD.open("trials.txt", O_READ);
  if (dataFile) {
    dataFile.seek(0);
    now = rtc.now();
    unix_time_24  = now.unixtime() - 86400; // last 24 hr = 24 * 60 *60 sec
    while (dataFile.available()) {
      watchdogReset();
      buffer_tmp = dataFile.readStringUntil(' ');
      time_stamp = atol(buffer_tmp.c_str());
      if (time_stamp > unix_time_24) {
        SerialUSB.print('A'); // All the data in last 24 hr
        SerialUSB.print(time_stamp);
        SerialUSB.print(' ');
        buffer_tmp = dataFile.readStringUntil(' ');
        buffer_tmp = dataFile.readStringUntil(' ');
        buffer_tmp = dataFile.readStringUntil(' ');
        buffer_tmp = dataFile.readStringUntil(' ');
        trial_type = (byte)buffer_tmp.toInt();
        SerialUSB.print(trial_type);
        SerialUSB.print(' ');
        buffer_tmp = dataFile.readStringUntil(' ');
        trial_outcome = (byte)buffer_tmp.toInt();
        SerialUSB.print(trial_outcome);
        SerialUSB.println();
        buffer_tmp = dataFile.readStringUntil('\n');
      } else {
        buffer_tmp = dataFile.readStringUntil('\n');
      }
    }
    SerialUSB.print("N"); // end of data sending
    SerialUSB.println(now.unixtime());
  } else {
    SerialUSB.println("E: error opening trials.txt for read");
    return -1;
  }
  dataFile.close();
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

    // Add state transitions.
    for (int i = 0; i < state->nStateChangeConditions; i++) {
      int CandidateEventCode = find_idx(EventNames, 40, state->StateChangeCondition[i].StateChangeTrigger);
      if (CandidateEventCode < 0) {
        sma->nStates--;
        return -1;
      }
      String TargetState = state->StateChangeCondition[i].StateChangeTarget;
      int TargetStateNumber = 0;
      if (TargetState.compareTo("exit") == 0) {
        TargetStateNumber = sma->nStates; 
      } else {
        TargetStateNumber = find_idx(sma->StateNames, sma->nStates, TargetState);
      }

      sma->InputMatrix[CurrentState][CandidateEventCode] = TargetStateNumber;
    }


    // Add output actions.
    for (int i = 0; i < 17; i++) {
      sma->OutputMatrix[CurrentState][i] = 0;
    }
    for (int i = 0; i < state->nOutputs; i++) {
        int TargetEventCode = find_idx(OutputActionNames, 17, state->Output[i].OutputType);
        if (TargetEventCode >= 0) {
          int Value = state->Output[i].Value;
          sma->OutputMatrix[CurrentState][TargetEventCode] = Value;
        } else {
          sma->nStates--;
          return -1;
        }
    }

    // Add self timer.
    sma->StateTimers[CurrentState] = state->StateTimer;
  }

  // Return 0 if success.
  return 0;
}


/*
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
  stateNum = sma->nStates;
  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 40; j++) {
	  InputStateMatrix[i][j] = sma->InputMatrix[i][j];
    }
  }

  for (int i = 0; i < stateNum; i++) {
    for (int j = 0; j < 17; j++) {
	  OutputStateMatrix[i][j] = sma->OutputMatrix[i][j];
    }
  }

  for (int i = 0; i < 8; i++) {
	PortInputsEnabled[i] = PortInputsEnabled[i];
  }

  for (int i = 0; i < stateNum; i++) {	
	StateTimers[i] = sma->StateTimers[i]* TimerScaleFactor;
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
	analogWrite(ledPin, 100);
	NewStateIdx = 0;
	CurrentState = 0;
	nEvents = 0;
	MatrixFinished = false;
			
	// Read initial state of sensors
	for (int x = 0; x < 8; x++) {
		if (PortInputsEnabled[x] == 1) {
			PortInputLineValue[x] = digitalRead(PortDigitalInputLines[x]);
			if (PortInputLineValue[x] == HIGH) {
				PortInputLineLastKnownStatus[x] = HIGH; // Update last known state of input line
			} else {
				PortInputLineLastKnownStatus[x] = LOW;
			}
		} else {
			PortInputLineLastKnownStatus[x] = LOW;
			PortInputLineValue[x] = LOW;
		}
		PortInputLineOverride[x] = false;
	}
			
	MatrixStartTime = millis();;
	StateStartTime = MatrixStartTime;
	CurrentTime = MatrixStartTime;

	nTransition = 0;
	state_visited[nTransition++] = CurrentState;

	// Adjust outputs, scheduled waves, Serial codes and sync port for first state
	setStateOutputs(CurrentState);
	Timer3.start(); // Runs every 100us
			
return 0;
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

void valve_control(byte on_off) { //0: both off; 1:left open; 2:right open; 3: both open 
  for (int x = 0; x < 2; x++) {
	digitalWrite(ValveDigitalOutputLines[x], bitRead(on_off, x));
  }
}


byte SerialUSBReadByte() {
  while (SerialUSB.available() == 0) {}
  LowByte = SerialUSB.read();
  return LowByte;
}

// write an unsigned long number to PC
void SerialUSBWriteLong(unsigned long num) { // NOT USED
  SerialUSB.write((byte)num);
  SerialUSB.write((byte)(num >> 8));
  SerialUSB.write((byte)(num >> 16));
  SerialUSB.write((byte)(num >> 24));
}

// write an unsigned short number to PC
void SerialUSBWriteShort(word num) {
  SerialUSB.write((byte)num);
  SerialUSB.write((byte)(num >> 8));
}

void printCurrentTime() {
  if (SerialUSB.dtr() && SerialUSB_connected()) {
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
}

byte SerialUSB_connected() {
  // Check if the usb cable is connected with the PC
  uint32_t Count = (UOTGHS->UOTGHS_DEVFNUM & UOTGHS_DEVFNUM_FNUM_Msk) >> UOTGHS_DEVFNUM_FNUM_Pos;
  delay(2);
  if (Count == (UOTGHS->UOTGHS_DEVFNUM & UOTGHS_DEVFNUM_FNUM_Msk) >> UOTGHS_DEVFNUM_FNUM_Pos) {
    return 0;
  } else {
    return 1;
  }
}

void stopGocue(){
	pwm_pin53.stop();
	analogWrite(PortPWMOutputLines[0], 0); 
}

void stopMaskingflash() {
	pwm_pin44.stop();
	analogWrite(PortPWMOutputLines[1], 0); 
}

void OptoStim() {
    sample_ind = 0; // first output
    if (laserByte == 1) {
      dac.setVoltage(uint16_t(powerWeight * thorlab_flat_sine[0]), false);
    } else {
      dac.setVoltage(uint16_t(powerWeight * (ultra_flat_sine[0] - 655) + 655), false); // 655 is 0.8 V
    }
    Timer4.start();
}

void timer_updateDAC() { // total 1.3 sec stim
  sample_ind ++;
  if (laserByte == 1) { // Thorlab laser
    if (sample_ind < 2400) { // flat cos, 1.2 sec
      dac.setVoltage(uint16_t(powerWeight * thorlab_flat_sine[sample_ind % 50]), false);
    } else if (sample_ind < 2600) { // ramp down, 0.1 sec
      dac.setVoltage(uint16_t(powerWeight * thorlab_ramp_sine[sample_ind - 2400]), false);
    } else {
      Timer4.stop();
      dac.setVoltage(0, false);
    }
  } else { // Ultra laser 
    if (sample_ind < 2400) { // flat cos, 1.2 sec
      dac.setVoltage(uint16_t(powerWeight * (ultra_flat_sine[sample_ind % 50] - 655) + 655), false); // 655 is 0.8 V
    } else if (sample_ind < 2600) { // ramp down, 0.1 sec
      dac.setVoltage(uint16_t(powerWeight * (ultra_ramp_sine[sample_ind - 2400] - 655) + 655), false); // 655 is 0.8 V
    } else {
      Timer4.stop();
      dac.setVoltage(0, false);
    }
  }
}

//////Task function JL 11/19/2024
void handler() {
	nCurrentEvents = 0;
	CurrentEvent[0] = 254; // Event 254 = No event
	CurrentTime = millis();  //use real time to record the time instead of counting handler visit times.  JL 11/19/2024
	// Refresh state of sensors and inputs
	for (int x = 0; x < 8; x++) {
		if ((PortInputsEnabled[x] == 1) && (!PortInputLineOverride[x])) {
			PortInputLineValue[x] = digitalRead(PortDigitalInputLines[x]);
		}
	}
		// Determine which port event occurred
	int Ev = 0; // Since port-in and port-out events are indexed sequentially, Ev replaces x in the loop.
	for (int x = 0; x < 8; x++) {
		// Determine port entry events
		if ((PortInputLineValue[x] == HIGH) && (PortInputLineLastKnownStatus[x] == LOW)) {
			PortInputLineLastKnownStatus[x] = HIGH;
			CurrentEvent[nCurrentEvents] = Ev;
			nCurrentEvents++;
		}
		Ev = Ev + 1;
		// Determine port exit events
		if ((PortInputLineValue[x] == LOW) && (PortInputLineLastKnownStatus[x] == HIGH)) {
			PortInputLineLastKnownStatus[x] = LOW;
			CurrentEvent[nCurrentEvents] = Ev;
			nCurrentEvents++;
		}
		Ev = Ev + 1;
	}
		
	Ev = 39;
	// Determine if a state timer expired
	TimeFromStart = CurrentTime - StateStartTime;
	if ((TimeFromStart >= StateTimers[CurrentState]) && (MeaningfulStateTimer == true)) {		
		CurrentEvent[nCurrentEvents] = Ev;
		nCurrentEvents++;
	}

	// Now determine if a state transition should occur. The first event linked to a state transition takes priority.
	byte StateTransitionFound = 0;
	int i = 0;
	while ((!StateTransitionFound) && (i < nCurrentEvents)) {			
		NewStateIdx = InputStateMatrix[CurrentState][CurrentEvent[i]];	
		if (NewStateIdx != CurrentState) {
			StateTransitionFound = 1;
		}				
		i++;
	}
		// Store timestamp of events captured in this cycle
	if ((nEvents + nCurrentEvents) < MaxTimestamps) {
		for (int x = 0; x < nCurrentEvents; x++) {
			eventTimeStamps[nEvents] = CurrentTime - MatrixStartTime; //CurrentTime;
			Events[nEvents] = CurrentEvent[x]; // new
			nEvents++;
		}
	}
	// Make state transition if necessary
	if (StateTransitionFound) {
		if (nTransition < MaxTransitions) {
			state_visited[nTransition++] = NewStateIdx;
		}

		if (NewStateIdx == stateNum) {
			MatrixFinished = true;
			Timer3.stop();
		} else {
			setStateOutputs(NewStateIdx);
			StateStartTime = CurrentTime;
			CurrentState = NewStateIdx;
		}
	} 
	return;
} // End timer handler


void setStateOutputs(byte State) {
	byte CurrentTimer = 0; // Used when referring to the timer currently being triggered
	byte CurrentCounter = 0; // Used when referring to the counter currently being reset
	valve_control(OutputStateMatrix[State][0]);        // Cols0 = Valves; Value: 0, 1, 2, 3
	digitalWrite(BncOutputLines[0], OutputStateMatrix[State][1]);

	// 9-16=PWM values 
	if (OutputStateMatrix[State][9] == 255) {
		pwm_pin53.start(PWM_PERIOD_PIN_53, PWM_DUTY_PIN_53);
	}else if (OutputStateMatrix[State][12] == 255) {
		pwm_pin53.start(PWM_PERIOD_PIN_53_H, PWM_DUTY_PIN_53_H);
	}else if (OutputStateMatrix[State][13] == 255) {
		pwm_pin53.start(PWM_PERIOD_PIN_53_L, PWM_DUTY_PIN_53_L);
	}else {
		stopGocue();
	}
	
	if (OutputStateMatrix[State][10] == 255) {
		pwm_pin44.start(PWM_PERIOD_PIN_44, PWM_DUTY_PIN_44);
	}
	
	if (OutputStateMatrix[State][11] == 255) {
		OptoStim();
	}
		
	if (InputStateMatrix[State][39] != State) {
		MeaningfulStateTimer = true;
	} else {
		MeaningfulStateTimer = false;
	}
}


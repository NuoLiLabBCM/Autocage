/* by Yaoyao
  changes from the original version from Sanworks.
  1. the whole structure was changed to adapt the slow commnucation via serial1 (instead of SerialUSB),
    see https://sanworks.io/forum/showthread.php?tid=142
  2. All the Bpod communication with PC was shut down
  3. comment green/blue led pin; CHANGE updateStatusLED()
  4. ValveRegWrite() w/ reg => ValveWrite() w/ digitalwritedirect()
  5. comment SyncRegWrite()
  6. do not send back envent during running.
  7. pwm1: true pwm used for go cue, etc. pwm2-8: stay the same, i.e., analogwrite()
   also UpdatePWMOutputStates() only work on pwm2-8
   in setStateOutputs(), change pwm1 to true pwm

  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod repository
  Copyright (C) 2016 Sanworks LLC, Sound Beach, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTIC ULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
// Bpod Finite State Machine v 0.5
// Requires the DueTimer library from:
// https://github.com/ivanseidel/DueTimer
#include <DueTimer.h>
#include <SPI.h>
#include "pwm_lib.h" // pwm lib for go cue sound

byte FirmwareBuildVersion = 6;
//////////////////////////////
// Hardware mapping:        //
//////////////////////////////

//     8 Sensor/Valve/LED Ports
byte PortDigitalInputLines[8] = {28, 30, 32, 34, 36, 38, 40, 42};
byte PortPWMOutputLines[8] = {53, 8, 7, 6, 5, 4, 3, 2}; // pwm1: gocue; pwm2-8: digital
//PWMH0_PC3  // PWM_CH0 pin35
//PWMH1_PA19 // PWM_CH1 pin42
//PWMH2_PB14 // PWM_CH2 pin53
using namespace arduino_due::pwm_lib;
// for go cue sound
#define PWM_PERIOD_PIN_53 28600 // 3.5 kHz        //10000 // 100 us, 10 kHz
#define PWM_DUTY_PIN_53   14300 // 50% duty
// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH2_PB14> pwm_pin53;




byte PortAnalogInputLines[8] = {0, 1, 2, 3, 4, 5, 6, 7}; // not used!!!

//     wire
byte WireDigitalInputLines[4] = {35, 33, 31, 29};
byte WireDigitalOutputLines[4] = {43, 41, 39, 37};

//     SPI device latch pins
//byte ValveRegisterLatch = 22;
//byte SyncRegisterLatch = 23;
byte ValveDigitalOutputLines[2] = {22, 23};

//      Bnc
byte BncOutputLines[2] = {10, 11};
byte BncInputLines[2] = {24, 25};

//      Indicator
byte RedLEDPin = 13;
//byte GreenLEDPin = 14;
//byte BlueLEDPin = 12;


//////////////////////////////////
// Initialize system state vars: /
//////////////////////////////////

byte PortPWMOutputState[8] = {0}; // State of all 8 output lines (As 8-bit PWM value from 0-255 representing duty cycle. PWM cycles at 1KHZ)
byte PortValveOutputState = 0;   // State of all 8 valves
byte PortInputsEnabled[8] = {0}; // Enabled or disabled input reads of port IR lines
byte WireInputsEnabled[4] = {0}; // Enabled or disabled input reads of wire lines
boolean PortInputLineValue[8] = {0}; // Direct reads of digital values of IR beams
boolean PortInputLineOverride[8] = {0}; // set to 1 if user created a virtual input, to prevent hardware reads until user returns it to low
boolean PortInputLineLastKnownStatus[8] = {0}; // Last known status of IR beams
boolean BNCInputLineValue[2] = {0}; // Direct reads of BNC input lines
boolean BNCInputLineOverride[2] = {0}; // Set to 1 if user created a virtual BNC high event, to prevent hardware reads until user returns low
boolean BNCInputLineLastKnownStatus[2] = {0}; // Last known status of BNC input lines
boolean WireInputLineValue[4] = {0}; // Direct reads of Wire terminal input lines
boolean WireInputLineOverride[2] = {0}; // Set to 1 if user created a virtual wire high event, to prevent hardware reads until user returns low
boolean WireInputLineLastKnownStatus[4] = {0}; // Last known status of Wire terminal input lines
boolean MatrixFinished = false; // Has the system exited the matrix (final state)?
boolean MatrixAborted = false; // Has the user aborted the matrix before the final state?
boolean MeaningfulStateTimer = false; // Does this state's timer get us to another state when it expires?
int CurrentState = 1; // What state is the state machine currently in? (State 0 is the final state)
int NewState = 1;
byte state_visited[1024] = {};
uint16_t nTransition = 0;

byte CurrentEvent[10] = {0}; // What event code just happened and needs to be handled. Up to 10 can be acquired per 30us loop.
byte nCurrentEvents = 0; // Index of current event
byte SoftEvent = 0; // What soft event code just happened


//////////////////////////////////
// Initialize general use vars:  /
//////////////////////////////////

byte CommandByte = 0;  // Op code to specify handling of an incoming USB Serial message
byte VirtualEventTarget = 0; // Op code to specify which virtual event type (Port, BNC, etc)
byte VirtualEventData = 0; // State of target
int nWaves = 0; // number of scheduled waves registered
byte CurrentWave = 0; // Scheduled wave currently in use
byte LowByte = 0; // LowByte through FourthByte are used for reading bytes that will be combined to 16 and 32 bit integers
byte SecondByte = 0;
byte ThirdByte = 0;
byte FourthByte = 0;
byte Byte1 = 0;
byte Byte2 = 0;
byte Byte3 = 0;
byte Byte4 = 0; // Temporary storage of command and param values read from Serial port
unsigned long LongInt = 0;
int nStates = 0;
uint16_t nEvents = 0;
int LEDBrightnessAdjustInterval = 5;
byte LEDBrightnessAdjustDirection = 1;
byte LEDBrightness = 0;
byte InputStateMatrix[128][40] = {0}; // Matrix containing all of Bpod's inputs and corresponding state transitions
// Cols: 0-15 = IR beam in...out... 16-19 = BNC1 high...low 20-27 = wire1high...low 28-37=SoftEvents 38=Unused  39=Tup

byte OutputStateMatrix[128][17] = {0}; // Matrix containing all of Bpod's output actions for each Input state
// Cols: 0=Valves 1=BNC 2=Wire 3=Hardware Serial 1 (UART) 4=Hardware Serial 2 (UART) 5 = SoftCode 6=GlobalTimerTrig 7=GlobalTimerCancel
// 8 = GlobalCounterReset 9-17=PWM values (LED channel on port interface board)

byte GlobalTimerMatrix[128][5] = {0}; // Matrix contatining state transitions for global timer elapse events
byte GlobalCounterMatrix[128][5] = {0}; // Matrix contatining state transitions for global counter threshold events
boolean GlobalTimersActive[5] = {0}; // 0 if timer x is inactive, 1 if it's active.
unsigned long GlobalTimerEnd[5] = {0}; // Future Times when active global timers will elapse
unsigned long GlobalTimers[5] = {0}; // Timers independent of states
unsigned long GlobalCounterCounts[5] = {0}; // Event counters
byte GlobalCounterAttachedEvents[5] = {254}; // Event each event counter is attached to
unsigned long GlobalCounterThresholds[5] = {0}; // Event counter thresholds (trigger events if crossed)
unsigned long TimeStamps[1024] = {0}; // TimeStamps for events on this trial
byte Events[1024] = {0};
int MaxTimestamps  = 1024; // Maximum number of timestamps (to check when to start event-dropping)
int MaxTransitions = 1024;
int CurrentColumn = 0; // Used when re-mapping event codes to columns of global timer and counter matrices
unsigned long StateTimers[128] = {0}; // Timers for each state
unsigned long StartTime = 0; // System Start Time
unsigned long MatrixStartTime = 0; // Trial Start Time
unsigned long MatrixStartTimeMillis = 0; // Used for 32-bit timer wrap-over correction in client
unsigned long StateStartTime = 0; // Session Start Time
unsigned long NextLEDBrightnessAdjustTime = 0;
byte ConnectedToClient = 0;
unsigned long CurrentTime = 0; // Current time (units = timer cycles since start; used to control state transitions)
unsigned long TimeFromStart = 0;
unsigned long Num2Break = 0; // For conversion from int32 to bytes
unsigned long SessionStartTime = 0;
byte connectionState = 0; // 1 if connected to MATLAB
byte RunningStateMatrix = 0; // 1 if state matrix is running

void setup() {
	for (int x = 0; x < 8; x++) {
		pinMode(PortDigitalInputLines[x], INPUT_PULLUP);
	}
	for(int x = 1; x < 8; x++) {
		pinMode(PortPWMOutputLines[x], OUTPUT);
		analogWrite(PortPWMOutputLines[x], 0);
	}
	for (int x = 0; x < 4; x++) {
		pinMode(WireDigitalInputLines[x], INPUT_PULLUP);
		pinMode(WireDigitalOutputLines[x], OUTPUT);
	}
	for (int x = 0; x < 2; x++) {
		pinMode(BncInputLines[x], INPUT);
		pinMode(BncOutputLines[x], OUTPUT);
	}
	for (int x = 0; x < 2; x++) {
		pinMode(ValveDigitalOutputLines[x],OUTPUT);
	}
	//pinMode(ValveRegisterLatch, OUTPUT); //  SPI device latch pins
	//pinMode(SyncRegisterLatch, OUTPUT);

	pinMode(RedLEDPin, OUTPUT);
	//pinMode(GreenLEDPin, OUTPUT);
	//pinMode(BlueLEDPin, OUTPUT);

	Serial1.begin(115200);
	SerialUSB.begin(115200);
	//SPI.begin();
	SetWireOutputLines(0);
	SetBNCOutputLines(0);
	updateStatusLED(0); // LED OFF
	ValveWrite(0);
	Timer3.attachInterrupt(handler);
	Timer3.setPeriod(100); // Runs every 100us
}

void loop() {
	// Check connection
	if (connectionState == 0) {
		updateStatusLED(1); // BLINK: waiting for connection
	}

	if (Serial1.available() > 0) {
		CommandByte = Serial1.read();  // P for Program, R for Run, O for Override, 6 for Device ID
		//SerialUSB.print("commond: ");
		//SerialUSB.println(CommandByte);
		switch (CommandByte) {
		case '6':  // Initialization handshake
			connectionState = 1;
			updateStatusLED(2); // ON: CONNECTED
			Serial1.print(5);   // returns a byte: '5' (ASCII 53) to confirm the connection
			//delayMicroseconds(100000);
			delay(100);
			Serial1.flush();
			SessionStartTime = millis();
			break;
		case 'F':  // Return firmware build number
			//Serial1.write(FirmwareBuildVersion);
			ConnectedToClient = 1;
			break;
		case 'O':  // Override hardware state
			manualOverrideOutputs();
			break;
		case 'I': // Read and return digital input line states
			Byte1 = Serial1ReadByte();
			Byte2 = Serial1ReadByte();
			switch (Byte1) {
			case 'B': // Read BNC input line
				Byte3 = digitalReadDirect(BncInputLines[Byte2]);
				break;
			case 'P': // Read port digital input line
				Byte3 = digitalReadDirect(PortDigitalInputLines[Byte2]);
				break;
			case 'W': // Read wire digital input line
				Byte3 = digitalReadDirect(WireDigitalInputLines[Byte2]);
				break;
			}
			//Serial1.write(Byte3);
			break;
		case 'Z':  // Bpod governing machine has closed the client program
			//SerialUSB.println("Z");
			ConnectedToClient = 0;
			connectionState = 0;
			//Serial1.write('1');
			updateStatusLED(0); // LED OFF
			break;
		case 'S': // Soft code.
			VirtualEventTarget = Serial1ReadByte();
			VirtualEventData = Serial1ReadByte();
			break;
		case 'H': // Recieve byte from USB and send to Serial module 1 or 2
			Byte1 = Serial1ReadByte();
			Byte2 = Serial1ReadByte();
			if (Byte1 == 1) {
				//Serial1.write(Byte2);
			} else if (Byte1 == 2) {
				//Serial2.write(Byte2);
			}
			break;
		case 'V': // Manual override: execute virtual event
			VirtualEventTarget = Serial1ReadByte();
			VirtualEventData = Serial1ReadByte();
			if (RunningStateMatrix) {
				switch (VirtualEventTarget) {
				case 'P': // Virtual poke PortInputLineLastKnownStatus
					if (PortInputLineLastKnownStatus[VirtualEventData] == LOW) {
						PortInputLineValue[VirtualEventData] = HIGH;
						PortInputLineOverride[VirtualEventData] = true;
					} else {
						PortInputLineValue[VirtualEventData] = LOW;
						PortInputLineOverride[VirtualEventData] = false;
					}
					break;
				case 'B': // Virtual BNC input
					if (BNCInputLineLastKnownStatus[VirtualEventData] == LOW) {
						BNCInputLineValue[VirtualEventData] = HIGH;
						BNCInputLineOverride[VirtualEventData] = true;
					} else {
						BNCInputLineValue[VirtualEventData] = LOW;
						BNCInputLineOverride[VirtualEventData] = false;
					}
					break;
				case 'W': // Virtual Wire input
					if (WireInputLineLastKnownStatus[VirtualEventData] == LOW) {
						WireInputLineValue[VirtualEventData] = HIGH;
						WireInputLineOverride[VirtualEventData] = true;
					} else {
						WireInputLineValue[VirtualEventData] = LOW;
						WireInputLineOverride[VirtualEventData] = false;
					}
					break;
				case 'S':  // Soft event
					SoftEvent = VirtualEventData;
					break;
				}
			}
			break;
		case 'P':  // Get new state matrix from client
			//Serial.println("Start Transmitting Matrix.\n");
			//Serial.flush();
			//SerialUSB.println("P");
			nStates = Serial1ReadByte();
			//SerialUSB.println(nStates);
			// Get Input state matrix
			for (int x = 0; x < nStates; x++) {
				for (int y = 0; y < 40; y++) {
					InputStateMatrix[x][y] = Serial1ReadByte();
				}
			}
			// Get Output state matrix
			for (int x = 0; x < nStates; x++) {
				for (int y = 0; y < 17; y++) {
					OutputStateMatrix[x][y] = Serial1ReadByte();
				}
			}

			// Get global timer matrix
			for (int x = 0; x < nStates; x++) {
				for (int y = 0; y < 5; y++) {
					GlobalTimerMatrix[x][y] = Serial1ReadByte();
				}
			}

			// Get global counter matrix
			for (int x = 0; x < nStates; x++) {
				for (int y = 0; y < 5; y++) {
					GlobalCounterMatrix[x][y] = Serial1ReadByte();
				}
			}

			// Get global counter attached events
			for (int x = 0; x < 5; x++) {
				GlobalCounterAttachedEvents[x] = Serial1ReadByte();
			}

			// Get input channel configurtaion
			for (int x = 0; x < 8; x++) {
				PortInputsEnabled[x] = Serial1ReadByte();
			}

			for (int x = 0; x < 4; x++) {
				WireInputsEnabled[x] = Serial1ReadByte();
			}


			// Get state timers
			for (int x = 0; x < nStates; x++) {
				StateTimers[x] = Serial1ReadLong();
			}

			// Get global timers
			for (int x = 0; x < 5; x++) {
				GlobalTimers[x] = Serial1ReadLong();
			}

			// Get global counter event count thresholds
			for (int x = 0; x < 5; x++) {
				GlobalCounterThresholds[x] = Serial1ReadLong();
			}
			//Serial.println("Transmit Completed.\n");
			//Serial.flush();
			Serial1.write(1);
			break;
		case 'R':  // Run State Matrix
			//SerialUSB.println("R");
			Serial1.write(1);
			if(RunningStateMatrix == 1){
				RunningStateMatrix = 0;
				Timer3.stop();
			}
			updateStatusLED(3); // HALF ON: RUNNING...
			NewState = 0;
			CurrentState = 0;
			nEvents = 0;
			SoftEvent = 254; // No event
			MatrixFinished = false;

			// Reset event counters
			for (int x = 0; x < 5; x++) {
				GlobalCounterCounts[x] = 0;
			}
			// Read initial state of sensors
			for (int x = 0; x < 8; x++) {
				if (PortInputsEnabled[x] == 1) {
					PortInputLineValue[x] = digitalReadDirect(PortDigitalInputLines[x]); // Read each photogate's current state into an array
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
			for (int x = 0; x < 2; x++) {
				BNCInputLineValue[x] = digitalReadDirect(BncInputLines[x]);
				if (BNCInputLineValue[x] == HIGH) {
					BNCInputLineLastKnownStatus[x] = true;
				} else {
					BNCInputLineLastKnownStatus[x] = false;
				}
				BNCInputLineOverride[x] = false;
			}
			for (int x = 0; x < 4; x++) {
				if (WireInputsEnabled[x] == 1) {
					WireInputLineValue[x] = digitalReadDirect(WireDigitalInputLines[x]);
					if (WireInputLineValue[x] == HIGH) {
						WireInputLineLastKnownStatus[x] = true;
					} else {
						WireInputLineLastKnownStatus[x] = false;
					}
				}
				WireInputLineOverride[x] = false;
			}
			// Reset timers
			MatrixStartTime = 0;
			StateStartTime = MatrixStartTime;
			CurrentTime = MatrixStartTime;
			MatrixStartTimeMillis = millis();

			nTransition = 0;
			state_visited[nTransition++] = CurrentState;

			// Adjust outputs, scheduled waves, Serial codes and sync port for first state
			setStateOutputs(CurrentState);
			RunningStateMatrix = 1;

			//Serial.println("start Timer3");
			Timer3.start(); // Runs every 100us

			break;
		case 'X':   // Exit state matrix and return data
			MatrixFinished = true;
			RunningStateMatrix = false;
			updateStatusLED(0); // OFF
			Timer3.stop();      // stop timer
			// setStateOutputs(0); // Returns all lines to low by forcing final state
			break;
		case 'K':  // get struck
			//SerialUSB.println("R");
			if(RunningStateMatrix == 1){
				RunningStateMatrix = 0;
				Timer3.stop();
			}
			for(int x = 0; x < 6148; x++){
				Serial1.write(byte(0));
			}
     break;

		default:
			break;
		} // End switch commandbyte
	} // End Serial1.available


	if (MatrixFinished) {
		MatrixFinished = 0;
		//SyncRegWrite(0); // Reset the sync lines
		ValveWrite(0); // Reset valves
		for (int x = 0; x < 8; x++) { // Reset PWM lines
			PortPWMOutputState[x] = 0;
		}
		//UpdatePWMOutputStates();
		SetBNCOutputLines(0); // Reset BNC outputs
		SetWireOutputLines(0); // Reset wire outputs
		Serial1.write(1); // Op Code for sending events
		//delay(1);
		//Serial1.write(1); // Read one event
		//Serial1.write(255); // Send Matrix-end code
		// Send trial-start timestamp (in milliseconds, basically immune to microsecond 32-bit timer wrap-over)
		//Serial1WriteLong(MatrixStartTimeMillis - SessionStartTime);
		// Send matrix start timestamp (in microseconds)
		//Serial1WriteLong(TimeFromStart);//MatrixStartTime // current trial duration new

		//SerialUSB.println(nEvents);
		Serial1WriteShort(nEvents);
		delayMicroseconds(100);
		for (int x = 0; x < nEvents; x++) {
			Serial1.write(Events[x]); // new
			Serial1WriteLong(TimeStamps[x]);
		}
		//SerialUSB.println(nTransition);
		Serial1WriteShort(nTransition);
		delayMicroseconds(100);
		for (int x = 0; x < nTransition; x++) {
			Serial1.write(state_visited[x]); // new
			delayMicroseconds(50);
			//SerialUSB.println(state_visited[x]);
		}

		updateStatusLED(0); // OFF
		for (int x = 0; x < 5; x++) { // Shut down active global timers
			GlobalTimersActive[x] = false;
		}
	} // End Matrix finished
}


//***************** Timer Callback Function ***************
void handler() {
	if (RunningStateMatrix) {
		nCurrentEvents = 0;
		CurrentEvent[0] = 254; // Event 254 = No event
		CurrentTime++;         // 0.1 ms step increment
		// Refresh state of sensors and inputs
		for (int x = 0; x < 8; x++) {
			if ((PortInputsEnabled[x] == 1) && (!PortInputLineOverride[x])) {
				PortInputLineValue[x] = digitalReadDirect(PortDigitalInputLines[x]);
			}
		}
		for (int x = 0; x < 2; x++) {
			if (!PortInputLineOverride[x]) {
				BNCInputLineValue[x] = digitalReadDirect(BncInputLines[x]);
			}
		}
		for (int x = 0; x < 4; x++) {
			if ((WireInputsEnabled[x] == 1) && (!WireInputLineOverride[x])) {
				WireInputLineValue[x] = digitalReadDirect(WireDigitalInputLines[x]);
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
		// Determine which BNC event occurred
		for (int x = 0; x < 2; x++) {
			// Determine BNC low-to-high events
			if ((BNCInputLineValue[x] == HIGH) && (BNCInputLineLastKnownStatus[x] == LOW)) {
				BNCInputLineLastKnownStatus[x] = HIGH;
				CurrentEvent[nCurrentEvents] = Ev;
				nCurrentEvents++;
			}
			Ev = Ev + 1;
			// Determine BNC high-to-low events
			if ((BNCInputLineValue[x] == LOW) && (BNCInputLineLastKnownStatus[x] == HIGH)) {
				BNCInputLineLastKnownStatus[x] = LOW;
				CurrentEvent[nCurrentEvents] = Ev;
				nCurrentEvents++;
			}
			Ev = Ev + 1;
		}
		// Determine which Wire event occurred
		for (int x = 0; x < 4; x++) {
			// Determine Wire low-to-high events
			if ((WireInputLineValue[x] == HIGH) && (WireInputLineLastKnownStatus[x] == LOW)) {
				WireInputLineLastKnownStatus[x] = HIGH;
				CurrentEvent[nCurrentEvents] = Ev;
				nCurrentEvents++;
			}
			Ev = Ev + 1;
			// Determine Wire high-to-low events
			if ((WireInputLineValue[x] == LOW) && (WireInputLineLastKnownStatus[x] == HIGH)) {
				WireInputLineLastKnownStatus[x] = LOW;
				CurrentEvent[nCurrentEvents] = Ev;
				nCurrentEvents++;
			}
			Ev = Ev + 1;
		}
		// Map soft events to event code scheme
		if (SoftEvent < 254) {
			CurrentEvent[nCurrentEvents] = SoftEvent + Ev - 1;
			nCurrentEvents++;
			SoftEvent = 254;
		}
		Ev = 40;
		// Determine if a global timer expired
		for (int x = 0; x < 5; x++) {
			if (GlobalTimersActive[x] == true) {
				if (CurrentTime >= GlobalTimerEnd[x]) {
					CurrentEvent[nCurrentEvents] = Ev;
					nCurrentEvents++;
					GlobalTimersActive[x] = false;
				}
			}
			Ev = Ev + 1;
		}
		// Determine if a global event counter threshold was exceeded
		for (int x = 0; x < 5; x++) {
			if (GlobalCounterAttachedEvents[x] < 254) {
				// Check for and handle threshold crossing
				if (GlobalCounterCounts[x] == GlobalCounterThresholds[x]) {
					CurrentEvent[nCurrentEvents] = Ev;
					nCurrentEvents++;
				}
				// Add current event to count (Crossing triggered on next cycle)
				for (int i = 0; i < nCurrentEvents; i++) {
					if (CurrentEvent[i] == GlobalCounterAttachedEvents[x]) {
						GlobalCounterCounts[x] = GlobalCounterCounts[x] + 1;
					}
				}
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
			if (CurrentEvent[i] < 40) {
				NewState = InputStateMatrix[CurrentState][CurrentEvent[i]];
			} else if (CurrentEvent[i] < 45) {
				CurrentColumn = CurrentEvent[0] - 40;
				NewState = GlobalTimerMatrix[CurrentState][CurrentColumn];
			} else if (CurrentEvent[i] < 50) {
				CurrentColumn = CurrentEvent[i] - 45;
				NewState = GlobalCounterMatrix[CurrentState][CurrentColumn];
			}
			if (NewState != CurrentState) {
				StateTransitionFound = 1;
			}
			i++;
		}
		// Store timestamp of events captured in this cycle
		if ((nEvents + nCurrentEvents) < MaxTimestamps) {
			for (int x = 0; x < nCurrentEvents; x++) {
				TimeStamps[nEvents] = CurrentTime;
				Events[nEvents] = CurrentEvent[x]; // new
				nEvents++;
			}
		}
		// Make state transition if necessary
		if (NewState != CurrentState) {
			//Serial.write("State changed to ");
			//Serial.write(NewState);
			//Serial.flush();
			if (nTransition < MaxTransitions) {
				state_visited[nTransition++] = NewState;
			}

			if (NewState == nStates) {
				RunningStateMatrix = false;
				MatrixFinished = true;

				Timer3.stop();
				//Serial.println("Timer3 Stop");

			} else {
				setStateOutputs(NewState);
				StateStartTime = CurrentTime;
				CurrentState = NewState;
			}
		}

		/*
		  // Write events captured to USB (if events were captured)
		  if (nCurrentEvents > 0) {
		  Serial1.write(1); // Code for returning events
		  Serial1.write(nCurrentEvents);
		  for (int x = 0; x < nCurrentEvents; x++) {
		    Serial1.write(CurrentEvent[x]);
		  }
		  }
		*/

	} // End running state matrix
	// Serial.write(CurrentTime);
	return;
} // End timer handler



void SetBNCOutputLines(int BNCState) {
	switch (BNCState) {
	case 0: {
		digitalWriteDirect(BncOutputLines[0], LOW);
		digitalWriteDirect(BncOutputLines[1], LOW);
	}
	break;
	case 1: {
		digitalWriteDirect(BncOutputLines[0], HIGH);
		digitalWriteDirect(BncOutputLines[1], LOW);
	}
	break;
	case 2: {
		digitalWriteDirect(BncOutputLines[0], LOW);
		digitalWriteDirect(BncOutputLines[1], HIGH);
	}
	break;
	case 3: {
		digitalWriteDirect(BncOutputLines[0], HIGH);
		digitalWriteDirect(BncOutputLines[1], HIGH);
	}
	break;
	}
}

int ValveWrite(int WireState) {
	// // Write to water chip
	// SPI.transfer(value);
	// digitalWriteDirect(ValveRegisterLatch, HIGH);
	// digitalWriteDirect(ValveRegisterLatch, LOW);

	for (int x = 0; x < 2; x++) {
		digitalWriteDirect(ValveDigitalOutputLines[x], bitRead(WireState, x));
	}
}

/*
  int SyncRegWrite(int value) {
  // Write to LED driver chip
  SPI.transfer(value);
  digitalWriteDirect(SyncRegisterLatch, HIGH);
  digitalWriteDirect(SyncRegisterLatch, LOW);
  }
*/

void UpdatePWMOutputStates() {
	if (PortPWMOutputState[0] == 255) {
		pwm_pin53.start(PWM_PERIOD_PIN_53, PWM_DUTY_PIN_53);
	} else {
		pwm_pin53.stop();
	}
	for (int x = 1; x < 8; x++) {
		analogWrite(PortPWMOutputLines[x], PortPWMOutputState[x]);
	}
}
void SetWireOutputLines(int WireState) {
	for (int x = 0; x < 4; x++) {
		digitalWriteDirect(WireDigitalOutputLines[x], bitRead(WireState, x));
	}
}

void updateStatusLED(int Mode) {
	CurrentTime = millis();
	switch (Mode) {
	case 0: { // OFF
		analogWrite(RedLEDPin, 0);
		//digitalWriteDirect(GreenLEDPin, 0);
		//analogWrite(BlueLEDPin, 0);
	}
	break;
	case 1: { // Waiting for matrix: BLINK
		if (ConnectedToClient == 0) {
			if (CurrentTime > NextLEDBrightnessAdjustTime) {
				NextLEDBrightnessAdjustTime = CurrentTime + LEDBrightnessAdjustInterval;
				if (LEDBrightnessAdjustDirection == 1) {
					if (LEDBrightness < 255) {
						LEDBrightness = LEDBrightness + 1;
					} else {
						LEDBrightnessAdjustDirection = 0;
					}
				}
				if (LEDBrightnessAdjustDirection == 0) {
					if (LEDBrightness > 0) {
						LEDBrightness = LEDBrightness - 1;
					} else {
						LEDBrightnessAdjustDirection = 2;
					}
				}
				if (LEDBrightnessAdjustDirection == 2) {
					NextLEDBrightnessAdjustTime = CurrentTime + 500;
					LEDBrightnessAdjustDirection = 1;
				}
				analogWrite(RedLEDPin, LEDBrightness);
			}
		}
	}
	break;
	case 2: { // ON
		//analogWrite(BlueLEDPin, 0);
		digitalWriteDirect(RedLEDPin, 1);
	}
	break;
	case 3: { // HALF ON
		//analogWrite(BlueLEDPin, 0);
		//digitalWriteDirect(GreenLEDPin, 1);
		analogWrite(RedLEDPin, 100);
	}
	break;
	}
}

void setStateOutputs(byte State) {
	byte CurrentTimer = 0; // Used when referring to the timer currently being triggered
	byte CurrentCounter = 0; // Used when referring to the counter currently being reset
	ValveWrite(OutputStateMatrix[State][0]);        // Cols0 = Valves; Value: 0, 1, 2, 3
	SetBNCOutputLines(OutputStateMatrix[State][1]); // 1=BNC
	SetWireOutputLines(OutputStateMatrix[State][2]);// 2=Wire
	//Serial1.write(OutputStateMatrix[State][3]);   // 3=Hardware Serial 1 (UART)
	//Serial2.write(OutputStateMatrix[State][4]);   // 4=Hardware Serial 2 (UART)
	if (OutputStateMatrix[State][5] > 0) {          // 5 = SoftCode
		//Serial1.write(2); // Code for soft-code byte
		//Serial1.write(OutputStateMatrix[State][5]); // Code for soft-code byte
	}

	// 9-16=PWM values (LED channel on port interface board)
	if (OutputStateMatrix[State][9] == 255) {
		pwm_pin53.start(PWM_PERIOD_PIN_53, PWM_DUTY_PIN_53);
	} else {
		pwm_pin53.stop();
	}
	for (int x = 1; x < 8; x++) {
		analogWrite(PortPWMOutputLines[x], OutputStateMatrix[State][x + 9]);
	}
	// Trigger global timers
	CurrentTimer = OutputStateMatrix[State][6];
	if (CurrentTimer > 0) {
		CurrentTimer = CurrentTimer - 1; // Convert to 0 index
		GlobalTimersActive[CurrentTimer] = true;
		GlobalTimerEnd[CurrentTimer] = CurrentTime + GlobalTimers[CurrentTimer];
	}
	// Cancel global timers
	CurrentTimer = OutputStateMatrix[State][7];
	if (CurrentTimer > 0) {
		CurrentTimer = CurrentTimer - 1; // Convert to 0 index
		GlobalTimersActive[CurrentTimer] = false;
	}
	// Reset event counters
	CurrentCounter = OutputStateMatrix[State][8];
	if (CurrentCounter > 0) {
		CurrentCounter = CurrentCounter - 1; // Convert to 0 index
		GlobalCounterCounts[CurrentCounter] = 0;
	}
	if (InputStateMatrix[State][39] != State) {
		MeaningfulStateTimer = true;
	} else {
		MeaningfulStateTimer = false;
	}
	//SyncRegWrite((State + 1)); // Output binary state code, corrected for zero index
}

void manualOverrideOutputs() {
	byte OutputType = 0;
	OutputType = Serial1ReadByte();
	switch (OutputType) {
	case 'P':  // Override PWM lines
		for (int x = 0; x < 8; x++) {
			PortPWMOutputState[x] = Serial1ReadByte();
		}
		UpdatePWMOutputStates();
		break;
	case 'V':  // Override valves
		Byte1 = Serial1ReadByte();
		ValveWrite(Byte1);
		break;
	case 'B': // Override BNC lines
		Byte1 = Serial1ReadByte();
		SetBNCOutputLines(Byte1);
		break;
	case 'W':  // Override wire terminal output lines
		Byte1 = Serial1ReadByte();
		SetWireOutputLines(Byte1);
		break;
	case 'S': // Override Serial module port 1
		Byte1 = Serial1ReadByte();  // Read data to send
		//Serial1.write(Byte1);
		break;
	case 'T': // Override Serial module port 2
		Byte1 = Serial1ReadByte();  // Read data to send
		//Serial2.write(Byte1);
		break;
	}
}

void digitalWriteDirect(int pin, boolean val) {
	if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
	else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

byte digitalReadDirect(int pin) {
	return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

void Serial1WriteLong(unsigned long num) {
	Serial1.write((byte)num);
	Serial1.write((byte)(num >> 8));
	Serial1.write((byte)(num >> 16));
	Serial1.write((byte)(num >> 24));
	delayMicroseconds(100);
}

void Serial1WriteShort(word num) {
	Serial1.write((byte)num);
	Serial1.write((byte)(num >> 8));
	delayMicroseconds(100);
}

unsigned long Serial1ReadLong() {
	while (Serial1.available() == 0) {}
	LowByte = Serial1.read();
	while (Serial1.available() == 0) {}
	SecondByte = Serial1.read();
	while (Serial1.available() == 0) {}
	ThirdByte = Serial1.read();
	while (Serial1.available() == 0) {}
	FourthByte = Serial1.read();
	LongInt =  (unsigned long)(((unsigned long)FourthByte << 24) | ((unsigned long)ThirdByte << 16) | ((unsigned long)SecondByte << 8) | ((unsigned long)LowByte));
	return LongInt;
}

byte Serial1ReadByte() {
	while (Serial1.available() == 0) {}
	LowByte = Serial1.read();
	return LowByte;
}

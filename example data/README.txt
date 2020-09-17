/* There are six files in the folder for each mouse, They are:
 * PARAS.TXT:	parameters for task/trials
 * PARAF.TXT:	parameters for head-fixation
 * TRIALS.TXT:	data for each trial
 * EVENTS.TXT:	data for each event
 * EVENTST.TXT:	data for the detailed events for each trial
 * WEIGHT.TXT:	weight data read from scale (20 Hz sample rate, 
 * 				only record weights > 10 g); ONLY available in version 1.0
 */


 
/*************TRIALS.TXT*************
 * 
 * Each row is for one trial;
 * 
 * Column1: Unix time when the trial is finished;
 * Column2: trial number;
 * Column3: ProtocolType: 10-Fixation; 21-Sample; 22-Delay; 23-Optostim
 * Column4: subProtocol number: 0-fixation; 1~9-sample; 10~18-delay; 19-optostim
 *			0: from begining to head-fixation for 25 sec; Sample = 1.3 sec, random 
 *			   	delay (0.2~0.5 sec), no error trials
 *			1: autoLearn ON, random delay (0.3~0.6 sec), no error trials, 
 *				advance: reach 100 trials;
 *			3: autoLearn ON, random delay (0.3~0.6 sec), with error trials, advance: 
				reach 200 trials;
 *			4: autoLearn antiBias, random delay (0.3~0.6 sec), advance: performance > 75%
 *			13: antiBias, Delay = 0.3 sec, forced delay, advance: performance > 70%
 *			14: antiBias, Delay = 0.5 sec, forced delay, advance: performance > 70%
 *			15: antiBias, Delay = 0.8 sec, forced delay, advance: performance > 70%
 *			16: antiBias, Delay = 1.0 sec, forced delay, advance: performance > 70%
 *			17: antiBias, Delay = 1.3 sec, forced delay, advance: performance > 70%
 *			18: antiBias, Delay = 1.3 sec, forced delay, timeOut->4 sec, 
 *				advance: performance > 70%, fixation duration 30->60 sec
 *			19: antiBias, Delay = 1.3 sec, 15% trials with stimulation during sample/delay/response
 * Column5: Trial type: 0-right; 1-left; 2-either side
 * Column6: Trial outcome: 0-No Response; 1-Reward; 2-Time Out (error); 3-'Others'
 * Column7: Optostim flag: percentage of power (10, 30, 50, 70 100%) + stim epoch (1-stim during sample; 2-delay; 3-response)
 *			e.g., 11 means stim during sample with 10% power
 *			0 means no stimulation in this trial.
 * Column8: Forward/Backward motor position: 0-255
 * Column9: Left/Right motor position: 0-255 (70 is roughly the center)
 * Column10-end: States visited in this trial (see state machine coded in Arduino_Master.ino)
 * 
 **************TRIALS.TXT*************/


 
/*************EVENTS.TXT*************
 * 
 * Each row is one event recorded.
 * 
 * Column1: Time stamp for the event; if the event is 1, the time is Unix time; 
 * 			otherwise it is milisec after restart.
 * Column2: Event type: 1-Arduino restarted; 7-switchTriggered; 8-headfixation; 
 *			11-headfixation again immediately after release
			9-release1:timeup; 10-release2:escape; 12-release3:struggle
			20~22-user updating FB_motor_position, LR_motor_position and FB_final_position;
 * Column3: Event value: -1 means N/A, otherwise is value corresponding to the event.
 * 
 **************EVENTS.TXT*************/
 
 
 
/*************EVENTST.TXT*************
 * 
 * Each row is for one trial
 * 
 * Column1: Trial number;
 * Column2: Number of events in this trial;
 * Column3~end: pairs of event ID and timestamp
 * 				ID: 39-timeup (see state machine coded in Arduino_Master.ino for details); 
 *					0-left lickport in; 1-left lickport out; 
 *					2-right lickport in; 3-right lickport out;
 * 				timestamp: unit: 0.1 ms; 0 is start of a trial
 *
 **************EVENTST.TXT*************/


/*************WEIGHT.TXT*************
 *
 * Each row is a 2-sec data segment
 * 
 * Column1: Unix time when the data were sent;
 * Column2-41: weight data sampled at 20 Hz
 *
*************WEIGHT.TXT*************/

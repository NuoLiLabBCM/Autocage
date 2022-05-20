#include "pwm_lib.h" // pwm lib for mask flashing
#include <DueTimer.h>  // Timer interrupt
#include <Wire.h>
#include <Adafruit_MCP4725.h>

const byte ledPin = 13;
const byte flashOutputPin = 53;
const byte MaskflashPin = 22;
const byte OptostimPin = 23;

using namespace arduino_due::pwm_lib;
// for mask led
#define PWM_PERIOD_PIN_53 10000000 // 100 ms, 10 Hz      //10000 // 100 us, 10 kHz
#define PWM_DUTY_PIN_53   100000  //  1%                //2000  // 20% duty
// defining pwm object using pin 35, pin PC3 mapped to pin 35 on the DUE
// this object uses PWM channel 0
pwm<pwm_pin::PWMH2_PB14> pwm_pin53;

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


byte weightByte = 100;
float powerWeight = 1.0;
byte laserByte = 1; // 1-using thorlab laser; 2-using Ultra laser
byte led_state = LOW;

void setup() {
  //SerialUSB.begin(115200);

  Serial2.begin(115200);     // To Bpod
  while (Serial2.available()) {
    Serial2.read(); // clear serial1 dirty data
  }

  pinMode(ledPin, OUTPUT);
  digitalWriteDirect(ledPin, led_state);

  pinMode(flashOutputPin, OUTPUT);
  digitalWriteDirect(flashOutputPin, LOW);

  pinMode(MaskflashPin, INPUT_PULLUP);
  pinMode(OptostimPin, INPUT_PULLUP);

  randomSeed(analogRead(0));

  attachInterrupt(digitalPinToInterrupt(MaskflashPin), MaskFlash, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OptostimPin), OptoStim, CHANGE);

  Timer3.attachInterrupt(flash5sec_handler); // in case not receiving flash falling event, run flash max 5 sec
  Timer3.setPeriod(5000000); // Runs  5 sec later to stop flash
  flash5sec_handler();

  Timer4.attachInterrupt(timer_updateDAC); //
  Timer4.setPeriod(500); // every 0.5 ms to update DAC value

  // For Adafruit MCP4725A1 the address is 0x62 (default) or 0x63 (ADDR pin tied to VCC)
  // For MCP4725A0 the address is 0x60 or 0x61
  // For MCP4725A2 the address is 0x64 or 0x65
  dac.begin(0x62);
  dac.setVoltage(0, false);
}

void loop() {
  //digitalWrite(ledPin, state);
  if (Serial2.available()) { // receiving data from Controller
    if (led_state == LOW) {
      led_state = HIGH;
      digitalWriteDirect(ledPin, led_state);
    } else {
      led_state = LOW;
      digitalWriteDirect(ledPin, led_state);
    }
    weightByte = Serial2.read();
    powerWeight = (float)weightByte / 100;
    while(Serial2.available()==0){};
    laserByte = Serial2.read();
    //SerialUSB.println(weightByte);
    //SerialUSB.println(laserByte);
  }
}

void MaskFlash() {
  if (digitalReadDirect(MaskflashPin) == HIGH) { // rising
    //SerialUSB.println("mask rising");
    //digitalWriteDirect(ledPin, HIGH);
    pwm_pin53.start(PWM_PERIOD_PIN_53, PWM_DUTY_PIN_53);
    Timer3.start();
  }
  /*
    else
    { // falling
    //SerialUSB.println("mask falling");
    //digitalWriteDirect(ledPin, LOW);
    pwm_pin53.stop();
    Timer3.stop();
    pinMode(flashOutputPin, OUTPUT);
    digitalWrite(flashOutputPin, LOW);
    }
  */
}

void flash5sec_handler() {
  Timer3.stop();
  pwm_pin53.stop();
  pinMode(flashOutputPin, OUTPUT);
  digitalWriteDirect(flashOutputPin, LOW);
}


void OptoStim() {
  if (digitalReadDirect(OptostimPin) == HIGH) { // rising
    //SerialUSB.println("OptostimPin rising");
    //digitalWriteDirect(ledPin, HIGH);
    sample_ind = 0; // first output
    if (laserByte == 1) {
      dac.setVoltage(uint16_t(powerWeight * thorlab_flat_sine[0]), false);
    } else {
      dac.setVoltage(uint16_t(powerWeight * (ultra_flat_sine[0] - 655) + 655), false); // 655 is 0.8 V
    }
    Timer4.start();
  }
  /*
    else { // falling
    //SerialUSB.println("OptostimPin falling");
    //digitalWriteDirect(ledPin, LOW);
    Timer4.stop();
    dac.setVoltage(0, false); // output 0
    } */

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

void digitalWriteDirect(int pin, boolean val) {
  if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
  else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
}

byte digitalReadDirect(int pin) {
  return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
}

/*                                             ATC SYSTEM II {Code by RAJA MUHAMMAD AHSAN}
 *  Description:
 *            This code is intended controlling six (6) TECs mounted on the CTGP-II by utilization of PID controller.
 *            For controlling the TEC using PID. we first set the AMBIENT temperature level 
 *              1. If the SP is set above the AMBIENT level than the PID algorithm is activated, which helps to retain the PV to the desired SP.
 *                 For improving the PID response for the wide temperature window above the SP i.e. 23 - 50 [degC], the temperature window is 
 *                 divided into five (5) sub-windows for which different sets of PID gain constants are adopted.  
 *              For the SP below AMBIENT level the PID does not locks to the desired SP and the error of 1-1.5 [degC] is retained above the SP.
 *              The reason for this, is that the calculated PWM value by PID is not enough to cools the TEC to the SP. For this reason following
 *              considerations are adopted:
 *              1. A Hysterisis window is defined above and below the SP, initially the TEC is heated/cooled with PWM DC value (250)
 *                 if the PV reaches above the upper-threshold level (UTL) tnan the TEC is cooled with the optimum level of PWM DC value and if the  
 *                 PV reaches below the lower-threshold level (LTL) than the TEC is heated with the optimum level of PWM DC value. For this it is 
 *                 required to keep the track of previous SP
 *                 This condition is described below:
 *                 a. If the OLD_SP is set ABOVE the AMBIENT and the NEW_SP is set BELOW AMBIENT: 
 *                      then cool the TEC with PWM maximum DC (i.e. 250). 
 *                        i. When the PV reaches in the range of window-II below the NEW_SP 
 *                           [i.e. if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))]   
 *                              than switch the TEC polarity (HEATING) with the optimum level of PWM DC
 *                        ii. When the PV reaches in the range of window-I above the NEW_SP
 *                           [i.e. if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))]
 *                              than switch the TEC polarity (COOLING) with the optimum level of PWM DC                      
 *                              
 *                 b. If the OLD_SP is set BELOW the AMBIENT and the NEW_SP is also set BELOW AMBIENT, but greater than the OLD_SP: 
 *                      then heat the TEC with PID calculated PWM DC. 
 *                        i. When the PV reaches in the range of window-II below the NEW_SP 
 *                           [i.e. if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))]   
 *                              than switch the TEC polarity (HEATING) with the optimum level of PWM DC
 *                        ii. When the PV reaches in the range of window-I above the NEW_SP
 *                           [i.e. if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))]
 *                              than switch the TEC polarity (COOLING) with the optimum level of PWM DC                      
 *                              
 *                 c. If the OLD_SP is set BELOW the AMBIENT and the NEW_SP is also set BELOW AMBIENT, but lesser than the OLD_SP: 
 *                      then cool the TEC with PWM maximum DC (i.e. 250).
 *                        i. When the PV reaches in the range of window-II below the NEW_SP 
 *                           [i.e. if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))]   
 *                              than switch the TEC polarity (HEATING) with the optimum level of PWM DC
 *                        ii. When the PV reaches in the range of window-I above the NEW_SP
 *                           [i.e. if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))]
 *                              than switch the TEC polarity (COOLING) with the optimum level of PWM DC
 *                 
 *                 ----------------------------------------------------------------------------------------------------------------------
 *                 | NOTE: The reason for adoptation of PWM maximum DC value for cooling the TEC instead of PID calculated PWM value is | 
 *                 | that, the PWM DC value calculated by PID is lower than the DC maximum vale and not sufficient to cool the TEC to   |
 *                 | the desired SP.                                                                                                    |
 *                 ----------------------------------------------------------------------------------------------------------------------
 */
 
#include <PID_v1.h>

int count=1, c, time=0.5;  //time=0.5
double PWM_min = 0, PWM_max = 255;                // Specify Minimum and Maximum PWM value for controller

double Output1, gap;                              // Define Output variable of PID1 for TEC-1
double Output2, gap1;                             // Define Output variable of PID2 for TEC-2
double Output3, gap2;                             // Define Output variable of PID3 for TEC-3
double Output4, gap3;                             // Define Output variable of PID4 for TEC-4
double Output5, gap4;                             // Define Output variable of PID5 for TEC-5
double Output6, gap5;                             // Define Output variable of PID5 for TEC-6

double aggKp=0, aggKi=0, aggKd=0;                 // Define the PID1 Tuning Parameters (constants)
double aggKp2=0, aggKi2=0, aggKd2=0;              // Define the PID2 Tuning Parameters (constants)
double aggKp3=0, aggKi3=0, aggKd3=0;              // Define the PID3 Tuning Parameters (constants)
double aggKp4=0, aggKi4=0, aggKd4=0;              // Define the PID4 Tuning Parameters (constants)
double aggKp5=0, aggKi5=0, aggKd5=0;              // Define the PID5 Tuning Parameters (constants)
double aggKp6=0, aggKi6=0, aggKd6=0;              // Define the PID6 Tuning Parameters (constants)

double Ambient_temp = 22;                       // Specify Ambient temperature level

double temp_read1, temp_set1;                     // Specify PV1 and SP1 variable for PID1
double temp_read2, temp_set2;                     // Specify PV2 and SP2 variable for PID2
double temp_read3, temp_set3;                     // Specify PV3 and SP3 variable for PID3
double temp_read4, temp_set4;                     // Specify PV4 and SP4 variable for PID4
double temp_read5, temp_set5;                     // Specify PV5 and SP5 variable for PID5
double temp_read6, temp_set6;                     // Specify PV6 and SP6 variable for PID6

double temp_set1_OLD;                             // Specify variable to store the previous SP1 (TEC-1)
double temp_set2_OLD;                             // Specify variable to store the previous SP2 (TEC-2)
double temp_set3_OLD;                             // Specify variable to store the previous SP3 (TEC-3)
double temp_set4_OLD;                             // Specify variable to store the previous SP4 (TEC-4)
double temp_set5_OLD;                             // Specify variable to store the previous SP5 (TEC-5)
double temp_set6_OLD;                             // Specify variable to store the previous SP6 (TEC-6)

char mode;                                        // Specify flag to start calculating the PIDs PWM values

char mode1, flag1 = 0, flag2 = 0, flag3 = 0;      // Specify Variables and flags used for TEC-1
char mode2, flag4 = 0, flag5 = 0, flag6 = 0;      // Specify Variables and flags used for TEC-2 
char mode3, flag7 = 0, flag8 = 0, flag9 = 0;      // Specify Variables and flags used for TEC-3 
char mode4, flag10 = 0, flag11 = 0, flag12 = 0;   // Specify Variables and flags used for TEC-4
char mode5, flag13 = 0, flag14 = 0, flag15 = 0;   // Specify Variables and flags used for TEC-5
char mode6, flag16 = 0, flag17 = 0, flag18 = 0;   // Specify Variables and flags used for TEC-6

//Specify the links (by reference) and initial tuning parameters
PID act1PID(&temp_read1, &Output1, &temp_set1, aggKp, aggKi, aggKd ,DIRECT);
PID act2PID(&temp_read2, &Output2, &temp_set2, aggKp2, aggKi2, aggKd2 ,DIRECT);
PID act3PID(&temp_read3, &Output3, &temp_set3, aggKp3, aggKi3, aggKd3 ,DIRECT);
PID act4PID(&temp_read4, &Output4, &temp_set4, aggKp4, aggKi4, aggKd4 ,DIRECT);
PID act5PID(&temp_read5, &Output5, &temp_set5, aggKp5, aggKi5, aggKd5 ,DIRECT);
PID act6PID(&temp_read6, &Output6, &temp_set6, aggKp6, aggKi6, aggKd6 ,DIRECT);

int PWMpins1 = 2;                                 // Defining the ATMEGA PWM pin for controlling TEC-1
int direction_ctrl_pins1[] = {29,28};             // Defining the ATMEGA direction controlling pins for TEC-1

int PWMpins2 = 3;                                 // Defining the ATMEGA PWM pin for controlling TEC-2
int direction_ctrl_pins2[] = {27,26};             // Defining the ATMEGA direction controlling pins for TEC-2

int PWMpins3 = 12;                                // Defining the ATMEGA PWM pin for controlling TEC-3
int direction_ctrl_pins3[] = {34,35};             // Defining the ATMEGA direction controlling pins for TEC-3

int PWMpins4 = 5;                                 // Defining the ATMEGA PWM pin for controlling TEC-4
int direction_ctrl_pins4[] = {36,37};             // Defining the ATMEGA direction controlling pins for TEC-4

int PWMpins5 = 11;                                // Defining the ATMEGA PWM pin for controlling TEC-5
int direction_ctrl_pins5[] = {32,33};             // Defining the ATMEGA direction controlling pins for TEC-5

int PWMpins6 = 7;                                 // Defining the ATMEGA PWM pin for controlling TEC-6
int direction_ctrl_pins6[] = {30,31};             // Defining the ATMEGA direction controlling pins for TEC-6

void setup()
{
  Serial.begin(115200);                             // Setting baude-rate for communication between ATMEGA and PYTHON
  
  // ATMEGA pins for controlling TEC-1
  pinMode(PWMpins1,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins1[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins1[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  TCCR3B = TCCR3B & B11111000 | B00000010;        // Setting PWM frequency to 4kHz
  digitalWrite(PWMpins1,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins1[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins1[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // ATMEGA pins for controlling TEC-2
  pinMode(PWMpins2,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins2[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins2[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  digitalWrite(PWMpins2,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins2[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins2[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // ATMEGA pins for controlling TEC-3
  pinMode(PWMpins3,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins3[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins3[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  TCCR1B = TCCR1B & B11111000 | B00000010;        // Setting PWM frequency to 4kHz
  digitalWrite(PWMpins3,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins3[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins3[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // ATMEGA pins for controlling TEC-4
  pinMode(PWMpins4,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins4[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins4[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  digitalWrite(PWMpins4,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins4[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins4[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // ATMEGA pins for controlling TEC-5
  pinMode(PWMpins5,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins5[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins5[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  digitalWrite(PWMpins5,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins5[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins5[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // ATMEGA pins for controlling TEC-6
  pinMode(PWMpins6,OUTPUT);                       // Setting ATMEGA PWM pin for OUTPUT mode
  pinMode(direction_ctrl_pins6[0], OUTPUT);       // Setting ATMEGA direction control pin 1 for OUTPUT mode
  pinMode(direction_ctrl_pins6[1], OUTPUT);       // Setting ATMEGA direction control pin 2 for OUTPUT mode
  TCCR4B = TCCR4B & B11111000 | B00000010;        // Setting PWM frequency to 4kHz
  digitalWrite(PWMpins6,LOW);                     // Initially set LOW logic to the PWM pin
  digitalWrite(direction_ctrl_pins6[0], LOW);     // Initially set LOW logic to the direction control pin 1
  digitalWrite(direction_ctrl_pins6[1], LOW);     // Initially set LOW logic to the direction control pin 2

  // Setting-up PID1
  act1PID.SetMode(AUTOMATIC);                     // Setting PID1 calculation mode to AUTOMATIC
  act1PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act1PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations

  // Setting-up PID2
  act2PID.SetMode(AUTOMATIC);                     // Setting PID2 calculation mode to AUTOMATIC
  act2PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act2PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations 

  // Setting-up PID3
  act3PID.SetMode(AUTOMATIC);                     // Setting PID3 calculation mode to AUTOMATIC
  act3PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act3PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations 

  // Setting-up PID4
  act4PID.SetMode(AUTOMATIC);                     // Setting PID4 calculation mode to AUTOMATIC
  act4PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act4PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations

  // Setting-up PID5
  act5PID.SetMode(AUTOMATIC);                     // Setting PID5 calculation mode to AUTOMATIC
  act5PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act5PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations

  // Setting-up PID6
  act6PID.SetMode(AUTOMATIC);                     // Setting PID6 calculation mode to AUTOMATIC
  act6PID.SetSampleTime(time);                    // Setting sampling-time for calculation of integral and derivative controller values 
  act6PID.SetOutputLimits(PWM_min, PWM_max);      // Setting PWM minimum and maximum duty cycle value for PIC calculations
}

void loop()
{
  while (Serial.available()>0)
  {
    /* Grabbing the SP and PV values, transmitted by PYTHON */
    static char buffer[32];
    static size_t pos;
    char c = Serial.read();
    buffer[pos++] = c;

    // Acquire SP1 and PV1 for PID1
    if (c == 'a')
    {
      buffer[pos] = '\0';
      temp_set1 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'b')
    {
      buffer[pos] = '\0';
      temp_read1 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }

    // Acquire SP2 and PV2 for PID2
    if (c == 'c')
    {
      buffer[pos] = '\0';
      temp_set2 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'd')
    {
      buffer[pos] = '\0';
      temp_read2 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }

    // Acquire SP3 and PV3 for PID3
    if (c == 'e')
    {
      buffer[pos] = '\0';
      temp_set3 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'f')
    {
      buffer[pos] = '\0';
      temp_read3 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }

    // Acquire SP4 and PV4 for PID4
    if (c == 'g')
    {
      buffer[pos] = '\0';
      temp_set4 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'h')
    {
      buffer[pos] = '\0';
      temp_read4 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }

    // Acquire SP5 and PV5 for PID5
    if (c == 'i')
    {
      buffer[pos] = '\0';
      temp_set5 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'j')
    {
      buffer[pos] = '\0';
      temp_read5 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }

    // Acquire SP6 and PV6 for PID6
    if (c == 'k')
    {
      buffer[pos] = '\0';
      temp_set6 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'l')
    {
      buffer[pos] = '\0';
      temp_read6 = atof(buffer);
      pos = 0;
      buffer[32] = {};
    }
    /* ---------------------------------------------------- */

    /* Acquire the PID1 constants value, if the SP1 lies between the temperature range */ 
    if ((temp_set1 >= 23) && (temp_set1 <= 29))
    {
      aggKp=75, aggKi=1, aggKd=100;
    }
    else if ((temp_set1 > 29) && (temp_set1 <= 34))
    {
      aggKp=85, aggKi=3, aggKd=100;
    }
    else if ((temp_set1 > 34) && (temp_set1 <= 40))    
    {
      aggKp=85, aggKi=2, aggKd=100;
    }
    else if ((temp_set1 > 40) && (temp_set1 <= 45))
    {
      aggKp=87, aggKi=1, aggKd=150;
    }
    else if ((temp_set1 > 45) && (temp_set1 <= 50))
    {
      aggKp=95, aggKi=1, aggKd=110;
    }
    else   // if the SP1 is below AMBIENT LEVEL, than adopt these values for PID1
    {
      aggKp=30, aggKi=0.5, aggKd=50;
    }

    /* Acquire the PID2 constants value, if the SP2 lies between the temperature range */ 
    if ((temp_set2 >= 23) && (temp_set2 <= 29))
    {
      aggKp2=75, aggKi2=1, aggKd2=100;
    }
    else if ((temp_set2 > 29) && (temp_set2 <= 34))
    {
      aggKp2=85, aggKi2=3, aggKd2=100;
    }
    else if ((temp_set2 > 34) && (temp_set2 <= 40))    
    {
      aggKp2=85, aggKi2=2, aggKd2=100;
    }
    else if ((temp_set2 > 40) && (temp_set2 <= 45))
    {
      aggKp2=87, aggKi2=1, aggKd2=150;
    }
    else if ((temp_set2 > 45) && (temp_set2 <= 50))
    {
      aggKp2=95, aggKi2=1, aggKd2=110;
    }
    else   // if the SP2 is below AMBIENT LEVEL, than adopt these values for PID2
    {
      aggKp2=30, aggKi2=0.5, aggKd2=50;
    }

    /* Acquire the PID3 constants value, if the SP3 lies between the temperature range */ 
    if ((temp_set3 >= 23) && (temp_set3 <= 29))
    {
      aggKp3=75, aggKi3=1, aggKd3=100;
    }
    else if ((temp_set3 > 29) && (temp_set3 <= 34))
    {
      aggKp3=85, aggKi3=3, aggKd3=100;
    }
    else if ((temp_set3 > 34) && (temp_set3 <= 40))    
    {
      aggKp3=85, aggKi3=2, aggKd3=100;
    }
    else if ((temp_set3 > 40) && (temp_set3 <= 45))
    {
      aggKp3=87, aggKi3=1, aggKd3=150;
    }
    else if ((temp_set3 > 45) && (temp_set3 <= 50))
    {
      aggKp3=95, aggKi3=1, aggKd3=110;
    }
    else   // if the SP3 is below AMBIENT LEVEL, than adopt these values for PID3
    {
      aggKp3=30, aggKi3=0.5, aggKd3=50;
    }

    /* Acquire the PID4 constants value, if the SP4 lies between the temperature range */ 
    if ((temp_set4 >= 23) && (temp_set4 <= 29))
    {
      aggKp4=75, aggKi4=1, aggKd4=100;
    }
    else if ((temp_set4 > 29) && (temp_set4 <= 34))
    {
      aggKp4=85, aggKi4=3, aggKd4=100;
    }
    else if ((temp_set4 > 34) && (temp_set4 <= 40))    
    {
      aggKp4=85, aggKi4=2, aggKd4=100;
    }
    else if ((temp_set4 > 40) && (temp_set4 <= 45))
    {
      aggKp4=87, aggKi4=1, aggKd4=150;
    }
    else if ((temp_set4 > 45) && (temp_set4 <= 50))
    {
      aggKp4=95, aggKi4=1, aggKd4=110;
    }
    else   // if the SP3 is below AMBIENT LEVEL, than adopt these values for PID4
    {
      aggKp4=30, aggKi4=0.5, aggKd4=50;
    }

    /* Acquire the PID5 constants value, if the SP5 lies between the temperature range */ 
    if ((temp_set5 >= 23) && (temp_set5 <= 29))
    {
      aggKp5=75, aggKi5=1, aggKd5=100;
    }
    else if ((temp_set5 > 29) && (temp_set5 <= 34))
    {
      aggKp5=85, aggKi5=3, aggKd5=100;
    }
    else if ((temp_set5 > 34) && (temp_set5 <= 40))    
    {
      aggKp5=85, aggKi5=2, aggKd5=100;
    }
    else if ((temp_set5 > 40) && (temp_set5 <= 45))
    {
      aggKp5=87, aggKi5=1, aggKd5=150;
    }
    else if ((temp_set5 > 45) && (temp_set5 <= 50))
    {
      aggKp5=95, aggKi5=1, aggKd5=110;
    }
    else   // if the SP5 is below AMBIENT LEVEL, than adopt these values for PID5
    {
      aggKp5=30, aggKi5=0.5, aggKd5=50;
    }

    /* Acquire the PID6 constants value, if the SP6 lies between the temperature range */ 
    if ((temp_set6 >= 23) && (temp_set6 <= 29))
    {
      aggKp6=75, aggKi6=1, aggKd6=100;
    }
    else if ((temp_set6 > 29) && (temp_set6 <= 34))
    {
      aggKp6=85, aggKi6=3, aggKd6=100;
    }
    else if ((temp_set6 > 34) && (temp_set6 <= 40))    
    {
      aggKp6=85, aggKi6=2, aggKd6=100;
    }
    else if ((temp_set6 > 40) && (temp_set6 <= 45))
    {
      aggKp6=87, aggKi6=1, aggKd6=150;
    }
    else if ((temp_set6 > 45) && (temp_set6 <= 50))
    {
      aggKp6=95, aggKi6=1, aggKd6=110;
    }
    else   // if the SP6 is below AMBIENT LEVEL, than adopt these values for PID6
    {
      aggKp6=30, aggKi6=0.5, aggKd6=50;
    }
    
    mode = 's';
  }
  if (mode == 's')
  {   
    gap = abs(temp_set1 - temp_read1);                  // Calculate difference between SP and PV
    act1PID.SetTunings(aggKp, aggKi, aggKd);            // Set acquired tuning parameters for PID 

    if (temp_set1 >= Ambient_temp)                       // If SP is above AMBIENT than set the TEC for heating mode
    {
      act1PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins1[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC driver
      digitalWrite(direction_ctrl_pins1[0], HIGH);
      act1PID.Compute();
      analogWrite(PWMpins1, Output1);
      flag1 = 0;
      flag2 = 0;
      flag3 = 0;
      mode1 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP for temperature range below th Ambient level */
    // If OLD_SP is above AMBIENT and the NEW_SP is below AMBIENT, than set the TEC cooling mode initially until the
    // error between PV and SP become less that 0.5 [degC].
    //                                                    OR 
    // if the PV reaches the SP, than the first IF condition becomes false, for retain the lock of PV with the SP
    // the flag [mode1] is introduced. And for locking the PV to the SP the flag [flag1] is introduced.    
    if (((temp_set1_OLD > Ambient_temp) && (temp_set1 < Ambient_temp)) || ((mode1 == 'q')))
    {
      // if the PV is locked with SP and if NEW_SP is introduced which is lesser than AMBIENT but greater than OLD_SP.
      // Than terminate from this IF condition by setting [mode1 = 'r'] and to jump to the next IF condition.
      if (temp_set1 > temp_set1_OLD)
      {
        flag1 = 1;
        mode1 = 'r';
        flag2 = 0;
        flag3 = 0;
      }
      if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins1[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[1], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 11))
        {
          analogWrite(PWMpins1, 140);
        }
        else if ((temp_set1 >= 11) && (temp_set1 < 14))
        {
          analogWrite(PWMpins1, 130);
        }
        else if ((temp_set1 >= 14) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 120);          
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 60);          
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 30);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 25);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 15);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))   // This section is commented bcz when these lines added, the controller does not lock to the values due to low DC value
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag1 = 1;
      }
      else if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins1[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[0], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 10);
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 25);
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 35);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 40);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 50);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag1 = 1;
      }
      else if ((abs(temp_set1 - temp_read1) > 0.5) && (flag1 == 0))
      {
        act1PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins1[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[1], HIGH);
        analogWrite(PWMpins1, 250);
      }
      mode1 = 'q';
    }
    // If OLD_SP is below AMBIENT and the NEW_SP is also below AMBIENT, but the NEW_SP is greater than OLD_SP. Than set the TEC heating mode initially
    // If OLD_SP is below AMBIENT and the NEW_SP is also below AMBIENT but greater than OLD_SO, than set the TEC heating mode initially until the
    // error between PV and SP become less that 0.5 [degC].
    //                                                    OR 
    // if the PV reaches the SP, than the first IF condition becomes false, for retain the lock of PV with the SP
    // the flag [mode1] is introduced. And for locking the PV to the SP the flag [flag2] is introduced.
    if (((temp_set1_OLD < Ambient_temp) && (temp_set1 <= Ambient_temp) && (temp_set1 > temp_set1_OLD)) || ((mode1 == 'w')))
    {
      // if the PV is locked with SP and if NEW_SP is introduced which is lesser than AMBIENT but greater than OLD_SP.
      // Than set the heating of TEC with the calculated PID PWM DC value by setting the flag [flag2 = 0].
      if (temp_set1 > temp_set1_OLD)
      {
        flag2 = 0;
      }
      
      if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins1[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[1], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 11))
        {
          analogWrite(PWMpins1, 140);
        }
        else if ((temp_set1 >= 11) && (temp_set1 < 14))
        {
          analogWrite(PWMpins1, 130);
        }
        else if ((temp_set1 >= 14) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 120);          
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 60);          
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 30);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 25);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 15);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag2 = 1;
      }
      else if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins1[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[0], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 10);
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 25);
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 35);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 40);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 50);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag2 = 1;
      }
      else if ((abs(temp_set1 - temp_read1) > 0.5) && (flag2 == 0))
      {
        act1PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins1[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins1[0], HIGH);
        act1PID.Compute();
        analogWrite(PWMpins1, Output1);
      }
      mode1 = 'w';
      flag1 = 0;
    }
    // If OLD_SP is below AMBIENT and the NEW_SP is also below AMBIENT, but the NEW_SP is lesser than OLD_SP. Than set the TEC cooling mode initially
    // If OLD_SP is below AMBIENT and the NEW_SP is also below AMBIENT but lesser than OLD_SP, than set the TEC cooling mode initially until the
    // error between PV and SP become less that 0.5 [degC].
    //                                                    OR 
    // if the PV reaches the SP, than the first IF condition becomes false, for retain the lock of PV with the SP
    // the flag [mode1] is introduced. And for locking the PV to the SP the flag [flag3] is introduced.
    if (((temp_set1_OLD <= Ambient_temp) && (temp_set1 <= Ambient_temp) && (temp_set1 < temp_set1_OLD)) || ((mode1 == 'e')))
    {
      // if the PV is locked with SP and if NEW_SP is introduced which is lesser than AMBIENT but also lesser than OLD_SP.
      // Than set the cooling of TEC with the PWM maximum DC value by setting the flag [flag3 = 0].
      if (temp_set1 < temp_set1_OLD)
      {
        flag3 = 0;
      }
      
      if (((temp_set1 - temp_read1) <= -0.01) && ((temp_set1 - temp_read1) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins1[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[1], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 11))
        {
          analogWrite(PWMpins1, 140);
        }
        else if ((temp_set1 >= 11) && (temp_set1 < 14))
        {
          analogWrite(PWMpins1, 130);
        }
        else if ((temp_set1 >= 14) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 120);          
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 60);          
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 30);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 25);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 15);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag3 = 1;
      }
      else if (((temp_set1 - temp_read1) >= 0.01) && ((temp_set1 - temp_read1) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins1[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[0], HIGH);
        if ((temp_set1 >= 10) && (temp_set1 < 15))
        {
          analogWrite(PWMpins1, 10);
        }
        else if ((temp_set1 >= 15) && (temp_set1 < 17))
        {
          analogWrite(PWMpins1, 25);
        }
        else if ((temp_set1 >= 17) && (temp_set1 < 19))
        {
          analogWrite(PWMpins1, 35);          
        }
        else if ((temp_set1 >= 19) && (temp_set1 < 20))
        {
          analogWrite(PWMpins1, 40);          
        }
        else if ((temp_set1 >= 20) && (temp_set1 <= 23))
        {
          analogWrite(PWMpins1, 50);          
        }
//        else if ((temp_set1 >= 21) && (temp_set1 < 23))
//        {
//          analogWrite(PWMpins1, 1);          
//        }
        flag3 = 1;
      }
      else if ((abs(temp_set1 - temp_read1) > 0.5) && (flag3 == 0))
      {
        act1PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins1[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins1[1], HIGH);
        analogWrite(PWMpins1, 250);
      }
      mode1 = 'e';
      flag1 = 0;
      flag2 = 0;
    }

    // TEC2 controlling
    gap1 = abs(temp_set2 - temp_read2);                  // Calculate difference between SP2 and PV2
    act2PID.SetTunings(aggKp2, aggKi2, aggKd2);          // Set acquired tuning parameters for PID2 

    if (temp_set2 >= Ambient_temp)                       // If SP2 is above AMBIENT than set the TEC2 for heating mode
    {
      act2PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins2[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC2 driver
      digitalWrite(direction_ctrl_pins2[0], HIGH);
      act2PID.Compute();
      analogWrite(PWMpins2, Output2);
      flag4 = 0;
      flag5 = 0;
      flag6 = 0;
      mode2 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP2 for temperature range below th Ambient level */
    // If OLD_SP2 is above AMBIENT and the NEW_SP2 is below AMBIENT, than set the TEC2 cooling mode initially until the
    // error between PV2 and SP2 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV2 reaches the SP2, than the first IF condition becomes false, for retain the lock of PV2 with the SP2
    // the flag [mode2] is introduced. And for locking the PV2 to the SP the flag [flag4] is introduced.    
    if (((temp_set2_OLD > Ambient_temp) && (temp_set2 < Ambient_temp)) || ((mode2 == 'q')))
    {
      // if the PV2 is locked with SP2 and if NEW_SP2 is introduced which is lesser than AMBIENT but greater than OLD_SP2.
      // Than terminate from this IF condition by setting [mode2 = 'r'] and to jump to the next IF condition.
      if (temp_set2 > temp_set2_OLD)
      {
        flag4 = 1;
        mode2 = 'r';
        flag5 = 0;
        flag6 = 0;
      }
      if (((temp_set2 - temp_read2) <= -0.01) && ((temp_set2 - temp_read2) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins2[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[1], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 11))
        {
          analogWrite(PWMpins2, 140);
        }
        else if ((temp_set2 >= 11) && (temp_set2 < 14))
        {
          analogWrite(PWMpins2, 130);
        }
        else if ((temp_set2 >= 14) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 120);          
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 60);          
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 30);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 25);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 15);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag4 = 1;
      }
      else if (((temp_set2 - temp_read2) >= 0.01) && ((temp_set2 - temp_read2) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins2[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[0], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 10);
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 25);
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 35);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 40);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 50);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag4 = 1;
      }
      else if ((abs(temp_set2 - temp_read2) > 0.5) && (flag4 == 0))
      {
        act2PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins2[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[1], HIGH);
        analogWrite(PWMpins2, 250);
      }
      mode2 = 'q';
    }
    // If OLD_SP2 is below AMBIENT and the NEW_SP2 is also below AMBIENT, but the NEW_SP2 is greater than OLD_SP2. Than set the TEC2 heating mode initially
    // If OLD_SP2 is below AMBIENT and the NEW_SP2 is also below AMBIENT but greater than OLD_SP2, than set the TEC2 heating mode initially until the
    // error between PV2 and SP2 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV2 reaches the SP2, than the first IF condition becomes false, for retain the lock of PV2 with the SP2
    // the flag [mode2] is introduced. And for locking the PV2 to the SP2 the flag [flag5] is introduced.
    if (((temp_set2_OLD < Ambient_temp) && (temp_set2 <= Ambient_temp) && (temp_set2 > temp_set2_OLD)) || ((mode2 == 'w')))
    {
      // if the PV2 is locked with SP2 and if NEW_SP2 is introduced which is lesser than AMBIENT but greater than OLD_SP2.
      // Than set the heating of TEC2 with the calculated PID2 PWM DC value by setting the flag [flag5 = 0].
      if (temp_set2 > temp_set2_OLD)
      {
        flag5 = 0;
      }
      
      if (((temp_set2 - temp_read2) <= -0.01) && ((temp_set2 - temp_read2) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins2[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[1], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 11))
        {
          analogWrite(PWMpins2, 140);
        }
        else if ((temp_set2 >= 11) && (temp_set2 < 14))
        {
          analogWrite(PWMpins2, 130);
        }
        else if ((temp_set2 >= 14) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 120);          
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 60);          
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 30);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 25);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 15);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag5 = 1;
      }
      else if (((temp_set2 - temp_read2) >= 0.01) && ((temp_set2 - temp_read2) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins2[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[0], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 10);
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 25);
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 35);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 40);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 50);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag5 = 1;
      }
      else if ((abs(temp_set2 - temp_read2) > 0.5) && (flag5 == 0))
      {
        act2PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins2[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins2[0], HIGH);
        act2PID.Compute();
        analogWrite(PWMpins2, Output2);
      }
      mode2 = 'w';
      flag4 = 0;
    }
    // If OLD_SP2 is below AMBIENT and the NEW_SP2 is also below AMBIENT, but the NEW_SP2 is lesser than OLD_SP2. Than set the TEC2 cooling mode initially
    // If OLD_SP2 is below AMBIENT and the NEW_SP2 is also below AMBIENT but lesser than OLD_SP2, than set the TEC2 cooling mode initially until the
    // error between PV2 and SP2 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV2 reaches the SP2, than the first IF condition becomes false, for retain the lock of PV2 with the SP2
    // the flag [mode2] is introduced. And for locking the PV2 to the SP2 the flag [flag6] is introduced.
    if (((temp_set2_OLD <= Ambient_temp) && (temp_set2 <= Ambient_temp) && (temp_set2 < temp_set2_OLD)) || ((mode2 == 'e')))
    {
      // if the PV2 is locked with SP2 and if NEW_SP2 is introduced which is lesser than AMBIENT but also lesser than OLD_SP2.
      // Than set the cooling of TEC2 with the PWM maximum DC value by setting the flag [flag6 = 0].
      if (temp_set2 < temp_set2_OLD)
      {
        flag6 = 0;
      }
      
      if (((temp_set2 - temp_read2) <= -0.01) && ((temp_set2 - temp_read2) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins2[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[1], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 11))
        {
          analogWrite(PWMpins2, 140);
        }
        else if ((temp_set2 >= 11) && (temp_set2 < 14))
        {
          analogWrite(PWMpins2, 130);
        }
        else if ((temp_set2 >= 14) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 120);          
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 60);          
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 30);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 25);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 15);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag6 = 1;
      }
      else if (((temp_set2 - temp_read2) >= 0.01) && ((temp_set2 - temp_read2) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins2[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[0], HIGH);
        if ((temp_set2 >= 10) && (temp_set2 < 15))
        {
          analogWrite(PWMpins2, 10);
        }
        else if ((temp_set2 >= 15) && (temp_set2 < 17))
        {
          analogWrite(PWMpins2, 25);
        }
        else if ((temp_set2 >= 17) && (temp_set2 < 19))
        {
          analogWrite(PWMpins2, 35);          
        }
        else if ((temp_set2 >= 19) && (temp_set2 < 20))
        {
          analogWrite(PWMpins2, 40);          
        }
        else if ((temp_set2 >= 20) && (temp_set2 <= 23))
        {
          analogWrite(PWMpins2, 50);          
        }
//        else if ((temp_set2 >= 21) && (temp_set2 < 23))
//        {
//          analogWrite(PWMpins2, 1);          
//        }
        flag6 = 1;
      }
      else if ((abs(temp_set2 - temp_read2) > 0.5) && (flag6 == 0))
      {
        act2PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins2[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins2[1], HIGH);
        analogWrite(PWMpins2, 250);
      }
      mode2 = 'e';
      flag4 = 0;
      flag5 = 0;
    }

    // TEC3 controlling
    gap2 = abs(temp_set3 - temp_read3);                  // Calculate difference between SP3 and PV3
    act3PID.SetTunings(aggKp3, aggKi3, aggKd3);          // Set acquired tuning parameters for PID3 

    if (temp_set3 >= Ambient_temp)                       // If SP3 is above AMBIENT than set the TEC3 for heating mode
    {
      act3PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins3[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC3 driver
      digitalWrite(direction_ctrl_pins3[0], HIGH);
      act3PID.Compute();
      analogWrite(PWMpins3, Output3);
      flag7 = 0;
      flag8 = 0;
      flag9 = 0;
      mode3 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP3 for temperature range below the Ambient level */
    // If OLD_SP3 is above AMBIENT and the NEW_SP3 is below AMBIENT, than set the TEC3 cooling mode initially until the
    // error between PV3 and SP3 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV3 reaches the SP3, than the first IF condition becomes false, for retain the lock of PV3 with the SP3
    // the flag [mode3] is introduced. And for locking the PV2 to the SP the flag [flag7] is introduced.    
    if (((temp_set3_OLD > Ambient_temp) && (temp_set3 < Ambient_temp)) || ((mode3 == 'q')))
    {
      // if the PV3 is locked with SP3 and if NEW_SP3 is introduced which is lesser than AMBIENT but greater than OLD_SP3.
      // Than terminate from this IF condition by setting [mode3 = 'r'] and to jump to the next IF condition.
      if (temp_set3 > temp_set3_OLD)
      {
        flag7 = 1;
        mode3 = 'r';
        flag8 = 0;
        flag9 = 0;
      }
      if (((temp_set3 - temp_read3) <= -0.01) && ((temp_set3 - temp_read3) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins3[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[1], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 11))
        {
          analogWrite(PWMpins3, 140);
        }
        else if ((temp_set3 >= 11) && (temp_set3 < 14))
        {
          analogWrite(PWMpins3, 130);
        }
        else if ((temp_set3 >= 14) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 120);          
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 60);          
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 30);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 25);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 15);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag7 = 1;
      }
      else if (((temp_set3 - temp_read3) >= 0.01) && ((temp_set3 - temp_read3) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins3[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[0], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 10);
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 25);
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 35);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 40);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 50);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag7 = 1;
      }
      else if ((abs(temp_set3 - temp_read3) > 0.5) && (flag7 == 0))
      {
        act3PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins3[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[1], HIGH);
        analogWrite(PWMpins3, 250);
      }
      mode3 = 'q';
    }
    // If OLD_SP3 is below AMBIENT and the NEW_SP3 is also below AMBIENT, but the NEW_SP3 is greater than OLD_SP3. Than set the TEC3 heating mode initially
    // If OLD_SP3 is below AMBIENT and the NEW_SP3 is also below AMBIENT but greater than OLD_SP3, than set the TEC3 heating mode initially until the
    // error between PV3 and SP3 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV3 reaches the SP3, than the first IF condition becomes false, for retain the lock of PV3 with the SP3
    // the flag [mode3] is introduced. And for locking the PV3 to the SP3 the flag [flag8] is introduced.
    if (((temp_set3_OLD < Ambient_temp) && (temp_set3 <= Ambient_temp) && (temp_set3 > temp_set2_OLD)) || ((mode3 == 'w')))
    {
      // if the PV3 is locked with SP3 and if NEW_SP3 is introduced which is lesser than AMBIENT but greater than OLD_SP3.
      // Than set the heating of TEC3 with the calculated PID3 PWM DC value by setting the flag [flag8 = 0].
      if (temp_set3 > temp_set3_OLD)
      {
        flag8 = 0;
      }
      
      if (((temp_set3 - temp_read3) <= -0.01) && ((temp_set3 - temp_read3) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins3[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[1], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 11))
        {
          analogWrite(PWMpins3, 140);
        }
        else if ((temp_set3 >= 11) && (temp_set3 < 14))
        {
          analogWrite(PWMpins3, 130);
        }
        else if ((temp_set3 >= 14) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 120);          
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 60);          
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 30);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 25);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 15);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag8 = 1;
      }
      else if (((temp_set3 - temp_read3) >= 0.01) && ((temp_set3 - temp_read3) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins3[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[0], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 10);
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 25);
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 35);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 40);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 50);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag8 = 1;
      }
      else if ((abs(temp_set3 - temp_read3) > 0.5) && (flag8 == 0))
      {
        act3PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins3[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins3[0], HIGH);
        act3PID.Compute();
        analogWrite(PWMpins3, Output3);
      }
      mode3 = 'w';
      flag7 = 0;
    }
    // If OLD_SP3 is below AMBIENT and the NEW_SP3 is also below AMBIENT, but the NEW_SP3 is lesser than OLD_SP3. Than set the TEC3 cooling mode initially
    // If OLD_SP3 is below AMBIENT and the NEW_SP3 is also below AMBIENT but lesser than OLD_SP3, than set the TEC3 cooling mode initially until the
    // error between PV3 and SP3 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV3 reaches the SP3, than the first IF condition becomes false, for retain the lock of PV3 with the SP3
    // the flag [mode3] is introduced. And for locking the PV3 to the SP3 the flag [flag9] is introduced.
    if (((temp_set3_OLD <= Ambient_temp) && (temp_set3 <= Ambient_temp) && (temp_set3 < temp_set3_OLD)) || ((mode3 == 'e')))
    {
      // if the PV3 is locked with SP3 and if NEW_SP3 is introduced which is lesser than AMBIENT but also lesser than OLD_SP3.
      // Than set the cooling of TEC3 with the PWM maximum DC value by setting the flag [flag9 = 0].
      if (temp_set3 < temp_set3_OLD)
      {
        flag9 = 0;
      }
      
      if (((temp_set3 - temp_read3) <= -0.01) && ((temp_set3 - temp_read3) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins3[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[1], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 11))
        {
          analogWrite(PWMpins3, 140);
        }
        else if ((temp_set3 >= 11) && (temp_set3 < 14))
        {
          analogWrite(PWMpins3, 130);
        }
        else if ((temp_set3 >= 14) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 120);          
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 60);          
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 30);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 25);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 15);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag9 = 1;
      }
      else if (((temp_set3 - temp_read3) >= 0.01) && ((temp_set3 - temp_read3) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins3[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[0], HIGH);
        if ((temp_set3 >= 10) && (temp_set3 < 15))
        {
          analogWrite(PWMpins3, 10);
        }
        else if ((temp_set3 >= 15) && (temp_set3 < 17))
        {
          analogWrite(PWMpins3, 25);
        }
        else if ((temp_set3 >= 17) && (temp_set3 < 19))
        {
          analogWrite(PWMpins3, 35);          
        }
        else if ((temp_set3 >= 19) && (temp_set3 < 20))
        {
          analogWrite(PWMpins3, 40);          
        }
        else if ((temp_set3 >= 20) && (temp_set3 <= 23))
        {
          analogWrite(PWMpins3, 50);          
        }
//        else if ((temp_set3 >= 21) && (temp_set3 < 23))
//        {
//          analogWrite(PWMpins3, 1);          
//        }
        flag9 = 1;
      }
      else if ((abs(temp_set3 - temp_read3) > 0.5) && (flag9 == 0))
      {
        act3PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins3[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins3[1], HIGH);
        analogWrite(PWMpins3, 250);
      }
      mode3 = 'e';
      flag7 = 0;
      flag8 = 0;
    }

    // TEC4 controlling
    gap3 = abs(temp_set4 - temp_read4);                  // Calculate difference between SP4 and PV4
    act4PID.SetTunings(aggKp4, aggKi4, aggKd4);          // Set acquired tuning parameters for PID4

    if (temp_set4 >= Ambient_temp)                       // If SP3 is above AMBIENT than set the TEC4 for heating mode
    {
      act4PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins4[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC4 driver
      digitalWrite(direction_ctrl_pins4[0], HIGH);
      act4PID.Compute();
      analogWrite(PWMpins4, Output4);
      flag10 = 0;
      flag11 = 0;
      flag12 = 0;
      mode4 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP4 for temperature range below the Ambient level */
    // If OLD_SP4 is above AMBIENT and the NEW_SP4 is below AMBIENT, than set the TEC4 cooling mode initially until the
    // error between PV4 and SP4 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV4 reaches the SP4, than the first IF condition becomes false, for retain the lock of PV4 with the SP4
    // the flag [mode4] is introduced. And for locking the PV4 to the SP4 the flag [flag10] is introduced.    
    if (((temp_set4_OLD > Ambient_temp) && (temp_set4 < Ambient_temp)) || ((mode4 == 'q')))
    {
      // if the PV4 is locked with SP4 and if NEW_SP4 is introduced which is lesser than AMBIENT but greater than OLD_SP4.
      // Than terminate from this IF condition by setting [mode4 = 'r'] and to jump to the next IF condition.
      if (temp_set4 > temp_set4_OLD)
      {
        flag10 = 1;
        mode4 = 'r';
        flag11 = 0;
        flag12 = 0;
      }
      if (((temp_set4 - temp_read4) <= -0.01) && ((temp_set4 - temp_read4) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins4[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[1], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 11))
        {
          analogWrite(PWMpins4, 140);
        }
        else if ((temp_set4 >= 11) && (temp_set4 < 14))
        {
          analogWrite(PWMpins4, 130);
        }
        else if ((temp_set4 >= 14) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 120);          
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 60);          
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 30);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 25);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 15);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag10 = 1;
      }
      else if (((temp_set4 - temp_read4) >= 0.01) && ((temp_set4 - temp_read4) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins4[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[0], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 10);
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 25);
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 35);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 40);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 50);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag10 = 1;
      }
      else if ((abs(temp_set4 - temp_read4) > 0.5) && (flag10 == 0))
      {
        act4PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins4[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[1], HIGH);
        analogWrite(PWMpins4, 250);
      }
      mode4 = 'q';
    }
    // If OLD_SP4 is below AMBIENT and the NEW_SP4 is also below AMBIENT, but the NEW_SP4 is greater than OLD_SP4. Than set the TEC4 heating mode initially
    // If OLD_SP4 is below AMBIENT and the NEW_SP4 is also below AMBIENT but greater than OLD_SP4, than set the TEC4 heating mode initially until the
    // error between PV4 and SP4 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV4 reaches the SP4, than the first IF condition becomes false, for retain the lock of PV4 with the SP4
    // the flag [mode4] is introduced. And for locking the PV4 to the SP4 the flag [flag11] is introduced.
    if (((temp_set4_OLD < Ambient_temp) && (temp_set4 <= Ambient_temp) && (temp_set4 > temp_set4_OLD)) || ((mode4 == 'w')))
    {
      // if the PV4 is locked with SP4 and if NEW_SP4 is introduced which is lesser than AMBIENT but greater than OLD_SP4.
      // Than set the heating of TEC4 with the calculated PID4 PWM DC value by setting the flag [flag11 = 0].
      if (temp_set4 > temp_set4_OLD)
      {
        flag11 = 0;
      }
      
      if (((temp_set4 - temp_read4) <= -0.01) && ((temp_set4 - temp_read4) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins4[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[1], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 11))
        {
          analogWrite(PWMpins4, 140);
        }
        else if ((temp_set4 >= 11) && (temp_set4 < 14))
        {
          analogWrite(PWMpins4, 130);
        }
        else if ((temp_set4 >= 14) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 120);          
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 60);          
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 30);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 25);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 15);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag11 = 1;
      }
      else if (((temp_set4 - temp_read4) >= 0.01) && ((temp_set4 - temp_read4) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins4[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[0], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 10);
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 25);
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 35);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 40);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 50);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag11 = 1;
      }
      else if ((abs(temp_set4 - temp_read4) > 0.5) && (flag11 == 0))
      {
        act4PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins4[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins4[0], HIGH);
        act4PID.Compute();
        analogWrite(PWMpins4, Output4);
      }
      mode4 = 'w';
      flag10 = 0;
    }
    // If OLD_SP4 is below AMBIENT and the NEW_SP4 is also below AMBIENT, but the NEW_SP4 is lesser than OLD_SP4. Than set the TEC4 cooling mode initially
    // If OLD_SP4 is below AMBIENT and the NEW_SP4 is also below AMBIENT but lesser than OLD_SP4, than set the TEC4 cooling mode initially until the
    // error between PV4 and SP4 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV4 reaches the SP4, than the first IF condition becomes false, for retain the lock of PV4 with the SP4
    // the flag [mode4] is introduced. And for locking the PV4 to the SP4 the flag [flag11] is introduced.
    if (((temp_set4_OLD <= Ambient_temp) && (temp_set4 <= Ambient_temp) && (temp_set4 < temp_set4_OLD)) || ((mode4 == 'e')))
    {
      // if the PV4 is locked with SP4 and if NEW_SP4 is introduced which is lesser than AMBIENT but also lesser than OLD_SP4.
      // Than set the cooling of TEC4 with the PWM maximum DC value by setting the flag [flag12 = 0].
      if (temp_set4 < temp_set4_OLD)
      {
        flag12 = 0;
      }
      
      if (((temp_set4 - temp_read4) <= -0.01) && ((temp_set4 - temp_read4) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins4[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[1], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 11))
        {
          analogWrite(PWMpins4, 140);
        }
        else if ((temp_set4 >= 11) && (temp_set4 < 14))
        {
          analogWrite(PWMpins4, 130);
        }
        else if ((temp_set4 >= 14) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 120);          
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 60);          
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 30);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 25);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 15);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag12 = 1;
      }
      else if (((temp_set4 - temp_read4) >= 0.01) && ((temp_set4 - temp_read4) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins4[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[0], HIGH);
        if ((temp_set4 >= 10) && (temp_set4 < 15))
        {
          analogWrite(PWMpins4, 10);
        }
        else if ((temp_set4 >= 15) && (temp_set4 < 17))
        {
          analogWrite(PWMpins4, 25);
        }
        else if ((temp_set4 >= 17) && (temp_set4 < 19))
        {
          analogWrite(PWMpins4, 35);          
        }
        else if ((temp_set4 >= 19) && (temp_set4 < 20))
        {
          analogWrite(PWMpins4, 40);          
        }
        else if ((temp_set4 >= 20) && (temp_set4 <= 23))
        {
          analogWrite(PWMpins4, 50);          
        }
//        else if ((temp_set4 >= 21) && (temp_set4 < 23))
//        {
//          analogWrite(PWMpins4, 1);          
//        }
        flag12 = 1;
      }
      else if ((abs(temp_set4 - temp_read4) > 0.5) && (flag12 == 0))
      {
        act4PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins4[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins4[1], HIGH);
        analogWrite(PWMpins4, 250);
      }
      mode4 = 'e';
      flag10 = 0;
      flag11 = 0;
    }

    // TEC5 controlling
    gap4 = abs(temp_set5 - temp_read5);                  // Calculate difference between SP5 and PV5
    act5PID.SetTunings(aggKp5, aggKi5, aggKd5);          // Set acquired tuning parameters for PID5

    if (temp_set5 >= Ambient_temp)                       // If SP5 is above AMBIENT than set the TEC5 for heating mode
    {
      act5PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins5[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC5 driver
      digitalWrite(direction_ctrl_pins5[0], HIGH);
      act5PID.Compute();
      analogWrite(PWMpins5, Output5);
      flag13 = 0;
      flag14 = 0;
      flag15 = 0;
      mode5 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP5 for temperature range below the Ambient level */
    // If OLD_SP5 is above AMBIENT and the NEW_SP5 is below AMBIENT, than set the TEC5 cooling mode initially until the
    // error between PV5 and SP5 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV5 reaches the SP5, than the first IF condition becomes false, for retain the lock of PV5 with the SP5
    // the flag [mode5] is introduced. And for locking the PV5 to the SP5 the flag [flag13] is introduced.    
    if (((temp_set5_OLD > Ambient_temp) && (temp_set5 < Ambient_temp)) || ((mode5 == 'q')))
    {
      // if the PV5 is locked with SP5 and if NEW_SP5 is introduced which is lesser than AMBIENT but greater than OLD_SP5.
      // Than terminate from this IF condition by setting [mode5 = 'r'] and to jump to the next IF condition.
      if (temp_set5 > temp_set5_OLD)
      {
        flag13 = 1;
        mode5 = 'r';
        flag14 = 0;
        flag15 = 0;
      }
      if (((temp_set5 - temp_read5) <= -0.01) && ((temp_set5 - temp_read5) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins5[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[1], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 11))
        {
          analogWrite(PWMpins5, 140);
        }
        else if ((temp_set5 >= 11) && (temp_set5 < 14))
        {
          analogWrite(PWMpins5, 130);
        }
        else if ((temp_set5 >= 14) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 120);          
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 60);          
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 30);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 25);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 15);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag13 = 1;
      }
      else if (((temp_set5 - temp_read5) >= 0.01) && ((temp_set5 - temp_read5) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins5[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[0], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 10);
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 25);
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 35);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 40);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 50);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag13 = 1;
      }
      else if ((abs(temp_set5 - temp_read5) > 0.5) && (flag13 == 0))
      {
        act5PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins5[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[1], HIGH);
        analogWrite(PWMpins5, 250);
      }
      mode5 = 'q';
    }
    // If OLD_SP5 is below AMBIENT and the NEW_SP5 is also below AMBIENT, but the NEW_SP5 is greater than OLD_SP5. Than set the TEC5 heating mode initially
    // If OLD_SP5 is below AMBIENT and the NEW_SP5 is also below AMBIENT but greater than OLD_SP5, than set the TEC5 heating mode initially until the
    // error between PV5 and SP5 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV5 reaches the SP5, than the first IF condition becomes false, for retain the lock of PV5 with the SP5
    // the flag [mode5] is introduced. And for locking the PV5 to the SP5 the flag [flag14] is introduced.
    if (((temp_set5_OLD < Ambient_temp) && (temp_set5 <= Ambient_temp) && (temp_set5 > temp_set5_OLD)) || ((mode5 == 'w')))
    {
      // if the PV5 is locked with SP5 and if NEW_SP5 is introduced which is lesser than AMBIENT but greater than OLD_SP5.
      // Than set the heating of TEC5 with the calculated PID5 PWM DC value by setting the flag [flag14 = 0].
      if (temp_set5 > temp_set5_OLD)
      {
        flag14 = 0;
      }
      
      if (((temp_set5 - temp_read5) <= -0.01) && ((temp_set5 - temp_read5) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins5[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[1], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 11))
        {
          analogWrite(PWMpins5, 140);
        }
        else if ((temp_set5 >= 11) && (temp_set5 < 14))
        {
          analogWrite(PWMpins5, 130);
        }
        else if ((temp_set5 >= 14) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 120);          
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 60);          
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 30);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 25);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 15);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag14 = 1;
      }
      else if (((temp_set5 - temp_read5) >= 0.01) && ((temp_set5 - temp_read5) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins5[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[0], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 10);
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 25);
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 35);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 40);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 50);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag14 = 1;
      }
      else if ((abs(temp_set5 - temp_read5) > 0.5) && (flag14 == 0))
      {
        act5PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins5[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins5[0], HIGH);
        act5PID.Compute();
        analogWrite(PWMpins5, Output5);
      }
      mode5 = 'w';
      flag13 = 0;
    }
    // If OLD_SP5 is below AMBIENT and the NEW_SP5 is also below AMBIENT, but the NEW_SP5 is lesser than OLD_SP5. Than set the TEC5 cooling mode initially
    // If OLD_SP5 is below AMBIENT and the NEW_SP5 is also below AMBIENT but lesser than OLD_SP5, than set the TEC5 cooling mode initially until the
    // error between PV5 and SP5 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV5 reaches the SP5, than the first IF condition becomes false, for retain the lock of PV5 with the SP5
    // the flag [mode5] is introduced. And for locking the PV5 to the SP5 the flag [flag15] is introduced.
    if (((temp_set5_OLD <= Ambient_temp) && (temp_set5 <= Ambient_temp) && (temp_set5 < temp_set5_OLD)) || ((mode5 == 'e')))
    {
      // if the PV5 is locked with SP5 and if NEW_SP5 is introduced which is lesser than AMBIENT but also lesser than OLD_SP5.
      // Than set the cooling of TEC5 with the PWM maximum DC value by setting the flag [flag15 = 0].
      if (temp_set5 < temp_set5_OLD)
      {
        flag15 = 0;
      }
      
      if (((temp_set5 - temp_read5) <= -0.01) && ((temp_set5 - temp_read5) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins5[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[1], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 11))
        {
          analogWrite(PWMpins5, 140);
        }
        else if ((temp_set5 >= 11) && (temp_set5 < 14))
        {
          analogWrite(PWMpins5, 130);
        }
        else if ((temp_set5 >= 14) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 120);          
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 60);          
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 30);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 25);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 15);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag15 = 1;
      }
      else if (((temp_set5 - temp_read5) >= 0.01) && ((temp_set5 - temp_read5) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins5[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[0], HIGH);
        if ((temp_set5 >= 10) && (temp_set5 < 15))
        {
          analogWrite(PWMpins5, 10);
        }
        else if ((temp_set5 >= 15) && (temp_set5 < 17))
        {
          analogWrite(PWMpins5, 25);
        }
        else if ((temp_set5 >= 17) && (temp_set5 < 19))
        {
          analogWrite(PWMpins5, 35);          
        }
        else if ((temp_set5 >= 19) && (temp_set5 < 20))
        {
          analogWrite(PWMpins5, 40);          
        }
        else if ((temp_set5 >= 20) && (temp_set5 <= 23))
        {
          analogWrite(PWMpins5, 50);          
        }
//        else if ((temp_set5 >= 21) && (temp_set5 < 23))
//        {
//          analogWrite(PWMpins5, 1);          
//        }
        flag15 = 1;
      }
      else if ((abs(temp_set5 - temp_read5) > 0.5) && (flag15 == 0))
      {
        act5PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins5[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins5[1], HIGH);
        analogWrite(PWMpins5, 250);
      }
      mode5 = 'e';
      flag13 = 0;
      flag14 = 0;
    }

    // TEC6 controlling
    gap5 = abs(temp_set6 - temp_read6);                  // Calculate difference between SP6 and PV6
    act6PID.SetTunings(aggKp6, aggKi6, aggKd6);          // Set acquired tuning parameters for PID6

    if (temp_set6 >= Ambient_temp)                       // If SP5 is above AMBIENT than set the TEC6 for heating mode
    {
      act6PID.SetControllerDirection(DIRECT);
      digitalWrite(direction_ctrl_pins6[1], LOW); 
      delayMicroseconds(55);                            // delay of 55 microseconds to avoid shoot-through of the TEC6 driver
      digitalWrite(direction_ctrl_pins6[0], HIGH);
      act6PID.Compute();
      analogWrite(PWMpins6, Output6);
      flag16 = 0;
      flag17 = 0;
      flag18 = 0;
      mode6 = 'r';
    }
    /* Defining HYSTERISES windows ABOVE and BELOW the SP6 for temperature range below the Ambient level */
    // If OLD_SP6 is above AMBIENT and the NEW_SP6 is below AMBIENT, than set the TEC6 cooling mode initially until the
    // error between PV6 and SP6 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV6 reaches the SP6, than the first IF condition becomes false, for retain the lock of PV6 with the SP6
    // the flag [mode6] is introduced. And for locking the PV6 to the SP6 the flag [flag16] is introduced.    
    if (((temp_set6_OLD > Ambient_temp) && (temp_set6 < Ambient_temp)) || ((mode6 == 'q')))
    {
      // if the PV6 is locked with SP6 and if NEW_SP6 is introduced which is lesser than AMBIENT but greater than OLD_SP6.
      // Than terminate from this IF condition by setting [mode6 = 'r'] and to jump to the next IF condition.
      if (temp_set6 > temp_set6_OLD)
      {
        flag16 = 1;
        mode6 = 'r';
        flag17 = 0;
        flag18 = 0;
      }
      if (((temp_set6 - temp_read6) <= -0.01) && ((temp_set6 - temp_read6) >= -0.08))   // COOLING
      {
        digitalWrite(direction_ctrl_pins6[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[1], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 11))
        {
          analogWrite(PWMpins6, 140);
        }
        else if ((temp_set6 >= 11) && (temp_set6 < 14))
        {
          analogWrite(PWMpins6, 130);
        }
        else if ((temp_set6 >= 14) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 120);          
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 60);          
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 30);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 25);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 15);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag16 = 1;
      }
      else if (((temp_set6 - temp_read6) >= 0.01) && ((temp_set6 - temp_read6) <= 0.08))   // HEATING
      {
        digitalWrite(direction_ctrl_pins6[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[0], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 10);
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 25);
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 35);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 40);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 50);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag16 = 1;
      }
      else if ((abs(temp_set6 - temp_read6) > 0.5) && (flag16 == 0))
      {
        act6PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins6[0], LOW);  
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[1], HIGH);
        analogWrite(PWMpins6, 250);
      }
      mode6 = 'q';
    }
    // If OLD_SP6 is below AMBIENT and the NEW_SP6 is also below AMBIENT, but the NEW_SP6 is greater than OLD_SP6. Than set the TEC6 heating mode initially
    // If OLD_SP6 is below AMBIENT and the NEW_SP6 is also below AMBIENT but greater than OLD_SP6, than set the TEC6 heating mode initially until the
    // error between PV6 and SP6 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV6 reaches the SP6, than the first IF condition becomes false, for retain the lock of PV6 with the SP6
    // the flag [mode6] is introduced. And for locking the PV6 to the SP6 the flag [flag17] is introduced.
    if (((temp_set6_OLD < Ambient_temp) && (temp_set6 <= Ambient_temp) && (temp_set6 > temp_set6_OLD)) || ((mode6 == 'w')))
    {
      // if the PV6 is locked with SP6 and if NEW_SP6 is introduced which is lesser than AMBIENT but greater than OLD_SP6.
      // Than set the heating of TEC6 with the calculated PID6 PWM DC value by setting the flag [flag17 = 0].
      if (temp_set6 > temp_set6_OLD)
      {
        flag17 = 0;
      }
      
      if (((temp_set6 - temp_read6) <= -0.01) && ((temp_set6 - temp_read6) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins6[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[1], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 11))
        {
          analogWrite(PWMpins6, 140);
        }
        else if ((temp_set6 >= 11) && (temp_set6 < 14))
        {
          analogWrite(PWMpins6, 130);
        }
        else if ((temp_set6 >= 14) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 120);          
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 60);          
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 30);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 25);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 15);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag17 = 1;
      }
      else if (((temp_set6 - temp_read6) >= 0.01) && ((temp_set6 - temp_read6) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins6[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[0], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 10);
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 25);
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 35);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 40);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 50);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag17 = 1;
      }
      else if ((abs(temp_set6 - temp_read6) > 0.5) && (flag17 == 0))
      {
        act6PID.SetControllerDirection(DIRECT);
        digitalWrite(direction_ctrl_pins6[1], LOW);  
        delayMicroseconds(55);                  
        digitalWrite(direction_ctrl_pins6[0], HIGH);
        act6PID.Compute();
        analogWrite(PWMpins6, Output6);
      }
      mode6 = 'w';
      flag16 = 0;
    }
    // If OLD_SP6 is below AMBIENT and the NEW_SP6 is also below AMBIENT, but the NEW_SP6 is lesser than OLD_SP6. Than set the TEC6 cooling mode initially
    // If OLD_SP6 is below AMBIENT and the NEW_SP6 is also below AMBIENT but lesser than OLD_SP6, than set the TEC6 cooling mode initially until the
    // error between PV6 and SP6 become less that 0.5 [degC].
    //                                                    OR 
    // if the PV6 reaches the SP6, than the first IF condition becomes false, for retain the lock of PV6 with the SP6
    // the flag [mode6] is introduced. And for locking the PV6 to the SP6 the flag [flag18] is introduced.
    if (((temp_set6_OLD <= Ambient_temp) && (temp_set6 <= Ambient_temp) && (temp_set6 < temp_set6_OLD)) || ((mode6 == 'e')))
    {
      // if the PV6 is locked with SP6 and if NEW_SP6 is introduced which is lesser than AMBIENT but also lesser than OLD_SP6.
      // Than set the cooling of TEC6 with the PWM maximum DC value by setting the flag [flag18 = 0].
      if (temp_set6 < temp_set6_OLD)
      {
        flag18 = 0;
      }
      
      if (((temp_set6 - temp_read6) <= -0.01) && ((temp_set6 - temp_read6) >= -0.08))  // COOLING
      {
        digitalWrite(direction_ctrl_pins6[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[1], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 11))
        {
          analogWrite(PWMpins6, 140);
        }
        else if ((temp_set6 >= 11) && (temp_set6 < 14))
        {
          analogWrite(PWMpins6, 130);
        }
        else if ((temp_set6 >= 14) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 120);          
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 60);          
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 30);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 25);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 15);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag18 = 1;
      }
      else if (((temp_set6 - temp_read6) >= 0.01) && ((temp_set6 - temp_read6) <= 0.08))  // HEATING
      {
        digitalWrite(direction_ctrl_pins6[1], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[0], HIGH);
        if ((temp_set6 >= 10) && (temp_set6 < 15))
        {
          analogWrite(PWMpins6, 10);
        }
        else if ((temp_set6 >= 15) && (temp_set6 < 17))
        {
          analogWrite(PWMpins6, 25);
        }
        else if ((temp_set6 >= 17) && (temp_set6 < 19))
        {
          analogWrite(PWMpins6, 35);          
        }
        else if ((temp_set6 >= 19) && (temp_set6 < 20))
        {
          analogWrite(PWMpins6, 40);          
        }
        else if ((temp_set6 >= 20) && (temp_set6 <= 23))
        {
          analogWrite(PWMpins6, 50);          
        }
//        else if ((temp_set6 >= 21) && (temp_set6 < 23))
//        {
//          analogWrite(PWMpins6, 1);          
//        }
        flag18 = 1;
      }
      else if ((abs(temp_set6 - temp_read6) > 0.5) && (flag18 == 0))
      {
        act6PID.SetControllerDirection(REVERSE); 
        digitalWrite(direction_ctrl_pins6[0], LOW); 
        delayMicroseconds(55);                     
        digitalWrite(direction_ctrl_pins6[1], HIGH);
        analogWrite(PWMpins6, 250);
      }
      mode6 = 'e';
      flag16 = 0;
      flag17 = 0;
    }
                               
  }
  temp_set1_OLD = temp_set1;    // Store the previous SP1
  temp_set2_OLD = temp_set2;    // Store the previous SP2
  temp_set3_OLD = temp_set3;    // Store the previous SP3
  temp_set4_OLD = temp_set4;    // Store the previous SP4
  temp_set5_OLD = temp_set5;    // Store the previous SP5
  temp_set6_OLD = temp_set6;    // Store the previous SP6
  count=1;
}

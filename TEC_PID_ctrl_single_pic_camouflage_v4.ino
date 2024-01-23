
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 25
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
int total_devices=1;
DeviceAddress sensor_address; 

float temp_read, temp_set, temp_sensor;
int start = 0;
char mode;
int PWMpins = 5;
int direction_ctrl_pins[] = {36,37};    //  defining control pins for changing the logic to control the direction of TECs heat flow
float elapsed_time, Time, prev_time;    // initializing variables for calculation of D controller value
float PID_value = 0, PID_error = 0, prev_error = 0, pwm;
float P_ctrl = 0, I_ctrl = 0, D_ctrl = 0;   // initializing variable for storing calculated values of P,I & D controller
double kp = 0, ki = 0, kd = 0;  
int flag=0;
unsigned long currentMillis, previousMillis;

void setup() {
  Serial.begin(9600);
  pinMode(PWMpins,OUTPUT);
  TCCR3B = TCCR3B & B11111000 | B00000010;        // Setting PWM frequency to 4kHz
  digitalWrite(PWMpins,LOW); 
  pinMode(direction_ctrl_pins[0], OUTPUT); 
  digitalWrite(direction_ctrl_pins[0], LOW);
  pinMode(direction_ctrl_pins[1], OUTPUT); 
  digitalWrite(direction_ctrl_pins[1], LOW);

  // Initializing connection with DS18b20 temperature sensor
  sensors.begin();
  sensors. setResolution(12);
  sensors.setWaitForConversion(false);
  previousMillis = 0;
  sensors.requestTemperatures(); // Send the command to get temperatures
}

void loop() {
  
  if (Serial.available() > 0)
  {
    static char buffer[32];
    static size_t pos;
    char c = Serial.read();
    buffer[pos++] = c;
//    Serial.println(c);

    if (c == 'p')
    {
      buffer[pos] = '\0';
      temp_set = atof(buffer);
//      Serial.print(temp_set);
//      Serial.print('p');
//        Serial.println();
      pos = 0;
      buffer[32] = {};
    }
    if (c == 'e')
    {
      buffer[pos] = '\0';
      temp_read = atof(buffer);
//      Serial.print(temp_read);
//      Serial.println("e");
      pos = 0;
      buffer[32] = {};
    }
//    if (c == 'g')
//    {
/* ---------------------------------------------- */
//      Technique 1:
/* ---------------------------------------------- */
//      temp_sensor = sensors.getTempCByIndex(0);
//      Serial.println(temp_sensor);
//      sensors.setWaitForConversion(false);  // makes it async
//      sensors.requestTemperatures();
/* ---------------------------------------------- */

/* ---------------------------------------------- */
//      Technique 2:
/* ---------------------------------------------- */
//      currentMillis = millis();
//      if ((currentMillis - previousMillis) > 750)
//      {
//        previousMillis = currentMillis; 
//        temp_sensor = sensors.getTempCByIndex(0);
//        Serial.println(temp_sensor);
//        sensors.requestTemperatures();
//      }
//      else
//      {
//        Serial.println('0');
//      }
/* ---------------------------------------------- */
//      start = 0;
//      pos = 0;
//      buffer[32] = {};
//    }
    if (c == 'R')
    {
      buffer[pos] = '\0';
      Serial.print('1');
      pos = 0;
      buffer[32] = {};
      start = 1;
    }
  
    // At the start, the camou-plate is heated for contour detection
    if (start == 1)
    {
      digitalWrite(direction_ctrl_pins[1], LOW);
      delayMicroseconds(55);
      digitalWrite(direction_ctrl_pins[0], HIGH);
      analogWrite(PWMpins,50);   //50
    }

      if ((temp_set >= 10) && (temp_set < 14))
      {
        kp = 30000, ki = 3000, kd = 7000;
      }
      if ((temp_set >= 14) && (temp_set < 20))
      {
        kp = 20000, ki = 1500, kd = 7000;
      }
      if ((temp_set >= 20) && (temp_set < 39))
      {
        kp = 150, ki = 65, kd = 7500;
      }
      if ((temp_set >= 39) && (temp_set < 45))
      {
        kp = 1500, ki = 500, kd = 7500;
      }
      if ((temp_set >= 45) && (temp_set <= 50))
      {
        kp = 3000, ki = 2000, kd = 10000;
      }
      
       if (temp_set > temp_read)                     // If set point (SP) is greater than process variable (PV) than switch TEC to heating
      {
        //Serial.println("Heating...");
        digitalWrite(direction_ctrl_pins[1], LOW);  // set logic LOW to control pin 3 to turn-off Q2 & Q3 MOSFETs
        delayMicroseconds(55);                      // delay to avoid shoot-through
        digitalWrite(direction_ctrl_pins[0], HIGH); // set logic HIGH to control pin 2 to turn-on Q1 & Q4 MOSFETs 

        mode = 's';
        analogWrite(PWMpins,255);

        //Serial.println(mode);
      }
      if (temp_set < temp_read)                     // If set point (SP) is lesser than process variable (PV) than set the TEC to cooling
      {
        //Serial.println("Cooling...");
        digitalWrite(direction_ctrl_pins[0], LOW);  // set logic LOW to pin 2 to turn-off Q1 & Q4 MOSFETs 

        delayMicroseconds(55);                      // delay to avoid shoot-through
        digitalWrite(direction_ctrl_pins[1], HIGH); // set logic HIGH to pin 3 to turn-on Q2 & Q3 MOSFETs

        mode = 's'; 
        analogWrite(PWMpins,255);

      }
  }
  
  if (mode == 's')
    {
   
      PID_error = temp_set - temp_read;   // calculating the error
      
      /* If SP input is less than the value of temperature read than the error calculated is negative 
         In my case than the PID value calculated for pwm is not suffient to cool down the load 
         so setting the condition to take only the magnitude of the error in this case    */
      PID_error = abs(PID_error);         // if error is negative than only read the magnitude
      // Calculaing value of proportional controller
      P_ctrl = 0.01 * kp * PID_error;
      //P_ctrl = kp*PID_error;
      
  
      // Calculating value of integral controller
      I_ctrl = 0.01 * I_ctrl + (ki * PID_error);
      if (I_ctrl < 0)
      {
        I_ctrl = 0;
      }
      if (I_ctrl > 255)
      {
        I_ctrl = 255;
      }
      //Serial.print(I_ctrl);
      // Calculating value of Derivative controller
      prev_time = Time;
      Time = millis();
      elapsed_time = (Time - prev_time)/1000;
      D_ctrl = 0.01 * kd * ((PID_error - prev_error) / elapsed_time);

      // Calculating final value of PID controller (PID = P + I + D)
      PID_value = P_ctrl + I_ctrl + D_ctrl;

      // Defining range of PWM (0 - 255) by using PID value
      if (PID_value < 0)
      {
        PID_value = abs(PID_value);
      }
      else if (PID_value > 255)
      {
        PID_value = 255;
      }
      else {
        PID_value = PID_value;
      }

      // Writing value at PWM pins 
      if (temp_set > temp_read)
      {
        //Serial.print("11Heating...");
        digitalWrite(direction_ctrl_pins[1], LOW);  // set logic LOW to control pin 3 to turn-off Q2 & Q3 MOSFETs
   
        delayMicroseconds(55);                      // delay to avoid shoot-through
        digitalWrite(direction_ctrl_pins[0], HIGH); // set logic HIGH to control pin 2 to turn-on Q1 & Q4 MOSFETs 
      }
      if (temp_set < temp_read)
      {
        //Serial.print("11Cooling...");
        digitalWrite(direction_ctrl_pins[0], LOW);  // set logic LOW to pin 2 to turn-off Q1 & Q4 MOSFETs 
     
        delayMicroseconds(55);                      // delay to avoid shoot-through
        digitalWrite(direction_ctrl_pins[1], HIGH); // set logic HIGH to pin 3 to turn-on Q2 & Q3 MOSFETs
      }
      if (temp_set == temp_read)
      {
        digitalWrite(direction_ctrl_pins[0], LOW);  // set logic LOW to pin 2 to turn-off Q1 & Q4 MOSFETs 
   
        digitalWrite(direction_ctrl_pins[1], LOW); // set logic HIGH to pin 3 to turn-off Q2 & Q3 MOSFETs
      }
      //int pwm = 255 - PID_value;
      //Serial.print(",");
      //Serial.print(PID_value);
      //Serial.print(",");
      //Serial.print(pwm);
      //Serial.println();
      //Serial.print(pwm);
      
      analogWrite(PWMpins, PID_value);
      //Serial.print(",");
      //Serial.println(PID_value);
      //Serial.println();
      prev_error = PID_error;
    }
}

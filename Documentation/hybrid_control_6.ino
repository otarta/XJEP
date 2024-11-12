#include <Servo.h>
#include "ADS1X15.h"

//Define all of the GSU engine state strings that could be received over serial
#define trim_low "Trim Low"
#define stick_low "StickLo!"
#define ready "Ready"
#define prime_vap "PrimeVap"
#define burner_on "BurnerOn"
#define start_on "Start On"
#define ignition "Ignition"
#define stage_1 "Stage1"
#define stage_2 "Stage2"
#define stage_3 "Stage3"
#define time_out "Time Out"
#define running "Running"
#define cooling "Cooling"
#define stop "Stop"
#define glow_bad "GlowBad"
#define start_bad "StartBad"
#define low_rpm "Low RPM"
#define high_temp "HighTemp"
#define flame_out "FlameOut"

//Define how often data is sent to the laptop
#define serial_dt 200

#define engine_pin 2     //The digital pin the engine PWM goes to
#define voltage_pin A0   //The analog pin that the voltage is read by, hooked up to the voltage divider
#define hall_pin A1      //The analog pin that the current is read by, hooked up to the hall effect current sensor
//#define idle_voltage 20  //Voltage at idle (undershoot rather than overshoot) UNUSED IN CURRENT CODE

//Define how long the hybrid system has to stabilize the voltage before it shuts off the engine for safety reasons
#define stability_time 2000

//The digital pin that the load switch is connected to, if theres a load switch
#define load_pin 3

//Start button debouncing time and the pin of the start button. (debouncing helps with silly stuff that buttons do so they actually work)
#define button_debounce_millis 20
#define start_button_pin 4

//#define conductivity_shunt 2667

#define R1_V 220000                  //Resistor 1 in voltage divider
#define R2_V 4700                    //Resistor 2 in voltage divider
#define VD_correction 0.96         //Resistors aren't always accurate so this correction factor uses measured data to calibrate the divider.
//#define voltage_setpoint 33        //Setpoint of the voltage, this should be set to the bus voltage that the load is expecting, and the voltage of the backup batteries.
#define min_acceptable_voltage 40  //This is the minimum voltage that will be tolerated before the engine shuts off
#define max_acceptable_voltage 58  //This is the maximum voltage that will be tolerated before the engine shuts off so it doesn't kill the batteries or the load.

#define dt_ms 100          //PID loop time delta
#define full_throttle 160  //Angle representative of full throttle
#define trim_down 30       //Angle representative of min throttle and trim down (this is the OFF state)
#define min_throttle 45    //Angle representative of min throttle

#define KP -0.0002    //Proportionality constant, if it is negative its cause I wired the current sensor backwards so a positive value would be current going into the battery
#define KI 0         //Integral constant, init guess: 0.0000002 START AT ZERO FOR TUNING
#define KD 0         //Derivative constant, init guess: 0.05 START AT ZERO FOR TUNING
#define IMax 115   //Maximum angle representation of throttle that the I term can contribute
#define IMin -100  //Minimum angle representation of throttle that the I term can contribute

#define current_sense_offset 597

//These variables represent all of the data coming in from the data relay module.
unsigned char rver0;
unsigned char rver1;
unsigned char Pw_raw[2];
uint16_t pw_raw;
unsigned char Rpm_raw[2];
uint16_t rpm_raw;
unsigned char Tmp_raw[2];
uint16_t temp_raw;
unsigned char Rpm_disp[2];
uint16_t rpm_disp;
unsigned char Pw_bar_pcent;
unsigned char rver2;
unsigned char rver3;
unsigned char rver4;
unsigned char rver5;
unsigned char rver6;
char model[9];
char engine_state[17];
char throttle_string[9];
char b_vol[9];
char tp_rpm[9];
char pwr[9];
char tmp[9];

//String variables to store the string values of the character arrays so that they can be converted into actual math and stuff data
String model_string;
String engine_state_string;
String throttle_str;
String battery_voltage_str;
String ts_rpm_str;
String power_str;
String EGT_str;

//Aforementioned math and stuff versions of the data
uint16_t EGT;
uint16_t power;
uint8_t ts_rpm;
uint8_t throttle_val;
float battery_voltage;

//Time adjusted versions of KP, KI, and KD. This is necessary so that the PID frequency can be changed without needing to retune.
const float KPt = KP * dt_ms;
const float KIt = KI * dt_ms * dt_ms;
const float KDt = KD;

//Variables to store the P, I, and D values.
int16_t P = 0;
int16_t I = 0;
int16_t D = 0;

//Variable to store the output angle that will be sent to the engine.
unsigned int output_angle = min_throttle;

//A bunch of constants that convert from the voltage to the analog representation of the voltage using the voltage divider formula and the arduino conversion from 5 volts to a 16 bit integer.
//const float shunt_constant = conductivity_shunt*((float)R1_C/(float)R2_C + 1);
//const unsigned int analog_target = VD_correction * ((voltage_setpoint * 1023.0 * R2_V) / (5 * (R1_V + R2_V)));
//const unsigned int analog_idle = VD_correction * ((idle_voltage * 1023.0 * R2_V) / (5 * (R1_V + R2_V))); UNUSED IN CURRENT CODE
const unsigned int analog_min_acc = VD_correction * ((min_acceptable_voltage * 1023.0 * R2_V) / (3.3 * (R1_V + R2_V)));
const unsigned int analog_max_acc = VD_correction * ((max_acceptable_voltage * 1023.0 * R2_V) / (3.3 * (R1_V + R2_V)));
const float voltage_constant = 3.3 * (R1_V + R2_V) / (VD_correction * 1023.0 * R2_V);  //constant to multiply analog value by to yield actual voltage, the first factor is just to get 10x the voltage
unsigned long last_PID_time = 0;                                               //Variable to store the last time the PID loop was run
int last_PID_error = 0;                                                   //Variable to store the last PID_error value for PID derivative term

uint8_t state = 0;  //0->off, 1->on, 2->starting, 3->stopping
uint8_t last_state = 0;

//Variables to store start button information
bool start_button_state;
bool last_start_button_state;
unsigned long start_button_downtime = 0;
unsigned long start_button_uptime = 0;

//Variable to store the voltage error.
int16_t PID_error = 0;

//Last time that serial information was sent. This is necessary to send serial information every N milliseconds instead of every loop.
unsigned long last_serial_time = 0;

//I don't think we actually use these variables anymore.
bool rapid_idle_state = 0;
bool last_rapid_idle_state = 0;
unsigned long idle_up_time = 0;
unsigned long idle_down_time = 0;

//Variables that store information as to whether or not the voltage is chill like that, and how long it hasn't been chill for if that is the case.
bool voltage_adequacy = 1;
bool last_voltage_adequacy = 1;
unsigned long voltage_inadequate_time = 0;

uint8_t error_message[2] = { 0, 0 };
/*
{x,0} (electronics): 0 - no electronics error, 1 - voltage timeout
{0,x} (engine codes): 0 - no engine error, 1 - glow plug bad, 2 - bad starter motor, 3 - low RPM, 4 - high EGT, 5 - flame out
*/
uint8_t start_byte = 193;  //Start byte for Serial data transmission
uint8_t end_byte = 195;    //End byyte for Serial data transmission

Servo engine;  //Servo object that represents the engine

void setup() {
  // put your setup code here, to run once:
  engine.attach(engine_pin);                //Attach the engine servo to the correct pin
  engine.write(trim_down);                  //Write the OFF servo command to ensure engine isn't recieving throttle initially
  pinMode(voltage_pin, INPUT);              //Set the voltage input pin to be input, duh
  pinMode(start_button_pin, INPUT_PULLUP);  //Set the start button pin to input that is pulled high, the other end of the button is thus hooked to ground
  pinMode(hall_pin, INPUT);
  Serial.begin(9600);                       //Begin Serial port
  Serial2.begin(19200);
  
}

void loop() {
  // put your main code here, to run repeatedly:

  unsigned long current_time = millis();                  //Store the current time of the program running in milliseconds
  start_button_state = digitalRead(start_button_pin);     //Read the state of the start button, either on or off
  unsigned int voltage_analog = analogRead(voltage_pin);  //Read the voltage of the hybrid system output, this has been passed through a voltage divider
  int16_t battery_current_analog = (int16_t)analogRead(hall_pin) - current_sense_offset;
  
  //float shunt_voltage_diff = ADS.toVoltage(ADS.readADC_Differential_0_1());
  PID_error = battery_current_analog;                 //The error is the same as the current supplied to the batteries, or taken from, idk, depends on the sign

  if (Serial2.available() >= 60) {  //If the Serial2 buffer has a lot of data in it, assume it has received a message from the GSU, each message starts with 0xFC (or 0x00), 0x52, 0x00, 0x00, 0x40, hence the nested if statements
    uint8_t inByte1 = Serial2.read();
    if (inByte1 == 0xFC || inByte1 == 0x0) {
      if (Serial2.read() == 0xC5) {
        if (Serial2.read() == 0x52) {
          if (Serial2.read() == 0x00) {
            if (Serial2.read() == 0x00) {
              if (Serial2.read() == 0x40) {
                rver0 = Serial2.read();  //Every time a byte is read from the serial buffer it clears that byte, so each byte is read individually and stored to the appropriate variable.
                rver1 = Serial2.read();

                Pw_raw[0] = Serial2.read();  //I dont actually know what this variable is, I think it is pulse width.
                Pw_raw[1] = Serial2.read();
                pw_raw = Pw_raw[0] + (Pw_raw[1] << 8);

                Rpm_raw[0] = Serial2.read();  //Engine RPM, a 16 bit unsigned int.
                Rpm_raw[1] = Serial2.read();
                rpm_raw = Rpm_raw[0] + (Rpm_raw[1] << 8);

                Tmp_raw[0] = Serial2.read();  //Exhaust gas temperature
                Tmp_raw[1] = Serial2.read();
                temp_raw = Tmp_raw[0] + (Tmp_raw[1] << 8);
                EGT = temp_raw;

                Rpm_disp[0] = Serial2.read();  //No idea
                Rpm_disp[1] = Serial2.read();
                rpm_disp = Rpm_disp[0] + (Rpm_disp[1] << 8);

                Pw_bar_pcent = Serial2.read();  //No idea

                rver2 = Serial2.read();

                rver3 = Serial2.read();

                rver4 = Serial2.read();

                rver5 = Serial2.read();

                rver6 = Serial2.read();

                for (int i = 0; i < sizeof(model) - 1; i++) {  //Model name
                  model[i] = Serial2.read();
                }
                model[sizeof(model) - 1] = '\0';
                model_string = String(model);

                for (int i = 0; i < sizeof(engine_state) - 1; i++) {  //State of the engine according to the GSU, define statements at the beginning detail the possible states.
                  engine_state[i] = Serial2.read();
                }
                engine_state[sizeof(engine_state) - 1] = '\0';
                engine_state_string = String(engine_state);

                for (int i = 0; i < sizeof(throttle_string) - 1; i++) {  //Throttle value according to the GSU.
                  throttle_string[i] = Serial2.read();
                }
                throttle_string[sizeof(throttle_string) - 1] = '\0';
                throttle_str = String(throttle_string);
                throttle_str.remove(0, 3);
                throttle_val = throttle_str.toInt();

                for (int i = 0; i < sizeof(b_vol) - 1; i++) {  //Battery voltage of the 2 cell battery powering the GSU and arduino and whatnot
                  b_vol[i] = Serial2.read();
                }
                b_vol[sizeof(b_vol) - 1] = '\0';
                battery_voltage = String(b_vol).toFloat();

                for (int i = 0; i < sizeof(tp_rpm) - 1; i++) {  //RPM of the turboprop, not currently working for some reason
                  tp_rpm[i] = Serial2.read();
                }
                tp_rpm[sizeof(tp_rpm) - 1] = '\0';
                ts_rpm_str = String(tp_rpm);
                ts_rpm_str.remove(0, 3);
                ts_rpm = ts_rpm_str.toInt();

                for (int i = 0; i < sizeof(pwr) - 1; i++) {  //Power? maybe?
                  pwr[i] = Serial2.read();
                }
                pwr[sizeof(pwr) - 1] = '\0';
                power_str = String(pwr);

                for (int i = 0; i < sizeof(tmp) - 1; i++) {  //EGT String
                  tmp[i] = Serial2.read();
                }
                tmp[sizeof(tmp) - 1] = '\0';


                if (engine_state_string.indexOf(prime_vap) != -1 || engine_state_string.indexOf(burner_on) != -1 || engine_state_string.indexOf(start_on) != -1 || engine_state_string.indexOf(ignition) != -1 || engine_state_string.indexOf(stage_1) != -1 || engine_state_string.indexOf(stage_2) != -1 || engine_state_string.indexOf(stage_3) != -1) {
                  //Details all of the starting states, the engine progresses through multiple states while starting and this if statement sets the state to "2" if it is in any of them.
                  state = 2;
                } else if (engine_state_string.indexOf(cooling) != -1) {
                  //If the engine is cooling then the engine is stopping.
                  state = 3;
                } else if (engine_state_string.indexOf(stop) != -1 || engine_state_string.indexOf(trim_low) != -1 || engine_state_string.indexOf(stick_low) != -1 || engine_state_string.indexOf(ready) != -1) {
                  //If the engine is in any of the states that imply that it isn't running then the state is off or "0".
                  state = 0;
                } else if (engine_state_string.indexOf(running) != -1 && state != 3 && state != 0) {
                  //If the engine is running, set the state to 1. However do not do this if the state is in "stopping" or "off", because that transition should never happen,
                  //in addition, the state is set to "stopping" by the code further down, but the GSU does not register this since they are two seperate systems, thus
                  //we must ensure that the state is not set to "on" (thereby activating the PID loop) when the engine is supposed to be stopping.
                  state = 1;
                } else if (engine_state_string.indexOf(glow_bad) != -1) {  //If the engine has an error, log the error to the engine error message and stop the engine.
                  state = 3;
                  error_message[1] = 1;
                } else if (engine_state_string.indexOf(start_bad) != -1) {
                  state = 3;
                  error_message[1] = 2;
                } else if (engine_state_string.indexOf(low_rpm) != -1) {
                  state = 3;
                  error_message[1] = 3;
                } else if (engine_state_string.indexOf(high_temp) != -1) {
                  state = 3;
                  error_message[1] = 4;
                } else if (engine_state_string.indexOf(flame_out) != -1) {
                  state = 3;
                  error_message[1] = 5;
                }
              }
            }
          }
        }
      }
    }
  }

  if (Serial.available() > 0) {                   //If there is serial data from the laptop, do the following
    String message = Serial.readStringUntil(13);  //Read the incoming serial data until 13 (craig or cairrage however u spell it return character)
    if (message.length() == 2) {                  //If the message length is 2 bytes, we assume it is a valid message
      uint8_t cmd = message[0];                   //The command is the first byte
      uint8_t val = message[1];                   //The value of the command is the second byte

      if (cmd == 0) {                  //Command 0 means to either start or stop the engine
        if (val == 1 && state == 0) {  //Command 0 value 1 means start the engine
          //Start procedure for the engine
          engine.write(trim_down);
          delay(1000);
          engine.write(min_throttle);
          delay(1000);
          engine.write(1.5 * (min_throttle + full_throttle) / 2);
          delay(500);
          engine.write(min_throttle);
        } else if (val == 0) {  //Command 0 value 0 means stop the engine
          //To stop the engine all that is required is for the throttle value to be set to the trim down position
          engine.write(trim_down);
          //Set the state to stopping so that the PID loop does not remain active, thereby overwriting the trim_down command.
          state = 3;
        }
      }
    }
  }
  /*
  //Start button debouncing, same logic as previous start procedure just with a button instead
  if (start_button_state != last_start_button_state && start_button_state == 0) {
    start_button_downtime = current_time;
  }
  if (start_button_state != last_start_button_state && start_button_state == 1) {
    start_button_uptime = current_time;
    if (start_button_uptime - start_button_downtime >= button_debounce_millis) {
      if (state == 0) {
        engine.write(trim_down);
        delay(1000);
        engine.write(min_throttle);
        delay(1000);
        engine.write(1.5 * (min_throttle + full_throttle) / 2);
        delay(500);
        engine.write(min_throttle);
      } else {
        engine.write(trim_down);
        state = 3;
      }
    }
  }
  last_start_button_state = start_button_state;
  */

  //Beginning the PID loop
  if (state == 1) {
    digitalWrite(load_pin, 1);  //Activate the load, if connected via load switch, if not then this does nothing.
    error_message[0] = 0;
    //Reset the PID values if the PID loop is just starting (AKA the last state was not PID, thus the loop just started)
    if (last_state != 1) {
      P = 0;
      I = 0;
      D = 0;

      output_angle = min_throttle;
    }

    //Run the PID every dt_ms milliseconds
    if (current_time - last_PID_time >= dt_ms) {
      //Calculate P, I, and D values, this may be broken considering it shoots it to full throttle at the current settings.
      P = KPt * PID_error;
      I += KIt * PID_error;
      D = KDt * (PID_error - last_PID_error);

      //I term limiting to make sure I buildup doesn't occur.
      if (I >= IMax) {
        I = IMax;
      } else if (I <= IMin) {
        I = IMin;
      }

      //Set the output_angle (AKA the throttle) to the sum of P, I, and D, plus the min_throttle value since 0 is not the minimum.
      output_angle += P + I + D;
      //Clamp the output angle to full throttle and min throttle.
      if (output_angle > full_throttle) {
        output_angle = full_throttle;
      } else if (output_angle < min_throttle) {
        output_angle = min_throttle;
      }

      last_PID_error = PID_error;
      last_PID_time = current_time;

      //Write the output_angle to the engine pin, it is refered to as an angle because the "write" command expects a servo rather than an engine, so it sends an angle command.
      engine.write(output_angle);
    } 


    //Low and high voltage protection stuff.
    //If the voltage is above the maximum acceptable voltage, or it is below the minimum acceptable voltage while at full throttle, the voltage is considered inadequate
    if (voltage_analog < analog_min_acc && output_angle == full_throttle || voltage_analog > analog_max_acc) {
      voltage_adequacy = 0;
    } else if (voltage_analog >= analog_min_acc && voltage_analog <= analog_max_acc) {
      voltage_adequacy = 1;
    }
    if (voltage_adequacy != last_voltage_adequacy && voltage_adequacy == 0) {
      voltage_inadequate_time = current_time;
    }

    //If the voltage is inadequate and has been that way for more than the stability time, throw an error, disconnect the load, and stop the engine.
    if (voltage_inadequate_time != 0 && voltage_adequacy == 0 && current_time - voltage_inadequate_time >= stability_time) {
      error_message[0] = 1;
      state = 3;
      digitalWrite(load_pin, 0);
      engine.write(trim_down);
    }
    last_voltage_adequacy = voltage_adequacy;
  }

  //Do serial math and write all the serial information to the laptop for data monitoring purposes.
  if (current_time - last_serial_time >= serial_dt) {
    uint16_t current_time_seconds = 0.001 * current_time;
    uint8_t current_time_seconds_byte_1 = current_time_seconds & 0xFF;
    uint8_t current_time_seconds_byte_2 = current_time_seconds >> 8;
    uint8_t throttle = map(output_angle, min_throttle, full_throttle, 0, 100);
    uint8_t voltage = voltage_analog * voltage_constant;
    uint8_t rpm_byte_1 = rpm_raw & 0xFF;
    uint8_t rpm_byte_2 = rpm_raw >> 8;
    uint8_t EGT_byte_1 = EGT & 0xFF;
    uint8_t EGT_byte_2 = EGT >> 8;
    uint8_t PID_err_1 = PID_error & 0xFF;
    uint8_t PID_err_2 = PID_error >> 8;
    uint8_t P_1 = P & 0xFF;
    uint8_t P_2 = P >> 8;
    uint8_t I_1 = I & 0xFF;
    uint8_t I_2 = I >> 8;
    uint8_t D_1 = D & 0xFF;
    uint8_t D_2 = D >> 8;
    uint8_t message[22] = { start_byte, 
                            current_time_seconds_byte_1, 
                            current_time_seconds_byte_2, 
                            throttle, 
                            voltage, 
                            rpm_byte_1, 
                            rpm_byte_2, 
                            ts_rpm, 
                            EGT_byte_1, 
                            EGT_byte_2, 
                            state, 
                            error_message[0], 
                            error_message[1], 
                            PID_err_1,
                            PID_err_2,
                            P_1,
                            P_2,
                            I_1,
                            I_2,
                            D_1,
                            D_2,
                            end_byte 
                          };
    Serial.write(message, 22);
    last_serial_time = current_time;
  }

  last_state = state;
}

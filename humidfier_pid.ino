#include "pin_define.h"


//LCD config
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

int PWMplate_pin = 9;  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)

//----- Variables ------

float set_temp_plate = 0;  //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float set_temp_air = 0;  //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float up_threshold;
float down_threshold;
float PID_error1 = 0;
float previous_error1 = 0;
float elapsedTime1, Time1, timePrev1;
float PID_value1 = 0;

float temperature_read2 = 0.0;
float PID_error2 = 0;
float previous_error2 = 0;
float elapsedTime2, Time2, timePrev2;
float PID_value2 = 0;

int menu_activated = 0;



//----- PID constants -----
int kp = 90, ki = 30, kd = 80;
int PID_p1 = 0, PID_i1 = 0, PID_d1 = 0;
int PID_p2 = 0, PID_i2 = 0, PID_d2 = 0;
float last_kp1 = 0, last_ki1 = 0, last_kd1 = 0;
float last_kp2 = 0, last_ki2 = 0, last_kd2 = 0;
int PID_values_fixed1 = 0, PID_values_fixed2 = 0;

// ----- Messages define -----
String err[3] = {"Plate Overheat", "Tube Overheat", "Sensor error"};
String cwm[3] = {"Setup", "Heating up", "Idle"};
boolean isErr;


// ----- NTC reader ----- //        TODO: Improve samples average value (3, 5, 10)
double ntcRead(int ntc_pin, int sampleRate) {
  double input, Vo, R2, lnR2, T, Tc;
  double R1 = 100000;
  double c1 = 0.8272069482e-3, c2 = 2.087897328e-4, c3 = 0.8062131944e-7;

  double avg = 0, avgT = 0;

  for (int i = 0; i < sampleRate; i++) {
    input = analogRead(ntc_pin);
    Vo = 1023 - input;
    R2 = R1 * ((1023 / Vo) - 1);
    Serial.println(R2);
    lnR2 = log(R2);
    T = (1 / (c1 + c2 * lnR2 + c3 * lnR2 * lnR2));
    Tc = T - 273.15;
    Serial.println(Tc);

    avg += Tc;
  }
  avgT = avg / sampleRate;
  Serial.println(avgT);

  return avgT;
}

double read_ntc_AirIn() {
  sensor_DS18B20_AirIn.requestTemperatures();
  return sensor_DS18B20_AirIn.getTempCByIndex(0);
}

double read_ntc_AirOut() {
  sensor_DS18B20_AirOut.requestTemperatures();
  return sensor_DS18B20_AirOut.getTempCByIndex(0);
}

void error(int code) {
    return err[code]
}

void lcdPrint(int workmode, float val1, float val2) {
    lcd.clear();
    switch (workmode) {
        case 0: // Setup mode
            lcd.setCursor(0,0);
            lcd.print(cwm[0]);
            lcd.setCursor(0,1);
            lcd.print("> Select mode");
            break;
        case 1: // Working mode //          TODO: Consider LCD Refresh Rate
            lcd.setCursor(0,0);
            lcd.print("Set: %.2f °C", val1);
            lcd.setCursor(0,1);
            lcd.print("Curr: %.2f °C", val2);
            break;
        case 2:
            lcd.setCursor(0,0);
            lcd.print(cwm[2]);
            lcd.setCursor(0,1);
            lcd.print(err[(int)val1]);
            break;
    };

}

// ----- Main -----
void setup() {
  Serial.begin(9600);
  sensor_DS18B20_plate.begin();
  sensor_DS18B20_AirIn.begin();
  sensor_DS18B20_AirOut.begin();
  pinMode(PWMplate_pin, OUTPUT);
  pinMode(Relay, OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BTN3, INPUT_PULLUP);
  Time1 = millis();
  lcd.init();
  lcd.backlight();

  lcdPrint(0, 0, 0);

  while (set_temp_air == 0) {
    if (digitalRead(BTN1) == 0) {
      set_temp_air = 35;
    }
    if (digitalRead(BTN2) == 0) {
      set_temp_air = 37;
    }
    if (digitalRead(BTN3) == 0) {
      set_temp_air = 39;
    }
  }

  delay(500);
  lcd.clear();

  up_threshold = set_temp_air - 2;
  down_threshold = set_temp_air - 5;
}

void loop() {


  temp_read_AirIn = read_ntc_AirIn();
  temp_read_AirOut = read_ntc_AirOut();
  temp_read_plate = ntcRead(ntc_plate, 1);
  float delta_wire_temp = temp_read_AirOut - temp_read_AirIn;

  if (temp_read_plate > 150) {
      isErr = true;
      lcdPrint(2, 0, 0);
      digitalWrite(Relay, LOW);
      analogWrite(PWMheaterwire, 0);
  }

  if (temp_read_AirOut > 42) {
      isErr = true;
      lcdPrint(2, 1, 0);
      digitalWrite(Relay, LOW);
      analogWrite(PWMheaterwire, 0);
  }


  while (isErr != false) {
    if ( temp_read_AirIn < up_threshold && temp_read_plate <= 75 ) {
      digitalWrite(Relay, HIGH);
    }

    if ( temp_read_AirIn > down_threshold ) {
      digitalWrite(Relay, LOW);
    }

    temp_read_AirOut = read_ntc_AirOut();                     // First we read the real value of temperature
    PID_error1 = temp_read_AirOut - temp_read_AirOut + 3;     //Next we calculate the error between the setpoint and the real value
    PID_p1 = 0.01 * kp * PID_error1;                          //Calculate the P value
    PID_i1 = 0.01 * PID_i1 + (ki * PID_error1);               //Calculate the I value in a range on +-3

    //For derivative we need real time to calculate speed change rate
    timePrev1 = Time1;  // the previous time is stored before the actual time read
    Time1 = millis();  // actual time read
    elapsedTime1 = (Time1 - timePrev1) / 1000;
    PID_d1 = 0.01 * kd * ((PID_error1 - previous_error1) / elapsedTime1);     //Calculate the D calue
    PID_value1 = PID_p1 + PID_i1 + PID_d1;                    //Final total PID value is the sum of P + I + D

    //We define PWM range between 0 and 255
    if (PID_value1 < 0) {
      PID_value1 = 0;
    }
    if (PID_value1 > 255) {
      PID_value1 = 255;
    }
    //Now we can write the PWM signal to the mosfet on digital pin D3
    //Since we activate the MOSFET with a 0 to the base of the BJT, we write PID value
    analogWrite(PWMheaterwire, PID_value1);
    previous_error1 = PID_error1;  //Remember to store the previous error for next loop.

    delay(150);  //Refresh rate + delay of LCD print
    lcdPrint(1, set_temp_air, temp_read_AirOut);
  }
}

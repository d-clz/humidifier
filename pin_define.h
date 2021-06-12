#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS_1 12  //DS18B20 pin
#define ONE_WIRE_BUS_2 11  //DS18B20 pin
#define ONE_WIRE_BUS_3 8  //DS18B20 pin

OneWire oneWire_DS18B20_plate(ONE_WIRE_BUS_1);
DallasTemperature sensor_DS18B20_plate(&oneWire_DS18B20_plate);
OneWire oneWire_DS18B20_AirIn(ONE_WIRE_BUS_2);
DallasTemperature sensor_DS18B20_AirIn(&oneWire_DS18B20_AirIn);
OneWire oneWire_DS18B20_AirOut(ONE_WIRE_BUS_3);
DallasTemperature sensor_DS18B20_AirOut(&oneWire_DS18B20_AirOut);

#define PWMheaterwire 10

#define BTN1 2
#define BTN2 3
#define BTN3 4

#define Relay 13

#define ntc_plate A0       // which analog pin to connect
  
#define plateTHERMISTORNOMINAL 100000      // resistance at 25 degrees C
#define plateTEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
#define plateBCOEFFICIENT 3950        // The beta coefficient of the thermistor (usually 3000-4000)
#define plateSERIESRESISTOR 100000    // the value of the series resistor (GND -> Analog)

int plateNTCraw;
float plateSteinhart;


#define AirInNTCpin A1       // which analog pin to connect  
#define AirOutNTCpin A2       // which analog pin to connect  

#define AirTHERMISTORNOMINAL 2000      // resistance at 25 degrees C
#define AirTEMPERATURENOMINAL 25   // temp. for nominal resistance (almost always 25 C)
#define AirBCOEFFICIENT 3950        // The beta coefficient of the thermistor (usually 3000-4000)
#define AirSERIESRESISTOR 2200    // the value of the series resistor (GND -> Analog)

int AirInNTCraw;
float AirInSteinhart;
int AirOutNTCraw;
float AirOutSteinhart;

float temp_read_AirIn;
float temp_read_AirOut;
float temp_read_plate;

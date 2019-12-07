/*
Project: SmartBox
Created: Primoz Flander 4.5.2019

v12: added NRF24

v11: added MQ-135 gas sensor

v9: added rotary encoder, optimized program

v8: added bugzapper auto on
           
v7: summer time update
    added PIR startup calibration delay

Board: Arduino Nano + breakout board
Arduino pin   /      I/O:
DI2          ->      DT encoder
DI3          ->      CLK encoder
DI4          ->      SW encoder
DI5          ->      PIR
DI6          ->      Light sensor
DI7          ->      DHT sensor
DO8          ->      RF transmitter
DI9          ->      NRF24 CE
DI10         ->      NRF24 CS
DI11         ->      NRF24 MOSI
DI12         ->      NRF24 MISO
DI13         ->      NRF24 SCK
A0           ->      MQ-135 gas sensor
A4           ->      SPI rtc, SPI lcd
A5           ->      SPI rtc, SPI lcd

*/

/*=======================================================================================
                                    Includes
========================================================================================*/
//#define MY_DEBUG
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_DATARATE (RF24_250KBPS)
#include <SPI.h>
#include <MySensors.h> 
#include <RCSwitch.h>
#include <DS3231.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Rotary.h>
#include <EEPROM.h>
#include <math.h>

/*=======================================================================================
                                    Definitions
========================================================================================*/

#define rotSwitch                      4
#define AIRQUALITY_SENSOR_ANALOG_PIN   A0
#define PIR                            5
#define lightSensor                    6
#define DHT_DATA_PIN                   7
#define led                            13
#define menuItemsNr                    12
#define CHILD_ID_HUMS                  0
#define CHILD_ID_TEMP                  1
#define CHILD_ID_MQ                    2
#define CHILD_ID_MS                    3
#define CHILD_ID_LS                    4

/*=======================================================================================
                                User Configurarations
========================================================================================*/

MyMessage msg0(CHILD_ID_HUMS, V_HUM);
MyMessage msg1(CHILD_ID_TEMP, V_TEMP);
MyMessage msg2(CHILD_ID_MQ, V_LEVEL);
MyMessage msg3(CHILD_ID_MS, V_TRIPPED);
MyMessage msg4(CHILD_ID_LS, V_STATUS);

DHT dht;
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
RCSwitch mySwitch = RCSwitch();
DS3231  rtc(SDA, SCL);
Rotary r = Rotary(2, 3);
Time  t;                      //init a Time-data structure
uint32_t delayMS;
bool S_light = false;         //light status
bool S_weekend = false;
bool lcdShow = true;
bool dateAqFlag = true;
bool S_PIR = false;
bool S_lightS = false;
bool S_lastPIR = false;
bool S_lastLightS = false; 
int addr = 0;
int menuVal=0;
int menuLvl=1;
int lightTime = 0;
int pirState;             // the current reading from the input pin
int lastPirState = 0;   // the previous reading from the input pin
int lastButtonState = 0;   // the previous reading from the input pin
int buttonState;             // the current reading from the input pin
int airQuality = 0;
int lastAirQuality = 1;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
unsigned long nLoop = 0;
unsigned long debugLoop = 0;
float humidity;
float temperature;
float lastTemp = 1;
float lastHum = 1;
String menuItems[menuItemsNr] = {"<-Nazaj", "Kal. ura", "Kal. min", "Kal. temp", "Kal. vlaz.", "PIR luc zak.", "Vklop luci (h)", "Vklop luci (min)", "Izklop luci(h)", "Izklop luci(min)", "Cas izklopa lcd", "Cas vklopa lcd"};  // max 16 char
String unitItems[menuItemsNr] = {"<-Nazaj", "h", "min", "C", "%", "sekund", "h", "min", "h", "min", "h", "h"};
int valueItems[menuItemsNr];

void presentation()  
{ 
  
  sendSketchInfo("SmartBox", "1.0");
  present(CHILD_ID_HUMS, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_MQ, S_AIR_QUALITY);
  present(CHILD_ID_MS, S_MOTION);
  present(CHILD_ID_LS, S_BINARY);

}
/*=======================================================================================
                                   Setup function
========================================================================================*/

void setup()  {
  Serial.begin(115200);
  rtc.begin();
  dht.setup(DHT_DATA_PIN);
  sleep(dht.getMinimumSamplingPeriod());
  lcd.begin(16,2);         // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.clear();
  mySwitch.enableTransmit(8);
  pinMode(rotSwitch, INPUT_PULLUP);
  pinMode(lightSensor,INPUT);
  pinMode(PIR,INPUT);          
  pinMode(led, OUTPUT);        
  mySwitch.send("110111001000111001000100"); //shutdown light
  startupText();
  for (int i=0; i < menuItemsNr; i++){
    valueItems[i] = (int8_t) EEPROM.read(addr+i);
  }
  //interrupts
  PCICR |= (1 << PCIE2);
  PCMSK2 |= (1 << PCINT18) | (1 << PCINT19);
  sei();
}

/*=======================================================================================
                                            Loop
========================================================================================*/

void loop(){

  lightControl();
  loopTiming();
  //serialDebug();
  //isWeekend();
  readSw();
  timers();
  if (lcdShow == true) {
    showMenu();
    lcdShow = false;
  }
  send2Domoticz();
}

/*=======================================================================================
                                         Functions
========================================================================================*/

ISR(PCINT2_vect) {
  unsigned char result = r.process();
  if (result == DIR_NONE) {
  }
  else if (result == DIR_CW) {
    Serial.println("ClockWise");  
    if (menuLvl == 0) {
      menuLvl = 1;
    }
    else if (menuLvl == 1)  {
      menuVal--;
      if (menuVal < 0)  {
        menuVal = (menuItemsNr-1);
      }
    }
    else if (menuLvl == 2) {
      decVal();
    }
    lcdShow = true;   
  }
  
  else if (result == DIR_CCW) {
    Serial.println("CClockWise"); 
    if (menuLvl == 0) {
      menuLvl = 1;
    }
    else if (menuLvl == 1)  {
      menuVal++;
      if (menuVal > (menuItemsNr-1))  {
        menuVal = 0;
      }
    }
     else if (menuLvl == 2) {
      incVal();
    }
    lcdShow = true;
  }
}

void lightControl(void) {
  S_PIR = digitalRead(PIR);
  S_lightS = digitalRead(lightSensor);
  if (S_PIR == true) {
    if ((S_light == false) && (S_lightS == true))  {
     mySwitch.send("110111001000111001001100");
     S_light = true;
     Serial.println("L on");
    }  
  }
  else if (S_light == true) {
   mySwitch.send("110111001000111001000100");
   S_light = false;
   Serial.println("L off");
  }
}

void isWeekend(void) {

  if (t.dow >= 5 && t.hour > 8 && digitalRead(PIR) == true) {
    S_weekend = true;
    lcd.setCursor(15,0);
    lcd.print("*");
  }
  else if (t.dow < 5 && t.hour == 4)  {
    S_weekend = false;
    lcd.setCursor(15,0);
    lcd.print(" ");
  }
}

void timers(void) {

  //turn on light
  if ((t.dow < 6) && (t.hour == valueItems[6]) && (t.min == valueItems[7]) && (S_light == false))  { 
    mySwitch.send("110111001000111001001100");
    S_light = true;
  }
  //turn off light
  if ((t.dow < 6) && (t.hour == valueItems[8]) && (t.min == valueItems[9]) && (S_light == true))  { 
    mySwitch.send("110111001000111001000100");
    S_light = false;
  }
    
  //turn off lcd backlight
  if (t.hour == valueItems[10] && t.min == 0 && t.sec <= 5)  { 
    lcd.noBacklight();
  }
  //turn on lcd backlight
  if (t.hour == valueItems[11] && t.min == 40 && t.sec <= 5) { 
    lcd.backlight();
  }
}

void startupText(void) {
  
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("Smart Box v11");
  lcd.setCursor(0,1);
  lcd.print("Starting up");
  delay(1000);
  lcd.print(".");
  delay(1000);
  lcd.print(".");
  delay(1000);
  lcd.print(".");
  delay(1000);
  lcd.clear();
}

void readSw() {
  
  int reading = digitalRead(rotSwitch);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
    }
    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (reading != buttonState) {
        buttonState = reading; 
        if (buttonState == HIGH) {      
          switch (menuLvl) { 
            case 0:
              menuLvl = 1;
            break;
            
            case 1:
              if (menuVal == 0) {
                menuLvl = 0;
              }
              else  {
                menuLvl = 2;
              }
            break;     
            case 2:
              if (menuVal == 1)  {
                t = rtc.getTime();
                rtc.setTime(valueItems[1], t.min, 0);
              }
              else if (menuVal == 2) {
                t = rtc.getTime();
                rtc.setTime(t.hour, valueItems[2], 0);          
              }
              menuLvl = 1;
            break;
            default:
              menuLvl = 0;  // SET DEFAULT SCREEN
            break;
          }
          lcdShow = true;
        }
      }
    }
    lastButtonState = reading;
}

void incVal() {

  valueItems[menuVal] ++;
  if ((menuVal == 1) || (menuVal == 6) || (menuVal == 8) || (menuVal == 8) || (menuVal == 10) || (menuVal == 11)) {
    if (valueItems[menuVal] > 23) {
    valueItems[menuVal] = 23;
    } 
  }
  else if ((menuVal == 2) || (menuVal == 5) || (menuVal == 7) || (menuVal == 9))  {
    if (valueItems[menuVal] > 59) {
    valueItems[menuVal] = 59;
    } 
  }
  else if ((menuVal == 3) || (menuVal == 4))  {
    if (valueItems[menuVal] > 10) {
    valueItems[menuVal] = 10;
    } 
  }
  else if (valueItems[menuVal] > 127) {
    valueItems[menuVal] = 127;
  }
  EEPROM.update(addr + menuVal, valueItems[menuVal]);
}

void decVal() {
  
  valueItems[menuVal] --;
  if ((menuVal == 3) || (menuVal == 4))  {
    if (valueItems[menuVal] < -10) {
    valueItems[menuVal] = -10;
    } 
  }
  else if (valueItems[menuVal] < 0) {
    valueItems[menuVal] = 0;
  }
  EEPROM.update(addr + menuVal, valueItems[menuVal]);
}

void showMenu() {

  if (menuLvl == 0)  {
    displayMainMenu();
  }
  else  {
    displayMenuItem();
  }
}


void loopTiming() {
  
   nLoop++;
   delay(1);
   if (nLoop > 5000) {
      nLoop = 0;  //reset loop timer  
      if (menuLvl == 0) {
        lcdShow = true;
      }
   }
}

void displayMenuItem()  {
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(menuItems[menuVal]);
  if (menuVal != 0) {
    lcd.setCursor(0, 1);
    lcd.print(valueItems[menuVal]);
    lcd.print(" ");
    lcd.print(unitItems[menuVal]);
  }
}

void displayMainMenu()  {

  dht.readSensor(true);
  temperature = dht.getTemperature();  
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("T:");
  lcd.print(temperature + valueItems[3]);
  lcd.setCursor(6,0);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("H:");
  humidity = dht.getHumidity();
  lcd.print(humidity + valueItems[4]);
  lcd.setCursor(6,1);
  lcd.print("%");

  t = rtc.getTime();   // Get data from the DS3231
  lcd.setCursor(9,0);
  if (dateAqFlag) {
    lcd.print("Q:");
    airQuality = analogRead(AIRQUALITY_SENSOR_ANALOG_PIN);
    lcd.print(airQuality);
    lcd.print("p");
  }
  else  {
    lcd.print(rtc.getDateStr());
    lcd.print("  ");
  }
  dateAqFlag = !dateAqFlag;
  lcd.setCursor(15,0);
  lcd.print(" ");
  lcd.setCursor(9,1);
  lcd.print(rtc.getTimeStr());
  lcd.setCursor(14,1);
  lcd.print("  ");
}

void serialDebug() {

  debugLoop++;
  if  (debugLoop > 1000)  {
    Serial.println("-----------------------------");
    Serial.print("LightSensor:");
    Serial.println(digitalRead(lightSensor));
    Serial.print("PIR:");
    Serial.println(digitalRead(PIR));
    Serial.print("pirState:");
    Serial.println(pirState);
    Serial.print("lastPirState:");
    Serial.println(lastPirState);
    Serial.print("lightTime:");
    Serial.println(lightTime);
    Serial.print("S_light:");
    Serial.println(S_light);
    debugLoop = 0;
  } 
}

void send2Domoticz(void)  {

    if (humidity != lastHum) {
      send(msg0.set(humidity,1));
      lastHum = humidity;
    }
    if (temperature != lastTemp) {
      send(msg1.set(temperature,1));
      lastTemp = temperature;
    }
    if (airQuality != lastAirQuality) {
      send(msg2.set(airQuality));
      lastAirQuality = airQuality;
    }
    if (S_PIR != S_lastPIR) {
      send(msg3.set(S_PIR));
      S_lastPIR = S_PIR;
    }
    if (S_lightS != S_lastLightS) {
      send(msg4.set(S_lightS));
      S_lastLightS = S_lightS;
    }
}


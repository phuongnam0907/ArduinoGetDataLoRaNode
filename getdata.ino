/***************************************************
INCLUDE LIBRARIES AND DEFINE VARIABLES IN EACH FUNCTION
****************************************************/
//  SW1 - A0: pH Sensor                   https://www.dfrobot.com/wiki/index.php/PH_meter(SKU:_SEN0161)
//  SW2 - A1: High Temperature Sensor     https://www.dfrobot.com/wiki/index.php/HighTemperatureSensor_SKU:SEN0198
//  SW3 - A2: Liquid Level Sensor         https://www.dfrobot.com/wiki/index.php/Liquid_Level_Sensor-FS-IR02_SKU:_SEN0205
//  SW4 - A3: Dissolved Oxygen Sensor     https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_Dissolved_Oxygen_Sensor_SKU:SEN0237
//  SW5 - A4: Total Dissolved Solids      https://www.dfrobot.com/wiki/index.php/Gravity:_Analog_TDS_Sensor_/_Meter_For_Arduino_SKU:_SEN0244
//  SW6 - A5: ...

uint8_t dataSensor[15];

//================= pH sensor ===================
#define SensorPin A0            //pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00            //deviation compensate
#define samplingInterval 20
#define printInterval 800
#define ArrayLenth  40    //times of collection
int pHArray[ArrayLenth];   //Store the average value of the sensor feedback
int pHArrayIndex=0;    

//=========== high temperature sensor ===========
#include "DFRobotHighTemperatureSensor.h"
#define SensorTemperaturePin A1
const float voltageRef = 5.000; //Set reference voltage,you need test your IOREF voltage. 
//const float voltageRef = 3.300; 
int HighTemperaturePin = A1;  //Setting pin
DFRobotHighTemperature PT100 = DFRobotHighTemperature(voltageRef); //Define an PT100 object

//============== liquid level sensor ============
#define SensorLiquidLevelPin A2
int Liquid_level=0;

//================= oxygen sensor ===============
#include <avr/pgmspace.h>
#include <EEPROM.h>

#define DoSensorPin A3    //dissolved oxygen sensor analog output pin to arduino mainboard
#define VREF 5000    //for arduino uno, the ADC reference is the AVCC, that is 5000mV(TYP)
float doValue;      //current dissolved oxygen value, unit; mg/L
float temperatureDO = 25;    //default temperature is 25^C, you can use a temperature sensor to read it

#define EEPROM_write(address, p) {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) EEPROM.write(address+i, pp[i]);}
#define EEPROM_read(address, p)  {int i = 0; byte *pp = (byte*)&(p);for(; i < sizeof(p); i++) pp[i]=EEPROM.read(address+i);}

#define ReceivedBufferLength 20
char receivedBuffer[ReceivedBufferLength+1];    // store the serial command
byte receivedBufferIndex = 0;

#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    //store the analog value in the array, readed from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;

#define SaturationDoVoltageAddress 12          //the address of the Saturation Oxygen voltage stored in the EEPROM
#define SaturationDoTemperatureAddress 16      //the address of the Saturation Oxygen temperature stored in the EEPROM
float SaturationDoVoltage,SaturationDoTemperature;
float averageVoltage;

const float SaturationValueTab[41] PROGMEM = {      //saturation dissolved oxygen concentrations at various temperatures
14.46, 14.22, 13.82, 13.44, 13.09,
12.74, 12.42, 12.11, 11.81, 11.53,
11.26, 11.01, 10.77, 10.53, 10.30,
10.08, 9.86,  9.66,  9.46,  9.27,
9.08,  8.90,  8.73,  8.57,  8.41,
8.25,  8.11,  7.96,  7.82,  7.69,
7.56,  7.43,  7.30,  7.18,  7.07,
6.95,  6.84,  6.73,  6.63,  6.53,
6.41,
};

//================= solids sensor ===============
#define TdsSensorPin A4

#include <EEPROM.h>
#include "GravityTDS.h"

GravityTDS gravityTds;

float temperatureTDS = 25,tdsValue = 0;


/***************************************************
                      MAIN
****************************************************/
void setup() {
  // put your setup code here, to run once:
  init_start();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(getdata());
  getdata();
  delay(1000);
}

/***************************************************
                SETUP FOR SENSOR
****************************************************/
void init_start(){
  Serial.begin(9600);
  init_ph();
  init_temp();
  init_liq();
  init_oxy();
}

//================= pH sensor ===================
void init_ph(){
  pinMode(SensorPin,INPUT);
}
//=========== high temperature sensor ===========
void init_temp(){
  //do nothing
}
//============== liquid level sensor ============
void init_liq(){
  pinMode(SensorLiquidLevelPin,INPUT);
}
//================= oxygen sensor ===============
void init_oxy(){
  pinMode(DoSensorPin,INPUT);
}
//================= solids sensor ===============
void init_solids(){
  gravityTds.setPin(TdsSensorPin);
  gravityTds.setAref(5.0);  //reference voltage on ADC, default 5.0V on Arduino UNO
  gravityTds.setAdcRange(1024);  //1024 for 10bit ADC;4096 for 12bit ADC
  gravityTds.begin();  //initialization
}
/***************************************************
                FUNCTION GET DATA
****************************************************/
void getdata(){

  float pHvalue;
  int tempvalue;
  int liqvalue;

  getdata_ph();
  getdata_temp();
  getdata_liq();
  getdata_oxy();
  getdata_solids();

  //Create data of sensor then import into array dataSensor
  dataSensor[0] = 0xFF;   //Which port of analog sensor doesn't work, 1111 1111 is all working
  dataSensor[13] = 0x00;  //Number of analog sensor is not working
  dataSensor[14] = 0;     //Or '\0' is the end of data sensor
}

//================= pH sensor ===================
void getdata_ph(){
  static unsigned long samplingTime = millis();
  static unsigned long printTime = millis();
  static float pHValue,voltage;
  if(millis()-samplingTime > samplingInterval)
  {
    pHArray[pHArrayIndex++]=analogRead(SensorPin);
    if(pHArrayIndex==ArrayLenth)pHArrayIndex=0;
    voltage = avergearray(pHArray, ArrayLenth)*5.0/1024;
    pHValue = 3.5*voltage+Offset;
    samplingTime=millis();
  }
  if(millis() - printTime > printInterval)   //Every 800 milliseconds, print a numerical, convert the state of the LED indicator
  {
    Serial.print("Voltage:");
    Serial.print(voltage,2);
    Serial.print("    pH value: ");
    Serial.println(pHValue,2);
    printTime=millis();
  }
}
//=========== high temperature sensor ===========
void getdata_temp(){
  int temperature = PT100.readTemperature(HighTemperaturePin);  //Get temperature
  Serial.print("temperature:  ");
  Serial.print(temperature);
  Serial.println("  ^C");
  delay(1000); //just here to slow down the output so it is easier to read
}
//============== liquid level sensor ============
void getdata_liq(){
  Liquid_level=digitalRead(5);
  Serial.print("Liquid_level= ");
  Serial.println(Liquid_level,DEC);
  delay(500);
}
//================= oxygen sensor ===============
void getdata_oxy(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 30U)     //every 30 milliseconds,read the analog value from the ADC
  {
   analogSampleTimepoint = millis();
   analogBuffer[analogBufferIndex] = analogRead(DoSensorPin);    //read the analog value and store into the buffer
   analogBufferIndex++;
   if(analogBufferIndex == SCOUNT) 
       analogBufferIndex = 0;
  }
  
  static unsigned long tempSampleTimepoint = millis();
  if(millis()-tempSampleTimepoint > 500U)  // every 500 milliseconds, read the temperature
  {
    tempSampleTimepoint = millis();
    //temperature = readTemperature();  // add your temperature codes here to read the temperature, unit:^C
  }
  
  static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 1000U)
  {
    printTimepoint = millis();
    for(copyIndex=0;copyIndex<SCOUNT;copyIndex++)
    {
      analogBufferTemp[copyIndex]= analogBuffer[copyIndex];
    }
    averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 1024.0; // read the value more stable by the median filtering algorithm
    Serial.print(F("Temperature:"));
    Serial.print(temperatureDO,1);
    Serial.print(F("^C"));
    doValue = pgm_read_float_near( &SaturationValueTab[0] + (int)(SaturationDoTemperature+0.5) ) * averageVoltage / SaturationDoVoltage;  //calculate the do value, doValue = Voltage / SaturationDoVoltage * SaturationDoValue(with temperature compensation)
    Serial.print(F(",  DO Value:"));
    Serial.print(doValue,2);
    Serial.println(F("mg/L"));
  }
  
  if(serialDataAvailable() > 0)
  {
    byte modeIndex = uartParse();  //parse the uart command received
    doCalibration(modeIndex);    // If the correct calibration command is received, the calibration function should be called.
  }
}
//================= solids sensor ===============
void getdata_solids(){
  gravityTds.setTemperature(temperatureTDS);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate 
  tdsValue = gravityTds.getTdsValue();  // then get the value
  Serial.print(tdsValue,0);
  Serial.println("ppm");
  delay(1000);
}
/***************************************************
                    SUB-FUNCTION
****************************************************/
double avergearray(int* arr, int number){
  int i;
  int max,min;
  double avg;
  long amount=0;
  if(number<=0){
    Serial.println("Error number for the array to avraging!/n");
    return 0;
  }
  if(number<5){   //less than 5, calculated directly statistics
    for(i=0;i<number;i++){
      amount+=arr[i];
    }
    avg = amount/number;
    return avg;
  }else{
    if(arr[0]<arr[1]){
      min = arr[0];max=arr[1];
    }
    else{
      min=arr[1];max=arr[0];
    }
    for(i=2;i<number;i++){
      if(arr[i]<min){
        amount+=min;        //arr<min
        min=arr[i];
      }else {
        if(arr[i]>max){
          amount+=max;    //arr>max
          max=arr[i];
        }else{
          amount+=arr[i]; //min<=arr<=max
        }
      }//if
    }//for
    avg = (double)amount/(number-2);
  }//if
  return avg;
}
//===============================================//
boolean serialDataAvailable(void)
{
  char receivedChar;
  static unsigned long receivedTimeOut = millis();
  while ( Serial.available() > 0 ) 
  {   
    if (millis() - receivedTimeOut > 500U) 
    {
      receivedBufferIndex = 0;
      memset(receivedBuffer,0,(ReceivedBufferLength+1));
    }
    receivedTimeOut = millis();
    receivedChar = Serial.read();
    if (receivedChar == '\n' || receivedBufferIndex == ReceivedBufferLength)
    {
  receivedBufferIndex = 0;
  strupr(receivedBuffer);
  return true;
    }else{
        receivedBuffer[receivedBufferIndex] = receivedChar;
        receivedBufferIndex++;
    }
  }
  return false;
}
//===============================================//
byte uartParse()
{
    byte modeIndex = 0;
    if(strstr(receivedBuffer, "CALIBRATION") != NULL) 
        modeIndex = 1;
    else if(strstr(receivedBuffer, "EXIT") != NULL) 
        modeIndex = 3;
    else if(strstr(receivedBuffer, "SATCAL") != NULL)   
        modeIndex = 2;
    return modeIndex;
}
//===============================================//
void doCalibration(byte mode)
{
    char *receivedBufferPtr;
    static boolean doCalibrationFinishFlag = 0,enterCalibrationFlag = 0;
    float voltageValueStore;
    switch(mode)
    {
      case 0:
      if(enterCalibrationFlag)
         Serial.println(F("Command Error"));
      break;
      
      case 1:
      enterCalibrationFlag = 1;
      doCalibrationFinishFlag = 0;
      Serial.println();
      Serial.println(F(">>>Enter Calibration Mode<<<"));
      Serial.println(F(">>>Please put the probe into the saturation oxygen water! <<<"));
      Serial.println();
      break;
     
     case 2:
      if(enterCalibrationFlag)
      {
         Serial.println();
         Serial.println(F(">>>Saturation Calibration Finish!<<<"));
         Serial.println();
         EEPROM_write(SaturationDoVoltageAddress, averageVoltage);
         EEPROM_write(SaturationDoTemperatureAddress, temperatureDO);
         SaturationDoVoltage = averageVoltage;
         SaturationDoTemperature = temperatureDO;
         doCalibrationFinishFlag = 1;
      }
      break;

        case 3:
        if(enterCalibrationFlag)
        {
            Serial.println();
            if(doCalibrationFinishFlag)      
               Serial.print(F(">>>Calibration Successful"));
            else 
              Serial.print(F(">>>Calibration Failed"));       
            Serial.println(F(",Exit Calibration Mode<<<"));
            Serial.println();
            doCalibrationFinishFlag = 0;
            enterCalibrationFlag = 0;
        }
        break;
    }
}
//===============================================//
int getMedianNum(int bArray[], int iFilterLen) 
{
      int bTab[iFilterLen];
      for (byte i = 0; i<iFilterLen; i++)
      {
    bTab[i] = bArray[i];
      }
      int i, j, bTemp;
      for (j = 0; j < iFilterLen - 1; j++) 
      {
    for (i = 0; i < iFilterLen - j - 1; i++) 
          {
      if (bTab[i] > bTab[i + 1]) 
            {
    bTemp = bTab[i];
          bTab[i] = bTab[i + 1];
    bTab[i + 1] = bTemp;
       }
    }
      }
      if ((iFilterLen & 1) > 0)
  bTemp = bTab[(iFilterLen - 1) / 2];
      else
  bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
      return bTemp;
}
//===============================================//
void readDoCharacteristicValues(void)
{
    EEPROM_read(SaturationDoVoltageAddress, SaturationDoVoltage);  
    EEPROM_read(SaturationDoTemperatureAddress, SaturationDoTemperature);
    if(EEPROM.read(SaturationDoVoltageAddress)==0xFF && EEPROM.read(SaturationDoVoltageAddress+1)==0xFF && EEPROM.read(SaturationDoVoltageAddress+2)==0xFF && EEPROM.read(SaturationDoVoltageAddress+3)==0xFF)
    {
      SaturationDoVoltage = 1127.6;   //default voltage:1127.6mv
      EEPROM_write(SaturationDoVoltageAddress, SaturationDoVoltage);
    }
    if(EEPROM.read(SaturationDoTemperatureAddress)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+1)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+2)==0xFF && EEPROM.read(SaturationDoTemperatureAddress+3)==0xFF)
    {
      SaturationDoTemperature = 25.0;   //default temperature is 25^C
      EEPROM_write(SaturationDoTemperatureAddress, SaturationDoTemperature);
    }    
}

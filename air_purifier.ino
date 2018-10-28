#include <string.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <Wire.h>

/**** 핀 안내 ****/
/*
 * *************** 미세먼지 센서 선 연결 방법(왼쪽부터) ********************
 * { 1: 쓸데없음(우리가 자른 선), 2(PM10): 디지털 12번 핀, 3: 5V 핀, 4(PM25): 사용 안함(만약 측정값이 이상한 경우 2번 선 대신 이 선을 디지털 12번에 꽂아도 좋음), 5: GND 핀 }
 * ************** CO2 NDIR 센서 연결 방법(기판 오른쪽 핀 안내 참조) ******************
 *  { T: 아날로그 4번 핀, R: 아날로그 5번 핀, V+: 5V 핀, V-: GND 핀 }
 *  ************* CO2 화학식 센서 연결 방법 *******************
 *  { 빨간 선: 5V 핀, 검은 선: GND 핀, 파란 선: 아날로그 1번 핀 }
 *  ************* 온습도 센서 연결 방법 ***************
 *  { 빨간 선: 5V 핀, 검은 선: GND 핀, 고동색 선: 디지털 11번 핀 }
 *  
 *  ************* LCD 쉴드 **********
 *  LCD 쉴드에 연결되어 있는 선을 아두이노위에 얹는다고 가정할 때 정확히 일치하는 곳에 연결하면 됨.
 *  
 *  ************* 절대 건드리지 말아야 할 핀!! ********
 *  아날로그 3번 핀 - 모터 쉴드용
 *  디지털 2번 핀 - 모터 쉴드용
 */
#define DHTPIN 11 // DHT sensor pin(Digital 11)
#define         MG_PIN                       (A1)     //define which analog input channel you are going to use for co2 chemical sensor
SoftwareSerial mySerial(A4, A5); // NDIR SENSOR PINS(TX, RX)
#define MD_PIN 12 //Microdust sensor pin(Digital 12)



/******************************** Humidity and Temperature sensor *****************************/

#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);
int temp = 0;
int humid = 0;

/********************* CO2 Sensor (Chemical Way) **********************/
/************************Hardware Related Macros************************************/
#define         BOOL_PIN                     (13)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier

/***********************Software Related Macros************************************/
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
//These two values differ from sensor to sensor. user should derermine this value.
#define         ZERO_POINT_VOLTAGE           (0.258) // CO2 센서 보정 시 입력해야 하는 부분(자세한 내용은 아래 참조)
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

/*****************************Globals***********************************************/
float           CO2Curve[3]  =  {2.602, ZERO_POINT_VOLTAGE, (REACTION_VOLTGAE / (2.602 - 3))};
int percentage;
float volts;

/********************** CO2 Sensor (Chemical Way) END **********************/

/*********************************** CO2 SENSOR(NDIR) WAY) ******************************/
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
char response[9]; 
String ppmString = " ";


/******************************** MICRODUST SENSOR **************************************/
unsigned long duration;  //지속시간
unsigned long starttime; //시작시간
unsigned long curtime;   //종료시간
unsigned long sampletime_ms = 30000; // 30초 = 먼지 측정 주기(간격)-조정하세요 ; (1초=1000으로 설정)
unsigned long lowpulseoccupancy = 0; //지속시간 누적
float ratio = 0;
float concentration = 0; // using spec sheet curve
float pcsPerCF = 0;  //입자당 CF를 0으로 초기화
float ugm3 = 0;  //세제곱미터(큐빅 미터)당 마이크로 그램(㎍/㎥)
int i = 0;
int dht_sw = 0;
int air_sw = 0;
// Motor Shield
int pwmPin = 3; // Analog Pin 3
int dirPin = 2; //Digital Pin 2

static int iSpeed = 0;
static int iAcc = 5;


/**** DELAYED LOOP SETTINGS ****/
#define LCD_UPDATE_TIMES 250
#define CO2_NDIR_UPDATE_TIMES 5000
#define CO2_CHEM_UPDATE_TIMES 500
#define DHT_UPDATE_TIMES 1000
int last_lcd_update = 0;
int last_co2_ndir_update = 0;
int last_co2_chem_update = 0;
int last_dht_update = 0;

/**** Serial Connection Input Settings ****/
#define INPUT_SIZE 30

void setup() {
  Serial.begin(9600);
  /**** Motor Reset ****/
  analogWrite(pwmPin, 0);
  digitalWrite(dirPin, LOW);
  /**** CO2 Sensor(Chem) Register ****/
  pinMode(BOOL_PIN, INPUT);                        //set pin to input
  digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors
  /**** CO2 Sensor(NDIR) Register ****/
  mySerial.begin(9600);
  /**** Microdust Sensor Register ****/
  pinMode(3, INPUT);
  starttime = millis();
  last_lcd_update = millis() - LCD_UPDATE_TIMES;
  last_co2_ndir_update = millis() - CO2_NDIR_UPDATE_TIMES;
  last_co2_chem_update = millis() - CO2_CHEM_UPDATE_TIMES;
  last_dht_update = millis() - DHT_UPDATE_TIMES;
  /**** LCD Register ****/
  Serial.println("ready");
}

void loop() {
  /** Collect Microdust samples and control motor by settings ***/
  collectSamples();
  /*
   * TIMED LOOPS
   */
  if((millis() - last_lcd_update) >= LCD_UPDATE_TIMES) {
    if (iSpeed >= 0) {
      analogWrite(pwmPin, iSpeed);
      digitalWrite(dirPin, LOW);
    } else {
      analogWrite(pwmPin, -iSpeed);
      digitalWrite(dirPin, HIGH);
    }
  }
  if((millis() - last_co2_chem_update) >= CO2_CHEM_UPDATE_TIMES) {
    volts = MGRead(MG_PIN);
    percentage = MGGetPercentage(volts, CO2Curve);
    //Serial.print("Current Chem Sensor Volt:");
    //Serial.print(volts);
    //Serial.print("V");
    //Serial.print("\n");
    last_co2_chem_update = millis();
  }
  if((millis() - last_co2_ndir_update) >= CO2_NDIR_UPDATE_TIMES) {
    mySerial.write(cmd, 9);
    mySerial.readBytes(response, 9);
    int responseHigh = (int) response[2];
    int responseLow = (int) response[3];
    int ppm = (256*responseHigh)+responseLow;
    ppmString = String(ppm);
    /*
    for(int x=0;x<9;x++)
    {
      Serial.print((int)response[x], HEX);
      Serial.print(" ");
    }
    Serial.print("\n");
    //Serial.println((int)response, HEX);
    
    Serial.println("High:" + (String)responseHigh + " Low:" + (String)responseLow);
    Serial.print("PPM ");
    Serial.println(ppm);
    Serial.println("-----------------------------------");
    */
  }
  if((millis() - last_dht_update) >= DHT_UPDATE_TIMES) {
    temp = dht.readTemperature();
    humid = dht.readHumidity();
    /* Send Infomations */
    Serial.print(getNDIRPPM());
    Serial.print(" ");
    Serial.print(getChemPPM());
    Serial.print(" ");
    Serial.print(getDustInfo());
    Serial.print(" ");
    Serial.print(temp);
    Serial.print(" ");
    Serial.print(humid);
    Serial.print("\n");
  }
  /*
   * SERIAL CONNECTION
   */
   while(Serial.available() > 0) {
    String command = Serial.readStringUntil(' ');
    String data = Serial.readStringUntil('\n');
    /*
    Serial.print(command);
    Serial.print(" ");
    Serial.print(data);
    Serial.print("\n");
    */
    if(command == "setmotorspeed") {
      int serialSpeed = data.toInt();
      if(serialSpeed > 255 || serialSpeed < 0) {
        Serial.println("false"); 
      } else {
        iSpeed = serialSpeed;
        Serial.println("true");
      }
    }
   }
}
/**** MOTOR RELATED FUNCTIONS ****/

void controlMotor(int level) {
  iSpeed += level;
  if(iSpeed >= 255) {
    iSpeed = 255;
  } else if(iSpeed < 0) {
    iSpeed = 0;
  }
}


/**** CO2 SENSOR(CHEM) RELATED FUNCTIONS ****/

String getChemPPM() {
  if (percentage == -1) {
    return "<400";
  } else {
    return String(percentage);
  }
}

/*****************************  MGRead *********************************************
  Input:   mg_pin - analog channel
  Output:  output of SEN-000007
  Remarks: This function reads the output of SEN-000007
************************************************************************************/
float MGRead(int mg_pin)
{
  int i;
  float v = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    v += analogRead(mg_pin);
    delay(READ_SAMPLE_INTERVAL);
  }
  v = (v / READ_SAMPLE_TIMES) * 5 / 1024 ;
  return v;
}

/*****************************  MQGetPercentage **********************************
  Input:   volts   - SEN-000007 output measured in volts
         pcurve  - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(MG-811 output) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MGGetPercentage(float volts, float *pcurve)
{
  if ((volts / DC_GAIN ) >= ZERO_POINT_VOLTAGE) {
    return -1;
  } else {
    return pow(10, ((volts / DC_GAIN) - pcurve[1]) / pcurve[2] + pcurve[0]);
  }
}

/**** CO2 SENSOR(NDIR) RELATED FUNCTIONS ****/
String getNDIRPPM() {
  return ppmString;
}

/************************************ MICRODUST SENSOR RELATED FUNCTIONS ****************************/

void collectSamples() {
  duration = pulseIn(MD_PIN, LOW);
  lowpulseoccupancy = lowpulseoccupancy + duration;
  curtime = millis(); //아두이노 내부, 현재 시간;
  if ((curtime - starttime) >= sampletime_ms) //대기시간 > = 30s
  {
    ratio = lowpulseoccupancy / (sampletime_ms * 10.0);  // // 비율 정수값 0=>100
// concentration = 미세먼지 센서 제조사에서 출력하는 값
    concentration = 1.1 * pow(ratio, 3) - 3.8 * pow(ratio, 2) + 520 * ratio + 0.62; // using spec sheet curve
    pcsPerCF = concentration * 100;  // 입자 농도에 100을 곱하면 입자당 CF값
// org    ugm3 = pcsPerCF / 13000;  //입자당 CF를 나누면 세제곱미터당 마이크로그램 미세먼지 값
    ugm3 = pcsPerCF / 1300;  //입자당 CF를 1300으로 미세먼지 값

    // Serial.print("duration:");
    // Serial.print(duration);
    // Serial.print(" / ");
    // Serial.print("ugm3:");
    // Serial.print(ugm3);
    // Serial.print(" / ");
    // Serial.print("lowpulseoccupancy:");
    // Serial.print(lowpulseoccupancy);
    // Serial.print(" / ");
    // Serial.print("ratio:");
    // Serial.print(ratio);
    // Serial.print(" / ");
    // Serial.print("concentration:");
    // Serial.print(concentration);
    // Serial.print(" / ");
    // Serial.print(" pcs/0.01cf:");
    // Serial.println(pcsPerCF);
    // Serial.println("\n");
    // Serial.print("f-concentration:");
    // Serial.println(float(concentration/500));
    lowpulseoccupancy = 0;
    starttime = millis();
  }
}

String getDustInfo() {
  if(concentration == 0) {
    return "Waiting...";
  } else {
    return String(ugm3);
  }
}




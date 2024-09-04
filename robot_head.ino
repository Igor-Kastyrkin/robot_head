

//#define MY_FILTER


//#define MYSDFAT

#ifdef MYSDFAT

#include "SdFat.h"
// Setting ENABLE_DEDICATED_SPI to zero saves over 200 more bytes.
#if ENABLE_DEDICATED_SPI
#warning "Set ENABLE_DEDICATED_SPI zero in SdFat/src/SdFatConfig.h for minimum size"
#endif  // ENABLE_DEDICATED_SPI
// Insure FAT16/FAT32 only.
SdFat32 SD;

// FatFile does not support Stream functions, just simple read/write.
FatFile dataFile;
#else
#include <SD.h>
File dataFile;
#endif

//#include "serial.h"

//#define OTLADKA

//#define MICROUART

#ifdef MICROUART
//#define MU_PRINT
//#define MU_STREAM
#include <MicroUART.h>
MicroUART uart;
#endif

#include "Adafruit_PWMServoDriver2.h"
#ifdef MY_FILTER
#include "Kalman.h"
#endif
#include "FlexiTimer2.h"
#include "ServoDriverSmooth.h"

#define GYVER

#define pwmAddr 0x40

#ifndef GYVER
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
#endif

#define SERVO_FREQ 50


#define SDchipSelect 9

#define ServoSpeed 100
#define ServoAccel 0.4
#define HeadSpeed 100
#define HeadAccel 0.4

//File dataFile;//, playFile;

void commandHandle(char *RF_data);
void readFromMemory(void);

//unsigned long StartRecordTime = 0;
//unsigned long data_rate = 100;  //millis

//byte sizeOfGlbData = 30;
//char globalBlueRead[sizeOfGlbData];
const byte sizeOfRFdata = 40;
char RF_data[sizeOfRFdata];


//String DataString = "";
//char DataString[35]; // для 8 переменных

//bool bAngleChanged = 0;

bool bPlay_fl = 0;
//bool bRec_fl = 0;

bool bRecModeOn = 0;

const uint8_t LHAZerro = 48;  // 60
const uint8_t LHSZerro = 33;  // 45

const uint8_t RHAZerro = 73;  // 65
const uint8_t RHSZerro = 122; // 110 

const uint8_t LHandZerro = 80;   //120
const uint8_t RHandZerro = 125;  //145

#ifndef GYVER
const uint16_t hdSt = 1277;
uint16_t cur_pulse[8] = { 1166, 833, 1722, 1685,
                          1544, 1270, hdSt, hdSt,
                        };
uint16_t new_pulse[8] = { 1166, 833, 1722, 1685,
                          1544, 1270, hdSt, hdSt,
                        };
#endif


#define FileName "data.txt"


bool volatile plu = 0;  // флаг таймера

struct main_data {
  short Azimuth = 0;
  short UgolMesta = 0;
  uint8_t LHandSholder = 0;
  uint8_t LHandArm = 0;
  uint8_t RHandSholder = 0;
  uint8_t RHandArm = 0;
  uint8_t LHand = 35;
  uint8_t RHand = 35;
};


  main_data param;

#ifdef GYVER
#define AMOUNT 8  // кол-во серво
ServoDriverSmooth servos[AMOUNT];

uint32_t turnTimer;
#endif

//#include "serial.h"
void buffer_setup()
{
    DDRD |= (1 << PD3);
    PORTD &= ~(1 << PD3);
}

template<typename T> void myPrint(T data);
template<typename T> void myPrintln(T data);

void tik() {
  plu = 1;
}


template<typename T> void myPrint(T data)
{
#ifdef OTLADKA
#ifdef MICROUART
    uart.write(data, sizeof(data));
#else
	Serial.print(data);
#endif
#endif
}

template<typename T> void myPrintln(T data)
{
#ifdef OTLADKA
#ifdef MICROUART
    uart.write(data, sizeof(data));
#else
	Serial.println(data);
//	Serial.write("\r\n");
#endif
#endif
}

void myPrintln(const char* data, uint16_t length=35);
void myPrintln(const char* data, uint16_t length)
{
	char str[length+2];
#ifdef OTLADKA
#ifdef MICROUART	
    uart.write(data, strlen(data));
#else
//	if(strlen(data)>length) return;
//	strcpy(str, data);
//	strcat(str,"\r\n");
//	Serial.write(str, strlen(str));
	Serial.write(data, strlen(data));
	Serial.write("\r\n");
#endif
#endif
}

void myPrint(const char* data)
{
#ifdef OTLADKA
#ifdef MICROUART
    uart.write(data, strlen(data));
#else
	Serial.write(data, strlen(data));
#endif
#endif
}



int read_bluetooth_noblock(void) {
  static byte serialAmount;

#ifdef MICROUART	
  if (uart.available()) {
    char temp = uart.read();
#else
  if (Serial.available()) {
    char temp = Serial.read();
#endif
    RF_data[serialAmount] = temp;
    //    globalBlueRead.push_back(temp);
    if (temp == ':')  // контроль корректности принятых данных
    {
//      if (serialAmount >= 30) {
//        myPrintln("BigData");
//        return 0;
 //     }
      RF_data[serialAmount] = '\n';
  //    for (int j = 0; j < serialAmount; j++) myPrint(RF_data[j]);
      //      myPrint("-->");
      //      myPrintln(serialAmount);
      //      myPrintln(RF_data);

      commandHandle(RF_data);
      /*	  for(int i = 0; i < sizeOfRFdata-15; i++)
      	  {
      	    RF_data[i] = '\0';
      	  }
      */
      serialAmount = 0;
      return 0;
    }
    serialAmount++;
  }  // while Serial availible

  return 0;
}


void commandHandle(char *RF_data) {
  //  myPrintln("ComHnd");
  bool lArmChanged = 0;

  // идентификация команды
  char cmd = RF_data[0];
  //  myPrint("dt[0]=");
  //  myPrintln(RF_data[0]);
  //  delay(20);
  // подсчет длины строки
  byte D_Len = strlen(RF_data);
//  myPrint("dt_lngth:");
//  myPrintln(D_Len);
  // 		delay(20);
//  if (D_Len >= 30) {
//    myPrintln("TBgDt");  // слишком длинная строка
//  }
  // парсинг строки в новую переменную
//  char cpyData[D_Len + 1];
//  for (int f = 0; f < (D_Len - 1); f++) {
//    cpyData[f] = RF_data[f + 1];
 // }
  // копирование обратно
  for (int f = 0; f < (D_Len - 1); f++) {
    RF_data[f] = RF_data[f+1];
  }

  RF_data[D_Len - 1] = 0;

  short x = atoi(&RF_data[0]);
  //  long x_l = atol(&RF_data[0]);
  switch (cmd) {
    case 'P':
      FlexiTimer2::start();
      plu = 0;
	  #ifdef OTLADKA
	  myPrintln("Start Play",11);
	  #endif
      Play();
      break;

    case 'R':
      bRecModeOn ? bRecModeOn = 0 : bRecModeOn = 1;
      if ((bRecModeOn == 0) || (1 == bPlay_fl)) {
        if (dataFile) dataFile.close();
        FlexiTimer2::stop();
		
		#ifdef OTLADKA
        myPrintln("Record stopped, FILE CLOSED", 26);
#endif
        break;
      }
      //      start_timer();
//      if (SD.exists(FileName)) {
//        SD.remove(FileName);
//        delay(5);
//      }
 #ifdef MYSDFAT
      dataFile.open(FileName, O_WRONLY|O_TRUNC);
#else
      if (SD.exists(FileName)) {
        SD.remove(FileName);
        delay(5);
	  }
	  dataFile = SD.open(FileName, FILE_WRITE);
#endif
#ifdef OTLADKA
      myPrintln("Start Rec",9);
	  #endif
      plu = 0;
      FlexiTimer2::start();
      
      //     saveToMemory(0, AZ, UM);
      break;

    case 'H':  // азимут
      if (1 == bPlay_fl) {
        //        myPrintln("ERR4");
        //       delay(20);
        break;
      }
      param.Azimuth = x;
      //     servoA.write(filterAZ(Azimuth + 90));
    //  bAngleChanged = 1;
//      myPrint("Az=");
//      myPrintln(param.Azimuth);
        moveHead();
      break;

    case 'V':  // угол места
      if (1 == bPlay_fl) {
        //       myPrintln("ERR3");
        break;
      }
      param.UgolMesta = x;
      //      servoU.write(filterUM(UgolMesta + 90));
   //   bAngleChanged = 1;
//      myPrint("Ug=");
      moveHead();
	  #ifdef OTLADKA
      myPrintln(param.UgolMesta);
#endif
      break;

    case 'A':
      if (1 == bPlay_fl) break;
      //      lArmChanged = 1;
      param.LHandSholder = x;
      moveLeftHand();
      break;
    case 'B':
      if (1 == bPlay_fl) break;
      param.LHandArm = x;
      moveLeftHand();
      break;
    case 'C':
      if (1 == bPlay_fl) break;
      param.RHandSholder = x;
      moveRightHand();
      break;
    case 'D':
      if (1 == bPlay_fl) break;
      param.RHandArm = x;
      moveRightHand();
      break;
    case 'E':
      if (1 == bPlay_fl) break;
      param.RHand = x;
      moveRightHand();
      break;
    case 'F':
      if (1 == bPlay_fl) break;
      param.LHand = x;
      moveLeftHand();
      break;
    case 'G':
      if (1 == bPlay_fl) break;
      param.LHandSholder = x;
      param.RHandSholder = x;
      moveLeftHand();
      moveRightHand();
      break;
    case 'I':
      if (1 == bPlay_fl) break;
      param.LHandArm = x;
      param.RHandArm = x;
      moveLeftHand();
      moveRightHand();
      break;
#ifdef OTLADKA
    default:
      myPrint(F("errCmd="));
      for (byte i = 0; i < D_Len; i++)
        myPrint((char)RF_data[i]);
      myPrintln("",1);
      //      delay(2000);
#endif
  }

//  if (bRecModeOn) {
//    saveToMemory();
//  }

//  if (bAngleChanged) {
//	fluctation = -fluctation;
//	param.UgolMesta+= 5*fluctation;
//    bAngleChanged = 0;
//  }
  if (lArmChanged) {

    lArmChanged = 0;
  }
}


void Rec(void) {
  if (bRecModeOn == 0) return;
  //  unsigned long time = RecordingTime();

  saveToMemory();

  return;
}


void addToString(char *DataString, short data)
{
  char str1[5];
//  char *punct = ",";
  strcat(DataString, ",");
  itoa(data,str1,10);
  strcat(DataString, str1);
  myPrint(str1);
  myPrint(", ");
}


void Play(void) {
  bPlay_fl ? bPlay_fl = 0 : bPlay_fl = 1;
  if ((0 == bPlay_fl) || (bRecModeOn)) return;
  //  start_timer();

  readFromMemory();
}

// функция кроме всего должна каждой записи присваивать индекс
// чтобы кроме всего прочего при воспроизведении можно было легко
// понять сколько записей имеется в наличии и сколько осталось до конца
void saveToMemory() {
  //  myPrintln("WrtMmry");
//  static uint32_t count = 0;
//  static uint32_t deltaTime = 0;
  if (plu == 1) {

    if (isnan(param.Azimuth) && isnan(param.UgolMesta) && 
	isnan(param.LHandSholder) && isnan(param.LHandArm) && 
	isnan(param.RHandSholder) && isnan(param.RHandArm)) {
#ifdef OTLADKA
      myPrintln("isnan", 6);
#endif
    } else {
      //      myPrintln("isan");
//	  char str1[5];
      char DataString[35]; // для 8 переменных
//	  char *punct = ",";
	  itoa(param.Azimuth, DataString,10);
//	  strcat(DataString, ",");
	  addToString(DataString, param.UgolMesta);
      addToString(DataString, param.LHandSholder);
      addToString(DataString, param.LHandArm);
      addToString(DataString, param.LHand);
      addToString(DataString, param.RHandSholder);
      addToString(DataString, param.RHandArm);
      addToString(DataString, param.RHand);
	  strcat(DataString, "\n");
//	  Serial.write('\n');
//	  DataString +=param.Azimuth; 		DataString+=",";
//	  DataString +=param.UgolMesta; 	DataString+=",";
//	  DataString +=param.LHandSholder; 	DataString+=",";
//	  DataString +=param.LHandArm;	 	DataString+=",";
//	  DataString +=param.LHand; 		DataString+=",";
//	  DataString +=param.RHandSholder; 	DataString+=",";
//	  DataString +=param.RHandArm; 		DataString+=",";
//	  DataString +=param.RHand;	

      //    DataString += ";";
      //    DataString += String(0x0D,DEC);
//      myPrintln(DataString);
      if (dataFile) {
 #ifdef MYSDFAT
      dataFile.write(DataString,sizeof(DataString));
#else
	  dataFile.write(DataString,strlen(DataString));
      myPrintln(int(strlen(DataString)));
//	  count++;
//	  Serial.println(millis()-deltaTime);
//	  Serial.write(DataString, strlen(DataString));
#endif
//        DataString = "";
// 	  memset(&DataString[0], 0, sizeof(DataString));
      } 
#ifdef OTLADKA
		else  myPrintln("FileError",10);
#endif
    }
    plu = 0;
//	myPrint(F("T="));

//	Serial.print(" ");
//    Serial.println(count);
//	Serial.write("\n");
//	deltaTime = millis();
  }
}

void readFromMemory(void) {
//  myPrintln("RdMmry");
  uint16_t outArray[35]; // unsigned short
  char *buffer;
  char array[35];
  int16_t symbol2 = 0;  // попробовать заменить на int8_t или char вместо short
  byte h = 0;

//  if (SD.exists(FileName)) {
//    dataFile.openExistingSFN(FileName);
    //    myPrint(F("opnng"));
    //    myPrintln(FileName);
//  } else {
//    myPrint(FileName);
//    myPrintln(F("notExist"));
//  }
 #ifdef MYSDFAT
  if (dataFile.openExistingSFN(FileName)) {
 #else
  if (SD.exists(FileName)){
  dataFile = SD.open(FileName, FILE_READ);
  myPrint("opnng ");
  myPrintln(FileName);
 #endif
//  static uint32_t deltaTime=0;
	  for(int i = 0; i<AMOUNT;i++)
      {
        servos[i].setSpeed(ServoSpeed+80);
        servos[i].setAccel(ServoAccel+0.59);
      }
	    char *p;
        byte t = 0;
    while (dataFile.available() > 0) {
      //     myPrintln("RdStrng");

      if (plu == 1) {

        while ((symbol2 = dataFile.read()) != 0x0A) {
          //          считываем байт входящего файла
#ifdef MYSDFAT
          dataFile.read(symbol2,1);
#else
       //   symbol2 = dataFile.read();	
#endif
		  array[h] = symbol2;
          h++;
          if (h >= 34) {
//            myPrint(F("error1 = "));
//            myPrintln(h);
//            myPrintln(array);
            h = 0;
            delay(5);
            break;
          }
//          delay(1);

//          	    myPrint((char)symbol2);
        }
//        myPrintln("");
        array[h] = '\0';
  //      char *p;
        t = 0;

        //	  if(plu == 1)
        //	  {
        for (buffer = strtok_r(array, ",", &p); buffer; buffer = strtok_r(NULL, ",", &p)) {
//                     myPrint(t);  myPrint(" = ");  myPrintln(atoi(buffer));
          delay(1);
          outArray[t] = atoi(buffer);
          switch (t % 9) {
            case 0:
              param.Azimuth = outArray[t];
              break;
            case 1:
              param.UgolMesta = outArray[t];
              break;
            case 2:
              param.LHandSholder = outArray[t];
              break;
            case 3:
              param.LHandArm = outArray[t];
              break;
            case 4:
              param.LHand = outArray[t];
              break;
            case 5:
              param.RHandSholder = outArray[t];
              break;
            case 6:
              param.RHandArm = outArray[t];
              break;
            case 7:
              param.RHand = outArray[t];
              break;
            case 8:
              break;
          }

          t++;
          if (t >= 37) {
#ifdef OTLADKA
            myPrintln("error2", 7);
#endif
            delay(100);
            t = 0;
          }
        }
        h = 0;
        moveHead();
        moveLeftHand();
        moveRightHand();
        makeMove();

        myPrint(param.Azimuth);
        myPrint(',');
        myPrint(param.UgolMesta);
        myPrint(',');
        myPrint(param.LHandSholder);
        myPrint(',');
        myPrint(param.LHandArm);
        myPrint(',');
        myPrint(param.LHand);
        myPrint(',');
        myPrint(param.RHandSholder);
        myPrint(',');
        myPrint(param.RHandArm);
        myPrint(',');
        myPrintln(param.RHand);

//		myPrint(F("T="));
//		myPrintln(millis() - deltaTime);
//	    deltaTime = millis();
        plu = 0;
      }
    }
    dataFile.close();
    FlexiTimer2::stop();
#ifdef OTLADKA
    myPrintln("playFileClosed",14);
#endif
    bPlay_fl = 0;
	  for(int i = 0; i<AMOUNT;i++)
      {
        servos[i].setSpeed(ServoSpeed);
        servos[i].setAccel(ServoAccel);
      }

    delay(500);
  } else {
#ifdef OTLADKA
    myPrint(("ErrOpnngFl"));
    myPrintln(FileName);
#endif
    delay(2000);
  }
}

void moveHead() {
  int corrUm = 90;
  int corrAz = 180;
#ifdef MY_FILTER
  short fAZ = filterAZ(param.Azimuth);
  short fUM = filterUM(param.UgolMesta);
  //  servoA.write(fAZ + 90);
  //  servoU.write(fUM + 90);
  headUpDn(fUM + corrUm);
  headLfRt(fAZ + corrAz);
#else
  //  servoA.write(param.Azimuth + 90);
  //  servoU.write(param.UgolMesta + 90);
  headUpDn(param.UgolMesta + corrUm);
  headLfRt(param.Azimuth + corrAz);

#endif
  if (bRecModeOn == 0) {
//    myPrint(param.Azimuth + corr);
//    myPrint(',');
//    myPrintln(param.UgolMesta + corr);
  }
}

void headUpDn(int16_t angle) {
  uint8_t servoNum = 7;

#ifdef GYVER
  servos[servoNum].setTargetDeg(angle);
#else  
  new_pulse[servoNum] = map(angle, 0, 270, 500, 2500);
#endif
#ifdef OTLADKA
//  myPrint("headUpDn angle = ");
//  myPrintln(angle);
#endif
}

void headLfRt(int16_t angle) {
  uint8_t servoNum = 6;
#ifdef GYVER
  servos[servoNum].setTargetDeg(angle);
#else  
  new_pulse[servoNum] = map(angle, 0, 270, 500, 2500);
#endif
//  myPrint(F("headLfRt pulse = "));
//  myPrintln(new_pulse[servoNum]);
}

void moveLeftHand() {
  if (param.LHandSholder > 130) param.LHandSholder = 130;
  if (param.LHandArm > 97) param.LHandArm = 97;
  if (param.LHand > 150) param.LHand = 150;
  moveServoSholder(0, param.LHandSholder);
  moveServoArm(1, param.LHandArm);
  moveServoHand(4, param.LHand);  
  if (bRecModeOn == 0) {
    //   myPrint(LHandSholder);
    //    myPrint(',');
    //    myPrintln(LHandArm);
    //    myPrint(',');
    //    myPrintln(LHand);
  }
}


void moveRightHand() {
  if (param.RHandSholder > 127) param.RHandSholder = 127;
  if (param.RHandArm > 97) param.RHandArm = 97;
  if (param.RHand > 150) param.RHand = 150;
  moveServoSholder(2, param.RHandSholder);
  moveServoArm(3, param.RHandArm);
  moveServoHand(5, param.RHand); 
  if (bRecModeOn == 0) {
    //   myPrint(RHandSholder);
    //   myPrint(',');
    //   myPrintln(RHandArm);
    //   myPrint(',');
    //   myPrintln(RHand);
  }
}



void moveServoHand(uint8_t n, uint8_t angle) {
  uint16_t data;
  if (n == 4) {
    data = angle + LHandZerro;
#ifdef GYVER
//  myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(data);
#else  
    new_pulse[n] = map(data, 0, 270, 500, 2500);
#endif
    //    myPrint(F("Hand pulse = "));
    //    myPrintln(new_pulse[n]);
  } else if (n == 5) {
    data = angle + RHandZerro;
#ifdef GYVER
//  myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(data);
#else  
    new_pulse[n] = map(data, 270, 0, 500, 2500);
#endif
    //    myPrint(F("Hand pulse = "));
    //    myPrintln(new_pulse[n]);
  }
}

// углы от 0 до 270
void moveServoSholder(uint8_t n, uint8_t angle) {
  //  uint16_t getSlower = 150;
  //  long pulse = 0;
  uint16_t data;
  if (n == 0) {
    data = angle + LHSZerro;
#ifdef GYVER
//  myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(data);
#else  
    new_pulse[n] = map(data, 0, 270, 500, 2500);
#endif

  } else if (n == 2) {
    data = angle + RHSZerro;
#ifdef GYVER
//  myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(data);
#else  
    new_pulse[n] = map(data, 270, 0, 500, 2500);
#endif
    //    myPrint(F("Arm pulse = "));
    //    myPrintln(new_pulse[n]);
  }
  //  myPrint("Arm pulse = ");  myPrintln(pulse);
  //  pulseSlower(n, pulse, getSlower);
  //   setServoPulse(n, pulse/1000000.);
}

// углы от 0 до 180
void moveServoArm(uint8_t n, uint8_t angle) {
  //  uint16_t getSlower = 50;
  if (n == 1) {
    angle = angle + LHAZerro;
#ifdef GYVER
//  myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(angle);
#else  
    new_pulse[n] = map(angle, 0, 180, 500, 2500);
#endif
  } else if (n == 3) {
    angle = angle + RHAZerro;
#ifdef GYVER
// myPrint(n + ": ");
//  myPrintln(angle);
  servos[n].setTargetDeg(angle);
#else  
    new_pulse[n] = map(angle, 180, 0, 500, 2500);
#endif
  }
  //  myPrint("Sholder pulse = ");  myPrintln(pulse);
  //  pulseSlower(n, pulse, getSlower);
  //  setServoPulse(n, pulse/1000000.);
}

#ifndef GYVER

void pulseSlower(uint8_t n, /* const long &pulse,*/ const uint16_t slower) {
  static uint16_t i[8] = { 1166, 833, 1722, 1685,
                           1544, 1270, hdSt, hdSt,
                         };
  if (cur_pulse[n] == new_pulse[n]) {
    i[n] = cur_pulse[n];
    return;
  }
  if (cur_pulse[n] < new_pulse[n]) {
    //   for (int i = cur_pulse[n]; i < new_pulse[n]; i++) {
    setServoPulse(n, i[n]++ / 1000000.);
    delayMicroseconds(slower);
    //   }
  } else if (cur_pulse[n] > new_pulse[n]) {
    //    for (int i = cur_pulse[n]; i > new_pulse[n]; i--) {
    setServoPulse(n, i[n]-- / 1000000.);
    delayMicroseconds(slower);
    //    }
  }
  if (i[n] == new_pulse[n]) cur_pulse[n] = new_pulse[n];
}



void setServoPulse(uint8_t n, double pulse) {
  double pulselength;

  //==== for RDS3218 servo ====
  // pulse = 0.5ms = 0 grad
  // pulse = 1.5ms = 90 grad(for 180) and 135 (for 270)
  // pulse = 2.5ms = 180 grad(for 180) and 270 (for 270)
  //  myPrintln(pulse);
  pulselength = 1000000;
  pulselength /= SERVO_FREQ;
  //  myPrint(pulselength);
  //  myPrintln(" us per period");
  pulselength /= 4096;
  //  myPrint(pulselength);
  //  myPrintln(" us per bit");
  pulse *= 1000000;
  pulse /= pulselength;
  //  myPrintln(pulse);
  pwm.setPWM(n, 0, pulse);
}
#endif


void makeMove(void)
{
#ifdef GYVER
  if (millis() - turnTimer >= 20) {	// взводим таймер на 20 мс (как в библиотеке)
    turnTimer = millis();
    for (byte i = 0; i < AMOUNT; i++) {
      servos[i].tickManual();   // двигаем все сервы. Такой вариант эффективнее отдельных тиков
    }
  }
#else
  for (int i = 0; i < 8; i++)
//    pulseSlower(i, 1 /*150 - (i%2)*100*/);  // остаток от деления нужен чтобы у локтей была пауза 50 вместо 150.
     setServoPulse(i,new_pulse[i]/1000000.);
#endif
}



void setup() {
#ifdef MICROUART
  uart.begin(115200);
#else
  Serial.begin(115200);
#endif

  // ШИМ пины на ардуино нано: 3, 5, 6, 9, 10, 11
  // servoA.attach(5);
  // servoU.attach(6);

#ifdef GYVER
  servos[0]=ServoDriverSmooth(pwmAddr, 270);
  servos[1]=ServoDriverSmooth(pwmAddr, 180);
  servos[2]=ServoDriverSmooth(pwmAddr, 270);
  servos[3]=ServoDriverSmooth(pwmAddr, 180);
  servos[4]=ServoDriverSmooth(pwmAddr, 270);
  servos[5]=ServoDriverSmooth(pwmAddr, 270);
  servos[6]=ServoDriverSmooth(pwmAddr, 270);
  servos[7]=ServoDriverSmooth(pwmAddr, 270);

  servos[5].setDirection(REVERSE);
  servos[2].setDirection(REVERSE);
  servos[3].setDirection(REVERSE);
  delay(50);
 
//  for(int i = 0; i<AMOUNT;i++) 
//  {
  servos[0].attach(0, LHSZerro);  //  поднять левое плечо
  servos[0].smoothStart();
  servos[1].attach(1, LHAZerro);  //  поднять левое предплечье
  servos[1].smoothStart();
  servos[2].attach(2, RHSZerro); //  поднять правое плечо
  servos[2].smoothStart();
  servos[3].attach(3, RHAZerro);  //  поднять правое предплечье
  servos[3].smoothStart();
  servos[4].attach(4, LHandZerro+40); //  вращать левое плечо
  servos[4].smoothStart();
  servos[5].attach(5, RHandZerro+40); //  вращать правое плечо
  servos[5].smoothStart();
  servos[6].attach(6,180); // влево вправо
  servos[6].smoothStart();
  servos[7].attach(7,90);  // вверх вниз
  servos[7].smoothStart();
  
///  }

  for(int i = 0; i<AMOUNT;i++)
  {
      servos[i].setSpeed(ServoSpeed);
      servos[i].setAccel(ServoAccel);
  }
//  servos[6].setSpeed(30);
//  servos[6].setAccel(0.2);
//  servos[7].setSpeed(30);
 // servos[7].setAccel(0.2);
  delay(100);
#else
  if (!pwm.begin()) myPrintln("Servo failed",13);
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
#endif

  delay(10);

  if (!SD.begin(SDchipSelect)) {
	  #ifdef OTLADKA
    myPrintln("CardFailed", 11);
	#endif
    // don't do anything more:
    //   return;
  } else {
	  #ifdef OTLADKA
    myPrintln("cardInit", 8);
	#endif
  }
  delay(2000);
  FlexiTimer2::set(40, tik);
  #ifdef OTLADKA
  myPrintln("Starting working", 17);
  #endif
}

void loop() {
//	  static int8_t fluctation = 1;
  read_bluetooth_noblock();
  makeMove();
  if (bRecModeOn) {
    saveToMemory();
  }else{

  }
//	fluctation = -fluctation;
//	param.UgolMesta+= 4*fluctation;

//  moveHead();


  //  for (int i = 0; i < 8; i++)
  //    pulseSlower(i, 1 /*150 - (i%2)*100*/);  // остаток от деления нужен чтобы у локтей была пауза 50 вместо 150.
  //  delayMicroseconds(75);

}


// вызывается в прерывании при приёме байта
void MU_serialEvent() {
  static bool val = 0;
  digitalWrite(13, val = !val);
  //Serial.write(Serial.read());
}

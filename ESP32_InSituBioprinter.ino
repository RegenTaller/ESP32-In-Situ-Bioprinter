#include <Arduino.h>
#include <SPI.h>
#include <BluetoothSerial.h>
#include "driver/gpio.h"
#include <EEPROM.h>

#include "TMC2130Stepper.h"
#include "esp_system.h"
#include "driver/gptimer.h"
#include "esp_log.h"

#include <SimpleKalmanFilter.h>

BluetoothSerial SerialBT;

#define _sign(x) ((x) > 0 ? 1 : -1)  //Сигнум для смены приращения скорости на уменьшение
// PWM configuration
#define PWM_FREQUENCY 20000  // Frequency in Hz (1kHz)
#define PWM_RESOLUTION 10    // Resolution in bits (0-1023)
#define PWM_CHANNEL 0        // PWM channel (0-15)

#define APH1_pin 32
#define BPH1_pin 33
gpio_num_t APHASE1_PIN = GPIO_NUM_32;
gpio_num_t BPHASE1_PIN = GPIO_NUM_33;

gpio_num_t PWM1_PIN = GPIO_NUM_21;
gpio_num_t IN1_PIN = GPIO_NUM_27;
gpio_num_t IN2_PIN = GPIO_NUM_14;
#define SPEED_1 21
#define IN_1 27
#define IN_2 14

#define APH2_pin 34
#define BPH2_pin 35
gpio_num_t APHASE2_PIN = GPIO_NUM_34;
gpio_num_t BPHASE2_PIN = GPIO_NUM_35;

gpio_num_t PWM2_PIN = GPIO_NUM_22;
gpio_num_t IN3_PIN = GPIO_NUM_12;
gpio_num_t IN4_PIN = GPIO_NUM_13;
#define SPEED_2 22
#define IN_3 12
#define IN_4 13

gpio_num_t LED = GPIO_NUM_2;

volatile long enc1;

static const int spiClk = 1000000;  // 1 MHz
//uninitalised pointers to SPI objects
SPIClass* vspi = NULL;

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
const bool level = 1;  //направление для всех пинов

int Dduty = 0;


int _accel = 30000;    //ускорение в отсчётах энкодера в секунду
int _maxSpeed = 8344;  //максимальная скорость в отсчётах энкодера в секунду

const uint8_t _dt = 20;          //Временной шаг вызова функции регулирования
float _dts = (float)_dt / 1000;  //Временной шаг для подсчёта в единицах в секунду теор. значений скоростей V и позиций pos
uint32_t _tmr2 = 0;              //Переменная таймера comp_cur_pos для подсчёта значений V и pos
long _targetPos = 0;             //Целевое положение в отсчётах энкодера
int dir = -1;                    //Направление 1 - вперёд // -1 - назад

long controlPos = 0;     //Теоретическое положение, идущее в PID
float controlSpeed = 0;  //Теоретическая скорость на шаге

int _minDuty = 0, _maxDuty = 1023;  //Минимальное значение страгивания ДПТ: (_minDuty-255)
float _k = 1.0;
/////////////////////////////////////////////////////
/////////////////////////////////////////////////////

///////////////////////////////////////////////////////////
/////////ПЕРЕМЕННЫЕ ЭНКОДЕРОВ И РЕАЛЬНОЙ СКОРОСТИ//////////

volatile int lastEncoded1 = 0;    // Here updated value of encoder store.
volatile long encoderValue1 = 0;  // Raw encoder value
volatile long encREF1 = 0;        //сбрасываемое значениеэнкодера для скорости

volatile int lastEncoded2 = 0;    // Here updated value of encoder store.
volatile long encoderValue2 = 0;  // Raw encoder value
volatile long encREF2 = 0;        //сбрасываемое значениеэнкодера для скорости

volatile long encREF1N = 0;  // Raw encoder value
volatile long encREF2N = 0;

int32_t encREF1copy = 0;
int32_t encREF2copy = 0;
float Velocity1 = 0.0;  //Переменная мгновенной скорости 1
float Velocity2 = 0.0;  //Переменная мгновенной скорости 2

float Velocity1N = 0.0;  //Переменная мгновенной скорости 1
float Velocity2N = 0.0;  //Переменная мгновенной скорости 2
int32_t encREF1copyN = 0;
int32_t encREF2copyN = 0;
//uint8_t freq1 = 10; // частота опроса

uint8_t per1 = 50;  ///freq1; //период опроса скоростей 1000/60 = 17 мс
uint32_t t1 = 0;    //начальный момент времени unsigned long
uint32_t t2 = 0;    //unsigned long 4

int _buf[3];
byte _count = 0;
float _middle_f = 0;

float T_Theoretic = 0;

int ratio = 8344;  //8433 = 28*298
///////////////////////////////////////////////////////////////////////////
////////////////////////////////ПИД РЕГУЛЯТОР//////////////////////////////
int PWM1 = 0, PWM2 = 0;
long tpid = 0;

long _previnput1 = 0, _previnput2 = 0;
float integral1 = 0.0000;
float integral2 = 0.0000;

bool cutoff = 0;
int stopzone1 = 10, stopzone2 = 10;


float kp = 0.55;  // 2x: 0.35;  4x: 0.085		// (знач. по умолчанию)0.1 0.5 0.05
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
double ki = 0.00000005;  //2x: 0.000005 4x: 0.00000011 0.00000015 для высок.
// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
float kd = 0.35;  //2x: 35 - 2 прерывания 4x: 0.75 7.75 для высок скоростей

float kpVel = 0.55;  // 2x: 0.35;  4x: 0.085		// (знач. по умолчанию)0.1 0.5 0.05
  // интегральный - позволяет нивелировать ошибку со временем, имеет накопительный эффект
double kiVel = 0.00000005;  //2x: 0.000005 4x: 0.00000011 0.00000015 для высок.
// дифференциальный. Позволяет чуть сгладить рывки, но при большом значении
// сам становится причиной рывков и раскачки системы!
float kdVel = 0.35;  //2x: 35 - 2 прерывания 4x: 0.75 7.75 для высок скоростей
//////////////////////////////////////////////////////////////////////

////////////////Альтернативная "АНАЛИТИЧЕСКАЯ" кривая/////////////////
float ta = 0;  //конечный момент времени
float tb = 0;  //время окончания разгона
float tc = 0;  //время окончания равномерного движения

float xb = 0;
float xc = 0;
float curTime = 0;
long controlPos2 = 0;
int N = 0;  // Число шагов дискретизации

int8_t curve = 1;  //Тип кривой. -1 - пересечение парабол 2 - нормальный случай, 1 - только ускорение и равномерное

long f = 0;  //счётчик приращения

/////////////////////////////////////////////////////////////////////

//////////////////////////////Сбор данных////////////////////////////
uint8_t marker = 0;  //маркер прекращения
#define arr_sizePos1 40
#define arr_sizePos2 4
long EncStat1[arr_sizePos1][arr_sizePos2];
long EncStat2[arr_sizePos1][arr_sizePos2];
unsigned long del = 0;  //Обнуление таймера микрос

long tmrexp = 0;  //таймер цикла сбора

uint16_t cnn = 0, cnn1 = 0, cnn2 = 0;  //Счётчики вызовов

int16_t du = 0, du1 = 0;
/////////////////////////////////////////////////////////////////////

bool offtrig = 0;

bool PrintDataFlag = 1;

double rkp1 = 0.010000, rkp2 = 0.010000;

//////////////////Подавитель наростания//////////////////
uint32_t reduceFlag1, reduceFlag2;
int velfl1 = 0, velfl2 = 0;
float rconst;
int defaultPWM1 = _minDuty, defaultPWM2 = _minDuty;
int defaultPWMFlag1, defaultPWMFlag2;
int prevPWM1, prevPWM2;
/////////////////////////////////////////////////////////

/////////////////////ГЛАВНЫЙ ПРИВОД//////////////////////
/////////////////////////////////////////////////////////
#define MICROSTEPS 16  // Set to the microsteps that the motor needs

uint16_t controlSteps_Saved = 0;  //Сохранённая позиция моторов. Нужна для отсутствия сбития положения при отключении контроллера.
float MM_na_oborot = 31.141592;   // Миллиметров будет пройдено за один оборот. Диаметр шестерни 10 мм.
int microsteps = 16;              // количество микрошагов двигателя
// Motor Settings
float MM_na_Shag = MM_na_oborot / (200 * MICROSTEPS);  // Миллиметров будет пройдено за один шаг. 200 - полных шагов двигателя на оборот
float speedMMs = 20;                                   // Скорость в мм/с
float maxSP = 20;
float minSP = 10;
long newSP;

float Freq_MOT = speedMMs / MM_na_Shag;        // Шагов в секунду при данной скорости
float period_MOT = 1000000.0 / Freq_MOT;       // Микросекунд на шаг при данной скорости
float MOT_ISR_Tact = (period_MOT / (10)) / 2;  // Сколько тактов нужно для данной конфигурации таймера 519 мкс для 150 мм/с, значит 51,9 тактов для N = 159

volatile int MOT_ISR_NMAX = round(MOT_ISR_Tact);;
volatile int MOT_ISR_N = round(MOT_ISR_Tact);  // ОКРУГЛЁННОЕ ЗНАЧЕНИЕ ШАГОВ. Определяет скорость вращения мотора.

bool en = 1;  //ОТКЛЮЧЕНИЕ ЛОГИКИ ДРАЙВЕРА

volatile uint16_t counterSteps = 0;  //Реальное количество поданных импульсов/сделанных шагов, получаемое в ISR. Без учёта потерь

int direction = 0;                //Направление
float controlPosStepper = 0;      //Необходимая позиция в мм. Пропорциональна количеству шагов
unsigned int controlStepPos = 0;  //Необходимая позиция в шагах, зависящая от позиции в миллиметрах

unsigned int MAXSteps = 40000;  //ОГРАНИЧИТЕЛЬ ШАГОВ
int MINSteps = 0;               //НИЖНИЙ ОГРАНИЧИТЕЛЬ ШАГОВ
int shift = 0;                  //ОТСТУП ДЛЯ КАЛИБРОВКИ

SimpleKalmanFilter Filter(1, 1, 0.015);  //1-ый коэффициент - амплитуда разлёта показаний от реального, третий - скорость изменения показаний
float filtered = 0;
float dist = 0;
const float coeff = (float)1.0 / 10000.0 * 335.0 / 2.0;
// Define SPI pins
#define CS 5
#define SCK 18
#define MOSI 23
#define MISO 19

// TMC2130 configuration
#define EN 15   // No enable pin used
#define DIR 16  // No direction pin used
#define STEP 17
gpio_num_t EN_PIN = GPIO_NUM_15;
gpio_num_t DIR_PIN = GPIO_NUM_16;
gpio_num_t STEP_PIN = GPIO_NUM_17;

gpio_num_t TRIG_PIN = GPIO_NUM_2;
gpio_num_t ECHO_PIN = GPIO_NUM_4;
#define TRIG 2
#define ECHO 4

// TMC2130 driver instance
TMC2130Stepper driver = TMC2130Stepper(EN, DIR, STEP, CS);
bool dirStepper = true;

/////////////////////Trigger/////////////////////////////
volatile int counternew = 0;
volatile int per = 200;  // Equivalent to your 'per' variable

// Timer configuration
hw_timer_t* timer = NULL;
volatile int pulseCount = 0;  // Count how many pulses have been triggered
void IRAM_ATTR onTimer() {

  if (pulseCount <= 5) {          // 10мкс * 5 = 50 мкс импульс
    gpio_set_level(TRIG_PIN, 1);  // Set the pin HIGH      // Set pulse active flag
  } else {
    gpio_set_level(TRIG_PIN, 0);
  }
  pulseCount++;
  if (pulseCount >= 1000) {
    pulseCount = 0;
  }

  static uint16_t MOT_counter;
  static bool flagMove;

  MOT_counter++;  //Отсчёт тактов.

  if (MOT_counter == MOT_ISR_N) {  //Если настало время, обнуляется счётчик

    MOT_counter = 0;
    flagMove = 1;
  }

  if (en == 0 && flagMove == 1 && counterSteps < MAXSteps && counterSteps != controlStepPos) {

    gpio_set_level(STEP_PIN, 1);
    gpio_set_level(STEP_PIN, 0);
    flagMove = 0;

    if (direction == 1) {

      counterSteps++;

    } else if (direction == 0) {

      counterSteps--;
    }
  }

  flagMove == 0;
}

/////////////////////ECHO////////////////////////
volatile uint32_t pulse_duration = 0;  // Variable to store the pulse duration
volatile uint32_t countEcho = 0;
hw_timer_t* timer1 = NULL;
void IRAM_ATTR echoISR() {

  if (digitalRead(ECHO_PIN)) {                     // Rising edge detected
    timerWrite(timer1, 0);                         // Reset timer
    timerAlarm(timer1, 999999, false, 1000000);  // Start the timer
    countEcho++;
  } else {                               // Falling edge detected
    pulse_duration = timerRead(timer1);  // Get the timer count in µs
    timerWrite(timer1, 0);               // Stop the timer
  }
}

void setup() {

  Serial.begin(115200);
  SerialBT.begin("InSitu-Bioprinter");

  MainPinMode();   // Setting pins for main drivers and sensor
  setTRIGTimer();  // Setting timer for sensor and main drivers
  setECHOTimer();  // Setting ECHO pulse width measurement timer
  StepperSetup();  // Stepper motor drivers settings

  DispenserPinMode();  // Setting pins for dispanser encoders and motors
  Calculus(_accel, _maxSpeed, _targetPos, _dts);

  SetDispenser();

  _tmr2 = tpid = millis();
}

int8_t fl = 1;

void IRAM_ATTR loop() {

  //comp_cur_pos();
  prevPWM2 = PWM2;
  prevPWM1 = PWM1;

  //CommunicationBT();
  static uint32_t tim0;
  if (millis() - tpid >= _dt) {  // Расчёт ПИД

    comp_cur_pos();
    Velocities(_dt);
    velfl1 = velfl2 = 1;

    PWM2 = PIDcalc2(controlPos, encoderValue2, kp, ki, kd, _dt, dir, 0, offtrig);
    PWM1 = PIDcalc1(controlPos, encoderValue1, kp, ki, kd, _dt, dir, 0, offtrig);
    tpid = millis();
  }

  static uint32_t tim2;

  if (millis() - tim2 >= 20) {  //Расчёт редукции ПИД и движение

    VelocitiesN();

    staticReduction();

    if (abs(PWM2) >= abs(defaultPWM2) * 1.2 && abs(Velocity2N) >= _maxSpeed - 25 && rconst != 0 && defaultPWM2 >= _minDuty) {  //Отсечка увеличенного ШИМ, если скорость выше нужной
      PWM2 = defaultPWM2;
    }
    if (abs(PWM1) >= abs(defaultPWM1) * 1.2 && abs(Velocity1N) >= _maxSpeed - 25 && rconst != 0 && defaultPWM1 >= _minDuty) {  //Отсечка увеличенного ШИМ, если скорость выше нужной
      PWM1 = defaultPWM1;
    }

    movement(PWM2, 2);
    movement(PWM1, 1);
    
    tim2 = millis();
  }

  static uint32_t tim1;
  if (millis() - tim1 >= 100) {  // Вывод в COM
    if (PrintDataFlag == 1) {
      POSITIONS();
      Serial.println("");
      Serial.println(newSP);
      //Serial.println(" ");
      //Serial.println(PWM2);
      //VELS();
    }
    // Serial.println(PWM2);
    tim1 = millis();
  }

  static long t2;
  if (millis() - t2 >= 750) {  // Сохранение позиции главного привода

    t2 = millis();

    if (PrintDataFlag == 1) {
    }

    inputData();
    CommunicationBT();
    EEPROM.put(32, counterSteps);  //Обновление сохранённого значения позиции моторов
    EEPROM.commit();
  }

  static long timp2;
  if (millis() - timp2 >= 10) {

    dist = pulse_duration * coeff;
    filtered = Filter.updateEstimate(dist);  //Фильтрация значения дистанции
    //filtered = dist;
    //speedCORR();
    StepMotGen();

    timp2 = millis();
  }
}

void SetDispenser() {

  setRatio(8344, 4);
  setObor(0);
  setSpeedMMS(1);
  setMinDuty(250);

  static long controlPos_Saved, encoderValue1_Saved, encoderValue2_Saved;
  EEPROM.begin(1024);
  EEPROM.get(0, controlPos_Saved);
  EEPROM.get(8, encoderValue1_Saved);
  EEPROM.get(16, encoderValue2_Saved);
  controlPosStepper = _targetPos = controlPos_Saved;
  encoderValue1 = encoderValue1_Saved;
  encoderValue2 = encoderValue2_Saved;
}

void DispenserPinMode() {

  while (!Serial)
    ;
  // Set BTN_STOP_ALARM to input mode
  pinMode(LED, OUTPUT);

  pinMode(APHASE1_PIN, INPUT_PULLDOWN);
  pinMode(BPHASE1_PIN, INPUT_PULLDOWN);
  pinMode(APHASE2_PIN, INPUT_PULLDOWN);
  pinMode(BPHASE2_PIN, INPUT_PULLDOWN);

  pinMode(SPEED_1, OUTPUT);
  pinMode(IN_1, OUTPUT);
  pinMode(IN_2, OUTPUT);

  pinMode(SPEED_2, OUTPUT);
  pinMode(IN_3, OUTPUT);
  pinMode(IN_4, OUTPUT);

  attachInterrupt(APH2_pin, pin_A2_ISR, CHANGE);
  attachInterrupt(BPH2_pin, pin_B2_ISR, CHANGE);
  attachInterrupt(APH1_pin, pin_A1_ISR, CHANGE);
  attachInterrupt(BPH1_pin, pin_B1_ISR, CHANGE);

  // Attach the PWM channel to the chosen motor pin
  ledcAttach(PWM2_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWM1_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  delay(1000);
}

void pin_A2_ISR() {

  if (!(gpio_get_level(APHASE2_PIN)) == !(gpio_get_level(BPHASE2_PIN))) {  //2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000
    encoderValue2--;
    encREF2--;
    encREF2N--;

  } else {

    encoderValue2++;
    encREF2++;
    encREF2N++;
  }
}

void pin_B2_ISR() {
  if (!(gpio_get_level(APHASE2_PIN)) == !(gpio_get_level(BPHASE2_PIN))) {  // 2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    encoderValue2++;
    encREF2++;
    encREF2N++;

  } else {

    encoderValue2--;
    encREF2--;
    encREF2N--;
  }
  //enc1++;
}

void pin_A1_ISR() {

  if (!(gpio_get_level(APHASE1_PIN)) == !(gpio_get_level(BPHASE1_PIN))) {  //2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    encoderValue1++;
    encREF1++;
    encREF1N++;
  } else {

    encoderValue1--;
    encREF1--;
    encREF1N--;
  }
}

void pin_B1_ISR() {
  if (!(gpio_get_level(APHASE1_PIN)) == !(gpio_get_level(BPHASE1_PIN))) {  // 2 pin - PE4 PINE & 0b00010000 3 pin - PE5 PINE & 0b00100000

    //enc1--;
    encoderValue1--;
    encREF1--;
    encREF1N--;
  } else {

    //enc1++;
    encoderValue1++;
    encREF1++;
    encREF1N++;
  }
  //enc1++;
}

bool thisDir;
void comp_cur_pos() {  // USes timpos & _tmr2  (long encoder, uint8_t motor)

  unsigned long timpos = millis();

  if (timpos - _tmr2 >= _dt) {
    _dts = (_dt) / 1000.0;  //(timpos - _tmr2) / 1000.0;
    _tmr2 = millis();
    long err = _targetPos - controlPos;  // "ошибка" позиции
    if (err != 0) {
      if (_accel != 0) {

        thisDir = (controlSpeed * controlSpeed / _accel / 2.0 >= abs(err));  // пора тормозить (false до приближения к параболе торможения обр. от ускор.)
        controlSpeed += _accel * _dts * (thisDir ? -_sign(controlSpeed) : _sign(err));
        //Serial.println("Cont SPEED VAR 1: " + String(controlSpeed));

      } else {
        controlSpeed = err / _dts;  // профиль постоянной скорости
        //Serial.println("Cont SPEED VAR 2: " + String(controlSpeed));
      }
      controlSpeed = constrain(controlSpeed, -_maxSpeed, _maxSpeed);
      controlPos += controlSpeed * _dts;
      controlPos = constrain(controlPos, -150000, 150000);
    }
    //return controlPos;
    //Serial.print(encoderValue1);
    //Serial.print(' ');
    // Serial.print(Velocity1);
    // Serial.print(' ');
    // Serial.println();
  }
}

float rkpmax = 0.00;
int PIDcalc2(long setPoint, long current, float kp, float ki, float kd, float DT, int dir, bool cutoff, bool off) {  //ПИД для 1 первого мотора

  //if (!off) {
  static long prevVelocity2;
  long velocity2 = Velocity2;

  if (dir == -1) {
    setPoint = -setPoint;
  }

  float Duty2 = 0;
  long err2 = setPoint - current;
  long deltainput2 = _previnput2 - err2;
  _previnput2 = err2;
  Duty2 = (float)(err2 * kp);
  integral2 += (float)err2 * ki * DT;
  Duty2 += (float)deltainput2 * kd / DT + integral2;

  if (cutoff) {  // отсечка (для режимов позиции)
    if (abs(err2) < stopzone2) {
      integral2 = 0;
      Duty2 = 0;
      // Serial.println("null");
    }
  }

  Duty2 = constrain(Duty2, -_maxDuty, _maxDuty);

  int PWMnew2;
  PWMnew2 = Duty2;
  static bool flag2;

  if (abs(velocity2) > (abs(_maxSpeed)) && velfl2 == 1 && rkpmax != 0) {

    long errVel2 = abs(_maxSpeed) - abs(velocity2);
    float deltaVel2 = velocity2 - prevVelocity2;
    int deltaPWM2 = PWM2 - prevPWM2;  //rkp * errVel
    PWMnew2 = PWM2;
    prevPWM2 = PWM2;            //Сохранение предыдущего значения ШИМ
    prevVelocity2 = velocity2;  //Сохранение предыдущего значения скорости
    //if (abs(velocity1) > abs(controlspeed)) {
    if (rkp2 != 0.0) {
      flag2 = 1;
      PWMnew2 = constrain(abs(PWMnew2) + rkp2 * errVel2, defaultPWM2, PWM2);
    }
    if (PWM2 != prevPWM2 && velocity2 != prevVelocity2) {
      rkp2 = abs(velocity2 - prevVelocity2) / abs(PWM2 - prevPWM2);
    }
    if (_maxSpeed < 0) {
      PWMnew2 = -1 * abs(PWMnew2);
      defaultPWM2 = -1 * abs(defaultPWM2);
    }
    reduceFlag2++;
    velfl2 = 0;
    flag2 = 1;
  }

  prevVelocity2 = velocity2;
  prevPWM2 = PWM2;
  velfl2 = 0;
  if (flag2 == 1) {
    flag2 = 0;
  }

  flag2 = 0;
  return int(PWMnew2);
}

int PIDcalc1(long setPoint, long current, float kp, float ki, float kd, float DT, int dir, bool cutoff, bool off) {  //ПИД для 1 первого мотора

  //if (!off) {
  static long prevVelocity1;
  long velocity1 = Velocity1;

  if (dir == -1) {
    setPoint = -setPoint;
  }

  float Duty1 = 0;
  long err1 = setPoint - current;
  long deltainput1 = _previnput1 - err1;
  _previnput1 = err1;
  Duty1 = (float)(err1 * kp);
  integral1 += (float)err1 * ki * DT;
  Duty1 += (float)deltainput1 * kd / DT + integral1;

  if (cutoff) {  // отсечка (для режимов позиции)
    if (abs(err1) < stopzone1) {
      integral1 = 0;
      Duty1 = 0;
      // Serial.println("null");
    }
  }

  Duty1 = constrain(Duty1, -_maxDuty, _maxDuty);

  int PWMnew1;
  PWMnew1 = Duty1;
  static bool flag1;

  if (abs(velocity1) > (abs(_maxSpeed)) && velfl1 == 1 && rkpmax != 0) {

    long errVel1 = abs(_maxSpeed) - abs(velocity1);
    float deltaVel1 = velocity1 - prevVelocity1;
    int deltaPWM1 = PWM1 - prevPWM1;  //rkp * errVel
    PWMnew1 = PWM1;
    prevPWM1 = PWM1;            //Сохранение предыдущего значения ШИМ
    prevVelocity1 = velocity1;  //Сохранение предыдущего значения скорости
    //if (abs(velocity1) > abs(controlspeed)) {
    if (rkp1 != 0.0) {
      flag1 = 1;
      PWMnew1 = constrain(abs(PWMnew1) + rkp1 * errVel1, defaultPWM1, PWM1);
    }
    if (PWM1 != prevPWM1 && velocity1 != prevVelocity1) {
      rkp1 = abs(velocity1 - prevVelocity1) / abs(PWM1 - prevPWM1);
    }
    if (_maxSpeed < 0) {
      PWMnew1 = -1 * abs(PWMnew1);
      defaultPWM1 = -1 * abs(defaultPWM1);
    }
    reduceFlag1++;
    velfl1 = 0;
    flag1 = 1;
  }

  prevVelocity1 = velocity1;
  prevPWM1 = PWM1;
  velfl1 = 0;
  if (flag1 == 1) {
    flag1 = 0;
  }

  flag1 = 0;
  return int(PWMnew1);
}

void staticReduction() {

  if (abs(Velocity2N) >= _maxSpeed - 25 && defaultPWMFlag2 == 0) {  // Фиксация значения нормльного PWM

    defaultPWMFlag2 = 1;
    defaultPWM2 = PWM2;
  }
  if (abs(Velocity1N) >= _maxSpeed - 25 && defaultPWMFlag1 == 0) {  // Фиксация значения нормльного PWM

    defaultPWMFlag1 = 1;
    defaultPWM1 = PWM1;
  }

  static long adjDefPWMtimer1;
  static long adjDefPWMtimer2;
  if (abs(Velocity2N) > _maxSpeed && abs(prevPWM2) <= abs(defaultPWM2)) {

    if (millis() - adjDefPWMtimer2 >= 100) {
      defaultPWM2 = constrain(abs(defaultPWM2) - ((_maxDuty / 100) * (_dt * 2 / 10)), _minDuty, _maxDuty);
      if (_maxSpeed < 0) {
        defaultPWM2 = defaultPWM2 * -1;
      }
      adjDefPWMtimer2 = millis();
    }
  }

  if (abs(Velocity1N) > _maxSpeed && abs(prevPWM1) <= abs(defaultPWM1)) {

    if (millis() - adjDefPWMtimer1 >= 100) {
      defaultPWM1 = constrain(abs(defaultPWM1) - ((_maxDuty / 100) * (_dt * 2 / 10)), _minDuty, _maxDuty);
      if (_maxSpeed < 0) {
        defaultPWM1 = defaultPWM1 * -1;
      }
      adjDefPWMtimer1 = millis();
    }
  }

  if (abs(Velocity2N) > _maxSpeed && thisDir == false) {  //Оперативная редукция ШИМ
    if (rconst != 0) {
      static long deltaV2 = abs(Velocity2N - _maxSpeed);
      if (_maxSpeed > 0) {

        PWM2 = PWM2 - (rconst * deltaV2);
        if (PWM2 < defaultPWM2) {
          PWM2 = defaultPWM2;
        }
      } else {
        PWM2 = PWM2 + (rconst * deltaV2);
        if (PWM2 > defaultPWM2) {
          PWM2 = defaultPWM2;
        }
      }
    }
  }

  if (abs(Velocity1N) > _maxSpeed && thisDir == false) {  //Оперативная редукция ШИМ
    if (rconst != 0) {
      static long deltaV1 = abs(Velocity1N - _maxSpeed);
      if (_maxSpeed > 0) {

        PWM1 = PWM1 - (rconst * deltaV1);
        if (PWM1 < defaultPWM1) {
          PWM1 = defaultPWM1;
        }
      } else {
        PWM1 = PWM1 + (rconst * deltaV1);
        if (PWM1 > defaultPWM1) {
          PWM1 = defaultPWM1;
        }
      }
    }
  }
}

void movement(int _Duty, int motor) {  //Приведение ШИМ к минимальному. Передача сигнала

  if (motor == 1) {}
  Dduty = _Duty;

  if (Dduty > 0) {
    //Вращение при регулировании в направлении вперёд по умолчанию
    if (_minDuty != 0) {
      Dduty = Dduty * _k + _minDuty;
    }  // сжимаем диапазон
  } else {
    // Вращение в обратном напра
    if (_minDuty != 0) { Dduty = Dduty * _k - _minDuty; }  // сжимаем диапазон
  }

  int PWM = abs(Dduty);
  bool stopper1 = 0;
  bool stopper2 = 0;

  if ((abs(encoderValue1) + stopzone1 > _targetPos) && abs(_Duty) < 12) {
    //digitalWrite(IN_1, 1);
    gpio_set_level(IN1_PIN, 1);
    //digitalWrite(IN_2, 1);
    gpio_set_level(IN2_PIN, 1);
    ledcWrite(SPEED_1, _maxDuty);
    integral1 = 0;
    stopper1 = 1;

  } else {
    stopper1 = 0;
  }

  if ((abs(encoderValue2) + stopzone2 > _targetPos) && abs(_Duty) < 12) {

    gpio_set_level(IN3_PIN, 1);
    gpio_set_level(IN4_PIN, 1);
    //digitalWrite(IN_3, 1);
    //digitalWrite(IN_4, 1);
    ledcWrite(SPEED_2, _maxDuty);
    integral2 = 0;
    stopper2 = 1;
  } else {
    stopper2 = 0;
  }

  if (motor == 1 && stopper1 != 1) {  //Для первого мотора

    if (Dduty > 0) {

      gpio_set_level(IN1_PIN, level);
      gpio_set_level(IN2_PIN, !level);
      //digitalWrite(IN_1, level);
      //digitalWrite(IN_2, !level);
      ledcWrite(SPEED_1, PWM);
    } else {

      gpio_set_level(IN1_PIN, !level);
      gpio_set_level(IN2_PIN, level);
      // digitalWrite(IN_1, !level);
      // digitalWrite(IN_2, level);
      ledcWrite(SPEED_1, PWM);
    }

  } else if (motor == 2 && stopper2 != 1) {  //Для второго мотора

    if (Dduty > 0) {

      gpio_set_level(IN3_PIN, level);
      gpio_set_level(IN4_PIN, !level);
      // digitalWrite(IN_3, level);
      // digitalWrite(IN_4, !level);
      ledcWrite(SPEED_2, PWM);

    } else {

      gpio_set_level(IN3_PIN, !level);
      gpio_set_level(IN4_PIN, level);
      // digitalWrite(IN_3, !level);
      // digitalWrite(IN_4, level);
      ledcWrite(SPEED_2, PWM);
    }
  }
}

////////////////////////////////////////////////////////////////////////
/////////////////////////////СЕТЕРЫ SET/////////////////////////////////

void setMillimeters(float millimeters) {

  if (millimeters < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  _targetPos = (float)((millimeters * (float)ratio) / 1.0);  //1.0 - шаг винта.
  Serial.println("_targetPos: " + String(_targetPos));
  //_targetPos = abs(_targetPos);
}

void setSpeedMMS(float millimetersSec) {

  _maxSpeed = millimetersSec * ratio;
  constrain(_maxSpeed, 0, 25000);
  Serial.println("_maxSpeed: " + String(_maxSpeed));
}

void setTarget(long TargetPos) {

  if (TargetPos < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = abs(TargetPos);
}

void setObor(float ob) {

  dir = 1;
  if (ob < 0) {  //Инверсия направления для отрицательной позиции
    //dir = -1;
    Serial.println("Revers");
  }

  Serial.println("Oborotov: " + String(ob));
  _targetPos = (round(ob * ratio));  //ratio - число тиков на оборот
  Serial.println("_targetPos: " + String(_targetPos));
}

void setDeg(long Deg) {

  if (Deg < 0) {
    //dir = -1;
    Serial.println("Revers");
  } else {
    dir = 1;
  }  //Инверсия направления для отрицательной позиции

  Serial.println("Deg: " + String(Deg));
  Serial.println("_targetPos: " + String(_targetPos));
  _targetPos = Deg * ratio / 360;
}

void setRatio(uint16_t _ratio, uint8_t precision) {

  ratio = _ratio / (4 / precision);
  _maxSpeed = _maxSpeed / (4 / precision);
  _accel = _accel / (4 / precision);
}

void setMinDuty(int duty) {

  _minDuty = duty;
  _k = 1.0 - (float)_minDuty / _maxDuty;
}

void Calculus(int acceleration, int V, long targetPos, float dts) {  //Подсчитывает моменты времени и

  float Xta = (float)targetPos;  //Функция координаты от времени для t;
  float dti = dts;
  float Vmax = (float)V;
  float accel = acceleration;

  tb = Vmax / accel;  //момент времени завершения разгона
  tc = Xta / (Vmax);  //Момент времени завершения равномерного движения

  xb = (float)(Vmax * Vmax) / (2 * accel);  //координата завершения разгона

  float taorig = 0.0;


  if (_accel != 0) {

    if (xb < _targetPos / 2) {  //Если возможна кривая ускорения, равномерного движения и замедления

      taorig = (float)(Xta / Vmax + Vmax / accel);  //Завершающий момент времени 3 сегмента
      curve = 2;                                    //3 сегмента кривой

    } else if (xb > _targetPos / 2 && xb < _targetPos) {  //Ускорение и равномерное движение

      taorig = _targetPos / Vmax + tb / 2;
      tc = taorig;
      curve = 1;

    } else if (xb == _targetPos / 2) {  //Если возможно только ускорение и замедление

      taorig = (float)(2 * (Vmax / accel));
      curve = 0;

    } else if (xb >= _targetPos) {  //Если возможно только ускорение

      taorig = sqrt(2 * _targetPos / accel);
      curve = -1;
    }

  } else {

    taorig = _targetPos / Vmax;
  }

  ta = (int)(round(taorig * 100));
  ta = (float)ta / 100;

  N = round(ta / dti);

  Serial.println("Время ta_Calculus: " + String(ta));
  Serial.println("N: " + String(N));
  Serial.println("Время ускорения tb: " + String(tb));
  Serial.println("Позиция после ускорения xb: " + String(xb));

  Serial.println("Время начала торможения tc: " + String(tc));
  Serial.println("Curve: " + String(curve));
}

///////////////////////////////////////////////////////////////
/////////////////////СБОР ДАННЫХ И ВЫВОДЫ//////////////////////

bool posflag = 1;
void POSITIONS() {  //Вывод позиций столбцами

  if (posflag == 1) {

    Serial.println();
    //Serial.println(_targetPos);
    Serial.print(controlPos);
    //Serial.print(Velocity1);
    //Serial.print(_duty);
    //Serial.print(controlPos);
    Serial.print(", ");
    Serial.print(abs(encoderValue1));
    //Serial.print(Velocity1);
    //Serial.print(_duty);
    //Serial.print(controlPos);
    Serial.print(", ");

    //Serial.print(_duty2);
    Serial.print(abs(encoderValue2));

    // Serial.print(", ");
    // Serial.print(abs(integral));

    // Serial.print(", ");
    // Serial.print(abs(integral2));


    //Serial.print(controlSpeed/1000);
    //Serial.print(", ");
    //Serial.print(Velocity2);
  }
}

void PWMPORT() {

  Serial.println();

  Serial.print(", ");
  Serial.print(PWM1);

  Serial.print(", ");
  Serial.print(PWM2);

  Serial.print(", ");
  Serial.print(PWM1);
}

void VELS() {  //Вывод скоростей  столбцами

  Serial.println();

  Serial.print(millis());
  Serial.print(", ");
  //Serial.println(_targetPos);
  Serial.print(controlSpeed);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  //Serial.print(", ");
  //Serial.print(Velocity2);
  //Serial.print(Velocity1);
  //Serial.print(Velocity1);
  //Serial.print(_duty);
  //Serial.print(controlPos);
  Serial.print(", ");

  //Serial.print(_duty2);
  Serial.print(Velocity2N);
  //Serial.print(controlSpeed/1000);
  //Serial.print(", ");
  //Serial.print(Velocity2);
}

////////////////////ОБРАБОТКА ВХОДА//////////////////
void SavePos() {

  EEPROM.put(0, controlPos);
  EEPROM.put(8, encoderValue1);
  EEPROM.put(16, encoderValue2);
  EEPROM.commit();
}

void Velocities(int period) {  //Функция определения скорости моторов

  static long periodF;
  static long time;
  periodF = (micros() - time) / 1000;
  time = micros();
  //noInterrupts();
  encREF1copy = encREF1;  //int16_t
  encREF2copy = encREF2;  //int16_t
  encREF1 = encREF2 = 0;

  Velocity1 = (float)encREF1copy * 1000.0 / (float)period;
  Velocity2 = (float)encREF2copy * 1000.0 / (float)period;
}

void VelocitiesN() {  //Функция определения скорости моторов

  static long periodF;
  static long time;
  periodF = (micros() - time) / 1000;
  time = micros();

  encREF1copyN = encREF1N;  //int16_t
  encREF2copyN = encREF2N;  //int16_t
  encREF1N = encREF2N = 0;

  Velocity1N = (float)encREF1copyN * 1000.0 / (float)periodF;
  Velocity2N = (float)encREF2copyN * 1000.0 / (float)periodF;
}

int filter(int velocity) {
  _buf[_count] = velocity;
  if (++_count >= 3) _count = 0;
  int middle = 0;
  if ((_buf[0] <= _buf[1]) && (_buf[0] <= _buf[2])) {
    middle = (_buf[1] <= _buf[2]) ? _buf[1] : _buf[2];
  } else {
    if ((_buf[1] <= _buf[0]) && (_buf[1] <= _buf[2])) {
      middle = (_buf[0] <= _buf[2]) ? _buf[0] : _buf[2];
    } else {
      middle = (_buf[0] <= _buf[1]) ? _buf[0] : _buf[1];
    }
  }
  _middle_f += (middle - _middle_f);
  return _middle_f;
}

/////////////////////////////////////////////////////////////////////////

////////////////////////////Главный привод///////////////////////////////
void MainPinMode() {

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT_PULLUP);
  //pinMode(LED, OUTPUT);
}

void setTRIGTimer() {

  timer = timerBegin(10000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 100, true, 0);  // Set to trigger every 10 us
}

void setECHOTimer() {

  attachInterrupt(ECHO_PIN, echoISR, CHANGE);  // Attach the ISR to handle rising and falling edges
  timer1 = timerBegin(10000000);               // Timer 0, Prescaler 80 (1 µs per tick)
  timerWrite(timer1, 0);                       // No alarm yet
}

void StepperSetup() {

  while (!Serial)
    ;
  SPI.begin();
  // Initialize the TMC2130 driver
  driver.begin();
  driver.begin();  // Initiate pins and registeries
  driver.rms_current(200);
  driver.microsteps(MICROSTEPS);
  digitalWrite(EN, 0);
  driver.stealthChop(1);

  EEPROM.begin(1024);
  EEPROM.get(32, controlSteps_Saved);  //Получение, записанного перед выключением, положения мотора
  if (controlSteps_Saved > MAXSteps) {
    counterSteps = 0;
  } else {
    counterSteps = controlSteps_Saved;
  }

  Serial.println("TMC2130 configured!");
}

void StepMotGen() {

  if (filtered >= 82) {  //Ограничение максимальной дистанции

    controlPosStepper = 82 - shift;

  } else {

    controlPosStepper = filtered - shift;
  }

  controlStepPos = controlPosStepper / MM_na_Shag;  //Получение желаемой позиции в шагах

  if (counterSteps < controlStepPos && counterSteps <= MAXSteps) {  //Движение вниз

    direction = 1;
    en = 0;
    gpio_set_level(EN_PIN, en);
    gpio_set_level(DIR_PIN, direction);
    //digitalWrite(EN, en);
    //digitalWrite(DIR, direction);

  } else if (counterSteps > controlStepPos && counterSteps >= MINSteps) {  //Движение вверх

    direction = 0;
    en = 0;
    gpio_set_level(EN_PIN, en);
    gpio_set_level(DIR_PIN, direction);
    //digitalWrite(EN, en);
    //digitalWrite(DIR, direction);

  } else if (counterSteps >= controlStepPos && counterSteps <= controlStepPos) {  //Остановка, если в зоне равновесия

    en == 1;
    gpio_set_level(EN_PIN, en);
    gpio_set_level(DIR_PIN, 0);
    // digitalWrite(EN, en);
    // digitalWrite(DIR, 0);
  }
}

void speedCORR() {

  long deltaPOS = counterSteps - controlStepPos;
  if (deltaPOS < 0) { deltaPOS = deltaPOS * -1; }
  //newSP = (deltaPOS * MM_na_Shag) / (controlStepPos * MM_na_Shag) * maxSP + minSP;
  newSP = 3*MOT_ISR_NMAX - (((float)(deltaPOS) / (2048)) * 2*MOT_ISR_NMAX);
  //newSP = MOT_ISR_NMAX;
  //if (newSP < minSP) { newSP = minSP; }
  //if (newSP > maxSP) { newSP = maxSP; }
  if (newSP < MOT_ISR_NMAX) { newSP = MOT_ISR_NMAX; }
  if (newSP > 200) { newSP = 200;}
  MOT_ISR_N = newSP;
  //recalculation(1, newSP);
}

/////////////////////////////////////////////////////////////////////////

////////////////////////////COM Communication////////////////////////////

void inputData() {

  if (Serial.available() > 0) {  //если есть доступные данные

    //Serial.println("Serial.available()");
    char buffer[] = { "" };
    String dannie = "";
    int j = 0;

    while (Serial.available()) {

      while (Serial.available()) {
        dannie = dannie + Serial.readString();
      }
      //   if (Serial.readBytes(buffer, 1)) {
      //     //Serial.print("I received: ");
      //     //Serial.println(buffer[0]);
      //     dannie = dannie + buffer[0];
      //   }
      //   // j++;
      //   // Serial.print(j);
      //   // Serial.print(" :  ");
      //   //Serial.println(buffer[0]);
    }

    dannie.trim();
    //Serial.println("");
    Serial.println("Received: " + dannie);

    if (dannie.indexOf("sp") != -1) {

      uint8_t sppos = dannie.indexOf("sp") + 2;
      String speedrec = dannie.substring(sppos, sppos + 6);
      float receivedspeed = constrain(speedrec.toInt(), 1, 300);
      Serial.println("received speed: " + String(receivedspeed));
      maxSP = receivedspeed;
      recalculation(1, receivedspeed);
    }

    if (dannie.indexOf("sh") != -1) {

      uint8_t shpos = dannie.indexOf("sh") + 2;
      String Shiftrec = dannie.substring(shpos, shpos + 6);
      float receivedshift = constrain(Shiftrec.toInt(), 0.0, 82.0);
      Serial.println("received shift: " + String(receivedshift));
      recalculation(2, receivedshift);
    }

    if (dannie.indexOf("vel") != -1) {

      uint8_t vpos = dannie.indexOf("vel") + 3;
      String vel = dannie.substring(vpos, vpos + 6);
      int receivedvelocity = constrain(vel.toInt(), 0, 17000);
      Serial.println("received velocity: " + String(receivedvelocity));
      //recalculation(1, receivedspeed);
      _maxSpeed = receivedvelocity;
      defaultPWMFlag1 = defaultPWMFlag2 = _minDuty;
    }

    if (dannie.indexOf("velMMS") != -1) {

      uint8_t velMMSpos = dannie.indexOf("velMMS") + 6;
      String velMMS = dannie.substring(velMMSpos, velMMSpos + 6);
      float receivedvelMMS = constrain(velMMS.toFloat(), 0, 3.5);
      Serial.println("received velocity in mm/s: " + String(receivedvelMMS));
      //recalculation(1, receivedspeed);
      setSpeedMMS(receivedvelMMS);
      defaultPWMFlag1 = defaultPWMFlag2 = _minDuty;
    }

    if (dannie.indexOf("tpos") != -1) {

      uint8_t tppos = dannie.indexOf("tpos") + 4;
      String tposrec = dannie.substring(tppos);
      long receivedtpos = constrain(tposrec.toInt(), -250000, 250000);
      Serial.println("received target position: " + String(receivedtpos));
      _targetPos = receivedtpos;
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("acc") != -1) {

      uint8_t accpos = dannie.indexOf("acc") + 3;
      String accrec = dannie.substring(accpos, accpos + 6);
      int receivedacc = constrain(accrec.toInt(), 0, 40000);
      Serial.println("received acceleration: " + String(receivedacc));
      _accel = receivedacc;
    }

    if (dannie.indexOf("rk") != -1) {

      uint8_t rkpos = dannie.indexOf("rk") + 2;
      String rkrec = dannie.substring(rkpos, rkpos + 6);
      float receivedrk = constrain(rkrec.toFloat(), 0, 100);
      rkpmax = receivedrk;
      Serial.println("received REDUCTION rk: " + String(rkpmax));
    }

    if (dannie.indexOf("rconst") != -1) {

      uint8_t rconstpos = dannie.indexOf("rconst") + 6;
      String rconstrec = dannie.substring(rconstpos, rconstpos + 6);
      float receivedrconst = constrain(rconstrec.toFloat(), 0, 100);
      rconst = receivedrconst;
      Serial.println("received REDUCTION CONST: " + String(rconst));
    }

    if (dannie.indexOf("kp") != -1) {

      uint8_t kppos = dannie.indexOf("kp") + 2;
      String kprec = dannie.substring(kppos, kppos + 6);
      float receivedkp = constrain(kprec.toFloat(), 0, 100);
      Serial.println("received PID kp: " + String(receivedkp));
      kp = receivedkp;
    }

    if (dannie.indexOf("ki") != -1) {

      uint8_t kipos = dannie.indexOf("ki") + 2;
      String kirec = dannie.substring(kipos, kipos + 8);
      double receivedki = constrain(kirec.toDouble(), 0.0, 100.0);
      Serial.println("received PID ki: " + String(receivedki));
      ki = receivedki;
    }

    if (dannie.indexOf("kd") != -1) {

      uint8_t kdpos = dannie.indexOf("kd") + 2;
      String kdrec = dannie.substring(kdpos, kdpos + 6);
      double receivedkd = constrain(kdrec.toDouble(), 0.0, 100.0);
      Serial.println("received PID kd: " + String(receivedkd));
      kd = receivedkd;
    }

    if (dannie.indexOf("minDuty") != -1) {

      uint8_t minDutypos = dannie.indexOf("minDuty") + 7;
      String minDutyrec = dannie.substring(minDutypos, minDutypos + 6);
      int receivedminDuty = constrain(minDutyrec.toInt(), 0, _maxDuty);
      Serial.println("received PID MIN Duty: " + String(receivedminDuty));
      setMinDuty(receivedminDuty);
    }

    if (dannie.indexOf("maxDuty") != -1) {

      uint8_t maxDutypos = dannie.indexOf("maxDuty") + 7;
      String maxDutyrec = dannie.substring(maxDutypos, maxDutypos + 6);
      long receivedmaxDuty = constrain(maxDutyrec.toInt(), 0, 1000000);
      Serial.println("received PID MAX Duty: " + String(receivedmaxDuty));
      _maxDuty = receivedmaxDuty;
    }

    if (dannie.indexOf("Z") != -1) {

      uint8_t Zpos = dannie.indexOf("Z") + 1;
      String Zrec = dannie.substring(Zpos, Zpos + 6);
      long receivedZ = constrain(Zrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in mm: " + String(receivedZ));
      _targetPos += receivedZ * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Zob") != -1) {

      uint8_t Zobpos = dannie.indexOf("Zob") + 3;
      String Zobrec = dannie.substring(Zobpos, Zobpos + 6);
      long receivedZob = constrain(Zobrec.toInt(), -200, 200);
      Serial.println("received piston pusher Z shift in turns: " + String(receivedZob));
      _targetPos += receivedZob * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Ztick") != -1) {

      uint8_t Ztickpos = dannie.indexOf("Ztick") + 5;
      String Ztickrec = dannie.substring(Ztickpos, Ztickpos + 6);
      long receivedZtick = constrain(Ztickrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in ticks: " + String(receivedZtick));
      _targetPos += receivedZtick;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("tposMMS") != -1) {

      uint8_t tposMMSpos = dannie.indexOf("tposMMS") + 7;
      String tposMMSrec = dannie.substring(tposMMSpos, tposMMSpos + 7);
      float receivedtposMMS = constrain(tposMMSrec.toFloat(), -200, 200);
      Serial.println("received target position in mm: " + String(receivedtposMMS));
      setMillimeters(receivedtposMMS);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("tposob") != -1) {

      uint8_t tposobpos = dannie.indexOf("tposob") + 6;
      String tposobrec = dannie.substring(tposobpos, tposobpos + 6);
      float receivedtposob = constrain(tposobrec.toFloat(), -200, 200);
      Serial.println("received target position in revs: " + String(receivedtposob));
      setObor(receivedtposob);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Savepos") != -1) {

      //uint8_t Savepospos = dannie.indexOf("Savepos") + 7;
      //String Saveposrec = dannie.substring(Savepospos, Savepospos + 6);
      //float receivedSavepos = constrain(Saveposrec.toFloat(), -500000, 500000);
      Serial.println("Save position" + String(controlPos));
      SavePos();
      //defaultPWMFlag1 = defaultPWMFlag2 = 0;
      //defaultPWM1 = defaultPWM2 = _minDuty;
    }

    volatile long bauds[] = { 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 250000, 460800, 500000 };

    if (dannie.indexOf("COM") != -1) {

      bool supported = false;

      uint8_t COMBaudpos = dannie.indexOf("COM") + 3;
      PrintDataFlag = 0;
      Serial.println("/////Stop Data sending/////");
      long rec = dannie.substring(COMBaudpos).toInt();
      Serial.println("COM baud REC: " + String(rec));
      long NewBaud = constrain(rec, 1200, 500000);

      for (int i = 0; i < sizeof(bauds) / 4; i++) {
        Serial.println(bauds[i]);
        if (bauds[i] == NewBaud) {

          supported = true;
          Serial.println("Supported baud: " + String(bauds[i]));
          break;

        } else {
          supported = false;
        }
      }

      if (supported == false) {

        Serial.println("Invalid/Insupported baud");
        Serial.println("NewBaud Is: " + String(NewBaud));
        NewBaud = 115200;
      }

      Serial.println("NEW BAUD: " + String(NewBaud));

      if (NewBaud >= 1200) {

        Serial.end();
        delay(1500);

        Serial.begin(NewBaud);
        delay(1000);
        Serial.println("Successfully applied baud: " + String(NewBaud));
      }
    }

    if (dannie.indexOf("PD") != -1) {

      Serial.println("PD");
      PrintDataFlag = !PrintDataFlag;

      if (PrintDataFlag == 1) {

        Serial.println("///////////Data:///////////");

      } else {

        Serial.println("/////Stop Data sending/////");
      }
    }
    //Serial.println(dannie);
  }
}

void CommunicationBT() {

  if (SerialBT.available() > 0) {

    String dannie = "";

    while (SerialBT.available()) {

      while (SerialBT.available()) {

        dannie = dannie + SerialBT.readString();
      }
    }

    dannie.trim();
    //Serial.println("");
    Serial.println(dannie);
    SerialBT.println(dannie);

    if (dannie.indexOf("sp") != -1) {

      uint8_t sppos = dannie.indexOf("sp") + 2;
      String speedrec = dannie.substring(sppos, sppos + 6);
      int receivedspeed = constrain(speedrec.toInt(), 1, 300);
      Serial.println("received speed: " + String(receivedspeed));
      SerialBT.println("received speed: " + String(receivedspeed));
      maxSP = receivedspeed;
      recalculation(1, receivedspeed);
    }

    if (dannie.indexOf("sh") != -1) {

      uint8_t shpos = dannie.indexOf("sh") + 2;
      String Shiftrec = dannie.substring(shpos, shpos + 6);
      float receivedshift = constrain(Shiftrec.toInt(), 0.0, 82.0);
      Serial.println("received shift: " + String(receivedshift));
      SerialBT.println("received shift: " + String(receivedshift));
      recalculation(2, receivedshift);
    }

    if (dannie.indexOf("vel") != -1) {

      uint8_t vpos = dannie.indexOf("vel") + 3;
      String vel = dannie.substring(vpos, vpos + 6);
      int receivedvelocity = constrain(vel.toInt(), 0, 17000);
      Serial.println("received velocity: " + String(receivedvelocity));
      SerialBT.println("received velocity: " + String(receivedvelocity));
      //recalculation(1, receivedspeed);
      _maxSpeed = receivedvelocity;
      defaultPWMFlag1 = defaultPWMFlag2 = _minDuty;
    }

    if (dannie.indexOf("velMMS") != -1) {

      uint8_t velMMSpos = dannie.indexOf("velMMS") + 6;
      String velMMS = dannie.substring(velMMSpos, velMMSpos + 6);
      float receivedvelMMS = constrain(velMMS.toFloat(), 0, 3.5);
      Serial.println("received velocity in mm/s: " + String(receivedvelMMS));
      SerialBT.println("received velocity in mm/s: " + String(receivedvelMMS));
      //recalculation(1, receivedspeed);
      setSpeedMMS(receivedvelMMS);
      defaultPWMFlag1 = defaultPWMFlag2 = _minDuty;
    }

    if (dannie.indexOf("tpos") != -1) {

      uint8_t tppos = dannie.indexOf("tpos") + 4;
      String tposrec = dannie.substring(tppos);
      long receivedtpos = constrain(tposrec.toInt(), -250000, 250000);
      Serial.println("received target position: " + String(receivedtpos));
      SerialBT.println("received target position: " + String(receivedtpos));
      _targetPos = receivedtpos;
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("acc") != -1) {

      uint8_t accpos = dannie.indexOf("acc") + 3;
      String accrec = dannie.substring(accpos, accpos + 6);
      int receivedacc = constrain(accrec.toInt(), 0, 40000);
      Serial.println("received acceleration: " + String(receivedacc));
      SerialBT.println("received acceleration: " + String(receivedacc));
      _accel = receivedacc;
    }

    if (dannie.indexOf("rk") != -1) {

      uint8_t rkpos = dannie.indexOf("rk") + 2;
      String rkrec = dannie.substring(rkpos, rkpos + 6);
      float receivedrk = constrain(rkrec.toFloat(), 0, 100);
      rkpmax = receivedrk;
      Serial.println("received REDUCTION rk: " + String(rkpmax));
      SerialBT.println("received REDUCTION rk: " + String(rkpmax));
    }

    if (dannie.indexOf("rconst") != -1) {

      uint8_t rconstpos = dannie.indexOf("rconst") + 6;
      String rconstrec = dannie.substring(rconstpos, rconstpos + 6);
      float receivedrconst = constrain(rconstrec.toFloat(), 0, 100);
      rconst = receivedrconst;
      Serial.println("received REDUCTION CONST: " + String(rconst));
      SerialBT.println("received REDUCTION CONST: " + String(rconst));
    }

    if (dannie.indexOf("kp") != -1) {

      uint8_t kppos = dannie.indexOf("kp") + 2;
      String kprec = dannie.substring(kppos, kppos + 6);
      float receivedkp = constrain(kprec.toFloat(), 0, 100);
      Serial.println("received PID kp: " + String(receivedkp));
      SerialBT.println("received PID kp: " + String(receivedkp));
      kp = receivedkp;
    }

    if (dannie.indexOf("ki") != -1) {

      uint8_t kipos = dannie.indexOf("ki") + 2;
      String kirec = dannie.substring(kipos, kipos + 8);
      double receivedki = constrain(kirec.toDouble(), 0.0, 100.0);
      Serial.println("received PID ki: " + String(receivedki));
      SerialBT.println("received PID ki: " + String(receivedki));
      ki = receivedki;
    }

    if (dannie.indexOf("kd") != -1) {

      uint8_t kdpos = dannie.indexOf("kd") + 2;
      String kdrec = dannie.substring(kdpos, kdpos + 6);
      double receivedkd = constrain(kdrec.toDouble(), 0.0, 100.0);
      Serial.println("received PID kd: " + String(receivedkd));
      SerialBT.println("received PID kd: " + String(receivedkd));
      kd = receivedkd;
    }

    if (dannie.indexOf("minDuty") != -1) {

      uint8_t minDutypos = dannie.indexOf("minDuty") + 7;
      String minDutyrec = dannie.substring(minDutypos, minDutypos + 6);
      int receivedminDuty = constrain(minDutyrec.toInt(), 0, _maxDuty);
      Serial.println("received PID MIN Duty: " + String(receivedminDuty));
      SerialBT.println("received PID MIN Duty: " + String(receivedminDuty));
      setMinDuty(receivedminDuty);
    }

    if (dannie.indexOf("maxDuty") != -1) {

      uint8_t maxDutypos = dannie.indexOf("maxDuty") + 7;
      String maxDutyrec = dannie.substring(maxDutypos, maxDutypos + 6);
      long receivedmaxDuty = constrain(maxDutyrec.toInt(), 0, 1000000);
      Serial.println("received PID MAX Duty: " + String(receivedmaxDuty));
      SerialBT.println("received PID MAX Duty: " + String(receivedmaxDuty));
      _maxDuty = receivedmaxDuty;
    }

    if (dannie.indexOf("Z") != -1) {

      uint8_t Zpos = dannie.indexOf("Z") + 1;
      String Zrec = dannie.substring(Zpos, Zpos + 6);
      long receivedZ = constrain(Zrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in mm: " + String(receivedZ));
      SerialBT.println("received piston pusher Z shift in mm: " + String(receivedZ));
      _targetPos += receivedZ * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Zob") != -1) {

      uint8_t Zobpos = dannie.indexOf("Zob") + 3;
      String Zobrec = dannie.substring(Zobpos, Zobpos + 6);
      long receivedZob = constrain(Zobrec.toInt(), -200, 200);
      Serial.println("received piston pusher Z shift in turns: " + String(receivedZob));
      SerialBT.println("received piston pusher Z shift in turns: " + String(receivedZob));
      _targetPos += receivedZob * ratio;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Ztick") != -1) {

      uint8_t Ztickpos = dannie.indexOf("Ztick") + 5;
      String Ztickrec = dannie.substring(Ztickpos, Ztickpos + 6);
      long receivedZtick = constrain(Ztickrec.toInt(), -250000, 250000);
      Serial.println("received piston pusher Z shift in ticks: " + String(receivedZtick));
      SerialBT.println("received piston pusher Z shift in ticks: " + String(receivedZtick));
      _targetPos += receivedZtick;
      _targetPos = constrain(_targetPos, -250000, 250000);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("tposMMS") != -1) {

      uint8_t tposMMSpos = dannie.indexOf("tposMMS") + 7;
      String tposMMSrec = dannie.substring(tposMMSpos, tposMMSpos + 7);
      float receivedtposMMS = constrain(tposMMSrec.toFloat(), -200, 200);
      Serial.println("received target position in mm: " + String(receivedtposMMS));
      SerialBT.println("received target position in mm: " + String(receivedtposMMS));
      setMillimeters(receivedtposMMS);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("tposob") != -1) {

      uint8_t tposobpos = dannie.indexOf("tposob") + 6;
      String tposobrec = dannie.substring(tposobpos, tposobpos + 6);
      float receivedtposob = constrain(tposobrec.toFloat(), -200, 200);
      Serial.println("received target position in revs: " + String(receivedtposob));
      SerialBT.println("received target position in revs: " + String(receivedtposob));
      setObor(receivedtposob);
      defaultPWMFlag1 = defaultPWMFlag2 = 0;
      defaultPWM1 = defaultPWM2 = _minDuty;
    }

    if (dannie.indexOf("Savepos") != -1) {

      //uint8_t Savepospos = dannie.indexOf("Savepos") + 7;
      //String Saveposrec = dannie.substring(Savepospos, Savepospos + 6);
      //float receivedSavepos = constrain(Saveposrec.toFloat(), -500000, 500000);
      Serial.println("Save position" + String(controlPos));
      SerialBT.println("Save position" + String(controlPos));
      SavePos();
      //defaultPWMFlag1 = defaultPWMFlag2 = 0;
      //defaultPWM1 = defaultPWM2 = _minDuty;
    }

    volatile long bauds[] = { 1200, 2400, 4800, 9600, 19200, 31250, 38400, 57600, 74880, 115200, 230400, 250000, 460800, 500000 };

    if (dannie.indexOf("COM") != -1) {

      bool supported = false;

      uint8_t COMBaudpos = dannie.indexOf("COM") + 3;
      PrintDataFlag = 0;
      Serial.println("/////Stop Data sending/////");
      SerialBT.println("/////Stop Data sending/////");
      long rec = dannie.substring(COMBaudpos).toInt();
      Serial.println("COM baud REC: " + String(rec));
      SerialBT.println("COM baud REC: " + String(rec));
      long NewBaud = constrain(rec, 1200, 500000);

      for (int i = 0; i < sizeof(bauds) / 4; i++) {
        Serial.println(bauds[i]);
        if (bauds[i] == NewBaud) {

          supported = true;
          Serial.println("Supported baud: " + String(bauds[i]));
          SerialBT.println("Supported baud: " + String(bauds[i]));
          break;

        } else {
          supported = false;
        }
      }

      if (supported == false) {

        Serial.println("Invalid/Insupported baud");
        SerialBT.println("Invalid/Insupported baud");
        Serial.println("NewBaud Is: " + String(NewBaud));
        SerialBT.println("NewBaud Is: " + String(NewBaud));
        NewBaud = 115200;
      }

      Serial.println("NEW BAUD: " + String(NewBaud));
      SerialBT.println("NEW BAUD: " + String(NewBaud));

      if (NewBaud >= 1200) {

        Serial.end();
        delay(1500);

        Serial.begin(NewBaud);
        delay(1000);
        Serial.println("Successfully applied baud: " + String(NewBaud));
        SerialBT.println("Successfully applied baud: " + String(NewBaud));
      }
    }

    if (dannie.indexOf("PD") != -1) {

      Serial.println("PD");
      SerialBT.println("PD");
      PrintDataFlag = !PrintDataFlag;

      if (PrintDataFlag == 1) {

        Serial.println("///////////Data:///////////");
        SerialBT.println("///////////Data:///////////");

      } else {

        Serial.println("/////Stop Data sending/////");
        SerialBT.println("/////Stop Data sending/////");
      }
    }
  }
}

void recalculation(int opID, float val) {

  switch (opID) {

    case 1:  //смена скорости

      Freq_MOT = val / MM_na_Shag;             // Шагов в секунду при данной скорости
      period_MOT = 1000000.0 / Freq_MOT;       // Микросекунд на шаг при данной скорости
      MOT_ISR_Tact = (period_MOT / (10)) / 2;  // Сколько тактов нужно для данной конфигурации таймера 519 мкс для 150 мм/с, значит 51,9 тактов для N = 159

      MOT_ISR_N = round(MOT_ISR_Tact);  // ОКРУГЛЁННОЕ ЗНАЧЕНИЕ ШАГОВ. Определяет скорость вращения мотора.
      MOT_ISR_NMAX = MOT_ISR_N;

      break;

    case 2:

      shift = val;  //изменение отступа

      break;
  }
}
#include <Wire.h>                 // Подключаем библиотеку для работы с шиной I2C
#include <Adafruit_GFX.h>         // Hardware-specific library
//#include <Adafruit_SSD1306.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>
#include <iarduino_SensorPulse.h> // Подключаем библиотеку для работы с AD8232 и с датчиком пульса  PulseSensor
#include <RTClib.h>
#include <MAX30105.h>           // Подключаем библиотеку для работы с модулем
#include <heartRate.h>          // Подключаем блок для работы с ЧСС (пульс)
#include <spo2_algorithm.h> 
#include <string.h>
//#include <SoftwareSerial.h>

static const unsigned char PROGMEM pulseSmallPicture[] ={ 
  0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,              //две картинки bmp, которые отображаются на OLED при вызове
  0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
  0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
  0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00};

static const unsigned char PROGMEM pulseBigPicture[] ={ 
  0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00};

MCUFRIEND_kbv tft;
//для дисплея TFT 2.4"
  // #define LCD_RST A4             // Select goes to Analog 4
  // #define LCD_CS  A3             // Chip Select goes to Analog 3
  // #define LCD_RS  A2             // Command/Data goes to Analog 2
  // #define LCD_WR  A1             // LCD Write goes to Analog 1
  // #define LCD_RD  A0             // LCD Read goes to Analog 0
//
class MyTftSettings{
  public:
    MyTftSettings(){
      _text           = TFT_DARKGREY;
      _groundText     = TFT_BLACK;
      _groundScreen   = TFT_DARKGREY;
      _backEnd        = TFT_BLACK;
      _w              = 315;
      _x1             = 1;
      _y1             = 8;
      _h              = 32;
      _rotation       = 1;
      _textSize       = 2;
      _radius         = 8;
      _YP             = A1;             // must be an analog pin, use "An" notation!
      _XM             = A2;             // must be an analog pin, use "An" notation!
      _YM             = 7;              // can be a digital pin
      _XP             = 6;              // can be a digital pin
      _RX             = 300;            // По
      _TSMinX         = 84;             // колибровка тач-скрина
      _TSMinY         = 62;
      _TSMaxX         = 953;
      _TSMaxY         = 904;
    }

    void     setText(uint16_t text)                  {_text          = text;}
    void     setGroundText(uint16_t groundText)      {_groundText    = groundText;}
    void     setScreenText(uint16_t groundScreen)    {_groundScreen  = groundScreen;}
    void     setBackEnd(uint16_t backEnd)            {_backEnd       = backEnd;}
    uint16_t getText()                               {return _text;}
    uint16_t getGroundText()                         {return _groundText;}
    uint16_t getGroundScreen()                       {return _groundScreen;}
    uint16_t getBackEnd()                            {return _backEnd;}
    uint16_t getW()                                  {return _w;}
    uint8_t  getX1()                                 {return _x1;}
    uint8_t  getY1()                                 {return _y1;}
    uint8_t  getH()                                  {return _h;}
    uint8_t  getRotation()                           {return _rotation;}
    uint8_t  getTextSize()                           {return _textSize;}
    uint16_t getRadius()                             {return _radius;}
    uint8_t  getYP()                                 {return _YP;}
    uint8_t  getXM()                                 {return _XM;}
    uint8_t  getYM()                                 {return _YM;}
    uint8_t  getXP()                                 {return _XP;}
    uint16_t getRX()                                 {return _RX;}
    uint16_t getTSMinX()                             {return _TSMinX;}
    uint16_t getTSMinY()                             {return _TSMinY;}
    uint16_t getTSMaxX()                             {return _TSMaxX;}
    uint16_t getTSMaxY()                             {return _TSMaxY;}

  private:
   uint16_t _text, _groundText, _groundScreen, _backEnd, _w, _radius, _RX;
   uint16_t _TSMinX, _TSMinY, _TSMaxX, _TSMaxY;
   uint8_t  _n, _x1, _y1, _h, _rotation, _textSize, _YP, _XM, _YM, _XP;
};
MyTftSettings myTftSettings;

class Commands{
  public:
    Commands(){
      ledOff      = "led off";
      ledOn       = "led on";
      buzzerOff   = "buzzer off";
      buzzerOn    = "buzzer on";
      stateLed    = false;
      stateBuzzer = false;
      endCommand  = '#';
      delay       = 10;
      delayString = "Delay";
    }

    String    getLedOff()                  {return ledOff;}
    String    getLedOn()                   {return ledOn;}
    String    getBuzzerOff()               {return buzzerOff;}
    String    getBuzzerOn()                {return buzzerOn;}
    bool      getStateBuzzer()             {return stateBuzzer;}
    void      setStateBuzzer(bool state)   {stateBuzzer = state;}
    bool      getStateLed()                {return stateLed;}
    void      setStateLed(bool state)      {stateLed = state;}  
    char      getEndCommand()              {return endCommand;}
    uint16_t  getDelay()                   {return delay;}
    void      setDelay(uint16_t number)    {delay = number;}
    String    getDelayString()             {return delayString;}
  private:
    String    ledOff, ledOn, buzzerOff, buzzerOn, delayString;
    bool      stateLed, stateBuzzer;
    char      endCommand;
    uint16_t  delay;
};
Commands commands;

RTC_DS1307 RTC;

iarduino_SensorPulse sensor_Pulse(A7);   // сенсор PulseSensor к 7 аналоговому входу. Допускается указать еще один вывод, в качестве второго параметра.

iarduino_SensorPulse sensor_AD8232(A10); // сенсор AD8232 к 10 аналоговому входу

MAX30105 particleSensor;                             //  объект для работы с библиотекой
  #define  MAX_BRIGHTNESS 255                         //  переменная максимальной яркости свечения светодиода
  //  32-битная переменная занимает в памяти 4 байта. Дальше используется 2 массива, каждый из которых содержит 100 значений.
  //  Если эти переменные будут 32-х битные, то они займут 100*4*2 = 800 байт. Arduino UNO имеет всего 2 Килобайт (2048 байт) памяти,
  //  поэтому при создании всех остальных переменных её объёма памяти не хватит. В связи с этим при запуске скетча выполняется проверка
  //  типа платы. Если это Arduino UNO (имеющая на борту микроконтроллер ATmega328), то будут созданы 16-битные массивы, чтобы занимать
  //  в 2 раза меньше места. Так как данные с сенсора считываются в 32-битном формате, то при записи в 16-битный массив эти значения
  //  будут автоматически обрезаны.
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  uint16_t irBuffer[100];                             //  16-битный массив данных от сенсора со значениями от ИК-светодиода
  uint16_t redBuffer[100];                            //  16-битный массив данных от сенсора со значениями от красного светодиода
  #else
  uint32_t irBuffer[100];                             //  32-битный массив данных от сенсора со значениями от ИК-светодиода
  uint32_t redBuffer[100];                            //  32-битный массив данных от сенсора со значениями от красного светодиода
  #endif

  int32_t bufferLength;                               //  длина буфера данных
  int32_t spo2;                                       //  значение SpO2 (насыщенности крови кислородом)
  int8_t  validSPO2;                                  //  флаг валидности значений сенсора по SpO2
  int32_t heartRate;                                  //  значение ЧСС
  int8_t  validHeartRate;                             //  флаг валидности значений сенсора по ЧСС

  //функция работы с сенсором пульса MAX30102 (ЧСС)
  const byte RATE_SIZE = 4;   //  Коэффициент усреднения. ЧЕм больше число, тем больше усреднение показаний.
  byte rates[RATE_SIZE];      //  Массив со значениями ЧСС
  byte rateSpot = 0;          //  Переменная с порядковым номером значения в массиве
  long lastBeat = 0;          //  Время последнего зафиксированного удара
  float beatsPerMinute;       //  Создаём переменную для хранения значения ЧСС
  int beatAvg;                //  Создаём переменную для хранения усреднённого значения ЧСС
  //  PARTICLE_SENSOR.setPulseAmplitudeRed(0x0A);         //  Выключаем КРАСНЫЙ светодиод для того, чтобы модуль начал работу
  //  PARTICLE_SENSOR.setPulseAmplitudeGreen(0);          //  Выключаем ЗЕЛЁНЫЙ светодиод
//

void drawButtonsMode() {
  uint8_t n=1;
  tft.fillScreen(myTftSettings.getBackEnd());
  while(n<=3){
    tft.setCursor(myTftSettings.getX1() + 7,
                  myTftSettings.getY1() + 40 * (n - 1) + 4);
    tft.drawRoundRect(
        myTftSettings.getX1(), 
        myTftSettings.getY1() + 40 * (n - 1) - 7,
        myTftSettings.getW(), 
        myTftSettings.getH(),
        myTftSettings.getRadius(),
        myTftSettings.getGroundScreen());
    tft.setTextColor(myTftSettings.getText(), myTftSettings.getGroundText());
    if(n==1) tft.print("1 - pulse");
    if(n==2) tft.print("2 - spo");  
    if(n==3) tft.print("3 - cardio");
    n++;
  }
}

void drawButtonExit() {
  tft.fillScreen(myTftSettings.getBackEnd());
  tft.setCursor(myTftSettings.getX1() + 7, myTftSettings.getY1() + 4 + 198);
  tft.drawRoundRect(myTftSettings.getX1(), 
      myTftSettings.getY1() - 7 + 200,
      myTftSettings.getW(), 
      myTftSettings.getH(),
      myTftSettings.getRadius(), 
      myTftSettings.getGroundScreen());
  tft.setTextColor(myTftSettings.getText(), myTftSettings.getGroundText());
  tft.print("Exit");
}

void setupTft() {
  //Serial.println(F("TFT LCD test"));
  uint16_t ID = tft.readID(); 
  tft.begin(ID);           
  //Serial.println(ID);
  //Serial.print("tft.width() = ");
  //Serial.print(tft.width());
  // Serial.print("    tft.height() = ");
  // Serial.println(tft.height());
  tft.setRotation(myTftSettings.getRotation()); // Landscape
  tft.setTextSize(myTftSettings.getTextSize()); // TextSize
  drawButtonsMode();
}

void setupRTC() {
  while(!RTC.begin()) {
     tft.setTextSize(3); tft.setCursor(80,60);  tft.print("E R R O R");
     tft.setTextSize(2); tft.setCursor(35,125); tft.print("RTC не подключен");        
  }
  RTC.adjust(DateTime(2018, 7, 30, 14, 41, 12));  // год, месяц, день, час, минута, секунда
}

void setupBluetooth() {
  uint8_t Rx = 19;       // RX со стороны Arduino 
  uint8_t Tx = 18;       // TX со стороны Arduino
  pinMode(Rx,INPUT); 
  pinMode(Tx,OUTPUT);
  Serial1.begin(9600);      //скорость передачи данных по Serial1(Bluetooth)
}

void setupSensorPulse() {
  sensor_Pulse.begin();
  pinMode(A7, INPUT);
}

void setupAD8232() {
  pinMode(10, INPUT);   // Setup for leads off detection LO +
  pinMode(11, INPUT);   // Setup for leads off detection LO -
  sensor_AD8232.begin();
}

void setupMAX30102() {
  while (!particleSensor.begin(Wire, I2C_SPEED_FAST)){  // инициируем работу с сенсором. Если этого не произошло, то // Use default I2C port, 400kHz speed                                             
    tft.setTextSize(3); tft.setCursor(80,60);  tft.print("E R R O R");
    tft.setTextSize(2); tft.setCursor(35,125); tft.print("MAX30102 not avaible");
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
  }
                            // настройка сенсора для работы в режиме определения насыщенности крови кислородом и определения ЧСС:
  byte ledBrightness  = 50;    //  Задаём яркость работы светодиода, при этом потребление тока будет следующим: 0 - 0мА, 255 - 50 мА
  byte sampleAverage  = 4;     //  Устанавливаем коэффициент усреднения. Возможные варианты значений: 1, 2, 4, 8, 16, 32
  byte ledMode        = 2;     //  Устанавливаем режим работы светодиодов на сенсоре: 1 - только красный (Red only), 2 - красный и ИК (Red + IR), 3 - красный, ИК и зелёный (Red + IR + Green)
  byte sampleRate     = 100;   //  Устанавливаем частоту дискретизации (сглаживания сигнала). Варианты: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int  pulseWidth     = 411;   //  Устанавливаем ширину импульса. Варианты: 69, 118, 215, 411
  int  adcRange       = 4096;  //  Устанавливаем диапазон значений с АЦП. Варианты: 2048(11 бит), 4096(12 бит), 8192(13 бит), 16384(14 бит)
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void setupBuzzer() {
  pinMode(A6, OUTPUT);    
  digitalWrite(A6,HIGH);
}

void setupLed() {
  pinMode(A8, OUTPUT);    
  digitalWrite(A8,LOW);
}

TSPoint wasTouch() { // функция нажатия
  TouchScreen ts = TouchScreen(myTftSettings.getXP(), 
                               myTftSettings.getYP(), 
                               myTftSettings.getXM(), 
                               myTftSettings.getYM(), 
                               myTftSettings.getRX());  
  TSPoint p = ts.getPoint();
  pinMode(myTftSettings.getXM(), OUTPUT);              //почему без этого не работает ???
  pinMode(myTftSettings.getYP(), OUTPUT);              //
  if (p.z > ts.pressureThreshhold)
    return p;
  return;
}

byte getButton(TSPoint p){                   // функция перевода координаты в номер кнопки
  p.y = map(p.y, myTftSettings.getTSMinY(), myTftSettings.getTSMaxY(), tft.width(), 0);
  byte P_Button = p.y/48;                     // размер кнопки
  //Serial.print("P_X = "); Serial.print("  =:=   "); Serial.println(P_Button);
  return P_Button;
}

bool touchExit(){
  if (getButton(wasTouch()) == 0)
    return true;
  return false;
}

void sendData(){
  Serial1.print("Temp:");
  Serial1.print(analogRead(A7));
  Serial1.print("|");
  Serial1.print("rand:");
  Serial1.println(random(100));  //Temp:65|rand:95   
}

bool equlsDelay(String string){
  if(string.indexOf(commands.getDelayString())!= -1)
    return true;
  return false;
}

uint16_t parseCommandDelay(String string){
  return (string.substring(0, string.indexOf(commands.getDelayString()))).toInt();
}

void executeCommand(String command) {
  if (command.equals(commands.getBuzzerOn())) {
    commands.setStateBuzzer(true);
  } else if (command.equals(commands.getBuzzerOff())) {
    commands.setStateBuzzer(false);
  } else if (command.equals(commands.getLedOn())) {
    commands.setStateLed(true);
  } else if (command.equals(commands.getLedOff())) {
    commands.setStateLed(false);
  } else if(equlsDelay(command)){
    commands.setDelay(parseCommandDelay(command));
  }
}

void readCommand(String &command){
    if (Serial1.available() > 0) {
    char symbol = Serial1.read();
    if (symbol == commands.getEndCommand()) {           // проверяем на наличие конца команды (#)
      executeCommand(command); // выполняем команду
      command = "";                                     // ...и сбрасываем переменую для хранения команд
    } else {
      command += symbol;                                // формируем команду прибавляя каждый символ
    }
  }
}

void drawPulsation(uint16_t graphY0, uint16_t graphY, uint16_t graphX, uint16_t screenH){
  tft.drawLine(graphX, 0, graphX, screenH, myTftSettings.getBackEnd());
  if(graphX>0){
    tft.drawLine(graphX, graphY0, graphX, graphY, myTftSettings.getGroundScreen());
    tft.drawLine(graphX-1,graphY0-1,graphX-1,graphY-1, myTftSettings.getGroundScreen()); // утолщаем линию
  }
}

void workingOutAdditionalFeatures(bool state){
  if(state){                          //провека режима, которого нужно установить
    if(commands.getStateBuzzer())     //проверка состояний доп. возможностей
      digitalWrite(A6,LOW);
    if(commands.getStateLed())
      digitalWrite(A8,HIGH);
  }else{
    if(commands.getStateBuzzer())
      digitalWrite(A6,HIGH);
    if(commands.getStateLed())
      digitalWrite(A8,LOW);
  }
}

void workingOutAdditionalFeaturesInThePulse(){
  if(sensor_Pulse.check(ISP_BEEP)==0)
    workingOutAdditionalFeatures(false);
  if(sensor_Pulse.check(ISP_BEEP)==2)
    workingOutAdditionalFeatures(true);
}

void drawHeartRateValue(){
  uint16_t Y = 215;
  uint16_t X = 270;
  uint16_t R = 4;
  if(sensor_Pulse.check(ISP_BEEP)==0)
    tft.fillRect(X-35, Y-13, 32, 30, myTftSettings.getBackEnd());     //clear BigHeart
    tft.drawBitmap(X-25, Y-8, pulseSmallPicture, 24, 21, myTftSettings.getGroundScreen());
  if(sensor_Pulse.check(ISP_BEEP)==1)
    tft.fillRect(X-25, Y-8, 24, 21, myTftSettings.getBackEnd());      //clear littleHeart
    tft.drawBitmap(X-35, Y-14, pulseBigPicture, 32, 32, myTftSettings.getGroundScreen());
  if(sensor_Pulse.check(ISP_BEEP)==2){
    tft.setCursor(X, Y-R);
    tft.fillRect(X, Y-R, 35, 15, myTftSettings.getBackEnd());         //clear number
    tft.print(sensor_Pulse.check(ISP_PULSE));
  }
}

void drawDisconnect(){
  tft.setCursor(120, 100);
  tft.print("DISCONNECT");
}

//для сенсора на TCRT1000
  int x=0;
  int lastx=0;
  int lasty=0;
  int LastTime=0;
  int ThisTime;
  bool BPMTiming=false;
  bool BeatComplete=false;
  int BPM;
  #define LowerThreshold 500    // нижний уровень сигнала пульса  (500-505 для круглого pulse sensor)
  #define UpperThreshold 950    // верхний уровень сигнала пульса (520 для круглого pulse sensor)
//

void pulse_TCRT1000() {                       //функция работы с сенсором пульса TCRT1000
  static uint8_t n,flg,flg1;
  static uint16_t bpm;  
  digitalWrite(A6,HIGH);             // выключение BUZZER 
  digitalWrite(A8,LOW);              // гашение индикации на BLUE светодиоде  

  ThisTime=millis();
  int value=analogRead(A7); 

  //y=60-(value/16);  // координата y для круглого pulse sensor
  //y=40-(value/16);
  
  // вычисление величины пульса (BMP)
  if(value>UpperThreshold){
    if(BeatComplete){                  // полный удар  
      BPM=ThisTime-LastTime;
      BPM=int(60/(float(BPM)/1000));
      BPMTiming=false;
      BeatComplete=false;
      digitalWrite(A6,LOW);          // включение BUZZER
      digitalWrite(A8,HIGH);         // включение индикации на BLUE светодиоде        
    }
    if(BPMTiming==false){
      LastTime=millis();
      BPMTiming=true;
    }
  }
  if((value<LowerThreshold)&(BPMTiming))
    BeatComplete=true;

  if(value<10) n++;
  else n=0;

  if(n>10) BPM=0;
    // вывод на LCD числового значения пульса
  if(BPM){
    bpm+=BPM;                           // накопление 20 результатов ненулевых замеров пульса
    flg++; 
  }
  if(flg1>100) flg=20;                  // обнуление пульса на экране если сигналов нет 
  if(flg>=20){                           // вычислениен усредненного пульса        
    BPM=bpm/20;
    Serial.print(BPM);
    flg1=flg=bpm=0;
  }  
  flg1++;
  _delay_ms(20);
}

void spo_MAX30102() {                         //функция работы с модулем MAX30102 (SPO2)
  bufferLength = 100;                                //  Устанавливаем длину буфера равным 100 (куда будут записаны пакеты по 25 значений в течении 4 секунд)
  //  считываем первые 100 значений и определяем диапазон значений сигнала:
  for (byte i = 0 ; i < bufferLength ; i++) {        //  проходим в цикле по буферу и
    while (particleSensor.available() == false)      //  отправляем сенсору запрос на получение новых данных
      particleSensor.check();
    redBuffer[i] = particleSensor.getRed();          //  Записываем в массив значения сенсора, полученные при работе с КРАСНЫМ светодиодом
    irBuffer[i]  = particleSensor.getIR();           //  Записываем в массив значения сенсора, полученные при работе с ИК      светодиодом
    particleSensor.nextSample();                     //  Как только в буфер было записано 100 значений - отправляем сенсору команду начать вычислять значения ЧСС и SpO2
    Serial.print(F("red="));                         //  Выводим текст в монитор последовательного порта
    Serial.print(redBuffer[i], DEC);                 //  Выводим значение переменной redBuffer[i] в монитор последовательного порта
    Serial.print(F(", ir="));                        //  Выводим текст в монитор последовательного порта
    Serial.println(irBuffer[i], DEC);                //  Выводим значение переменной irBuffer[i] в монитор последовательного порта
  }

  //  Вычисляем значения ЧСС и SpO2 по первым полученным 100 значениям:
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //  Непрерывно считываем значений с сенсора и вычисляем значения ЧСС и SpO2 каждую секунду
  while (1) {
    //  Сбрасываем первые полученные 25 значений из буфера, а оставшиеся 75 сдвигаем влево в массиве
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }
    //  Получаем новые 25 значений прежде чем переходить к вычислению ЧСС
    for (byte i = 75; i < 100; i++) {
      while (particleSensor.available() == false) {  //  Опрашиваем сенсор на предмет наличия новых значений
        particleSensor.check();
      }
      redBuffer[i] = particleSensor.getRed();        //  Записываем в массив значения сенсора, полученные при работе с КРАСНЫМ светодиодом
      irBuffer[i] = particleSensor.getIR();          //  Записываем в массив значения сенсора, полученные при работе с ИК      светодиодом
      particleSensor.nextSample();                   //  Как только в буфер было записано 100 значений - отправляем сенсору команду начать вычислять значения ЧСС и SpO2

      Serial.print(F("red="));                       //  Выводим текст в монитор последовательного порта
      Serial.print(redBuffer[i], DEC);               //  Выводим значение переменной   redBuffer[i]   в монитор последовательного порта
      Serial.print(F(", ir="));                      //  Выводим текст в монитор последовательного порта
      Serial.print(irBuffer[i], DEC);                //  Выводим значение переменной   irBuffer[i]    в монитор последовательного порта
      Serial.print(F(", HR="));                      //  Выводим текст в монитор последовательного порта
      Serial.print(heartRate, DEC);                  //  Выводим значение переменной   heartRate      в монитор последовательного порта
      Serial.print(F(", HRvalid="));                 //  Выводим текст в монитор последовательного порта
      Serial.print(validHeartRate, DEC);             //  Выводим значение переменной   validHeartRate в монитор последовательного порта
      Serial.print(F(", SPO2="));                    //  Выводим текст в монитор последовательного порта
      Serial.print(spo2, DEC);                       //  Выводим значение переменной   spo2           в монитор последовательного порта
      Serial.print(F(", SPO2Valid="));               //  Выводим текст в монитор последовательного порта
      Serial.println(validSPO2, DEC);                //  Выводим значение переменной   validSPO2      в монитор последовательного порта
    }
    //  После получения очередного пакета из 25 значений повторно считаем значения ЧСС и SpO2
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}  

void cardio_AD8232() {                        // функция работы с модулем AD8232 
  uint16_t graphY0  = 0;                              // положение предыдущей точки графика по оси Y
  uint16_t graphY   = 0;                               // положение текущей    точки графика по оси Y
  uint16_t graphX   = 0;                               // положение текущей    точки графика по оси X
  uint16_t screenW  = tft.width() - 1;            // ширина дисплея
  uint16_t screenH  = tft.height() - 1 - 48;      // высота дисплея
  while(!touchExit()){
    graphX++; 
    if(graphX>=screenW)
      graphX=0;                
    graphY = map(analogRead(A10),4096,0,0,screenH); // определяем точку графика по оси Y
    if(!(digitalRead(10) == 1)||(digitalRead(11) == 1)){
      drawPulsation(graphY0, graphY, graphX, screenH);
    }else{
      tft.fillScreen(myTftSettings.getBackEnd());
      drawButtonExit();
      drawDisconnect();
      graphX=0;  
    }
    graphY0 = graphY;
    delay(commands.getDelay());
  }
  drawButtonsMode();
}

void pulse_MAX30102() {                       // функиця работы с модулем MAX30102 (pulse)
  long irValue = particleSensor.getIR();               //  Считываем значение отражённого ИК-светодиода (отвечающего за пульс) и
  if (checkForBeat(irValue) == true) 
  {                  //  если пульс был зафиксирован, то
    long delta = millis() - lastBeat;                   //  находим дельту по времени между ударами
    lastBeat = millis();                                //  Обновляем счётчик
    beatsPerMinute = 60 / (delta / 1000.0);             //  Вычисляем количество ударов в минуту
    if (beatsPerMinute < 255 && beatsPerMinute > 20)    //  Если количество ударов в минуту находится в промежутке между 20 и 255, то
    { rates[rateSpot++] = (byte)beatsPerMinute;         //  записываем это значение в массив значений ЧСС
      rateSpot %= RATE_SIZE;                            //  Задаём порядковый номер значения в массиве, возвращая остаток от деления и присваивая его переменной rateSpot
      beatAvg = 0;                                      //  Обнуляем переменную и
      for (byte x = 0 ; x < RATE_SIZE ; x++)            //  в цикле выполняем усреднение значений (чем больше RATE_SIZE, тем сильнее усреднение)
         beatAvg += rates[x];                           //  путём сложения всех элементов массива
      beatAvg /= RATE_SIZE;                             //  а затем деления всей суммы на коэффициент усреднения (на общее количество элементов в массиве)
    }
  }
  Serial.print("IR=");                                  //  Выводим в монитор последовательного порта текст про значение ИК-светдиода
  Serial.print(irValue);                                //  Выводим в монитор последовательного порта значение с ИК-светодиода
  Serial.print(", BPM=");                               //  Выводим в монитор последовательного порта текст про значение ЧСС
  Serial.print(beatsPerMinute);                         //  Выводим в монитор последовательного порта значение ЧСС
  Serial.print(", Avg BPM=");                           //  Выводим в монитор последовательного порта текст про усреднённую ЧСС
  Serial.print(beatAvg);                                //  Выводим в монитор последовательного порта значение усреднённой ЧСС
  if (irValue < 50000)  Serial.print(" No finger?");    //  Если значение ИК-светодиода меньше указанного, то выводим текст о том, что палец убран с датчика
  Serial.println();    
  delay(10);
}

// Volatile Variables, used in the interrupt service routine!
//volatile int BPM;           // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;              // holds the incoming raw data
volatile int IBI = 600;     // int that holds the time interval between beats! Must be seeded! 
volatile boolean Pulse = false;     // "True" when heartbeat is detected. "False" when not a "live beat". 
volatile boolean QS = false;        // becomes true when Arduino finds a beat.

// Regards Serial OutPut  -- Set This Up to your needs
static boolean serialVisual = true;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse 
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P = 512;                  // used to find peak in pulse wave, seeded
volatile int T = 512;               // used to find trough in pulse wave, seeded
volatile int thresh = 525;  // used to find instant moment of heart beat, seeded
volatile int amp = 100;     // used to hold amplitude of pulse waveform, seeded
volatile boolean firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile boolean secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

void ulse(){
  Signal = analogRead(A7);              // read the Pulse Sensor 
  sampleCounter += 2;                         // keep track of the time in mS
  int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise
    //  find the peak and trough of the pulse wave
  if(Signal < thresh && N > (IBI/5)*3){      // avoid dichrotic noise by waiting 3/5 of last IBI
    if (Signal < T){                         // T is the trough
      T = Signal;                            // keep track of lowest point in pulse wave 
    }
  }
  if(Signal > thresh && Signal > P){        // thresh condition helps avoid noise
    P = Signal;                             // P is the peak
  }                                         // keep track of highest point in pulse wave
  
  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
  // signal surges up in value every time there is a pulse
  if (N > 250){                                   // avoid high frequency noise
    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){        
      Pulse = true;                               // set the Pulse flag when there is a pulse
      digitalWrite(A8,HIGH);                // turn on pin 13 LED
      IBI = sampleCounter - lastBeatTime;         // time between beats in mS
      lastBeatTime = sampleCounter;               // keep track of time for next pulse
      if(secondBeat){                        // if this is the second beat
        secondBeat = false;                  // clear secondBeat flag
        for(int i=0; i<=9; i++){             // seed the running total to get a realistic BPM at startup
          rate[i] = IBI;                      
        }
      }
      if(firstBeat){                         // if it's the first time beat is found
        firstBeat = false;                   // clear firstBeat flag
        secondBeat = true;                   // set the second beat flag
        //sei();                               // enable interrupts again
        return;                              // IBI value is unreliable so discard it
      }   
      word runningTotal = 0;                  // clear the runningTotal variable    
      for(int i=0; i<=8; i++){                // shift data in the rate array
        rate[i] = rate[i+1];                  // and drop the oldest IBI value 
        runningTotal += rate[i];              // add up the 9 oldest IBI values
      }
      rate[9] = IBI;                          // add the latest IBI to the rate array
      runningTotal += rate[9];                // add the latest IBI to runningTotal
      runningTotal /= 10;                     // average the last 10 IBI values 
      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
      QS = true;                              // set Quantified Self flag 
      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
      Serial.println(BPM);
    }                       
  }
  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
    digitalWrite(A8,LOW);            // turn off pin 13 LED
    Pulse = false;                         // reset the Pulse flag so we can do it again
    amp = P - T;                           // get amplitude of the pulse wave
    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
    P = thresh;                            // reset these for next time
    T = thresh;
  }
  if (N > 2500){                           // if 2.5 seconds go by without a beat
    thresh = 512;                          // set thresh default
    P = 512;                               // set P default
    T = 512;                               // set T default
    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date        
    firstBeat = true;                      // set these to avoid noise
    secondBeat = false;                    // when we get the heartbeat back
  }
 // delay(200);
}

void pulse(){
  uint16_t graphY0  = 0;                              // положение предыдущей точки графика по оси Y
  uint16_t graphY   = 0;                               // положение текущей    точки графика по оси Y
  uint16_t graphX   = 0;                               // положение текущей    точки графика по оси X
  uint16_t screenW  = tft.width() - 1;            // ширина дисплея
  uint16_t screenH  = tft.height() - 1 - 48;      // высота дисплея
  String command;
  while(1){//!touchExit()
    graphX++; 
    readCommand(command);
    if(graphX>=screenW)
      graphX=0;                
    graphY = map(analogRead(A7),1024,0,0,screenH);//sensor_Pulse.check(ISP_ANALOG)
    if(false){//sensor_Pulse.check(ISP_VALID)==ISP_CHANGED
      tft.fillScreen(myTftSettings.getBackEnd());
      drawButtonExit();
      graphX=0;                                             
    }
    if(true){//sensor_Pulse.check(ISP_VALID)==ISP_CONNECTED
      sendData();
      drawPulsation(graphY0, graphY, graphX, screenH);
      drawHeartRateValue();
      workingOutAdditionalFeaturesInThePulse();
    }else{
      drawDisconnect();
    }
    graphY0 = graphY;
    delay(commands.getDelay());
  }
  drawButtonsMode();
}

void selectMode(byte button){                //функиця выбора режима работы
  switch (button)     
  {
  case 5:  // Если нажата кнопка "pulse"
    drawButtonExit();
    pulse();
    //pulse();
    break;
  case 4:  // Если нажата кнопка "spo"
    drawButtonExit();
    //spo_MAX30102();
    break;  
  case 3:  // Если нажата кнопка "cardio"
    drawButtonExit();
    //cardio_AD8232();
    break;       
  }
}

void setup(){
  Serial.begin(9600);
  setupTft();
  //setupRTC(); 
  setupLed();
  setupBuzzer();
  setupBluetooth();
  setupSensorPulse();
  //setupAD8232();                          
  //setupMAX30102();
  //prevTime = millis();
}

void loop() { selectMode(getButton(wasTouch())); }

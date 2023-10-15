#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <QTRSensors.h>
#include <ESP32Servo.h>

//:d
#define Servo1Pin D9
#define Servo2Pin D10
#define Servo4Pin D11
#define Servo3Pin D12

int Servo1_Min = 0;
int Servo1_Max = 180;
int Servo2_Min = 0;
int Servo2_Max = 180;
int Servo3_Min = 0;
int Servo3_Max = 180;
int Servo4_Min = 0;
int Servo4_Max = 180;

Servo Servo1;
Servo Servo2;
Servo Servo3;
Servo Servo4;

int Pot1Deger;
int Servo1Pozisyon;
int Pot2Deger;
int Servo2Pozisyon;
int Pot3Deger;
int Servo3Pozisyon;
int Pot4Deger;
int Servo4Pozisyon;


#define L_MOT_DIR1 D15
#define L_MOT_DIR2 D14

#define R_MOT_DIR1 D13
#define R_MOT_DIR2 D12
#define PWM_FREQ 5000 // PWM frekansi
#define LPWM_CH 0     // PWM kanali (0-15)
#define RPWM_CH 1
#define PWM_RES 8 // PWM cozunurlugu

#define LMOT_PIN PWM0
#define RMOT_PIN PWM1

#define DEADZONE 34
#define TURN_DEADZONE 34
#define checkNegative(x) ((x) > 0 ? 0 : 1)
#define checkPositive(x) ((x) > 0 ? 1 : 0)

#include <esp_now.h>
#include <WiFi.h>

uint8_t broadcastAddress[] = {0x7C, 0xDF, 0xA1, 0x93, 0x44, 0x3A};

esp_now_peer_info_t peerInfo;

int readingX = 2;
int readingY = 2;

int rightSpeed;
int leftSpeed;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
  }
}


typedef struct struct_joystick{
  int lx_axis;
  int ly_axis;
  bool l_switch;
    
  int rx_axis;
  int ry_axis_kol;
  bool r_switch;
    
  bool xbutton;
  bool ybutton;
  bool abutton;
  bool bbutton;
  bool manualbutton;
  
} struct_joystick;
struct_joystick joystickreading;


typedef struct struct_NSstatus
{
  bool manual;
} struct_NSstatus;
struct_NSstatus NSstatusReading;

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&joystickreading, incomingData, sizeof(joystickreading));

  NSstatusReading.manual = joystickreading.manualbutton;

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &NSstatusReading, sizeof(NSstatusReading));
}

#define Kp 0.4 // Kendi deneyimlerinizle bulmanız gerekli küçük bir değer ile başlayıp, büyütebilirsiniz en kararlı Kp değerini bulana kadar.. 0.4
#define Kd 2   // Bunu da küçük bir değerden başlatın ve deneyimleyerek büyütün. ( Not: Kp < Kd) 2.0
#define rightMaxSpeed 127
#define leftMaxSpeed 127
#define rightBaseSpeed 100 // robot için kp ve kd değerlerini tutturduysanız şayet motorların dönmesi gereken hız budur
#define leftBaseSpeed 100  // yukarıdaki değer ile aynıdır

QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup()
{
  Servo1.attach(Servo1Pin);
  Servo2.attach(Servo2Pin);
  Servo3.attach(Servo3Pin);
  Servo4.attach(Servo4Pin);

  Serial.begin(9600);

  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  int i;
  for (int i = 0; i < 100; i++)
  { // tercihe bağlı alandır ya robotunuzu hatta yürüterek kalibre edin veya bunu otomatik yapın.
    // otomatik kalibrasyon için burasının yorumunu kaldırın
    if (i < 25 || i >= 75)
    { // sensörleri, karşılaşılabilecek en aydınlık ve en karanlık okumalara maruz bırakmak için sola ve sağa çevirin.
      // saga dön
    }
    else
    {
      // sola dön
    }
    qtr.calibrate();
    delay(20);
  }
  // motorları durdur
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  delay(1000); // Ana döngüye girmeden önce botu konumlandırmak için 1 saniye bekleyin



ledcSetup(LPWM_CH, PWM_FREQ, PWM_RES);
ledcAttachPin(LMOT_PIN, LPWM_CH);

ledcSetup(RPWM_CH, PWM_FREQ, PWM_RES);
ledcAttachPin(RMOT_PIN, RPWM_CH);

// put your setup code here, to run once:
pinMode(L_MOT_DIR1, OUTPUT); // Motor yon secme pin1 cikis olarak ayarlandi
pinMode(L_MOT_DIR2, OUTPUT);
pinMode(R_MOT_DIR1, OUTPUT);
pinMode(R_MOT_DIR2, OUTPUT);

WiFi.mode(WIFI_STA);

// Init ESP-NOW
if (esp_now_init() != ESP_OK)
{
  Serial.println("Error initializing ESP-NOW");
  return;
}

// Once ESPNow is successfully Init, we will register for Send CB to
// get the status of Trasnmitted packet
esp_now_register_send_cb(OnDataSent);

// Register peer
memcpy(peerInfo.peer_addr, broadcastAddress, 6);
peerInfo.channel = 0;
peerInfo.encrypt = false;

// Add peer
while (esp_now_add_peer(&peerInfo) != ESP_OK)
{
  Serial.println("Failed to add peer");
  delay(100);
}
// Register for a callback function that will be called when data is received
esp_now_register_recv_cb(OnDataRecv);
}

void setSpeed()
{
  digitalWrite(L_MOT_DIR1, checkNegative(leftSpeed)); // motor ileri yönde hareket ediyor
  digitalWrite(L_MOT_DIR2, checkPositive(leftSpeed));
  digitalWrite(R_MOT_DIR1, checkNegative(rightSpeed)); // motor ileri yönde hareket ediyor
  digitalWrite(R_MOT_DIR2, checkPositive(rightSpeed));

  ledcWrite(LPWM_CH, constrain(abs(leftSpeed) * 2, 0, 255));
  ledcWrite(RPWM_CH, constrain(abs(rightSpeed) * 2, 0, 255));
}
void motorGo(int speed) {
    rightSpeed = speed;
    leftSpeed = speed;
// 0-128
    setSpeed();
}
unsigned int duzcizgi=0;
void kopruprocess() {
    motorGo(128);
    delay(2000);
    motorGo(0);
    // robot kol ileri
    // robot kol geri
    motorGo(-128);
    delay(2000);
    /*
    while(duzcizgi<5000) {
        motorGo(-128);
    }
    */
    motorGo(0);
}

int lastError = 0;
int otonom=1;


void loop()
{
  otonom = joystickreading.manualbutton;
  if(otonom==1) {

  
   // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 7000 (for a white line, use readLineWhite() instead)
  unsigned int position = qtr.readLineBlack(sensorValues);
  int kopruoldu = 0;
  duzcizgi = sensorValues[0]+sensorValues[1]+sensorValues[2]+sensorValues[3]+sensorValues[4];
  if(duzcizgi>=5000 && kopruoldu==0) {
    kopruprocess();
    kopruoldu = 1;
  }
  else {

  int error = (position - 2500);
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = rightBaseSpeed - motorSpeed;
  int leftMotorSpeed = leftBaseSpeed + motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed;
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed;
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;
  

    //digitalWrite(rightMotor1, HIGH);
    //digitalWrite(rightMotor2, LOW);
    //analogWrite(rightMotorPWM, rightMotorSpeed);
    
    Serial.print("right:");
    Serial.println(rightMotorSpeed);

    //digitalWrite(leftMotor1, HIGH);
    //digitalWrite(leftMotor2, LOW);
    //analogWrite(leftMotorPWM, leftMotorSpeed);
    rightSpeed = leftMotorSpeed;
    leftSpeed = rightMotorSpeed;

    setSpeed();
    Serial.print("left:");
    Serial.println(leftMotorSpeed);
  }

  }
  else { 
  Pot1Deger  = joystickreading.rx_axis;
  Pot2Deger  = joystickreading.ry_axis_kol;

  int posX=90;
  int posY=90;
  int posZ=90;

  if(Pot1Deger>8000) {
        posX++;
        if(posX>180) posX=180;
  }
  if(Pot1Deger<1000) {
        posX--;
        if(posX<0) posX=0;
  }
  if(Pot2Deger>8000) {
        posY++;
        if(posY>180) posY=180;
  }
  if(Pot2Deger<1000) {
        posY--;
        if(posY>0) posY=0;
  }

  if(joystickreading.xbutton)
  {
    posZ--;
    if(posZ>0) posZ=0;
  }
  else if(joystickreading.bbutton)
  {
      posZ++;
      if(posZ>180) posZ=180;
  }

  if(joystickreading.abutton) {
    Servo4.write(180);
  }
  else if(joystickreading.ybutton) {
    Servo4.write(0);
  }

  Servo1Pozisyon  = posX;
  Servo2Pozisyon  = posZ;
  Servo3Pozisyon  = posY;
  //Servo4Pozisyon  = posY;

  Servo1.write(Servo1Pozisyon);
  Servo2.write(Servo2Pozisyon);
  Servo3.write(Servo3Pozisyon);
  //Servo4.write(Servo4Pozisyon);


  // put your main code here, to run repeatedly:
  readingX = map(joystickreading.lx_axis, 0, 8191, -128, 128);
  readingY = map(joystickreading.ly_axis, 0, 8191, -128, 128);

  // Serial.print("x:");
  // Serial.println(readingX);

  // Serial.print("y:");
  // Serial.println(readingY);

  if (abs(readingX) > TURN_DEADZONE)
  {
    if (abs(readingX) > 100 || abs(readingY) > 115)
    {
      rightSpeed = (int)((float)((float)(readingX * readingY) / (float)-128.0));
      leftSpeed = (int)((float)(((float)(readingX * readingY) / (float)128.0)));
    }
    else
    {
      rightSpeed = -readingX;
      leftSpeed = readingX;
    }

    setSpeed();
  }
  else if (abs(readingY) > DEADZONE)
  {
    rightSpeed = readingY * 2;
    leftSpeed = readingY * 2;

    setSpeed();
  }
  else
  {
    // Serial.println("Stopoped");
    ledcWrite(LPWM_CH, 0);
    ledcWrite(RPWM_CH, 0);
  }
  // Serial.println();
  // Serial.print("Left speed");
  // Serial.print(leftSpeed);
  // Serial.print("Right speed");
  // Serial.println(rightSpeed);

    delay(100);
}
}

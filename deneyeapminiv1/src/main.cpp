#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>


float lx_axis;
float ly_axis;

float l_switch;

#define X_pin A0 // analog pin connected to X output
#define Y_pin A1 // analog pin connected to Y output

#define X_pin_1 A2 // analog pin connected to X output
#define Y_pin_1 A3 // analog pin connected to Y output

uint8_t broadcastAddress[] = {0xD4, 0xD4, 0xDA, 0x77, 0x4D, 0x30};


typedef struct struct_joystick{
  int lx_axis;
  int ly_axis;
  bool l_switch;
    
  int rx_axis;
  int ry_axis_kol;
  bool r_switch;
  
} struct_joystick;
struct_joystick joystickreading;


typedef struct struct_NSstatus{
  bool manual;
} struct_NSstatus;

struct_NSstatus NSstatusReading;

esp_now_peer_info_t peerinfo;
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status==0) {
    //başarılı
  }
}

void OnDataReceive(const uint8_t * mac_addr, uint8_t * incoming_data, int len) {
  memcpy(&NSstatusReading, incoming_data, sizeof(NSstatusReading));
}

// Arduino pin numbers



void setup() {
  pinMode(X_pin, INPUT);
  pinMode(Y_pin, INPUT);

  pinMode(X_pin_1, INPUT);
  pinMode(Y_pin_1, INPUT);
    
  //digitalWrite(SW_pin, HIGH);
  Serial.begin(9600);


  WiFi.mode(WIFI_STA);
  if(esp_now_init()!=ESP_OK) {
    Serial.println("error");
    return;
  }
    esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerinfo.peer_addr, broadcastAddress, 6);
  peerinfo.channel = 0;  
  peerinfo.encrypt = false;
  
  // Add peer        
  while (esp_now_add_peer(&peerinfo) != ESP_OK){
    Serial.println("Failed to add peer");
    delay(100);
  }
}


void loop() {

  joystickreading.lx_axis = analogRead(X_pin);
  joystickreading.ly_axis = analogRead(Y_pin);
    
  joystickreading.rx_axis = analogRead(X_pin_1);
  joystickreading.ry_axis_kol = analogRead(Y_pin_1);
    
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &joystickreading, sizeof(joystickreading));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(200);
}
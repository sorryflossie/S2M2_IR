// Include the Mona_ESP library
// Right: 85.71, Left: 85.71 RPM 
#include <Wire.h>
#include "Mona_ESP_lib.h"
#include <ESP32Encoder.h> // https://github.com/madhephaestus/ESP32Encoder.git 

// using Wifi and UDP
#include <WiFi.h>
#include <WiFiUdp.h>

#include <esp_now.h>

#define PACKET_SIZE 1460 // Can increase this with kconfig
#define UDP_PORT 54007
#define SERIAL_USB_BAUD 1000000

ESP32Encoder right_encoder;
ESP32Encoder left_encoder;


//centralhub mac: 8C:CE:4E:BB:4C:78

// WiFi network name and password:
const char* networkName = "MSc_IoT"; // replace with your network id
const char* networkPswd = "MSc_IoT@UCL"; // replace with your network password

// IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const int udpPort = UDP_PORT;

// Are we currently connected?
boolean connected = false;

// The UDP library class
WiFiUDP udp;
unsigned char packet[PACKET_SIZE];

int right_new_pos, right_old_pos, left_new_pos, left_old_pos;

float theta_d = 0.0, theta_f = 0.0;
float right_vel = 0.0, left_vel = 0.0, right_ref_vel = 0.0, left_ref_vel = 0.0;

char Bot_Id[32];
float X = 0.0, Y = 0.0, Z = 0.0, Orientation = 0.0, X_d = 0.0, Y_d = 0.0, Z_d = 0.0;
bool done = false;


// set sample time
float Ts = 0.01;
float start_time, current_time, elapsed_time;
const float rad2deg = 57.27;
const float deg2rad = -0.01745328;

// Wheel radius is 15mm, every revolute is 3500 pulse
const float robot_radius = 40;
const float wheel_radius = 15;
const float scale_encoder = 0.0043;    // 15/3500
const float pi = 3.14159; 


// initialise variables
float err = 0;
float control_theta=0, control_lin=0; 
float control_R=0, control_L=0, control = 0; 
float intError = 0;
bool read_flag = true;


typedef struct struct_message {
  char ID[32];              // 标识符
  float position[3];   // 位置，包含X, Y, Z坐标
  float headingY;// Y方向的朝向
  float position_d[3];  //目标地址    
} struct_message;

struct_message myData;

bool receive_flag = false;

bool phaseTwo = false; // flag for phase control

float stopThreshold = 20; 



void setup(){
  Mona_ESP_init();
  Serial.begin(1000000);

  // Connect to network
  //connectToWiFi(networkName, networkPswd);

  //激活WIFI基站模式
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);


  right_encoder.attachHalfQuad ( Mot_right_feedback, Mot_right_feedback_2 );
	left_encoder.attachHalfQuad( Mot_left_feedback_2, Mot_left_feedback );
	
	// clear starting values
	right_encoder.clearCount();
	left_encoder.clearCount();
	
	// set the lastToggle
	current_time = millis();
  // set the initial positions
  right_old_pos = 0;
  left_old_pos = 0;

  delay(5000);
}

void connectToWiFi(const char* ssid, const char* pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  // delete old config
  WiFi.disconnect(true);
  // register event handler
  WiFi.onEvent(WiFiEvent);

  // Initiate connection
  WiFi.begin(ssid, pwd);

  Serial.println("Waiting for WIFI connection...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }
}

// Wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // Initializes the UDP state
      // This initializes the transfer buffer
      udp.begin(WiFi.localIP(), udpPort);
      connected = true;
      WiFi.setTxPower(WIFI_POWER_19_5dBm);
      // When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      udp.stop();
      udp.flush();
      connected = false;
      break;
    default: break;
  }
}

void disp(){
  Serial.print("Mona :");
  Serial.print(Bot_Id);
  Serial.print("x :");
  Serial.print(X);
  Serial.print("\t z :");
  Serial.print(Z);
  Serial.print("\t theta :");
  Serial.print(Orientation);
  Serial.print("\t theta_d :");
  Serial.print(theta_d);
  Serial.print("\t Control Theta :");
  Serial.print(control_theta);
  Serial.print("\t Control XY :");
  Serial.print(control_lin);
  Serial.print("\t Right_ref :");
  Serial.print(right_ref_vel);
  Serial.print("\t Right vel :");
  Serial.print(right_vel);
  Serial.print("\t Left_ref :");
  Serial.print(left_ref_vel);
  Serial.print("\t Left_vel :");
  Serial.print(left_vel);
  Serial.print("\t CntR :");
  Serial.print(control_R);
  Serial.print("\t CntL :");
  Serial.print(control_L);
  Serial.print("\t X_d :");
  Serial.print(X_d);
  Serial.print("\t Z_d :");
  Serial.print(Z_d);
  Serial.println();
}

//void OnDataRecv(const esp_now_recv_info_t* info, const uint8_t *incomingData, int len) {
void OnDataRecv(const uint8_t *mac,const uint8_t *incomingData, int len) {
    if (len == sizeof(struct_message)) {
        memcpy(&myData, incomingData, sizeof(myData));
        receive_flag=true;

      strcpy(Bot_Id, myData.ID);
      X=myData.position[0];
      Y=myData.position[1];
      Z=myData.position[2];
      Orientation=normalizeAngle(myData.headingY*deg2rad);
      X_d=myData.position_d[0];
      Y_d=myData.position_d[1];
      Z_d=myData.position_d[2];

      Serial.print("Bot_Id:");
      Serial.print(Bot_Id);
      Serial.print(" ");

      Serial.print("X: ");
      Serial.print(X);
      Serial.print(" ");

      Serial.print("Y: ");
      Serial.print(Y);
      Serial.print(" ");

      Serial.print("Z: ");
      Serial.print(Z);
      Serial.print(" ");

      Serial.print("Orientation:");
      Serial.print(Orientation);
      Serial.print(" ");

      Serial.print("X_d:");
      Serial.print(X_d);
      Serial.print(" ");

      Serial.print("Y_d:");
      Serial.print(Y_d);
      Serial.print(" ");

      Serial.print("Z_d:");
      Serial.println(Z_d);
      Serial.print(" ");
    } else {
        Serial.println("Received data size does not match!");
    }
}


void position(){
  // int packetSize = udp.parsePacket();
  // // Received packets go to buffer
  // if (packetSize > 0) {
  //   int len = udp.read(packet, packetSize);

  //   unsigned char* readBuffer = &(packet[0]);
  //   memcpy((void*)&Bot_Id, (void*)readBuffer, sizeof(int)); readBuffer += sizeof(int);
  //   memcpy((void*)&X, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //   memcpy((void*)&Y, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //   memcpy((void*)&Z, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //   memcpy((void*)&Orientation, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);

  //   if (read_flag == true) {
  //     memcpy((void*)&X_d, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //     memcpy((void*)&Y_d, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //     memcpy((void*)&Z_d, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //     memcpy((void*)&theta_f, (void*)readBuffer, sizeof(float)); readBuffer += sizeof(float);
  //     read_flag = false;
  //   }
  //   Orientation = deg2rad * Orientation;
  // }
}
void velocity(){
  elapsed_time = (millis()-current_time)/1000;    // elapsed time in seconds
  right_new_pos = right_encoder.getCount();
  left_new_pos = left_encoder.getCount();
  right_vel = 2*pi*((right_new_pos-right_old_pos)/(3500*elapsed_time));
  left_vel = 2*pi*((left_new_pos-left_old_pos)/(3500*elapsed_time));
  right_old_pos = right_new_pos;
  left_old_pos = left_new_pos;
}

float angleDifference(float angle1, float angle2) {
    float diff = normalizeAngle(angle1 - angle2);
    if (diff > PI) {
        diff -= 2 * PI;
    } else if (diff < -PI) {
        diff += 2 * PI;
    }
    return diff;
}
void Propotional(float ref, float act, float Kp){
  //err = (ref-act);
  err = (ref-act);
  control = Kp*err;
  //if (control<-64){
  //  control = -64;
  //}
  control = bound(control, -150, 150);
}
void Propotional_theta(float ref, float act, float Kp){
  err = angleDifference(ref,act);
  control_theta = Kp*err;

  // err = normalizeAngle(ref - act);
  // // 使用幂函数调整误差，p为幂的指数，根据需要选择合适的p值
  // float p = 4; // 举例使用0.5，这会使得大误差的影响减弱
  // // 保证err和其幂次结果同号，使用pow函数计算幂次
  // float adjusted_err = pow(fabs(err), p) * (err >= 0 ? 1 : -1);
  // control_theta = Kp * adjusted_err;
}
void Propotional_XY(float ref_x, float ref_y, float act_x, float act_y,float Kp){
  float a = (ref_x-act_x)*(ref_x-act_x);
  float b = (ref_y-act_y)*(ref_y-act_y);
  err = sqrt(a+b);
  control_lin = Kp*err;

  // float a = (ref_x-act_x)*cos(Orientation);
  // float b = (ref_y-act_y)*sin(Orientation);
  // err = abs(a+b);
  // control_lin = Kp*err;

}
//void PI(float ref, float act, float Kp, float Ki){
//  err = (ref-act);
//  intError += err * Ts;
//  intError = bound(intError, -128, 128);
//  control = Kp*err+Ki*intError;
//}
float bound(float x, float x_min, float x_max){
  if(x < x_min){x = x_min;}
  if(x > x_max){x = x_max;}
  return x;
}

void dif_inverse(float Vb, float dot_theta, float&w_r, float&w_l){
  w_r = 0.0667*Vb+2.6667*dot_theta;
  w_l = 0.0667*Vb-2.6667*dot_theta;
}

float normalizeAngle(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

void loop(){
  if (true) {
    //position();
    velocity();
    current_time = millis();
     //theta_d =-normalizeAngle(atan2(Z_d-Z, X_d-X));
    theta_d = -normalizeAngle(atan2(X_d - X, Z_d - Z));
    
    //Propotional_theta(theta_d, Orientation, -0.1);
    
    
   
    float distanceToTarget = sqrt(pow(X_d - X, 2) + pow(Z_d - Z, 2));
    
    if (phaseTwo) {
        if (distanceToTarget > stopThreshold) {
            // Propotional_XY(X_d, Z_d, X, Z, 0.2);
            Propotional_XY(X_d, Z_d, X, Z, 0.01);
            Propotional_theta(theta_d, Orientation, -0.025);
        } else {
            //stop
            control_lin = 0;
            control_theta = 0;
            done=true;
        }
    } else {
        // phase one:turning
        if (abs(normalizeAngle(theta_d - Orientation)) < 0.1) { 
            phaseTwo = true;
            // Propotional_XY(X_d, Z_d, X, Z, 0.2);
        } else {
            Propotional_theta(theta_d, Orientation, -0.1);
            control_lin = 0.01;
        }
    }

    // 执行控制逻辑
    dif_inverse(control_lin, control_theta, right_ref_vel, left_ref_vel);
    Control_law();
    
    disp();
    //receive_flag = false;
    delay(Ts * 1000);
}
}

void Control_law()
{
// if(abs(control_lin)<0.01 && abs(control_theta<0.01)){
//  done = true;
// }
if (done==false){
  if (right_ref_vel<0){
      Propotional(-right_ref_vel, -right_vel,-75/right_ref_vel);
      control_R = control+25;
      Right_mot_backward(control_R);
      Serial.print("right down: ");
      Serial.println(control_R);
  }
  else{
    Propotional(right_ref_vel, right_vel,75/right_ref_vel);
    control_R = control+25;
    Right_mot_forward(control_R);
    Serial.print("right up: ");
    Serial.println(control_R);
  }
  if (left_ref_vel<0){
    Propotional(-left_ref_vel,-left_vel,-75/(left_ref_vel));
    control_L = control+25;
    Left_mot_backward(control_L);
    Serial.print("left down: ");
    Serial.println(control_L);
  }
  else{
    Propotional(left_ref_vel,left_vel,75/(left_ref_vel));
    control_L = control+25;
    Left_mot_forward(control_L);
    Serial.print("left up: ");
    Serial.println(control_L);
  }
}
else{
    Motors_stop();
    delay(2000);
}
}

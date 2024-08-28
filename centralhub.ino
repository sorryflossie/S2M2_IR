// #include <esp_now.h>
// #include <WiFi.h>
// #include <cstring>

// // REPLACE WITH YOUR RECEIVER MAC Address
// uint8_t broadcastAddress[] = {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x28};//(mona5)
// // uint8_t broadcastAddresses[2][6] = {
// //     {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x28}, // mona5
// //     {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x40}  // mona6
// // };




// typedef struct struct_message {
//   char ID[32];              // 标识符
//   float position[3];   // 位置，包含X, Y, Z坐标
//   float headingY;// Y方向的朝向
//   float position_d[3];  //目标地址    
// } struct_message;



// // Create a struct_message called myData
// struct_message myData;

// esp_now_peer_info_t peerInfo;

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//   Serial.print("\r\nLast Packet Send Status:\t");
//   Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
// }
 
// void setup() {
//   Serial.begin(115200);
//   WiFi.mode(WIFI_STA);

//   if (esp_now_init() != ESP_OK) {
//     Serial.println("Error initializing ESP-NOW");
//     return;
//   }

//   esp_now_register_send_cb(OnDataSent);
//   memcpy(peerInfo.peer_addr, broadcastAddress, 6);
//   peerInfo.channel = 0;  
//   peerInfo.encrypt = false;
  
//   if (esp_now_add_peer(&peerInfo) != ESP_OK){
//     Serial.println("Failed to add peer");
//     return;
//   }
// }
 
// void loop() {
//   if (Serial.available()) {
//     String input = Serial.readStringUntil('\n');
//     sscanf(input.c_str(), "%s\t%f,%f,%f\t%f", 
//            &myData.ID, 
//            &myData.position[0], &myData.position[1], &myData.position[2], 
//            &myData.headingY);
    
//     myData.position_d[0] = 500;
//     myData.position_d[1] = 500;
//     myData.position_d[2] = 500;

//     if (strcmp(myData.ID, "Mona5") == 0){
//       esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
//       if (result == ESP_OK) {
//         Serial.println("发送成功");
//       }
//       else {
//         Serial.println("发送数据出错");
//       }
//     }
    
//   }
// }


#include <esp_now.h>
#include <WiFi.h>
#include <cstring>

// 为机器人定义两个接收器的MAC地址
uint8_t broadcastAddresses[4][6] = {
    {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x28}, // mona5
    {0x8C, 0xCE, 0x4E, 0xBB, 0x4B, 0xEC}, // mona6
    {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x74}, // mona2
    {0x8C, 0xCE, 0x4E, 0xBB, 0x4C, 0x70}, // mona1
};

typedef struct struct_message {
  char ID[32];              // 标识符
  float position[3];        // 位置坐标：X, Y, Z
  float headingY;           // Y方向的朝向
  float position_d[3];      // 目标位置坐标
} struct_message;

// 创建一个名为myData的struct_message实例
struct_message myData;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\n上一个数据包发送状态:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "发送成功" : "发送失败");
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW 初始化失败");
        return;
    }

    esp_now_register_send_cb(OnDataSent);

    // 初始化每个设备
    for (int i = 0; i < 4; i++) {
        esp_now_peer_info_t peerInfo;
        memset(&peerInfo, 0, sizeof(esp_now_peer_info_t)); // 确保结构体全部字段初始化为0
        memcpy(peerInfo.peer_addr, broadcastAddresses[i], 6);
        peerInfo.channel = 0; // 可以设置为1或实际的WiFi频道
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.print("添加设备失败: ");
            for (int j = 0; j < 6; j++) {
                Serial.print(broadcastAddresses[i][j], HEX);
                if (j < 5) Serial.print(":");
            }
            Serial.println();
            return;
        }
    }
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    sscanf(input.c_str(), "%s\t%f,%f,%f\t%f\t%f,%f,%f", 
           myData.ID, 
           &myData.position[0], &myData.position[1], &myData.position[2], 
           &myData.headingY,
           &myData.position_d[0],&myData.position_d[1],&myData.position_d[2]);
//     char id[32];

// // 初始化随机数生成器
// randomSeed(analogRead(0)); // 如果在没有其他更好随机源的情况下

// // 生成一个随机数，如果随机数的低位是0，则选择"Mona5"，否则选择"Mona6"
// if (random(2) == 0) {
//     strcpy(id, "Mona2");
// } else {
//     strcpy(id, "Mona6");
// }

//     strcpy(myData.ID, id);
//     myData.position[0]=1;
//     myData.position[1]=1;
//     myData.position[2]=1;
//     myData.headingY=1;
    // myData.position_d[0] = 500;
    // myData.position_d[1] = 500;
    // myData.position_d[2] = 500;

    int robotIndex = -1;
    if (strcmp(myData.ID, "Mona5") == 0) {
      robotIndex = 0;
    } else if (strcmp(myData.ID, "Mona6") == 0) {
      robotIndex = 1;
    } else if (strcmp(myData.ID, "Mona2") == 0) {
      robotIndex = 2;
    } else if (strcmp(myData.ID, "Mona1") == 0) {
      robotIndex = 3;
    }
    

    if (robotIndex != -1) {
      esp_err_t result = esp_now_send(broadcastAddresses[robotIndex], (uint8_t *)&myData, sizeof(myData));
    }

  }
  

}


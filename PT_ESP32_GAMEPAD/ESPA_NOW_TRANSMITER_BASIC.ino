#include <Arduino.h>
#include "esp_now.h"
#include "WiFi.h"
#include <bluepad32.h>

// Dirección MAC de tu ESP32B (receptor)
uint8_t broadcastAddress[] = {0x6C, 0xC8, 0x40, 0x5D, 0x12, 0xE4};

// Estructura de datos para enviar
typedef struct struct_message {
  float yaw;
  float pitch;
  bool headTrackingActive;
} struct_message;
struct_message myData;

// Objeto para el Gamepad
GamepadPtr myGamepad = nullptr; 

// Callbacks de Bluepad32
void onConnectedGamepad(GamepadPtr gp) {
  myGamepad = gp;
}

void onDisconnectedGamepad(GamepadPtr gp) {
  myGamepad = nullptr;
}

// Callback de ESP-NOW para el estado de envío
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Opcional: imprimir estado de envío
}

// Tarea para el Core 1 (ESP-NOW)
void sendDataTask(void *pvParameters) {
  WiFi.mode(WIFI_STA);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    vTaskDelete(NULL);
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al agregar el peer");
    vTaskDelete(NULL);
  }
  Serial.println("Peer de ESP-NOW agregado con éxito.");

  while(true) {
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Tarea para el Core 0 (Control del Gamepad)
void gamepadControlTask(void *pvParameters) {
  Serial.begin(115200);

  // Inicializar Bluepad32
  BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
  BP32.enableNewBluetoothConnections(true);
  Serial.println("Configuración completa. Esperando datos del gamepad...");

  while(true) {
    BP32.update();

    if (myGamepad && myGamepad->isConnected()) {
      float joystickX = myGamepad->axisX();
      float joystickY = myGamepad->axisY();

      // Enviar siempre los datos del joystick
      myData.headTrackingActive = false;
      myData.yaw = joystickX;
      myData.pitch = joystickY;
    } else {
      // Si el gamepad se desconecta, enviar valores de 0
      myData.yaw = 0;
      myData.pitch = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void setup() {
  xTaskCreatePinnedToCore(
    sendDataTask,
    "sendDataTask",
    10000,
    NULL,
    1,
    NULL,
    1
  );

  xTaskCreatePinnedToCore(
    gamepadControlTask,
    "gamepadControlTask",
    10000,
    NULL,
    1,
    NULL,
    0
  );
}

void loop() {
}
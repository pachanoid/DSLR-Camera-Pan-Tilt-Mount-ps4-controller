#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <AccelStepper.h>
#include <ESP32Encoder.h>

// Estructura de datos para recibir
typedef struct struct_message {
  float yaw;
  float pitch;
  bool headTrackingActive;
} struct_message;
struct_message myData;

// Pines para el motor Pan
#define PAN_PUL_PIN 25
#define PAN_DIR_PIN 26
#define PAN_EN_PIN  33

// Pines para el motor Tilt
#define TILT_PUL_PIN 12
#define TILT_DIR_PIN 14
#define TILT_EN_PIN  27
#define TILT_POT_PIN 34 // Pin de entrada analógica para el potenciómetro Tilt

// Pines para el Encoder Pan (KY-040)
#define ENCODER_A_PIN 32
#define ENCODER_B_PIN 35
#define ENCODER_SW_PIN 23

// Objetos de AccelStepper
AccelStepper stepperPan(AccelStepper::DRIVER, PAN_PUL_PIN, PAN_DIR_PIN);
AccelStepper stepperTilt(AccelStepper::DRIVER, TILT_PUL_PIN, TILT_DIR_PIN);

// Objeto del Encoder
ESP32Encoder panEncoder;

// Constantes y variables para el PID Pan
float Kp_pan = 90.0;
float Ki_pan = 0.0;
float Kd_pan = 0.02;
float error_pan = 0;
float last_error_pan = 0;
float integral_pan = 0;
float derivative_pan = 0;
const float DEADZONE_PAN = 1.0; 

// Constantes y variables para el PID Tilt
float Kp_tilt = 45.0; 
float Ki_tilt = 0.0; 
float Kd_tilt = 0.8; 
float error_tilt = 0;
float last_error_tilt = 0;
float integral_tilt = 0;
float derivative_tilt = 0;
const float DEADZONE_TILT = 5.0;

// Variables y constantes del filtro de alfa
const float ALPHA_YAW = 0.2;
float smoothed_yaw = 0.0;
const float ALPHA_PITCH = 0.9;
float smoothed_pitch = 0.0;
const float ALPHA_TILT = 0.2;
float smoothed_tilt_angle = 0.0;

static unsigned long lastDataReceiveTime = 0;
const unsigned long TIMEOUT_MS = 500;

// Constantes de calibración
const float STEPS_PER_DEGREE = 8.88; 
const float ENCODER_STEPS_PER_DEGREE = 1.0;
const int MAX_PAN_ANGLE = 90; 
const int MAX_TILT_ANGLE = 90; 
const int TILT_MIN_ANALOG = 0; 
const int TILT_MAX_ANALOG = 4095; 

const float MAX_SPEED_HT = 3000.0;
const float ACCELERATION_HT = 3000.0; 

// Callback que se ejecuta cuando se reciben datos
void OnDataRecv(const esp_now_recv_info_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  lastDataReceiveTime = millis();
}
  
void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(PAN_EN_PIN, OUTPUT);
  pinMode(TILT_EN_PIN, OUTPUT);
  pinMode(TILT_POT_PIN, INPUT);
  digitalWrite(PAN_EN_PIN, HIGH);
  digitalWrite(TILT_EN_PIN, HIGH);

  stepperPan.setMaxSpeed(MAX_SPEED_HT);
  stepperPan.setAcceleration(ACCELERATION_HT);
  stepperTilt.setMaxSpeed(MAX_SPEED_HT);
  stepperTilt.setAcceleration(ACCELERATION_HT);
  
  // Inicializa los valores suavizados
  smoothed_tilt_angle = map(analogRead(TILT_POT_PIN), TILT_MIN_ANALOG, TILT_MAX_ANALOG, MAX_TILT_ANGLE, 0);
  smoothed_pitch = myData.pitch;
  smoothed_yaw = myData.yaw;
  
  // Inicializa los pines del Encoder con la librería ESP32Encoder
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  panEncoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
}

void loop() {
  bool dataReceivedRecently = (millis() - lastDataReceiveTime <= TIMEOUT_MS);

  if (dataReceivedRecently) {
    digitalWrite(PAN_EN_PIN, LOW);
    digitalWrite(TILT_EN_PIN, LOW);

    if (myData.headTrackingActive) {
      // Limitar los valores de yaw y pitch antes de calcular los pasos
      float constrained_yaw = constrain(myData.yaw, -MAX_PAN_ANGLE, MAX_PAN_ANGLE);
      float constrained_pitch = constrain(myData.pitch, -MAX_TILT_ANGLE, MAX_TILT_ANGLE);
      
      long panSteps = (long)(constrained_yaw * STEPS_PER_DEGREE);
      long tiltSteps = (long)(constrained_pitch * STEPS_PER_DEGREE);
      
      stepperPan.moveTo(panSteps);
      stepperTilt.moveTo(tiltSteps);
      
    } else { // Modo Joystick (Lazo Cerrado para ambos ejes)
      
      // --- Lazo Cerrado para EJE PAN con Encoder ---
      smoothed_yaw = (ALPHA_YAW * myData.yaw) + ((1 - ALPHA_YAW) * smoothed_yaw);
      float desired_pan_angle = map(smoothed_yaw, -512, 511, -MAX_PAN_ANGLE, MAX_PAN_ANGLE);
      
      float current_pan_angle = panEncoder.getCount() / ENCODER_STEPS_PER_DEGREE;
      
      float control_value_pan = 0;
      error_pan = desired_pan_angle - current_pan_angle;
      
      if (abs(error_pan) > DEADZONE_PAN) {
        integral_pan += error_pan;
        integral_pan = constrain(integral_pan, -100, 100); 
        
        derivative_pan = error_pan - last_error_pan;
        control_value_pan = (Kp_pan * error_pan) + (Ki_pan * integral_pan) + (Kd_pan * derivative_pan);
        control_value_pan *= -1;
      } else {
        integral_pan = 0;
        control_value_pan = 0;
      }
      stepperPan.setSpeed(control_value_pan);
      last_error_pan = error_pan;
      
      if (digitalRead(ENCODER_SW_PIN) == LOW) {
        panEncoder.clearCount();
        Serial.println("Pan Position Reset to 0");
      }
      
      // --- Lazo Cerrado para EJE TILT con Potenciómetro ---
      smoothed_pitch = (ALPHA_PITCH * myData.pitch) + ((1 - ALPHA_PITCH) * smoothed_pitch);
      float desired_tilt_angle = map(smoothed_pitch, -512, 511, 0, MAX_TILT_ANGLE);

      int raw_tilt_pot_value = analogRead(TILT_POT_PIN);
      float raw_tilt_angle = map(raw_tilt_pot_value, TILT_MIN_ANALOG, TILT_MAX_ANALOG, MAX_TILT_ANGLE, 0);
      smoothed_tilt_angle = (ALPHA_TILT * raw_tilt_angle) + ((1 - ALPHA_TILT) * smoothed_tilt_angle);
      
      float control_value_tilt = 0;
      error_tilt = desired_tilt_angle - smoothed_tilt_angle;
      
      if (abs(error_tilt) > DEADZONE_TILT) {
        integral_tilt += error_tilt;
        derivative_tilt = error_tilt - last_error_tilt;
        control_value_tilt = (Kp_tilt * error_tilt) + (Ki_tilt * integral_tilt) + (Kd_tilt * derivative_tilt);
      } else {
        integral_tilt = 0; 
        control_value_tilt = 0;
      }
      stepperTilt.setSpeed(control_value_tilt);
      last_error_tilt = error_tilt;
    }
  } else {
    stepperPan.setSpeed(0);
    stepperTilt.setSpeed(0);
    digitalWrite(PAN_EN_PIN, HIGH);
    digitalWrite(TILT_EN_PIN, HIGH);
  }

  if (myData.headTrackingActive && dataReceivedRecently) {
    stepperPan.run();
    stepperTilt.run();
  } else {
    stepperPan.runSpeed();
    stepperTilt.runSpeed();
  }
}
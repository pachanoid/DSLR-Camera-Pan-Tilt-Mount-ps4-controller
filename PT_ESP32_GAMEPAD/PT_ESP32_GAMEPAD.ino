// Código de prueba para el control de Pan-Tilt con joystick PS4
// Creado para el usuario para probar la funcionalidad de los motores y el gamepad
// Este código solo incluye la lógica del joystick, excluyendo todo lo demás del proyecto original

#include <Bluepad32.h>
#include <AccelStepper.h>

// Pines para el motor Pan (movimiento horizontal)
#define PAN_PUL_PIN 25 // Pin para pulsos del motor PAN
#define PAN_DIR_PIN 26 // Pin para dirección del motor PAN
#define PAN_EN_PIN  33 // Pin para habilitar/deshabilitar el driver PAN

// Pines para el motor Tilt (movimiento vertical)
#define TILT_PUL_PIN 12 // Pin para pulsos del motor TILT
#define TILT_DIR_PIN 14 // Pin para dirección del motor TILT
#define TILT_EN_PIN  27 // Pin para habilitar/deshabilitar el driver TILT

// Crear objetos de la clase AccelStepper para cada motor
AccelStepper stepperPan(AccelStepper::DRIVER, PAN_PUL_PIN, PAN_DIR_PIN);
AccelStepper stepperTilt(AccelStepper::DRIVER, TILT_PUL_PIN, TILT_DIR_PIN);

Bluepad32 bp32;
GamepadPtr myGamepad = nullptr;

// Callbacks para el estado del gamepad
void onConnectedGamepad(GamepadPtr gp) {
    Serial.println("Gamepad conectado");
    myGamepad = gp;
}

void onDisconnectedGamepad(GamepadPtr gp) {
    Serial.println("Gamepad desconectado");
    myGamepad = nullptr;
}

void setup() {
    Serial.begin(115200);

    // Inicializar la librería Bluepad32
    bp32.setup(&onConnectedGamepad, &onDisconnectedGamepad);
    bp32.enableNewBluetoothConnections(true);
    
    // Configurar los pines de habilitación como salida
    pinMode(PAN_EN_PIN, OUTPUT);
    pinMode(TILT_EN_PIN, OUTPUT);

    // Mantener los drivers deshabilitados al inicio (HIGH para deshabilitar, LOW para habilitar)
    digitalWrite(PAN_EN_PIN, HIGH);
    digitalWrite(TILT_EN_PIN, HIGH);

    // Configuración de los motores Pan y Tilt
    // SUGERENCIA: Si sigue saltando pasos, puedes intentar reducir este valor
    // stepperPan.setMaxSpeed(2000.0);
    stepperPan.setMaxSpeed(4000.0);    // Velocidad máxima del motor Pan
    stepperPan.setAcceleration(3000.0); // Reducir la aceleración del motor Pan

    // SUGERENCIA: Si sigue saltando pasos, puedes intentar reducir este valor
    // stepperTilt.setMaxSpeed(2000.0);
    stepperTilt.setMaxSpeed(4000.0);    // Velocidad máxima del motor Tilt
    stepperTilt.setAcceleration(3000.0); // Reducir la aceleración del motor Tilt
}

void loop() {
    bp32.update();
    
    // Lógica de control para los motores de la cámara con el stick derecho
    if (myGamepad && myGamepad->isConnected()) {
        int rawRX = myGamepad->axisRX(); 
        int rawRY = myGamepad->axisRY(); 

        const int deadzone = 20; 
        const float maxSpeed = 4000.0; 

        bool joystickPanMoving = abs(rawRX) > deadzone;
        bool joystickTiltMoving = abs(rawRY) > deadzone;

        // Habilita los drivers solo si el joystick se está moviendo
        if (joystickPanMoving || joystickTiltMoving) {
            digitalWrite(PAN_EN_PIN, LOW); // Habilita el driver PAN (LOW = Habilitado)
            digitalWrite(TILT_EN_PIN, LOW); // Habilita el driver TILT (LOW = Habilitado)
        } else {
            digitalWrite(PAN_EN_PIN, HIGH); // Deshabilita el driver PAN
            digitalWrite(TILT_EN_PIN, HIGH); // Deshabilita el driver TILT
        }

        float panSpeed = 0;
        if (joystickPanMoving) {
            panSpeed = map(rawRX, -512, 511, -maxSpeed, maxSpeed);
        }

        float tiltSpeed = 0;
        if (joystickTiltMoving) {
            tiltSpeed = map(-rawRY, -512, 511, -maxSpeed, maxSpeed);
        }

        stepperPan.setSpeed(panSpeed);
        stepperTilt.setSpeed(tiltSpeed);
    } else {
        stepperPan.setSpeed(0);
        stepperTilt.setSpeed(0);
        // Si el gamepad se desconecta, también deshabilitamos los drivers para ahorrar energía
        digitalWrite(PAN_EN_PIN, HIGH);
        digitalWrite(TILT_EN_PIN, HIGH);
    }
    
    stepperPan.runSpeed();
    stepperTilt.runSpeed();
}

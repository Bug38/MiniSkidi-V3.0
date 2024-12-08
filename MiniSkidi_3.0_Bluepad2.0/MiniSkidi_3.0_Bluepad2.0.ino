/* ProfessorBoots
   John Cheroske 1/6/2024
   MiniSkidi 3.0

   Thank you to the following people for contributing to this sketch
   -TomVerr99 "Excellent Job organizing what was a very messy intial sketch"
   -CrabRC "I dont even know where to start, but thank you for making numerous improvemnts/suggestions
   across both mechanical designs and software."
   -Fortinbra "Always willing to provide the discord group with a good meme or two, as well as lend a helping hand
   in multiple ways."

  Some tidbits to check

  -Install the esp32 boards manager into the arduino IDE"
  Programming Electronics Academy has a good tutorial: https://youtu.be/py91SMg_TeY?si=m1OWPBPlK-QHJ2Xx"
  -Select "ESP32 Dev Module" under tools>Board>ESP32 Arduino before uploading sketch.
  -The following include statements with comments "by -----" are libraries that can be installed
  directly inside the arduino IDE under Sketch>Include Library>Manage Libraries
*/
#include <Arduino.h>

#include <ESP32Servo.h>  // by Kevin Harrington
#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define quickFixServoPin 23
#define AuxServoPin 22
#define cabLights 32
#define auxLights 33

#define armMotor0 21   // Used for controlling auxiliary attachment movement
#define armMotor1 19   // Used for controlling auxiliary attachment movement
#define auxAttach0 18  // Used for controlling auxiliary attachment movement
#define auxAttach1 17  // Used for controlling auxiliary attachment movement

#define leftMotor0 33   // Used for controlling the left motor movement
#define leftMotor1 32   // Used for controlling the left motor movement
#define rightMotor0 25  // Used for controlling the right motor movementc:\Users\JohnC\Desktop\SOLIDWORKS Connected.lnk
#define rightMotor1 26  // Used for controlling the right motor movement

#define FORWARD 1
#define BACKWARD -1
#define STOP 0

Servo quickFixServo;
Servo AuxServo;



int lightSwitchTime = 0;
float adjustedSteeringValue = 86;
float steeringAdjustment = 1;
int steeringTrim = 0;
int auxTiltValue = 90;
int quickFixAngleValue = 145;

bool lightsOn = false;
bool moveAuxServoDown = false;
bool moveAuxServoUp = false;
bool hardLeft;
bool hardRight;

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      // Additionally, you can get certain gamepad properties like:
      // Model, VID, PID, BTAddr, flags, etc.
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                    properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}


void processGamepad(ControllerPtr ctl) {

  processMoving(ctl->dpad());

  processArm(ctl->y(), ctl->a());

  processQuickFixAngle(ctl->x(), ctl->b());

  processAux(ctl->l1(), ctl->r1());
}

int8_t auxServoUpdate = 0;
int8_t quickFixServoUpdate = 0;
uint8_t servoDelay = 0;
void processServos() {
  if (auxServoUpdate) {
    if (servoDelay == 8) {
      if (auxTiltValue + auxServoUpdate > 10 && auxTiltValue + auxServoUpdate < 170) {
        auxTiltValue += auxServoUpdate;
        AuxServo.write(auxTiltValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
  if (quickFixServoUpdate) {
    if (servoDelay == 8) {
      if (quickFixAngleValue + quickFixServoUpdate > 10 && quickFixAngleValue + quickFixServoUpdate < 170) {
        quickFixAngleValue += quickFixServoUpdate;
        quickFixServo.write(quickFixAngleValue);
      }
      servoDelay = 0;
    }
    servoDelay++;
  }
}

#define DPAD_FORWARD 0X01
#define DPAD_BACKWARD 0X02
#define DPAD_RIGHT 0X04
#define DPAD_LEFT 0X08
void processMoving(uint8_t dpad) {
  int rSpeed = 0;
  int lSpeed = 0;
  if (dpad & DPAD_RIGHT) {
    rSpeed = 255;
    if (dpad & DPAD_FORWARD || dpad & DPAD_BACKWARD) {
      rSpeed = 0;
      lSpeed = 255;
    } else {
      rSpeed = -255;
      lSpeed = 255;
    }
  } else if (dpad & DPAD_LEFT) {
    lSpeed = 255;
    if (dpad & DPAD_FORWARD || dpad & DPAD_BACKWARD) {
      rSpeed = 255;
      lSpeed = 0;
    } else {
      lSpeed = -255;
      rSpeed = 255;
    }
  } else if (dpad & DPAD_FORWARD) {
    rSpeed = 255;
    lSpeed = 255;
  }
  if (dpad & DPAD_BACKWARD) {
    if (!(dpad & DPAD_RIGHT || dpad & DPAD_LEFT)) {
      rSpeed = -255;
      lSpeed = -255;
    } else {
      int tmp = lSpeed * -1;
      lSpeed = rSpeed * -1;
      rSpeed = tmp;
    }
  }

  moveMotor(rightMotor0, rightMotor1, rSpeed);
  moveMotor(leftMotor0,  leftMotor1,  lSpeed);
}

int armSpeed = 255;
bool removeMomentum = false;
void processArm(bool up, bool down) {
  if (up) {
    moveMotor(armMotor0, armMotor1, armSpeed);
  } else if (down) {
    moveMotor(armMotor0, armMotor1, -armSpeed);
    removeMomentum = true;
  } else {
    if (removeMomentum) {
      moveMotor(armMotor0, armMotor1, armSpeed);
      delay(10);
      removeMomentum = false;
    }
    moveMotor(armMotor0, armMotor1, 0);
  }
}

void processQuickFixAngle(bool up, bool down) {
  if (up) {
    quickFixServoUpdate = 1;
  } else if (down) {
    quickFixServoUpdate = -1;
  } else {
    quickFixServoUpdate = 0;
  }
}

void processAux(bool up, bool down) {
  if (up) {
    auxServoUpdate = 1;
  } else if (down) {
    auxServoUpdate = -1;
  } else {
    auxServoUpdate = 0;
    Serial.printf("angle: %d\n", quickFixAngleValue);
  }
}

void moveMotor(int motorPin0, int motorPin1, int velocity) {
  if (velocity > 1) {
    analogWrite(motorPin0, velocity);
    analogWrite(motorPin1, LOW);
  } else if (velocity < -1) {
    analogWrite(motorPin0, LOW);
    analogWrite(motorPin1, (-1 * velocity));
  } else {
    analogWrite(motorPin0, 0);
    analogWrite(motorPin1, 0);
  }
}
void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
        processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

// Arduino setup function. Runs in CPU 1
void setup() {
  pinMode(armMotor0, OUTPUT);
  pinMode(armMotor1, OUTPUT);
  pinMode(auxAttach0, OUTPUT);
  pinMode(auxAttach1, OUTPUT);
  digitalWrite(auxAttach0, LOW);
  digitalWrite(auxAttach1, LOW);
  pinMode(leftMotor0, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(rightMotor0, OUTPUT);
  pinMode(rightMotor1, OUTPUT);


  quickFixServo.attach(quickFixServoPin);
  quickFixServo.write(quickFixAngleValue);
  AuxServo.attach(AuxServoPin);
  AuxServo.write(auxTiltValue);

  Serial.begin(115200);
  //   put your setup code here, to run once:
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t *addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // "forgetBluetoothKeys()" should be called when the user performs
  // a "device factory reset", or similar.
  // Calling "forgetBluetoothKeys" in setup() just as an example.
  // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
  // But it might also fix some connection / re-connection issues.
  BP32.forgetBluetoothKeys();

  // Enables mouse / touchpad support for gamepads that support them.
  // When enabled, controllers like DualSense and DualShock4 generate two connected devices:
  // - First one: the gamepad
  // - Second one, which is a "virtual device", is a mouse.
  // By default, it is disabled.
  BP32.enableVirtualDevice(false);
  // You could add additional error handling here,
  // such as logging the error or attempting to recover.
  // For example, you might attempt to reset the MCP23X17
  // and retry initialization before giving up completely.
  // Then, you could gracefully exit the program or continue
  // running with limited functionality.
}



// Arduino loop function. Runs in CPU 1.
void loop() {
  // This call fetches all the controllers' data.
  // Call this function in your main loop.
  bool dataUpdated = BP32.update();
  if (dataUpdated) {
    processControllers();
  }
  // The main loop must have some kind of "yield to lower priority task" event.
  // Otherwise, the watchdog will get triggered.
  // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
  // Detailed info here:
  // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

  //     vTaskDelay(1);
  else { processServos(); vTaskDelay(1); }
}
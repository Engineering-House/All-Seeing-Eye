#include <AccelStepper.h>

#define XB_STEP_PIN 26
#define XB_DIR_PIN 27

#define YB_STEP_PIN 33
#define YB_DIR_PIN 32

#define XT_STEP_PIN 16
#define XT_DIR_PIN 17

#define YT_STEP_PIN 23
#define YT_DIR_PIN 22

#define MAX_CMD_LEN 20
#define MAX_SPEED 200

AccelStepper xBot = AccelStepper(AccelStepper::DRIVER, XB_STEP_PIN, XB_DIR_PIN); //0
AccelStepper yBot = AccelStepper(AccelStepper::DRIVER, YB_STEP_PIN, YB_DIR_PIN); //1
AccelStepper xTop = AccelStepper(AccelStepper::DRIVER, XT_STEP_PIN, XT_DIR_PIN); //2
AccelStepper yTop = AccelStepper(AccelStepper::DRIVER, YT_STEP_PIN, YT_DIR_PIN); //3

AccelStepper* steppers[] = {&xBot, &yBot, &xTop, &yTop};

volatile float speed = 200;
volatile int pos[] = {0, 0, 0, 0};

TaskHandle_t uartMonitor;

char cmdBuffer[MAX_CMD_LEN];
char cmdBufferIndex = 0;
// penis 

uint32_t tick = 0;
char epicMode = 0;

void parseCmd() {
  if(!strncmp(cmdBuffer, "speed", 5)) {
    int len = strlen(cmdBuffer);
    if(len <= 6) {
      Serial.printf("Usage: speed <float>\n");
      return;
    }
    double spd = atof(cmdBuffer + 6);
    speed = spd;
    for(int i = 0; i < 4; i++) {
      steppers[i]->setMaxSpeed(speed);
    }
    Serial.printf("Speed set to %4.1f\n", speed);
  } else if(!strncmp(cmdBuffer, "pos", 3)) {
    int index;
    int posP;
    int c = sscanf(cmdBuffer, "pos %d %d", &index, &posP);
    if(c == 2) {
      pos[index] = posP;
      Serial.printf("pos[%d] = %d\n", index, posP);
    } else if(c == 1) {
      Serial.printf("Position: %d\n", pos[index]);
      return;
    } else {
      Serial.printf("Usage: pos <motor> <float>\n");
      return;
    }
  } else if(!strncmp(cmdBuffer, "reset", 5)) {
    int index;
    int c = sscanf(cmdBuffer, "reset %d", &index);
    if(c == 1) {
      steppers[index]->setCurrentPosition(0);
      pos[index] = 0;
      Serial.printf("stepper[%d] reset\n", index);
    } else {
      for(int i = 0; i < 4; i++) {
        steppers[i]->setCurrentPosition(0);
        pos[i] = 0;
      }
      Serial.printf("steppers reset\n");
    }
  } else if(!strncmp(cmdBuffer, "epic", 4)) {
    epicMode = !epicMode;
    Serial.printf("Epic mode set to %d\n", epicMode);
  }
}

void checkCmd() {
  while(Serial.available()) {
    char c = Serial.read();
    if(c == '\b' && cmdBufferIndex > 0) {
      Serial.print("\b \b");
      cmdBufferIndex--;
      cmdBuffer[cmdBufferIndex] = '\0';
    } else if(c == '\n') {

    } else if(c == '\r') {
      Serial.println();
      cmdBuffer[cmdBufferIndex] = '\0';
      if(cmdBufferIndex > 0) {
        parseCmd();
      }
      cmdBufferIndex = 0;
      Serial.print("> ");
    } else {
      cmdBuffer[cmdBufferIndex] = c;
      Serial.print(c);
      cmdBufferIndex++;
    }
  }
  if(epicMode) {
    pos[0] = (int) (cosf((float) tick * 0.000024f) * 500.0f);
    pos[1] = (int) (cosf((float) tick * 0.000024f + 2 * 3.14159f / 3) * 500.0f);
    pos[2] = (int) (cosf((float) tick * 0.000024f - 2 * 3.14159f / 3) * -500.0f);
  }
  for(int i = 0; i < 4; i++) {
    steppers[i]->moveTo(pos[i]);
  }
  
  xBot.run();
  yBot.run();
  xTop.run();
  yTop.run();
}

void runStepper() {

}

void setup() {
  Serial.begin(9600);
  for(int i = 0; i < 4; i++) {
    steppers[i]->setPinsInverted(false, true);
    steppers[i]->setMaxSpeed(250);
    steppers[i]->setAcceleration(300);
    steppers[i]->setCurrentPosition(0);
  }
}

void loop() {
  checkCmd();
  tick++;
}
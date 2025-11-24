// Arduino code for controlling 3d printed nerf turret see https://dylanhammond-11.github.io/projects/auto-nerf-turret/index/
// Use alongside turret_vision_face.py and turret_vision_red.py for full implementation with the turret
// Code by Dylan Hammond


# include <Servo.h>

Servo pan;
Servo tilt;
Servo load;

uint8_t panPin= 4 , tiltPin = 3 , loadPin = 2; // Servo pin numbers
uint8_t IN1 = 6, IN2 = 7, IN3 = 11, IN4 = 12; // Motor driver pin numbers
static int xCenter = 320 , yCenter = 240 ; // Pixel center of camera (640x480)
int pixelBuffer = 50; // Pixel buffer range that determines when it is on-target and ready to fire

int xTarget = 0;
int yTarget = 0;
int sampleRate  = 80; // Sample rate for pid loop
bool targetFound = false;
bool readyToFire = true; // This variable will turn the motors off until new CV2 data is recieved 
bool firingInProgress = false;




void fire() {
  // Controls firing timing sequence through a switch statement
  static enum { IDLE, SPINNING_UP, LOADING, RESETTING, DONE } state = IDLE;
  static unsigned long fireStartTime = 0;
  static const uint8_t shootingSpeed = 100;
  static const unsigned long motorDelay = 700;
  static const unsigned long servoDelay = 300;

  switch (state) {
    case IDLE:
      // Start the flywheels
      analogWrite(IN1, shootingSpeed); digitalWrite(IN2, LOW);
      analogWrite(IN3, shootingSpeed); digitalWrite(IN4, HIGH);
      fireStartTime = millis();
      state = SPINNING_UP;
      break;

    case SPINNING_UP:
      if (millis() - fireStartTime >= motorDelay) {
        load.write(105); // Push bullet
        fireStartTime = millis();
        state = LOADING;
      }
      break;

    case LOADING:
      if (millis() - fireStartTime >= servoDelay) {
        load.write(0); // Reset loader
        fireStartTime = millis();
        state = RESETTING;
      }
      break;

    case RESETTING:
      if (millis() - fireStartTime >= 200) {
        // Stop the flywheels
        digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
        digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
        state = DONE;
      }
      break;

   case DONE:
  state = IDLE;
  targetFound = false;
  readyToFire = false;
  firingInProgress = false;  // We're done firing!
  break;

  }
}


void aim() {
  if ((abs(xCenter - xTarget) < pixelBuffer) &&
      (abs(yCenter - yTarget) < pixelBuffer) &&
      targetFound  && !firingInProgress) {

    firingInProgress = true;  // Mark that firing has started
  }
}



float updatePID(float dt){
  

  float kpPan = 0.0725, kiPan = 0, kdPan = 0; // Gains for Pan and Tilt PID Controller
  float kpTilt = .065, kiTilt = 0, kdTilt = 0;
  static float pastIntegralPan= 0, pastIntegralTilt = 0;
  static float essPrevPan = 0, essPrevTilt = 0;

  int essPan = xCenter - xTarget;
  int essTilt = yCenter - yTarget;

  ///////PID for Pan//////////
  if(abs(essPan) > pixelBuffer){
  float integralPan  = essPan*dt + pastIntegralPan;
  float derivativePan = (essPan-essPrevPan)/dt;
  
  float panOutput = kpPan*essPan + kiPan*integralPan + kdPan*derivativePan;
  float panOutputConstrained = constrain(panOutput + 90,0,180);
  pan.write(panOutputConstrained);

  essPrevPan= essPan;
  pastIntegralPan = integralPan;
  }
  /////////////////////////////


  ///////PID for tilt//////////
  if(abs(essTilt)> pixelBuffer){
  float integralTilt  = essTilt*dt + pastIntegralTilt;
  float derivativeTilt = (essTilt-essPrevTilt)/dt;

  float tiltOutput = kpTilt*essTilt + kiTilt*integralTilt + kdTilt*derivativeTilt;
  float tiltOutputConstrained = constrain(tiltOutput + 90,0,180);

  tilt.write(tiltOutputConstrained);

  essPrevTilt = essTilt;
  pastIntegralTilt = integralTilt;
  ///////////////////////////////
  }

}


void setup() {
  pan.attach(panPin);
  tilt.attach(tiltPin);
  load.attach(loadPin);
  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);  // Set built-in LED pin as output
  Serial.begin(115200);

  pan.write(90);// Initialize all servos
  tilt.write(90);
  load.write(0);
}

void loop() {

  static unsigned long pastTime = 0; // Time counter that controls the timing of the pid updates
  unsigned long currentTime = millis();

    // Check if recieving data from python
if (Serial.available() >= 4) {
  uint8_t buffer[4];
  Serial.readBytes(buffer, 4);
  xTarget = buffer[0] | (buffer[1] << 8);
  yTarget = buffer[2] | (buffer[3] << 8);

  targetFound = true;
  readyToFire = true;  // Allow firing again
}


  if (targetFound && currentTime-pastTime >= sampleRate ) { // Controls PID update rate
    float dt = (currentTime - pastTime)/1000.00;
    updatePID(dt); 

    pastTime = currentTime;
    
  }
      aim(); // Check if we should start firing

  if (firingInProgress) {
    fire();  // Keep running fire() until DONE
  }

  }



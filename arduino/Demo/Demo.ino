# include <Servo.h>

Servo pan;
Servo tilt;
Servo load;
uint8_t panPin= 4 , tiltPin = 3 , loadPin = 2; // Servo pin numbers
uint8_t IN1 = 6, IN2 = 7, IN3 = 11, IN4 = 12; // Motor driver pin numbers
static int xCenter = 320 , yCenter = 240 ; // Pixel center of camera 
int pixelBuffer = 50; // Buffer range that determines when it is on-target and ready to fire

int xTarget = 0;
int yTarget = 0;
int sampleRate  = 100; // delay 
bool targetFound = false;
bool readyToFire = true; // This variable will turn the motors off until new CV2 data is recieved 
bool firingInProgress = false;




void fire() {
  int shootingSpeed = 255;
  

  analogWrite(IN1, shootingSpeed); digitalWrite(IN2, LOW);
  analogWrite(IN3, shootingSpeed); digitalWrite(IN4, HIGH);
  delay(700);
  

   
  load.write(105); // Push bullet
  delay(700);
  load.write(0); // Reset load
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);



  
}


void aim() {
  if ((abs(xCenter - xTarget) < pixelBuffer) &&
      (abs(yCenter - yTarget) < pixelBuffer) &&
      targetFound && readyToFire && !firingInProgress) {

    firingInProgress = true;  // Mark that firing has started
  }
}



float updatePID(float dt){
  

  float kpPan = 0.2, kiPan = 0, kdPan = 0; // Gains for Pan and Tilt PID Controller
  float kpTilt = 0.2, kiTilt = 0, kdTilt = 0;
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


  ///////PID for tilt/////////
  if(abs(essTilt)> pixelBuffer){
  float integralTilt  = essTilt*dt + pastIntegralTilt;
  float derivativeTilt = (essTilt-essPrevTilt)/dt;

  float tiltOutput = kpTilt*essTilt + kiTilt*integralTilt + kdTilt*derivativeTilt;
  float tiltOutputConstrained = constrain(tiltOutput + 90,0,180);

  tilt.write(tiltOutputConstrained);

  essPrevTilt = essTilt;
  pastIntegralTilt = integralTilt;
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
  delay(1000);
  tilt.write(90);
  delay(1000);
  tilt.write(100);
  delay(1000);
  tilt.write(120);
  //fire();
  delay(1000);
  tilt.write(130);
  delay(1000);
  //fire();
  delay(500);
  tilt.write(160);
  pan.write(100);
  delay(500);
  fire();
  tilt.write(140);
  delay(500);
  pan.write(130);
  fire();
  



  }



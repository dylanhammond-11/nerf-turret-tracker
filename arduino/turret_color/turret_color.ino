// Arduino code for controlling 3d printed nerf turret see https://dylanhammond-11.github.io/projects/auto-nerf-turret/index/
// Use alongside turret_vision_rgb.py for full implementation with the turret
// NOTE: This code is currently still in development alongside turret_vision_rgb.py and isn't fully functional
// Code by Dylan Hammond



# include <Servo.h>

Servo pan;
Servo tilt;
Servo load;
uint8_t panPin= 7 , tiltPin = 8 , loadPin = 10; // Servo pin numbers
uint8_t IN1 = 30, IN2 = 33, IN3 = 32, IN4 = 33; // Motor driver pin numbers
static int xCenter = 320 , yCenter = 240 ; // Pixel center of camera 
int pixelBuffer = 20; // Buffer range that determines when it is on-target and ready to fire
bool dataRecieved = false;
bool fired = false;

int xTarget = 0;
int yTarget = 0;
int sampleRate  = 100; // delay 

int maxBalloons = 6;// this number/2 is the balloon number
int redCoordinates[maxBalloons]; // Arrays to store coordinates, with maximum number of balloons each.
int greenCoordinates[maxBalloons];
int blueCoordinates[maxBalloons];


void fire(){
  uint8_t shootingSpeed = 100; // PWM for motor shooter
  unsigned long firingTime = millis();
  static unsigned long previousTime = 0;
  static unsigned long motorDelay = 100, servoDelay = 300; // Delays between servo and motor movement
  
  analogWrite(IN1, shootingSpeed); digitalWrite(IN2, LOW); // Wheel forwards
  analogWrite(IN3, shootingSpeed); digitalWrite(IN4, HIGH); // Wheel Backwards
  previousTime = firingTime;

  while (true){   
  if(firingTime - previousTime >= motorDelay){ 
    load.write(90); // Wait for the wheels to speed up before loading
    previousTime = firingTime;

    if(firingTime - previousTime >= servoDelay){  
  load.write(0); // Give the loader time to move before moving back to default position
  break;
  }
  }
}

digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); // Turn off the wheels
digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void aim(){

  if ((abs(xCenter-xTarget) < pixelBuffer) && (abs(yCenter-yTarget) < pixelBuffer)){
    fire();
    fired = true;

  }
}

float updatePID(float dt){

  float kpPan = 1, kiPan = 0, kdPan = 0; // Gains for Pan and Tilt PID Controller
  float kpTilt = 1, kiTilt = 0, kdTilt = 0;
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
  ////////////////////////////
  }

}

// function to operate shooting each color coordinate
void colorShoot(int coords[]){

  for(int i = 1, i<maxBalloons, i+=2){ //This loops through all of the coordinates in the array for a color
    xTarget = coords[i];
    yTarget = coords[i+1];

    while(abs(xTarget-xCenter)> pixelBuffer && abs()>pixelBuffer){
      updatePID();
    }
  }


}

void parseCoordinates( String coordinateData, int coords[]){
  int index = 0, lastIndex = 0, coordIndex = 0; //index: index of comma...lastIndex: index just after the last comma
 
  while(index = coordinateData.indexOf(',',lastIndex) != -1 && coordIndex < maxBalloons){
    coords[coordIndex++]=coordinateData.substring(lastIndex, index).toInt();
    lastIndex = index+1;
  }

  if(coordIndex < maxBalloons){
    coords[coordIndex++]=coordinateData.substring(lastIndex).toInt();
  }

  for(; coordIndex < maxBalloons){
    coords[coordIndex++]=-1;
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
  load.write(90);
}

void loop() {

  static unsigned long pastTime = 0; // Time counter that controls the timing of the pid updates
  unsigned long currentTime = millis();

    // Check if recieving data from python
 if(Serial.available()){
  String allCoordinates = Serial.ReadStringUntil('\n');
  allCoordinates.trim();

  int rStart = allCoordinates.indexOf('R:');
  int gStart = allCoordinates.indexOf('g:');
  int bStart = allCoordinates.indexOf('b:');

  if( rStart!=-1 && gStart!=-1 && bStart!=-1){ // Only operates if all colors found

    String allRCoords = allCoordinates.substring(rStart+2, gStart-1);
    String allGCoords = allCoordinates.substring(gStart+2,bStart-1);
    String allBCoords = allCoordinates.substring(bStart+2);

    parseCoordinates(allRCoords, redCoordinates);
    parseCoordinates(allGCoords, greenCoordinates);
    parseCoordinates(allBCoords, blueCoordinates);
  }
  dataRecieved = true;
}



}



  if (currentTime-pastTime >= sampleRate ) { // Controls PID update rate
    float dt = (currentTime - pastTime)/1000.00;
    updatePID(dt); 
    digitalWrite(LED_BUILTIN, LOW);

    pastTime = currentTime;
    
  }
    //aim(); // Check if on target, if so then fire
  }



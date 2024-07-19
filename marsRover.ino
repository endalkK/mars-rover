int board[24][12];
int startX;
int startY;
int orientation;
int already = 0;
//North = 0, east = 1, south = 2, west = 3
void displayBoard(){
  int x = 0;
  int y = 0;
  for (y=11;y>-1;y--){
    for (x=0; x<23; x++){
      Serial.print(board[x][y]);
      Serial.print("\t");
    }
    Serial.println(board[x][y]);
    
  }
}

//This creates objects and give them padding of 150
void createObjects(int a, int b){
  //padding
    for(int i = a-1; i < a+2; i++){
      for(int j = b-1; j < b+2; j++){
       if((0 <= i) && (i<24) && (0 <= j) && (j <12)){
          board[i][j] = 150;
        }        
      }
   }
}
//This function will implement wavefront planner motion planing. how far away you are from the goal determine your 
void waveFront(int goalX, int goalY){
  for(int i = 0; i < 24; i++){
      for(int j = 0; j < 12; j++){
        if(i <= goalX && j <= goalY){
          board[i][j] = (goalX - i) + (goalY - j);
        }
        else if(i >= goalX && j <= goalY){
          board[i][j] = (i - goalX) + (goalY - j);
        }
        else if(i <= goalX && j >= goalY){
          board[i][j] = (goalX - i) + (j - goalY);
        }
        else if(i >= goalX && j >= goalY){
          board[i][j] = (i - goalX) + (j - goalY);
        }
      }
   }
}
int return_orientation(){
  return orientation; 
}
void location(int i){
  if (i == 0)
    startY = startY + 1;
  else if (i == 1)
    startY = startY - 1;
  else if (i == 2)
    startX = startX + 1;
  else if (i == 3)
    startX = startX - 1;
}

void move(int orien, int dxn, int min){
  if (orien == dxn){
      Serial.print("F, ");
      Serial.println(min);
      MoveForward(10);
      Stop();
    }
  else{
    if((orien ==0 && dxn ==2) || (orien ==1 && dxn ==2) || (orien ==2 && dxn ==1) || (orien ==3 && dxn ==0)){
      orientation = dxn;
      Serial.print("R, ");
      Serial.println(min);
      TurnRight();
    }
    else if((orien ==0 && dxn ==3) || (orien ==1 && dxn ==3) || (orien ==2 && dxn ==0) || (orien ==3 && dxn ==1)){
      orientation = dxn;
      Serial.print("L, ");
      Serial.println(min);
      TurnLeft();
    }
    else if((orien ==0 && dxn ==1) || (orien ==1 && dxn ==0) || (orien ==2 && dxn ==3) || (orien ==3 && dxn ==2)){
      already = 1;
      orientation = dxn;
      Serial.print("180 F, ");
      Serial.println(min);
      MoveForward(10);
      Stop();
    }
  }
}

void findPath(){ 
 
  while(board[startX][startY] != 0){
  int min = 150; // float('inf') in python, since our obstacle is the maximum we will have on the board
  int index = 0;
  int negh[] = {150, 150, 150, 150}; //board[startX][startY+1], board[startX][startY-1], board[startX+1][startY], board[startX-1][startY]

  if (startY+1 < 12)
      negh[0] = board[startX][startY+1];
  if (startY-1 > 0)
      negh[1] = board[startX][startY-1];
  if (startX+1 < 24)
      negh[2] = board[startX+1][startY];
  if (startX-1 > 0)
      negh[3] = board[startX-1][startY];
  //
  if (already == 1)
    board[startX][startY] = 150;

  for(int i = 0; i < 4; i++){
    if (negh[i]< min){
      min = negh[i];
      index = i;
    }
  }
  move(return_orientation(), index, min);
  location(index);
  }
}

//_____________________________________

//Encoder Stuff!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#define RH_ENCODER_A 13 
#define RH_ENCODER_B 12
#define LH_ENCODER_A 11
#define LH_ENCODER_B 10

volatile int lastRightEncoded = 0;
volatile int lastLeftEncoded = 0;
volatile long rightEncoderValue = 0;
volatile long leftEncoderValue = 0;

volatile long leftCount = 0;
volatile long rightCount = 0;

//neokey stuff-----
#include <Adafruit_NeoPixel.h>
// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        9 
#define NUMPIXELS 2
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);
//------------------------------------------------------------------------------------
//motor junk--------------------------------------------------
#include <Adafruit_MotorShield.h>
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myLeftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *myRightMotor = AFMS.getMotor(1);
//-------------------------------------------------------------
int buttonCheck1 = 0; //checks if button is pressed down (there is no issue with bounce)
int buttonCheck2 = 0;



void ISR_ChangeColor1() {
  //Serial.println("Bob");
  //pixels.setPixelColor(0, pixels.Color(150, 150, 0));
  buttonCheck1 = 1;
  buttonCheck2 = 0;
}

void ISR_ChangeColor2() {
  //Serial.println("Fred");
  //pixels.setPixelColor(1, pixels.Color(0, 150, 0));
  buttonCheck2 = 1;
  buttonCheck1 = 0;
  Stop();
}

//ENCODERS---------------------------------------------------
void updateRightEncoder(){
  int MSB = digitalRead(RH_ENCODER_A); //MSB = most significant bit
  int LSB = digitalRead(RH_ENCODER_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastRightEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) rightEncoderValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) rightEncoderValue ++;

  lastRightEncoded = encoded; //store this value for next time
}

void updateLeftEncoder(){
  int MSB = digitalRead(LH_ENCODER_A); //MSB = most significant bit
  int LSB = digitalRead(LH_ENCODER_B); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (lastLeftEncoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) leftEncoderValue ++;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) leftEncoderValue --;

  lastLeftEncoded = encoded; //store this value for next time
}
void resetEncoderCount(){
  lastRightEncoded = 0;
  lastLeftEncoded = 0;
  rightEncoderValue = 0;
  leftEncoderValue = 0;
}



void TurnRight(){
    int kpl =  5;
  int kil = 0.5;
  int kdl= 0.05;
  

  int kpr = 5;
  int kir = 0.5;
  int kdr = 0.05;

  //int past_error = 0;
  int leftError = 0;
  int leftS_error = 0;
  int leftD_error = 0;
  int leftPast_error = 0;

  int rightError = 0;
  int rightS_error = 0;
  int rightD_error = 0;
  int rightPast_error = 0;

  //20cm is about 1360 - 1380 ticks

  int rightThreshold = 0;
  int rightGoal = 600;
  int leftThreshold = 0;
  int leftGoal = 600;//neg

  resetEncoderCount();

  //int avgEnc = 0;
  leftError = leftGoal - leftEncoderValue;
  rightError = rightGoal - rightEncoderValue;
  
  while(leftError > leftThreshold && rightError > rightThreshold){
    //Serial.println("in while loop - left");
    
    //avgEnc = (leftEncoderValue + rightEncoderValue)/2;
    //error = goal - avgEnc;
    leftError = leftGoal - leftEncoderValue;
    rightError = rightGoal - rightEncoderValue;
    //past_error = (lastLeftEncoded + lastRightEncoded)/2;
    leftPast_error = lastLeftEncoded;
    rightPast_error = lastRightEncoded;

    rightS_error = rightS_error + rightPast_error;
    rightD_error = rightPast_error - rightError;

    leftS_error = leftS_error + leftPast_error;
    leftD_error = leftPast_error - leftError;
    int leftSpeed = correctSpeed((kpl * leftError) + (kil * leftS_error) + (kdl * leftD_error));
    int rightSpeed = correctSpeed((kpr * rightError) + (kir * rightS_error) + (kdr * rightD_error));

    myLeftMotor->run(FORWARD);
    myRightMotor->run(BACKWARD);
    myLeftMotor->setSpeed(leftSpeed);
    myRightMotor->setSpeed(rightSpeed);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println("left threshold");
    // Serial.println(leftThreshold);
    // Serial.println("Left Error");
    // Serial.println(leftError);
 
  }
}
int correctSpeed(int speed){
  if (speed > 200){
    return 200;
  }
  else if (speed < 0){
    return 0;
  }
  return speed;
}
void TurnLeft(){
 
  int kpl =  5;
  int kil = 0.5;
  int kdl= 0.05;
  

  int kpr = 5;
  int kir = 0.5;
  int kdr = 0.05;

  //int past_error = 0;
  int leftError = 0;
  int leftS_error = 0;
  int leftD_error = 0;
  int leftPast_error = 0;

  int rightError = 0;
  int rightS_error = 0;
  int rightD_error = 0;
  int rightPast_error = 0;

  //20cm is about 1360 - 1380 ticks

  int rightThreshold = 0;
  int rightGoal = 530;
  int leftThreshold = 0;
  int leftGoal = 600;//neg

  resetEncoderCount();

  //int avgEnc = 0;
  leftError = leftGoal - leftEncoderValue;
  rightError = rightGoal - rightEncoderValue;
  
  while(leftError > leftThreshold && rightError > rightThreshold){
    //Serial.println("in while loop - left");
    
    //avgEnc = (leftEncoderValue + rightEncoderValue)/2;
    //error = goal - avgEnc;
    leftError = leftGoal - leftEncoderValue;
    rightError = rightGoal - rightEncoderValue;
    //past_error = (lastLeftEncoded + lastRightEncoded)/2;
    leftPast_error = lastLeftEncoded;
    rightPast_error = lastRightEncoded;

    rightS_error = rightS_error + rightPast_error;
    rightD_error = rightPast_error - rightError;

    leftS_error = leftS_error + leftPast_error;
    leftD_error = leftPast_error - leftError;
    int leftSpeed = correctSpeed((kpl * leftError) + (kil * leftS_error) + (kdl * leftD_error));
    int rightSpeed = correctSpeed((kpr * rightError) + (kir * rightS_error) + (kdr * rightD_error));

    myLeftMotor->run(BACKWARD);
    myRightMotor->run(FORWARD);
    myLeftMotor->setSpeed(leftSpeed);
    myRightMotor->setSpeed(rightSpeed);
    // Serial.println(leftSpeed);
    // Serial.println(rightSpeed);
    // Serial.println("left threshold");
    // Serial.println(leftThreshold);
    // Serial.println("Left Error");
    // Serial.println(leftError);
    //myLeftMotor->setSpeed(250);
    //myRightMotor->setSpeed(250);
  }
}

void Stop(){
  myLeftMotor->run(RELEASE);
  myRightMotor->run(RELEASE);
  myLeftMotor->setSpeed(0);
  myRightMotor->setSpeed(0);
}

void MoveForward(int cm){
  //Serial.println("MoveForward");

  int wheel_count = cm * (1395/20);
 
  int kpl =  5;
  int kil = 0.5;
  int kdl= 0.05;
  

  int kpr = 5;
  int kir = 0.5;
  int kdr = 0.05;

  //int past_error = 0;
  int leftError = 0;
  int leftS_error = 0;
  int leftD_error = 0;
  int leftPast_error = 0;

  int rightError = 0;
  int rightS_error = 0;
  int rightD_error = 0;
  int rightPast_error = 0;

  //20cm is about 1360 - 1380 ticks

  int rightThreshold = 0;
  int rightGoal = wheel_count;
  int leftThreshold = 0;
  int leftGoal = wheel_count;

  resetEncoderCount();

  //int avgEnc = 0;
  leftError = leftGoal - leftEncoderValue;
  rightError = rightGoal - rightEncoderValue;
  
  while(leftError > leftThreshold && rightError > rightThreshold){
    //Serial.println("in while loop");
    
    //avgEnc = (leftEncoderValue + rightEncoderValue)/2;
    //error = goal - avgEnc;
    leftError = leftGoal - leftEncoderValue;
    rightError = rightGoal - rightEncoderValue;
    //past_error = (lastLeftEncoded + lastRightEncoded)/2;
    leftPast_error = lastLeftEncoded;
    rightPast_error = lastRightEncoded;

    rightS_error = rightS_error + rightPast_error;
    rightD_error = rightPast_error - rightError;

    leftS_error = leftS_error + leftPast_error;
    leftD_error = leftPast_error - leftError;
    int leftSpeed = correctSpeed((kpl * leftError) + (kil * leftS_error) + (kdl * leftD_error));
    int rightSpeed = correctSpeed((kpr * rightError) + (kir * rightS_error) + (kdr * rightD_error));

    myLeftMotor->run(FORWARD);
    myRightMotor->run(FORWARD);
    myLeftMotor->setSpeed(leftSpeed);
    myRightMotor->setSpeed(rightSpeed);
    //myLeftMotor->setSpeed(250);
    //myRightMotor->setSpeed(250);
  }
}


void setup() {
  Serial.begin(9600);
  delay(1000);
    // This junk makes the keys glow_________
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  pixels.setPixelColor(1, pixels.Color(150, 0, 150));
  pixels.show();
  //________________________________________
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  attachInterrupt(7, ISR_ChangeColor1, LOW); //when button1 is LOW, execute ISR_ChangeColor1
  attachInterrupt(8, ISR_ChangeColor2, LOW);
  //Serial.println ("There.");
  AFMS.begin();
  //encoder
  pinMode(LH_ENCODER_A, INPUT_PULLUP);
  pinMode(LH_ENCODER_B, INPUT_PULLUP);
  pinMode(RH_ENCODER_A, INPUT_PULLUP);
  pinMode(RH_ENCODER_B, INPUT_PULLUP);
  //digitalWrite(RH_ENCODER_A, HIGH);
  //digitalWrite(RH_ENCODER_A, HIGH);
  // initialize hardware interrupts
  //attachInterrupt(4, leftEncoderEvent, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(2) , rightEncoderEvent, CHANGE);
  attachInterrupt(RH_ENCODER_A , updateRightEncoder, CHANGE);
  attachInterrupt(RH_ENCODER_B , updateRightEncoder, CHANGE);
  attachInterrupt(LH_ENCODER_A , updateLeftEncoder, CHANGE);
  attachInterrupt(LH_ENCODER_B , updateLeftEncoder, CHANGE);
  //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1
  //left most points of the resource
  int goalX[5] ={3, 4, 12, 11, 18};
  int goalY[5] = {2, 9,  9, 2, 5};

  waveFront(goalX[0], goalY[0]);

  //object 
  createObjects(2,9);
  createObjects(5,5);
  createObjects(10,8);
  createObjects(9,3);

  createObjects(13,5);
  createObjects(14,5);

  createObjects(17,9);
  createObjects(17,3);
  
  delay(2000);
  displayBoard();
  orientation = 2;
  startX = 0;
  startY = 10;
  delay(3000);
  findPath();
  /*
  for(int j = 0; j < 4; j++){
    Serial.println();
    startX = goalX[j];
    startY = goalY[j];
    waveFront(goalX[j+1], goalY[j+1]);
    //object 
    createObjects(2,9);
    createObjects(5,5);
    createObjects(10,8);
    createObjects(9,3);

    createObjects(13,5);
    createObjects(14,5);

    createObjects(17,9);
    createObjects(17,3);
    displayBoard();
    findPath();
    } */
}

void loop() {
  // put your main code here, to run repeatedly:
}

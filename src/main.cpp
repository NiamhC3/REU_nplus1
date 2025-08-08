#include <Arduino.h>
#include "ODriveCAN.h"

//  Defines the bitrate and node IDs of each odrive.
#define CAN_BAUDRATE 250000
#define ODRV0_NODE_ID 0
#define ODRV1_NODE_ID 1
#define ODRV2_NODE_ID 2
#define ODRV3_NODE_ID 3


// See lib_deps 
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // hack to prevent teensy compile error


// ---------------------CAN Setup---------------------------------------------------------
// Declares function onCanMessage to be defined later
void onCanMessage(const CanMsg& msg);

// Initializes the CAN bus on the teensy. This defines the RX and TX pins being used.
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can_intf;

// Sets up CAN on the teensy side
bool setupCan() {
  can_intf.begin();
  // Reset CAN controller to clear any previous error states
  can_intf.reset();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// UPDATE THIS for additional odrives
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID); // Standard CAN message ID
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
ODriveCAN odrv3(wrap_can_intf(can_intf), ODRV3_NODE_ID);
ODriveCAN* odrives[] = {&odrv0,&odrv1,&odrv2,&odrv3}; // Make sure all ODriveCAN instances are accounted for here

// General structure for ODrive data
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  Get_Encoder_Estimates_msg_t last_feedback;
  bool received_feedback = false;
};


// Keep some application-specific user data for every ODrive.
ODriveUserData odrv0_user_data;
ODriveUserData odrv1_user_data;
ODriveUserData odrv2_user_data;
ODriveUserData odrv3_user_data;

// Called every time a Heartbeat message arrives from the ODrive
// Sets variable "received_heartbeat" of "odrv_user_data" to true
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_heartbeat = msg;
  odrv_user_data->received_heartbeat = true;
}


// Called every time a feedback message arrives from the ODrive
// Sets variable "recieved_feedback" of "odrv_user_data" to true
void onFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  ODriveUserData* odrv_user_data = static_cast<ODriveUserData*>(user_data);
  odrv_user_data->last_feedback = msg;
  odrv_user_data->received_feedback = true;
}


// Called for every message that arrives on the CAN bus
void onCanMessage(const CanMsg& msg) {
  for (auto odrive : odrives) {
    onReceive(msg, *odrive);
  }
}


// -------------------Control Functions--------------------------------------------------

// Calculates the required motor tourqes to output desired joint torques. Updates array motorTorques with new values.
// Takes float minTorque to set the lowest torque of the motor torques to. This allows editing of the magnitude of motor torques
// Function has a hardcoded 0.025 Nm torque cap. Sets all torques to 0 and prints an error if this is exceeded.
// This function uses linear approximation instead of actual relationship between coupled angles
void motorTorqueCalculation(float jointTorques[3], float motorTorques[4], float minTorque, bool feedback){
  int i = 0, j = 0;
  for (i = 0; i < 4; i++){
    motorTorques[i] = 0;
  }
  float smallestTorque = 0;
  float nullSpaceAdjustment = 0;
  float pseudoInverse[4][3] = 
    {{-0.008562,  0.04838, -0.1696}
    ,{0.06498,    0.01748, -0.1446}
    ,{-0.06149,  -0.01341,  0.04701}
    ,{0.06498,    0.01748,  0.02205}};

  float nullSpace[4] = {1.145417, 1, 0.854583, 1};


  // Computes the product of the pseudo inverse and the desired joint torques
  for (i = 0; i < 4; i++){
    for (j = 0; j < 3; j++){
      motorTorques[i] += (pseudoInverse[i][j] * jointTorques[j])*7.5;
    }
  }

  // Determines the smallest torque out of the raw torques, and sets variables to be used to adjust the torques to reasonable values
  if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
    smallestTorque = motorTorques[0];
    nullSpaceAdjustment = nullSpace[0];
  }
  else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
    smallestTorque = motorTorques[1];
    nullSpaceAdjustment = nullSpace[1];
  }
  else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
    smallestTorque = motorTorques[2];
    nullSpaceAdjustment = nullSpace[2];
  }
  else{
    smallestTorque = motorTorques[3];
    nullSpaceAdjustment = nullSpace[3];
  }
  
  //Adjusts the lowest torque to 0.005. This values was chosen as the minimum value for the motors to overcome internal resistance
  for (i = 0; i < 4; i++){
    motorTorques[i] += (nullSpace[i] * ((minTorque - smallestTorque)/nullSpaceAdjustment));
  }


  float largestTorque = 0;
  if ((motorTorques[0] > 0.025) || (motorTorques[1] > 0.025) || (motorTorques[2] > 0.025) || (motorTorques[3] > 0.025)){
    if ((motorTorques[0] >= motorTorques[1]) && (motorTorques[0] >= motorTorques[2]) && (motorTorques[0] >= motorTorques[3])){
      largestTorque = motorTorques[0];
    }
    else if ((motorTorques[1] >= motorTorques[0]) && (motorTorques[1] >= motorTorques[2]) && (motorTorques[1] >= motorTorques[3])){
      largestTorque = motorTorques[1];
    }
    else if ((motorTorques[2] >= motorTorques[1]) && (motorTorques[2] >= motorTorques[0]) && (motorTorques[2] >= motorTorques[3])){
      largestTorque = motorTorques[2];
    }
    else {
      largestTorque = motorTorques[3];
    }


    if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
      smallestTorque = motorTorques[0];
      nullSpaceAdjustment = nullSpace[0];
    }
    else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
      smallestTorque = motorTorques[1];
      nullSpaceAdjustment = nullSpace[1];
    }
    else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
      smallestTorque = motorTorques[2];
      nullSpaceAdjustment = nullSpace[2];
    }
    else{
      smallestTorque = motorTorques[3];
      nullSpaceAdjustment = nullSpace[3];
    }
    
    for (i = 0; i < 4; i++){
      motorTorques[i] = (motorTorques[i] / (largestTorque - smallestTorque)) * 0.02;
    }

    if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
      smallestTorque = motorTorques[0];
      nullSpaceAdjustment = nullSpace[0];
    }
    else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
      smallestTorque = motorTorques[1];
      nullSpaceAdjustment = nullSpace[1];
    }
    else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
      smallestTorque = motorTorques[2];
      nullSpaceAdjustment = nullSpace[2];
    }
    else{
      smallestTorque = motorTorques[3];
      nullSpaceAdjustment = nullSpace[3];
    }
    
    for (i = 0; i < 4; i++){
      motorTorques[i] += (nullSpace[i] * ((minTorque - smallestTorque)/nullSpaceAdjustment));
    }
  }

  if (feedback){
    Serial.println("Motor Torques: ");
    Serial.println(motorTorques[0],6);
    Serial.println(motorTorques[1],6);
    Serial.println(motorTorques[2],6);
    Serial.println(motorTorques[3],6);
  }
}


// Balances torque values to be in the range minTorque to 0.025
void torqueBalancingCalculation(float motorTorques[4], float minTorque){
  int i = 0;
  float smallestTorque = 0, largestTorque = 0;
  float nullSpaceAdjustment = 0;

  float nullSpace[4] = {1.145417, 1, 0.854583, 1};

  // Determines the smallest torque out of the raw torques, and sets variables to be used to adjust the torques to reasonable values
  if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
    smallestTorque = motorTorques[0];
    nullSpaceAdjustment = nullSpace[0];
  }
  else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
    smallestTorque = motorTorques[1];
    nullSpaceAdjustment = nullSpace[1];
  }
  else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
    smallestTorque = motorTorques[2];
    nullSpaceAdjustment = nullSpace[2];
  }
  else{
    smallestTorque = motorTorques[3];
    nullSpaceAdjustment = nullSpace[3];
  }
  
  //Adjusts the lowest torque to 0.005. This values was chosen as the minimum value for the motors to overcome internal resistance
  for (i = 0; i < 4; i++){
    motorTorques[i] += (nullSpace[i] * ((minTorque - smallestTorque)/nullSpaceAdjustment));
  }

  //Scales torques to a range, and reshifts if any torque exceeds limit
  if ((motorTorques[0] > 0.025) || (motorTorques[1] > 0.025) || (motorTorques[2] > 0.025) || (motorTorques[3] > 0.025)){
    // Finds the largest torque
    if ((motorTorques[0] >= motorTorques[1]) && (motorTorques[0] >= motorTorques[2]) && (motorTorques[0] >= motorTorques[3])){
      largestTorque = motorTorques[0];
    }
    else if ((motorTorques[1] >= motorTorques[0]) && (motorTorques[1] >= motorTorques[2]) && (motorTorques[1] >= motorTorques[3])){
      largestTorque = motorTorques[1];
    }
    else if ((motorTorques[2] >= motorTorques[1]) && (motorTorques[2] >= motorTorques[0]) && (motorTorques[2] >= motorTorques[3])){
      largestTorque = motorTorques[2];
    }
    else {
      largestTorque = motorTorques[3];
    }

    // Finds the smallest torque
    if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
      smallestTorque = motorTorques[0];
      nullSpaceAdjustment = nullSpace[0];
    }
    else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
      smallestTorque = motorTorques[1];
      nullSpaceAdjustment = nullSpace[1];
    }
    else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
      smallestTorque = motorTorques[2];
      nullSpaceAdjustment = nullSpace[2];
    }
    else{
      smallestTorque = motorTorques[3];
      nullSpaceAdjustment = nullSpace[3];
    }
    
    // Scales torques to have range 0.02
    for (i = 0; i < 4; i++){
      motorTorques[i] = (motorTorques[i] / (largestTorque - smallestTorque)) * 0.02;
    }

    // Finds new smallest torque
    if ((motorTorques[0] <= motorTorques[1]) && (motorTorques[0]<= motorTorques[2]) && (motorTorques[0] <= motorTorques[3])){
      smallestTorque = motorTorques[0];
      nullSpaceAdjustment = nullSpace[0];
    }
    else if ((motorTorques[1] <= motorTorques[0]) && (motorTorques[1]<= motorTorques[2]) && (motorTorques[1] <= motorTorques[3])){
      smallestTorque = motorTorques[1];
      nullSpaceAdjustment = nullSpace[1];
    }
    else if ((motorTorques[2] <= motorTorques[0]) && (motorTorques[2]<= motorTorques[1]) && (motorTorques[2] <= motorTorques[3])){
      smallestTorque = motorTorques[2];
      nullSpaceAdjustment = nullSpace[2];
    }
    else{
      smallestTorque = motorTorques[3];
      nullSpaceAdjustment = nullSpace[3];
    }
    
    // Shifts set of torques up so that the lowest is minTorque
    for (i = 0; i < 4; i++){
      motorTorques[i] += (nullSpace[i] * ((minTorque - smallestTorque)/nullSpaceAdjustment));
    }
  }
  
}


// Calculates the required motor angle displacements in terms of angular displacement from most clockwise position, (fully upright)
// Does account for conversion between motor angles and "position" of the motors.
// Does account for any motor position offsets
// This function uses linear approximation instead of actual relationship between coupled angles
void  motorPositionCalculation(float jointAngles[3], float motorPositions[4], float motorOffsets[4], bool feedback){
  int i = 0, j = 0;
  // Initializes motor angles to 0
  float motorAngles[4] = {0,0,0,0};
  float jointJacobian[4][3] = 
  {{-5, 19.7308, 0},
   {5, -19.7308, -6},
   {-5, 0, 0},
   {5, -2.8692, 6}};

  for (i = 0; i < 4; i++){
    for (j = 0; j < 3; j++){
      motorAngles[i] += jointJacobian[i][j] * jointAngles[j];
    }
    motorAngles[i] = (motorAngles[i])/(7.5); // Confirm units
  }

  float angleConversion[4] = {-1,-1,1,1}; // Only relevant if it isn't possible to have the Odrives read the position accurately.
  for (i = 0; i < 4; i++){
    motorPositions[i] += motorOffsets[i];
    motorPositions[i] += ((motorAngles[i] / (6.2831853) )* angleConversion[i]);
  }

  if (feedback){
    Serial.println("Motor Positions");
    Serial.println(motorPositions[0],6);
    Serial.println(motorPositions[1],6);
    Serial.println(motorPositions[2],6);
    Serial.println(motorPositions[3],6);
  }
}


// Calculates the joint angles from the motor positions. Outputs the result in radians
void jointPositionCalculation(float jointAngles[3], float motorPositions[4], float motorOffsets[4]){
  int i = 0;
  float motorAngles[4] = {0,0,0,0};
  float angleConversion[4] = {-1,-1,1,1};
  float inverseJacobian[3][4] = // Pseudo inverse
  {{0.04843, 0.04228208, -0.16386647, 0.04228208},
   {0.04650544, -0.003646479, -0.0537984, -0.003646479},
   {-0.0653463, -0.0782095, 0.0755939, 0.0884571}};
  for (i = 0; i < 4; i++){
    motorAngles[i] = (motorPositions[i] - motorOffsets[i]) * 6.283185;
  }
  for (i = 0; i < 3; i++){
        jointAngles[i] = 7.5 * (inverseJacobian[i][0]*motorAngles[0]*angleConversion[0] + 
                                inverseJacobian[i][1]*motorAngles[1]*angleConversion[1] + 
                                inverseJacobian[i][2]*motorAngles[2]*angleConversion[2] + 
                                inverseJacobian[i][3]*motorAngles[3]*angleConversion[3]);
  }

}

// Prints the current position of the ODrive
float currentPosition(ODriveUserData &user_data, bool serialFeedback) {
  pumpEvents(can_intf);
  Get_Encoder_Estimates_msg_t feedback = user_data.last_feedback;
  user_data.received_feedback = false;
   if (serialFeedback){
    Serial.print("Current position: ");
    Serial.println(feedback.Pos_Estimate, 6);
  }
  return feedback.Pos_Estimate;
}

float currentVelocity(ODriveUserData &user_data, bool serialFeedback) {
  pumpEvents(can_intf);
  Get_Encoder_Estimates_msg_t feedback = user_data.last_feedback;
  user_data.received_feedback = false;
   if (serialFeedback){
    Serial.print("Current position: ");
    Serial.println(feedback.Vel_Estimate, 6);
  }
  return feedback.Vel_Estimate;
}

// Adds a delay until new feedback is recieved. This is used before refreshing positions to ensure the most recent position is updated
// Needs to be optimized! Takes too long, is restricting frequency of updates
void waitForNewFeedback(ODriveUserData &user_data, int userDelay) {
  unsigned long start = millis();
  user_data.received_feedback = false;
  while (!user_data.received_feedback && (millis() - start < userDelay)) {
    pumpEvents(can_intf);
    delayMicroseconds(100);
  }

}

// Calculates joint torques due to gravity, and returns the equivalent motor torques. (Subtract these values to counteract gravitation torque)
void gravityFeedforward(float motorPositions[4], float motorOffsets[4], float forwardJointTorques[3]){
  float jointAngles[3] = {0,0,0};
  float segmentMasses[4] = {0.033,0.027,0.019, 0.009};
  jointPositionCalculation(jointAngles, motorPositions, motorOffsets); // Calculates current joint angles
  // COMCoords is the coordinates of the {splay, proximal, medial, distal} segments
  float COMCoords[4][2] = {{0,0},{0,0},{0,0},{0,0}}; // x coordinate refers to torque applying direction from axis of rotation, perpendicular to gravity.
  float topOfSegment[4][2] = {{0,0},{0,0},{0,0},{0,0}};
  COMCoords[0][1] = 17.057;
  topOfSegment[0][1] = 29.88;
  COMCoords[1][0] = 30.339 * sin(jointAngles[1]);
  COMCoords[1][1] = 30.339 * cos(jointAngles[1]) + topOfSegment[0][1];
  topOfSegment[1][0] = 50 * sin(jointAngles[1]);
  topOfSegment[1][1] = 50 * cos(jointAngles[1]) + topOfSegment[0][1];
  COMCoords[2][0] = 17.897 * sin(2.6218 * jointAngles[1]) + topOfSegment[1][0];
  COMCoords[2][1] = 17.897 * cos(2.6218 * jointAngles[1]) + topOfSegment[1][1];
  topOfSegment[2][0] = 30 * sin(2.6218 * jointAngles[1]) + topOfSegment[1][0];
  topOfSegment[2][1] = 30 * cos(2.6218 * jointAngles[1]) + topOfSegment[1][1];
  COMCoords[3][0] = 16.674 * sin(2.6218 * jointAngles[1] + jointAngles[2]) + topOfSegment[2][0];
  COMCoords[3][1] = 16.674 * cos(2.6218 * jointAngles[1] + jointAngles[2]) + topOfSegment[2][1];

  forwardJointTorques[0] = -0.00475 + sin(jointAngles[0] - 0.37) * 3.4/1000 * 
                          (COMCoords[0][1] * segmentMasses[0] +
                          COMCoords[1][1] * segmentMasses[1] + 
                          COMCoords[2][1] * segmentMasses[2] +
                          COMCoords[3][1] * segmentMasses[3]);

  forwardJointTorques[1] = 9.81/1000 * (COMCoords[1][0] * segmentMasses[1] + 
                                        COMCoords[2][0] * segmentMasses[2] + 
                                        COMCoords[3][0] * segmentMasses[3])/(1.775) + (0.03 * jointAngles[0]);  
  
  forwardJointTorques[2] = 16.674 * sin(2.6218 * jointAngles[1] + jointAngles[2]) * segmentMasses[2] * 9.81/1000;

}


void readSerialInput(float tempJointAngles[3]) {
  static String inputString = "";
  
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      // Full message received, parse it
      float splay, proximal, distal;
      int numParsed = sscanf(inputString.c_str(), "%f,%f,%f", &splay, &proximal, &distal);

      if (numParsed == 3) {
        tempJointAngles[0] = splay;
        tempJointAngles[1] = proximal;
        tempJointAngles[2] = distal;
      } else {

      }

      inputString = "";  // Clear for next input
    } else {
      inputString += inChar;
    }
  }
}

// ------------------Odrive Calibration and Setup-----------------------------------------
// Run in main after setup. Moves motor to fully upright clockwise position, and records offsets
void positionOffsets(float motorOffsets[4], bool feedback){
  odrv0.setTorque(-0.014774); // Negative Applies Torque
  odrv1.setTorque(-0.01); // Negative Applies Torque
  odrv2.setTorque(0.02); // Positive Applies Torque
  odrv3.setTorque(0.01); // Positive Applies Torque

  delay(250);
  odrv0.setTorque(-0.0075); // Negative Applies Torque
  odrv1.setTorque(-0.023); // Negative Applies Torque
  odrv2.setTorque(0.025); // Positive Applies Torque
  odrv3.setTorque(0.0075); // Positive Applies Torque
  Serial.println("Getting netural positions...");
  delay(2000);

  waitForNewFeedback(odrv0_user_data, 100);
  waitForNewFeedback(odrv1_user_data, 100);
  waitForNewFeedback(odrv2_user_data, 100);
  waitForNewFeedback(odrv3_user_data, 100);
  motorOffsets[0] = currentPosition(odrv0_user_data,0);
  motorOffsets[1] = currentPosition(odrv1_user_data,0);
  motorOffsets[2] = currentPosition(odrv2_user_data,0);
  motorOffsets[3] = currentPosition(odrv3_user_data,0);
  if (feedback){
    Serial.println("Motor Offsets: ");
    Serial.println(motorOffsets[0],6);
    Serial.println(motorOffsets[1],6);
    Serial.println(motorOffsets[2],6);
    Serial.println(motorOffsets[3],6);  
  }
}


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) {
    delay(200);
  }



  odrv0.onFeedback(onFeedback, &odrv0_user_data);
  odrv0.onStatus(onHeartbeat, &odrv0_user_data);

  odrv1.onFeedback(onFeedback, &odrv1_user_data);
  odrv1.onStatus(onHeartbeat, &odrv1_user_data);

  odrv2.onFeedback(onFeedback, &odrv2_user_data);
  odrv2.onStatus(onHeartbeat, &odrv2_user_data);
  
  odrv3.onFeedback(onFeedback, &odrv3_user_data);
  odrv3.onStatus(onHeartbeat, &odrv3_user_data);

  if (!setupCan()) {
    Serial.println("CAN failed to initialize: reset required");
    while (true);
  }

  
  Serial.println("Waiting for ODrive 0...");
  while (!odrv0_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("Found ODrive 0");


  Serial.println("Waiting for ODrive 1...");
  while (!odrv1_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("Found ODrive 1");

  Serial.println("Waiting for ODrive 2...");
  while (!odrv2_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("Found ODrive 2");


  Serial.println("Waiting for ODrive 3...");
  while (!odrv3_user_data.received_heartbeat) {
    pumpEvents(can_intf);
  }
  Serial.println("Found ODrive 3");  


  Serial.println("Clearing errors...");
  odrv0.clearErrors();
  odrv1.clearErrors();
  odrv2.clearErrors();
  odrv3.clearErrors();
  delay(50);  // allow clearErrors to propagate

  odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv1.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv2.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  odrv3.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL, ODriveInputMode::INPUT_MODE_PASSTHROUGH);
  
  delay(1000);
  Serial.println("Enabling closed loop control...");
  pumpEvents(can_intf);
  Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
  Serial.println(odrv1_user_data.last_heartbeat.Axis_State);
  Serial.println(odrv2_user_data.last_heartbeat.Axis_State);
  Serial.println(odrv3_user_data.last_heartbeat.Axis_State);
  while ((odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) ||
        (odrv1_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) ||  
        (odrv2_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) ||
         (odrv3_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL)) {


    odrv0.clearErrors();
    odrv1.clearErrors();
    odrv2.clearErrors();
    odrv3.clearErrors();
    delay(500);
    if (odrv0_user_data.last_heartbeat.Axis_State != 8){
      odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }
    
    if (odrv1_user_data.last_heartbeat.Axis_State != 8){
      odrv1.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    if (odrv2_user_data.last_heartbeat.Axis_State != 8){
      odrv2.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    if (odrv3_user_data.last_heartbeat.Axis_State != 8){
      odrv3.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    }

    delay(1);
    pumpEvents(can_intf);
    Serial.println("Attempting closed loop...");
    Serial.println(odrv0_user_data.last_heartbeat.Axis_State);
    Serial.println(odrv1_user_data.last_heartbeat.Axis_State);
    Serial.println(odrv2_user_data.last_heartbeat.Axis_State);
    Serial.println(odrv3_user_data.last_heartbeat.Axis_State);
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(can_intf);
    }
  }
  
  Serial.println("ODrive running!");
  delay(1000);
}


// -------------------Odrive Main Function------------------------------------------------


void loop() {
  Serial.println("In main");
  // Initializing variables to store various motor values. All values are for motors {0, 1, 2, 3}, or joints {splay, proximal, distal}
  float tempMotorTorques[4] ={0,0,0,0}; // Stores calculated motor torques
  float tempMotorPositions[4] = {0,0,0,0}; // Stores the motor positions to get desired angle positions
  float tempMotorOffsets[4] = {0,0,0,0}; // Stores recorded motor position offsets
  float tempMotorAngles[4] = {0,0,0,0}; // Stores calculated motor angles 
  float forwardJointTorques[3] = {0,0,0};
  float gravityMotorTorques[4] = {0,0,0,0};
  float actualMotorPositions[4] = {0,0,0,0};
  float gravityJointTorques[3] = {0,0,0};
  float actualMotorVelocities[4] = {0,0,0,0};
  float tempJointTorques[3] = {0,0,0};
  int i = 0, j = 0;

  // User variables
  float tempJointAngles[3] = {0,0.6,0.6}; // {<splay>, <proximal>, <distal>}
  float actualJointAngles[3] = {0,0,0};
  float minTorque = 0.005; // Value the lowest motor torque is scaled to
  bool serialFeedback[5] = {0,0,0,0};   // A value of 0 prints no feedback. A value of 1 prints feedback, {<motorTorqueCalculation>, <motorPositionCalculation>, <currentPosition>, <positionOffsets>}

  // Runs a calibration sequence to collect the motor offsets
  positionOffsets(tempMotorOffsets, serialFeedback[3]);

  // Calculates the motor torques required to get tempJointTorques. Scales lowest to minTorques
  motorTorqueCalculation(tempJointTorques, tempMotorTorques, minTorque, serialFeedback[0]);

  // Calculates and prints the motor positions calculated from the inputted joint angles
  motorPositionCalculation(tempJointAngles, tempMotorPositions, tempMotorOffsets, 0);
  /*Serial.println("Desired motor positions");
  Serial.println(tempMotorPositions[0],6);
  Serial.println(tempMotorPositions[1],6);
  Serial.println(tempMotorPositions[2],6);
  Serial.println(tempMotorPositions[3],6);*/
  static unsigned long lastSend = 0; // For python

  while(1){
    i += 1;

    if (millis() - lastSend > 100) {
      lastSend = millis();
      Serial.print("DATA,");
      Serial.print(tempMotorTorques[0], 6);
      Serial.print(",");
      Serial.print(tempMotorTorques[1], 6);
      Serial.print(",");
      Serial.print(tempMotorTorques[2], 6);
      Serial.print(",");
      Serial.println(tempMotorTorques[3], 6);
      Serial.flush();
    }

    readSerialInput(tempJointAngles);
    // Check current motor positions
    waitForNewFeedback(odrv0_user_data, 1);
    waitForNewFeedback(odrv1_user_data, 1);
    waitForNewFeedback(odrv2_user_data, 1);
    waitForNewFeedback(odrv3_user_data, 1);
    actualMotorPositions[0] = currentPosition(odrv0_user_data, serialFeedback[2]);
    actualMotorPositions[1] = currentPosition(odrv1_user_data, serialFeedback[2]);
    actualMotorPositions[2] = currentPosition(odrv2_user_data, serialFeedback[2]);
    actualMotorPositions[3] = currentPosition(odrv3_user_data, serialFeedback[2]);

    actualMotorVelocities[0] = currentVelocity(odrv0_user_data,0);
    actualMotorVelocities[1] = currentVelocity(odrv1_user_data,0);
    actualMotorVelocities[2] = currentVelocity(odrv2_user_data,0);
    actualMotorVelocities[3] = currentVelocity(odrv3_user_data,0);

    // Calculate gravity feedforward
    gravityFeedforward(actualMotorPositions, tempMotorOffsets, forwardJointTorques);
  
    // Joint torques required to oppose gravity
    gravityJointTorques[0] =  - forwardJointTorques[0];
    gravityJointTorques[1] =  - forwardJointTorques[1];
    gravityJointTorques[2] =  - forwardJointTorques[2];
    
    motorTorqueCalculation(gravityJointTorques, gravityMotorTorques, minTorque, 0);

    // Feedforward, proportional, and derivative control
    tempMotorTorques[0] = gravityMotorTorques[0] 
                        - (tempMotorPositions[0] - actualMotorPositions[0]) * 0.02
                            + actualMotorVelocities[0] * 0.000225; //0.00337;

    tempMotorTorques[1] = gravityMotorTorques[1]
                        - (tempMotorPositions[1] - actualMotorPositions[1]) * 0.02
                            + actualMotorVelocities[1] * 0.000225; // 0.00337;

    tempMotorTorques[2] = gravityMotorTorques[2]
                        + (tempMotorPositions[2] - actualMotorPositions[2]) * 0.023
                           - actualMotorVelocities[2] * 0.000225; //0.00357;

    tempMotorTorques[3] = gravityMotorTorques[3]
                        + (tempMotorPositions[3] - actualMotorPositions[3]) * 0.023
                           - actualMotorVelocities[3] * 0.000225; // 0.00347

    // Restrics torques to a safe range. (min torque to 0.025)
    torqueBalancingCalculation(tempMotorTorques, minTorque);

    // Applies torques to the motors
    odrv0.setTorque(-tempMotorTorques[0]);// Negative Applies Torque
    odrv1.setTorque(-tempMotorTorques[1]); // Negative Applies Torque
    odrv2.setTorque(tempMotorTorques[2]); // Positive Applies Torque
    odrv3.setTorque(tempMotorTorques[3]); // Positive Applies Torque

    // Prints the seconds elapsed every 1000 updates
    if (i == 1000){
      //Serial.println(millis()/1000);
      i = 0;
    }

    delay(1);
  }
}
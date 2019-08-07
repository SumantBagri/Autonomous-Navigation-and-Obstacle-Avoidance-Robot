#include <motors.h>
#include <sensors.h>
#include <pid.h>

#include <Servo.h>
Servo gripperOpen;
int openAngle = 100;
int closeAngle = 45;
Servo gripperPitch;
int lowerAngle = 60;
int liftAngle = 45;

Sensors mySensors;
Motors myMotors;
PID B1_pid[4];
PID B2_pid;
PID B3_pid;
void localize();

unsigned long timer=3000;
int iter = 1;
unsigned long start_PID;
int counter=0;
/*
sensor_readings mapped to unique locations on the maze
{(1,0,1,0), (0,1,0,1)} --> 1
{(1,1,1,0), (0,1,1,1), (1,0,1,1), (1,1,0,1)} --> 2
{(1,1,0,0),(0,1,1,0), (0,0,1,1), (1,0,0,1)} --> 3
{(1,1,1,1)} --> 4
{(0,0,0,1), (1,0,0,0), (0,1,0,0), (0,0,1,0)} --> 5
*/
int obstacle[10][6]={
    {1,1,1,1,1,1},
    {1,0,0,0,0,1},
    {1,0,0,1,0,1},
    {1,0,1,0,0,1},
    {1,0,0,1,0,1},
    {1,1,0,1,0,1},
    {1,0,0,0,0,1},
    {1,1,0,1,1,1},
    {1,0,0,0,0,1},
    {1,1,1,1,1,1}
};

int seq[8];
int next_pos[2];
int curr_pos[2];
int prev_pos[2];
int curr_bot_head;
int next_bot_head;
int targetl[2]={1,1}; //Random values
int targetu[2]={6,1};
void find_next_position(int* next_pos, int current[2], int target[2]);
// function prototypes for localization
int return_seq_number(int* feasible_curr);
void find_adj(int **adj_arr, int x, int y);
int find_gen_seq(int ind, int x, int y, int** gen_seq, int** gen_seq2, int*** bigList, int* lengths_arr, int* seq);
void find_curr_pos(int* curr_pos, int* seq[]);
void go_to_target(int* target);
void block_pick_up();
void block_drop_off();
double blockDist;

void setup() {
  pinMode(22,OUTPUT);
  digitalWrite(22,LOW); // Indicator for localization, BLUE
  pinMode(23,OUTPUT);
  digitalWrite(23,LOW); // Indicator for reaching pickup location, RED
  pinMode(24,OUTPUT);
  digitalWrite(24,LOW); // Indicator for drop off, GREEEN
  pinMode(25, OUTPUT); 
  digitalWrite(25,LOW); // Indicator for block find, YELLOW
  
  gripperOpen.attach(8);
  gripperOpen.write(openAngle);
  gripperPitch.attach(9);
  gripperPitch.write(lowerAngle);

  Serial.begin(115200);
  myMotors.pinMode_init();
  mySensors.pinMode_init();

  // From B1 PID tuning
  for (int i = 0; i < 4; i++) {
    B1_pid[i].Kp = 11;    //15
    B1_pid[i].Ki = 0;     //0
    B1_pid[i].Kd = 55;    //60
  }

  // From B2 PID tuning
  B2_pid.Kp = 20;   //20 
  B2_pid.Ki = 0.1;  //0.1
  B2_pid.Kd = 10;   //10

  // From PID tuning
  B3_pid.Kp = 4;
  B3_pid.Ki = 0.04;
  B3_pid.Kd = 18;
  
  // wait for 3 secs before starting anything
  delay(3000);
  // Set the initial heading correctly
  for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
  mySensors.find_feasible_directions();
  mySensors.print_sensor_data();
  if (mySensors.feasible_curr[myMotors.heading] == 0) {
      //myMotors.heading_update(mySensors.feasible_curr);
      myMotors.orientation_update(&mySensors);
  }
  // Set the initial "seq" value
  seq[0] = return_seq_number(mySensors.feasible_curr);
}

void loop() {
    Serial.println("Starting localization");
    localize();
    Serial.println("Going for pickup");
    go_to_target(targetl);
    digitalWrite(23,HIGH); //Indicating it reached pickup location
    delay(2000);
    Serial.println("Picking up block");
    block_pick_up();
    go_to_target(targetu);
    digitalWrite(24,HIGH);
    block_drop_off();
    delay(100000);
}

void localize(){
  while(iter!=8){
    Serial.println(iter);
    //MOVE
    myMotors.drive_motors_move();
    delay(500);
    //MOVE PID
    start_PID=millis();
    while(millis()<start_PID+1700){
      for(int i=0; i < 8; i++){
        mySensors.read_ultrasonic(i);
      }
      mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
      myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
      myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
      myMotors.drive_motors();
    }
    myMotors.apply_brakes();
    delay(1000);
    //Check for rotate
    mySensors.find_feasible_directions();
    mySensors.print_sensor_data();
    if (mySensors.feasible_curr[myMotors.heading] == 0 && iter != 7) {
        myMotors.orientation_update(&mySensors);
        delay(500);
        //ROTATE PID
        start_PID=millis();
        while(millis()<start_PID+1700){
          for(int i=0; i < 8; i++){
            mySensors.read_ultrasonic(i);
          }
          //mySensors.print_sensor_data();
          mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
          myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
          myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
          myMotors.drive_motors();
        }
    }
    myMotors.apply_brakes();
    delay(1000);
    // Update "seq" array
    mySensors.find_feasible_directions();
    seq[iter] = return_seq_number(mySensors.feasible_curr);
    iter++;
  }
  Serial.print("{");
  for (int i=0;i<8;i++){
    Serial.print(seq[i]);
  }
  Serial.println("}");
  find_curr_pos(curr_pos, prev_pos, seq); // Done localizing :)
  Serial.println("\nDone localizing");
  Serial.print(curr_pos[0]);
  Serial.print(" ");
  Serial.println(curr_pos[1]);
  digitalWrite(22,HIGH);
}

void block_pick_up() {
  int Flag_Block_Pick = 0;
  unsigned long move_time;
  unsigned long curr_time;
  gripperPitch.write(lowerAngle);//lower gripper + open gripper
  delay (500);
  gripperOpen.write(openAngle);
  delay (1000);
  digitalWrite(25, LOW);
  /*for(int i=0; i < 8; i++){
    mySensors.read_ultrasonic(i);
  }
  mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
  myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
  myMotors.get_torques_slow(&B2_pid,mySensors.corrective_B2);
  myMotors.drive_motors(); //Until the block is in range
  myMotors.print_torque_vals();*/
  myMotors.drive_motors_slow(1);
  //MOVE FORWARD---------------------------------
  move_time = millis();
  while(Flag_Block_Pick == 0){
    start_PID=millis();
    while(millis()<start_PID+1700){
      for(int i=0; i < 8; i++){
        mySensors.read_ultrasonic(i);
      }
      mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
      myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
      myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
      myMotors.drive_motors();
    }
    myMotors.apply_brakes();
    delay(200);
    myMotors.drive_motors_slow(1);
    delay(500);
    myMotors.apply_brakes();

    //read sensors
    pinMode(11, INPUT);
    pinMode(10, OUTPUT);
    delayMicroseconds(2);
    digitalWrite(10, HIGH);
    delayMicroseconds(10);
    digitalWrite(10,LOW);
    blockDist = double(pulseIn(11, HIGH, 60000)/58.31);
    //if(blockDist == 0.0) {blockDist = 3000.0;} // With timeout pulseIn might return 0.0 indicating no wall/infinite distance
//    Serial.print("Reading: ");
//    Serial.print(blockDist);
//    Serial.println();
    if (blockDist <= 3.0){
      Flag_Block_Pick = 1;
    }
  }
  //delay(350); //continue motion delay
  // turn on LED light for visual representation
  digitalWrite(25, HIGH);
  move_time = millis() - move_time;
  myMotors.apply_brakes();
  delay(500);
  gripperOpen.write(closeAngle); 
  delay(500);
  gripperPitch.write(liftAngle);//close gripper+ move gripper up
  delay(2000);
  
  //move back same amount of distance
  curr_time = millis();
  /*Serial.print("Millis: ");
  Serial.print(millis());
  Serial.print("current time: ");
  Serial.print(curr_time);
  Serial.println();*/
  while ((millis()-curr_time) < 6000) {
    /*for(int i=0; i < 8; i++){
      mySensors.read_ultrasonic(i);
    }
    mySensors.update_corrective_factors(3, B1_pid, &B2_pid, &B3_pid);
    myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
    myMotors.get_torques_slow(&B2_pid,mySensors.corrective_B2);
    myMotors.drive_motors(); //Until the block is in range
    myMotors.print_torque_vals();*/
    myMotors.drive_motors_slow(-1);
    delay(500);
    myMotors.apply_brakes();
    delay(200);
    start_PID=millis();
    while(millis()<start_PID+1700){
      for(int i=0; i < 8; i++){
        mySensors.read_ultrasonic(i);
      }
      mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
      myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
      myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
      myMotors.drive_motors();
    }
    myMotors.apply_brakes();
    delay(200);
  }
  myMotors.apply_brakes();
  delay(2000);
}

void block_drop_off() {
  //move certain distance straight
  int move_delay = 1000;
  myMotors.drive_motors_slow(1);
  delay(move_delay);
  myMotors.apply_brakes();
  delay(1000);
  gripperPitch.write(lowerAngle); 
  delay(500);
  gripperOpen.write(openAngle);//lower gripper + open gripper
  delay(1000);
  digitalWrite(25, LOW);
  //move back same amount of distance
  myMotors.drive_motors_slow(-1);
  delay(move_delay);
  myMotors.apply_brakes();
  delay(1000);
  
  // Optional celebratory lightshow
  for (int i=0; i < 10; i++) {
    for (int j = 0; j < 4; j++) {
      digitalWrite((22+j), HIGH);
    }
    delay(200);
    for (int j = 0; j < 4; j++) {
      digitalWrite((22+j), LOW);
    }
    delay(100);
  }
}

void go_to_target(int* target){
  Serial.print("Heading:");
  Serial.println(myMotors.heading);
  Serial.println(" ");
  Serial.print("{");
  Serial.print(curr_pos[0]);
  Serial.print(" ");
  Serial.print(curr_pos[1]);
  Serial.println("}");
  //initial rotation to find target heading
  find_next_position(next_pos,curr_pos,target);
  curr_bot_head=find_alignment(curr_pos,prev_pos);
  next_bot_head=find_alignment(next_pos,curr_pos);
  Serial.print("Current_bot_head:");
  Serial.println(curr_bot_head);
  Serial.print("Next_bot_head:");
  Serial.println(next_bot_head);
  if (next_bot_head!=curr_bot_head){
    myMotors.rotate_bot((next_bot_head-curr_bot_head)*90);
    delay(1000);
  }
  //initial rotation PID
  start_PID=millis();
  while(millis()<start_PID+1700){
    for(int i=0; i < 8; i++){
      mySensors.read_ultrasonic(i);
    }
    mySensors.find_feasible_directions();
    mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
    myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
    myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
    myMotors.drive_motors();
  }
  myMotors.apply_brakes();
  delay(1000);

  while(abs(curr_pos[0]-target[0])+abs(curr_pos[1]-target[1])!=1){
    for(int i=0; i < 8; i++){
      mySensors.read_ultrasonic(i);
    }
    mySensors.find_feasible_directions();
    mySensors.print_sensor_data();
    if (mySensors.feasible_curr[myMotors.heading] == 0) {
      Serial.println("Current direction not feasible");
    /*   myMotors.orientation_update(&mySensors);
      start_PID=millis();
      while(millis()<start_PID+1500){
       for(int i=0; i < 8; i++){
          mySensors.read_ultrasonic(i);
        }
        //mySensors.print_sensor_data();
        mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
        myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
        myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
        myMotors.drive_motors();
      }
    //  localize();
      //go_to_target(target);
      myMotors.apply_brakes();*/
      digitalWrite(22,LOW);
      delay(10000);
      return;
    }
    
    //MOVE
    myMotors.drive_motors_move();
    myMotors.print_torque_vals();
    myMotors.apply_brakes();
    delay(1000);
    //MOVE PID
    start_PID=millis();
    while(millis()<start_PID+1700){
      for(int i=0; i < 8; i++){
        mySensors.read_ultrasonic(i);
      }
      mySensors.find_feasible_directions();
      mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
      myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
      myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
      myMotors.drive_motors();
    }
    myMotors.apply_brakes();
    delay(1000);
    //FIND NEXT ALIGNMENT
    prev_pos[0]=curr_pos[0]; prev_pos[1]=curr_pos[1];
    curr_pos[0]=next_pos[0]; curr_pos[1]=next_pos[1];
    find_next_position(next_pos,curr_pos,target);
    curr_bot_head=find_alignment(curr_pos,prev_pos);
    next_bot_head=find_alignment(next_pos,curr_pos);
    Serial.println(" ");
    Serial.print("{");
    Serial.print(curr_pos[0]);
    Serial.print(" ");
    Serial.print(curr_pos[1]);
    Serial.println("}");
    Serial.print("Current_bot_head:");
    Serial.println(curr_bot_head);
    Serial.print("Next_bot_head:");
    Serial.println(next_bot_head);
    //ROTATE
    if (next_bot_head!=curr_bot_head){
      myMotors.rotate_bot((next_bot_head-curr_bot_head)*90);
      myMotors.apply_brakes();
      delay(500);
      Serial.println("Rotating!");
      //ROTATE PID
      start_PID=millis();
      while(millis()<start_PID+1700){
        for(int i=0; i < 8; i++){
          mySensors.read_ultrasonic(i);
        }
        mySensors.find_feasible_directions();
        mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
        myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
        myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
        myMotors.drive_motors();
      }
      myMotors.apply_brakes();
      delay(1000);
    }
  }
  myMotors.apply_brakes();
  delay(1000);

  /*find_next_position(next_pos,curr_pos,target);
  curr_bot_head=find_alignment(curr_pos,prev_pos);
  next_bot_head=find_alignment(next_pos,curr_pos);
  if (next_bot_head!=curr_bot_head){
    myMotors.rotate_bot((next_bot_head-curr_bot_head)*90);
    delay(2000);
    //ROTATE PID
    start_PID=millis();
    while(millis()<start_PID+1500){
      for(int i=0; i < 8; i++){
        mySensors.read_ultrasonic(i);
      }
      mySensors.find_feasible_directions();
      mySensors.update_corrective_factors(myMotors.heading, B1_pid, &B2_pid, &B3_pid);
      myMotors.dir_update_PID(B1_pid,&B3_pid,mySensors.corrective_B1,mySensors.corrective_B3);
      myMotors.get_torques(&B2_pid,mySensors.corrective_B2);
      myMotors.drive_motors();
    }
    delay(2000);
  }*/
}

int find_alignment(int* curr_pos, int* prev_pos){
  int bot_head=0;
  if (curr_pos[0]>prev_pos[0]){
    bot_head=1;
  }
  else if(curr_pos[0]<prev_pos[0]){
    bot_head=3;
  }
  else if(curr_pos[1]<prev_pos[1]) {
    bot_head=2;
  }
  else {
    bot_head=4;
  }
  return bot_head;
}
// This function returns the sequence number correspoding to the colour for unique locations
// Check slide-40 of localization lecture for colour map
int return_seq_number(int* feasible_curr) {
  String dir_form;
  for (int i = 0; i < 4; i++) {
    dir_form += String(feasible_curr[i]);
  }
  if ((dir_form == "1010") || (dir_form == "0101")) return 1;
  else if ((dir_form == "1110") || (dir_form == "0111") || (dir_form == "1011") || (dir_form == "1101")) return 2;
  else if ((dir_form == "1100") || (dir_form == "0110") || (dir_form == "0011") || (dir_form == "1001")) return 3;
  else if ((dir_form == "1111")) return 4;
  else if ((dir_form == "1000") || (dir_form == "0100") || (dir_form == "0010") || (dir_form == "0001")) return 5;
  else return 99;
}

void find_adj(int **adj_arr, int x, int y) {
  adj_arr[0][0] = x-1; adj_arr[0][1] = y; // Left adjacent
  adj_arr[1][0] = x+1; adj_arr[1][1] = y; // Right adjacent
  adj_arr[2][0] = x;   adj_arr[2][1] = y-1; // Up adjacent
  adj_arr[3][0] = x;   adj_arr[3][1] = y+1; // Down adjacent
}

int find_gen_seq(int ind, int x, int y, int** gen_seq, int** gen_seq2, int*** bigList, int lengths_arr[], int seq[]) {
  //cout << "Finding.." << endl;
  int bool_check1;
  gen_seq[ind][0] = x;
  gen_seq[ind][1] = y;
  // Base case - Occurs when the last element of the "seq" is "reached"
  if (ind == 7) {
    //cout << "FOUND!" << endl;
    for (int i = 0; i < 8; i++) {
      gen_seq2[i][0] = gen_seq[i][0];
      gen_seq2[i][1] = gen_seq[i][1];
    }
    return 1;
  }
  // Again some initializations
  int **adj_arr = new int*[4];
  for (int i = 0; i < 4; i++) {
    adj_arr[i] = new int[2];
  }
  find_adj(adj_arr,x,y); // finds the adjacent cells as mentioned before

  // The main logic for recursion starts here
  ind++;
  // We go through each and every pair of (x,y) coordinates of the next element(ind++) which belong to the "unique location" in the seq array
  // and then try to compare if any of them correspond to the "adjacent cell/s" of our previous element(ind before doing (ind++))
  for (int k = 0; k < 4; k++) {
    // Compare each of the 4 adjacent cells with the above (x1,y1) pair
    for (int j = 0; j < lengths_arr[seq[ind]-1]; j++) {
      // Compute the (x,y) coordinates for the next element in the sequence
      // Did ind++ above indicating that we are looking at the next element
      int x1  = bigList[seq[ind]-1][j][0];
      int y1 = bigList[seq[ind]-1][j][1];
      //cout << "("<< x1 << "," << y1 << ")" << " ";
      // If we find a pair that matches (x1,y1) we call the function recursively to move forward from there
      if ((adj_arr[k][0] == x1) && (adj_arr[k][1] == y1)) {
        bool_check1 = find_gen_seq(ind,x1,y1,gen_seq,gen_seq2,bigList,lengths_arr,seq);
        if (bool_check1) {
          return bool_check1;
        }
      }
    }
    // If all the coordinate pairs for that unique location are looked at
    // and none of them match with any of the adjacent pair then a pattern cannot be found!!
    // THIS SHOULD NEVER HAPPEN!! EVER.....
    if (k==3) {
      //cout << "NOT FOUND!" << endl;
      return 0;
    }
  }
  delete adj_arr;
  return bool_check1;
}

// SHOULD CALL THIS AFTER BOT HAS MOVED 8 BLOCKS
void find_curr_pos(int* curr_pos, int* prev_pos, int seq[]) {
  // Test sequence values generated from the maze manually
  //int seq[8] = {3,1,2,1,1,3,1,4};
  //int seq[8] = {2,1,4,1,3,1,1,2};
  //int seq[8] = {5,4,1,3,3,1,2,3};
  //int seq[8] = {5,2,1,5,1,2,5,2};

  int*** bigList = new int**[5]; // Stores the coordinates for each unique locations
  int** gen_seq = new int*[8]; // Temporary storage of the sequence of unique locations
  int** gen_seq2 = new int*[8]; // Stores the actual sequence of unique locations
  int lengths_arr[5] = {9,4,6,1,4}; // Just identifies the number of blocks each unique location occupies on the maze

  // Some initializations. Dont bother!!
  for (int i = 0; i < 8; i++) {
    gen_seq[i] = new int[2];
    gen_seq2[i] = new int[2];
  }
  for (int i = 0; i < 5; i++) {
    bigList[i] = new int*[lengths_arr[i]];
    for (int j = 0; j < lengths_arr[i]; j++) {
      bigList[i][j] = new int[2];
    }
  }

  // World Initialization
  bigList[0][0][0] = 8; bigList[0][0][1] = 3;
  bigList[0][1][0] = 7; bigList[0][1][1] = 2;
  bigList[0][2][0] = 6; bigList[0][2][1] = 3;
  bigList[0][3][0] = 5; bigList[0][3][1] = 2;
  bigList[0][4][0] = 5; bigList[0][4][1] = 4;
  bigList[0][5][0] = 4; bigList[0][5][1] = 4;
  bigList[0][6][0] = 3; bigList[0][6][1] = 1;
  bigList[0][7][0] = 2; bigList[0][7][1] = 4;
  bigList[0][8][0] = 1; bigList[0][8][1] = 3;
  bigList[1][0][0] = 8; bigList[1][0][1] = 2;
  bigList[1][1][0] = 3; bigList[1][1][1] = 4;
  bigList[1][2][0] = 2; bigList[1][2][1] = 1;
  bigList[1][3][0] = 1; bigList[1][3][1] = 2;
  bigList[2][0][0] = 6; bigList[2][0][1] = 4;
  bigList[2][1][0] = 4; bigList[2][1][1] = 1;
  bigList[2][2][0] = 4; bigList[2][2][1] = 2;
  bigList[2][3][0] = 2; bigList[2][3][1] = 2;
  bigList[2][4][0] = 1; bigList[2][4][1] = 1;
  bigList[2][5][0] = 1; bigList[2][5][1] = 4;
  bigList[3][0][0] = 6; bigList[3][0][1] = 2;
  bigList[4][0][0] = 8; bigList[4][0][1] = 4;
  bigList[4][1][0] = 8; bigList[4][1][1] = 1;
  bigList[4][2][0] = 6; bigList[4][2][1] = 1;
  bigList[4][3][0] = 3; bigList[4][3][1] = 3;

  // THis is where everything starts
  // SO we start with the first element in the "seq" array
  // and keep going forward from there until the we are able to find a pattern of coordinates that matches the given sequence
  for (int i = 0; i < lengths_arr[seq[0]-1]; i++) {
    int ind=0;
    int x1  = bigList[seq[ind]-1][i][0];
    int y1 = bigList[seq[ind]-1][i][1];
    int bool_check=find_gen_seq(ind,x1,y1,gen_seq,gen_seq2,bigList,lengths_arr,seq);
    if (bool_check){
      break;
    }
  }

  curr_pos[0] = gen_seq2[7][0]; // x coordinate of current position
  curr_pos[1] = gen_seq2[7][1]; // y coordinate of current position

  prev_pos[0] = gen_seq2[6][0]; // x coordinate of previous position
  prev_pos[1] = gen_seq2[6][1]; // y coordinate of previous position

  // Deletes the dynamically allocated arrays. Very Important!!
  delete gen_seq;
  delete gen_seq2;
  delete bigList;
}

void find_next_position(int* next_pos, int current[2], int target[2]){
  //Loading zone
  if( target[0]==1 && target[1]==1){
    if(obstacle[current[0]-1][current[1]]==0){
      next_pos[1]=current[1];
      next_pos[0]=current[0]-1;
      return;
    }
    else if(obstacle[current[0]][current[1]-1]==0){
      next_pos[1]=current[1]-1;
      next_pos[0]=current[0];
      return;
    }
    else if(obstacle[current[0]][current[1]+1]==0){
      next_pos[1]=current[1]+1;
      next_pos[0]=current[0];
      return;
    }
  }
  //unloading zone (3,3)
  else if (target[0]==3){
    if(obstacle[current[0]-1][current[1]]==0 && counter==0){
      next_pos[1]=current[1];
      next_pos[0]=current[0]-1;
      return;
    }
    else if(obstacle[current[0]][current[1]+1]==0 && counter==0){
      next_pos[1]=current[1]+1;
      next_pos[0]=current[0];
      return;
    }
    else {
      if (counter==0){
        next_pos[0]=2;
        next_pos[1]=4;
        counter++;
        return;
      }
      else if(counter==1){
        next_pos[0]=3;
        next_pos[1]=4;
        counter++;
        return;
      }
      else if(counter==2){
        next_pos[0]=3;
        next_pos[1]=3;
        return;
      }
    }
  }
  //unloading zone (6,1),(8,11) and (8,4)
  else{
    if(obstacle[current[0]][current[1]-1]==0 && counter==0){
      next_pos[1]=current[1]-1;
      next_pos[0]=current[0];
      return;
    }
    else if(obstacle[current[0]+1][current[1]]==0 && counter==0){
      next_pos[1]=current[1];
      next_pos[0]=current[0]+1;
      return;
    }
    else{
      if(counter==0){
        next_pos[1]=2;
        next_pos[0]=4;
        counter++;
        return;
      }
      else if(counter==1){
        next_pos[1]=2;
        next_pos[0]=5;
        counter++;
        return;
      }
      else if(counter==2){
        next_pos[1]=2;
        next_pos[0]=6;
        counter++;
        return;
      }
      else if(counter==3 && target[0]==6){
        next_pos[1]=1;
        next_pos[0]=6;
        return;
      }
      else if(counter==3){
        next_pos[1]=2;
        next_pos[0]=7;
        counter++;
        return;
      }
      else if(counter==4){
        next_pos[1]=2;
        next_pos[0]=8;
        counter++;
        return;
      }
      else if(counter==5 && target[1]==1){
        next_pos[1]=1;
        next_pos[0]=8;
        counter++;
        return;
      }
      else{
        next_pos[1]=current[1]+1;
        next_pos[0]=current[0];
        return;
      }
    }
  }
}

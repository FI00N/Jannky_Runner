#include <Arduino.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"
#include "MovingAverageFilter.hpp"
#include "Lidar.hpp"
#include "Graph.hpp"
#include "ascii2graph.hpp"
#include "RobotMotionPlanner.hpp"
#include <Wire.h>
#include <VL6180X.h>
#include <string.h>

// Set up Lidars <name>(address, enablePin)
VL6180X flidar;
VL6180X lLidar;
VL6180X rLidar;
mtrn3100::Lidar r_lidar(0x20, 11, lLidar);
mtrn3100::Lidar l_lidar(0x22, 10, rLidar);
mtrn3100::Lidar f_lidar(0x25, 12, flidar);

// Motor pins.
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

// Set up PIDs.
mtrn3100::PIDController l_pos_pid(41, 0.95, 10, 0);
mtrn3100::PIDController r_pos_pid(46, 1.1, 10, 0);

// Encoder pins.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder r_encoder(18, 22, readRightEncoder);
mtrn3100::Encoder l_encoder(19, 23, readLeftEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

float ltarget = 0;
float rtarget = 0;

//Motion plan
char motionPlan[256] = {0};  // Assuming the motion plan will not exceed 256 characters
int motionPlanIndex = 0;
//Initialize array for storing maze
char maze[418] = {0};
int motionPlanLength = 418; //Initializing a check for when the motion plan finishes.
String incomingByte;

void setup() {
    delay(50);
    Serial.begin(115200);
    Serial3.begin(9600);

    //Make sure Wire is off.    
    Wire.end();
    delay(100);
    Wire.begin();
    
    r_lidar.lidarSetup();
    l_lidar.lidarSetup();
    f_lidar.lidarSetup();

    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, ltarget);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, rtarget);
    delay(50);
    
    // Map Solving
    int rowCount = 1;
    String mazeInput;   
    Serial3.println("ENTER MAZE");
    while (rowCount < 12) {
        if (Serial3.available() > 0) {
            incomingByte = Serial3.readString();
            Serial3.print("Row ");
            Serial3.print(rowCount);
            Serial3.println(" read in!");
            mazeInput += incomingByte;
            rowCount++;
        }
    }
    strcpy(maze, mazeInput.c_str());
    Serial3.println(maze);
   
    auto const actual = mtrn3100::ascii2graph(maze);
    mtrn3100::RobotMotionPlanner planner;

    for (auto e : actual.edges()) {
        int src = mtrn3100::get<0>(e.value);
        int dst = mtrn3100::get<1>(e.value);

        planner.addEdge(src, dst);

    }

    // Buffer for data received from Bluetooth
    char start[50] = "";
    int end;
    String startdata;
    String enddata;

    // Request start data
    Serial3.println("Please send the start cell and heading:");
    while (!Serial3.available()) {
        delay(10);
    }
    startdata = Serial3.readString();
    startdata.trim();
    startdata.toCharArray(start, sizeof(start));

    // Request end data
    Serial3.println("Please send the end cell:");
    while (!Serial3.available()) {
        delay(10);
    }
    enddata = Serial3.readString();
    enddata.trim();
    end = enddata.toInt();

    // robot head
    mtrn3100::RobotMotionPlanner::Direction head = planner.getStartDirection(start);

    // BFS Path
    int resultPath[mtrn3100::MAX_SIZE];
    int pathSize = planner.bfs(start, end, resultPath);

    planner.generateMotionPlan(resultPath, pathSize, head, motionPlan);

    Serial3.print("The motionPlan is: ");
    Serial3.println(motionPlan);    
}

void loop() {
    switch (motionPlan[motionPlanIndex]) {
      case 'L':
          ltarget = -4.15;
          rtarget = 4.15;
          l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, ltarget);
          r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, rtarget);
          l_pos_pid.tune(40, 20, 4);
          r_pos_pid.tune(40, 20, 4);
          break;
      case 'R':
          ltarget = 4.1;
          rtarget = -4.1;
          l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, ltarget);
          r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, rtarget);
          l_pos_pid.tune(40, 20, 4);
          r_pos_pid.tune(40, 20, 4);
          break;
      case 'F':
          ltarget = (250/(45/2))*0.91;
          rtarget = (250/(45/2))*0.91;
          r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, rtarget);
          l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, ltarget);
          // r_pos_pid.tune(21, 5, 4);
          // l_pos_pid.tune(21, 5, 4);
          r_pos_pid.tune(40, 20, 4);
          l_pos_pid.tune(40, 20, 4);
          break;
      default:
          motionPlanIndex = motionPlanLength;
          break;
    }

    //Stop car if drive plan is finished
    if (motionPlanIndex >= motionPlanLength) {
        while (1) {
        }
    }

    //Update and initialise timers
    //Variables to store Lidar Data so the Lidars only need to be read once
    float leftLidarRead = 0;
    float rightLidarRead = 0;
    float frontLidarRead = 1000;
    float leftMotorOffset = 1;
    float rightMotorOffset = 1;
    float motorSpeedLimiter = 1;
    //MAFs for PID error
    mtrn3100::MovingAverageFilter<float,50> mafLeft;
    mtrn3100::MovingAverageFilter<float,50> mafRight;
    //Variables to store the PID output
    float leftMotorsignal = 0;
    float rightMotorsignal = 0;

    //PID Timers
    uint32_t prev_time = micros();
    uint32_t curr_time = micros();
    float dt = static_cast<float>(curr_time - prev_time) / 1e6;
    //Lidar Timers
    uint32_t currWallTimer = millis();
    uint32_t prevWallTimer = millis();
    uint32_t dtWallTimer = currWallTimer - prevWallTimer;

    while (true) {
        //Update some timers
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;

        //If the car is going forward, a speed limiter becomes active to slow the whole robot down. This hopefully
        //helps the robot adjust more using the lidars.
        if (motionPlan[motionPlanIndex] == 'F') {
            motorSpeedLimiter = 0.2;
            currWallTimer = millis();
            dtWallTimer = currWallTimer - prevWallTimer;

            //dtWallTimer sets off the lidars every 200ms to check if the car is too close to the wall. If it is, the offset is active and will slow down a wheel,
            //so the car doesn't crash into the wall.
            if (dtWallTimer >= 200) {
                rightMotorOffset = 1;
                leftMotorOffset = 1;
                rightLidarRead = r_lidar.readDistance();
                leftLidarRead = l_lidar.readDistance();
                frontLidarRead = f_lidar.readDistance();
                
                //Activate offset to slow down a wheel if lidar detects wall is too close or far away.
                if ((rightLidarRead >= 80 && rightLidarRead <= 120) || leftLidarRead <= 60) {
                  rightMotorOffset = 0.9;
                } else if ((leftLidarRead >= 80 && leftLidarRead <= 120) || rightLidarRead <= 60) {
                  leftMotorOffset = 0.9;
                }
                
                prevWallTimer = currWallTimer;
            }
        }
        //Compute motor signal
        leftMotorsignal = l_pos_pid.compute(l_encoder.position);
        rightMotorsignal = r_pos_pid.compute(r_encoder.position);
        l_motor.setPWM(leftMotorsignal * motorSpeedLimiter * leftMotorOffset);
        r_motor.setPWM(rightMotorsignal * motorSpeedLimiter * rightMotorOffset);
        mafLeft.sample(l_pos_pid.getError());
        mafRight.sample(r_pos_pid.getError());

        // Check if error is within a certain range. If so, then stop and move on. Gives the robot 4 seconds each motion. 
        // Also stops if front is too close to wall.
        if (mafLeft.isFull() && mafRight.isFull()) {
            if ((abs(mafLeft.average()) <= 0.07 && abs(mafRight.average()) <= 0.07) || dt >= 4 || frontLidarRead <= 60) {
              l_motor.setPWM(0);
              r_motor.setPWM(0);
              break;
            }
        } 
    }

    motionPlanIndex++;

    // Reset the prev_error and prev_integral to zero.
    r_pos_pid.pidReset();
    l_pos_pid.pidReset();
    delay(500);
}

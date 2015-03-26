/** include used libraries* ***/
#include <ros/ros.h> // essential ros environment
#include <moveit/move_group_interface/move_group.h> // allow moveit commands 
#include <iostream>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <ctime>
#include <cstdlib>

/*** include directories for IK experiment ***/
#include <moveit/planning_scene_interface/planning_scene_interface.h> // allow planning commands
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>


#include <controller_manager/controller_manager.h>
//#include <joint_limits.h>
/*** uncomment to include specified code ***/
#define joint_angles_specification
#define test_mode
//#define cartesian_specification 

#define pi 3.14159265359

/*** declare global variables ***/

long int count = 0;
std::map<std::string, double> joints;

const char* p[] = { "p4_instrument_slide", 
                    "p4_hand_pitch", 
                    "p4_hand_roll", 
                    "p4_instrument_roll", 
                    "p4_instrument_pitch", 
                    "p4_instrument_jaw_right"};

/***
 *** FUNCTION: test_mode() 
 *** PURPOSE:  test each joint of da Vinci seperately to evaluate their extremums
 *** INPUTS:   none
 *** OUTPUTS:  nove   
***/

int test_mode_function() {
  moveit::planning_interface::MoveGroup group("gripper"); // setup the MoveGroup class for group gripper
  #ifdef test_mode
    int k;
    /*** iterate through all joints ***/
    for (k=0; k<3; k++) {
      int i, N; // declare temporary integers
      double limit;
      ROS_INFO("Enter number of test iterations for %s: ", p[k]); // ask for test length
      std::cin >> N; // stor length
      if (N == 0){
        continue;
      }
      ROS_INFO("Enter limit:");
      std::cin >> limit;
      //ROS_INFO("limit = %d", limit);
      /*** run test according to user input ***/
      ROS_INFO("Move to initial state");
      group.setNamedTarget("ready");
      group.move();
      for (i=0; i<N; i++) {
        ROS_INFO("iter = %i of %i", i+1, N); // show iteration number
        ROS_INFO("testing %s", p[k]); // show which part is currently under test
        joints[p[k]] = -limit; // go to first extremum
        group.setJointValueTarget(joints); // set joint values
        group.move(); // move da Vinci to specified position
        joints[p[k]] = limit; // go to second extremum
        group.setJointValueTarget(joints); // set joint values
        group.move(); // move da Vinci to specified position
        /*** detect maximum of specified iterations and provide an option to continue ***/
        if (i == N-1){
          std::string in; // declare variable
          std::cout << "is the test completed (y/n): " << std::endl; // user information
          std::cin >> in; // store user information
          /*** ask for further iterations  ***/
          if (in == "n") {
            ROS_INFO("write number of iterations:"); // user information
            std::cin >> N; // store new maximum
            ROS_INFO("write new limit:"); // user information
            std::cin >> limit; // store new limit
            group.setNamedTarget("ready");
            group.move();
            i = -1; // iteration is one behind
          }
        }
      }
    }
  #endif
  ROS_INFO("TEST MODE END!");
  return 0;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "MoveGroupInterface_new", ros::init_options::AnonymousName); // initiate ros file
  ROS_INFO("ROS node MoveGroupInterface has initiated"); // welcome message
  ros::AsyncSpinner spinner(1); // allow asynchronous subscriptions, services and callbacks from a single thread
  spinner.start(); // start spinner
  moveit::planning_interface::MoveGroup group("gripper"); // setup the MoveGroup class for group gripper

  ROS_INFO("\n\n *** STARTING *** \n");
  ROS_INFO("End effector link:   %s", group.getEndEffectorLink().c_str()); // provide the end-effector name to user
  ROS_INFO("Reference frame:     %s", group.getPlanningFrame().c_str());  // provide reference frame to user 
  ROS_INFO("Group name:          %s", group.getName().c_str());  // provide reference frame to user 
  ROS_INFO(" "); 

  int i;
  for (i=0; i<group.getJoints().size(); i++) {
    ROS_INFO("Joints %d:         %s", i, group.getJoints()[i].c_str());  // provide all joints to user
  }
  ROS_INFO(" "); 
  int j;
  for (j=0; j<group.getActiveJoints().size(); j++) {
    ROS_INFO("Active joints %d:  %s", j, group.getActiveJoints()[j].c_str());  // provide active joints to user
  }  
  ROS_INFO(" "); 

  ROS_INFO("Default goal joint tolerance: %f",              group.getGoalJointTolerance());
  group.setGoalTolerance(0.005); 
  ROS_INFO("Modified goal joint tolerance: %f",             group.getGoalJointTolerance());
  ROS_INFO("Default position tolerance: %f",                group.getGoalPositionTolerance());
  ROS_INFO("Default position tolerance: %f",                group.getGoalPositionTolerance());
  ROS_INFO("Default goal orientation tolerance (RPY): %f",  group.getGoalOrientationTolerance());
 
  ROS_INFO(" "); 
  ROS_INFO("DOF: %i", group.getVariableCount());

  #ifdef test_mode
    ROS_INFO("Move to initial state");
    group.setNamedTarget("ready");
    group.move();

    // ROS_INFO("Current joint values: %f", group.getCurrentJointValues()[0]);
    
  
  /*** get joint values ***/ 
  int N_controllers = sizeof(p)/sizeof(p[0]);
  int k;
  for (k=0; k<N_controllers; k++) {
    ROS_INFO("%d. %s = %f", k, p[k], group.getCurrentJointValues()[k]);  // provide active joints to user
  }  

  //ROS_INFO("End of strings: %i ", std::find(p,"p4_instrument_jaw_right"));

    //moveit::planning_interface::MoveGroup group("gripper"); // setup the MoveGroup class for group gripper
    std::string inputTest;
    std::cout << "\nPress 't' to run test mode or \nPress 'i' for IK test \nPress 'c' to run custom mode\nPress 'x' to enter custom IK mode\nPress 'd' to run demo mode" << std::endl;
    getline(std::cin, inputTest);
    std::cout << "input: " << inputTest << std::endl << std::endl;     
 

   if (inputTest == "t") {
      ROS_INFO("Entering test mode..");
      test_mode_function();
    }

    else if (inputTest == "c"){
      moveit::planning_interface::MoveGroup group("gripper"); // setup the MoveGroup class for group gripper
      char ans_c = 'c';
      while(ans_c != 'q') {
        double slide, p4_hand_pitch, p4_hand_roll, p4_instrument_roll, p4_instrument_pitch, p4_instrument_jaw_right;

        ROS_INFO("enter joint value for instrument_slide");
        std::cin >> slide;
        ROS_INFO("enter joint value for p4_hand_pitch");
        std::cin >> p4_hand_pitch;
        ROS_INFO("enter joint value for p4_hand_roll");
        std::cin >> p4_hand_roll;
        ROS_INFO("enter joint value for p4_instrument_roll");
        std::cin >> p4_instrument_roll;
        ROS_INFO("enter joint value for p4_instrument_pitch"); // 0.9
        std::cin >> p4_instrument_pitch;
        ROS_INFO("enter joint value for p4_instrument_jaw_right"); // -0.78 to 1.33
        std::cin >> p4_instrument_jaw_right;

        ROS_INFO("setting custom joint angles");
        joints[p[5]] =  p4_instrument_jaw_right;
        joints[p[4]] =  p4_instrument_pitch;
        joints[p[3]] =  p4_instrument_roll;
        joints[p[2]] =  p4_hand_roll;
        joints[p[1]] =  p4_hand_pitch;
        joints[p[0]] =  slide;

        group.setJointValueTarget(joints);
        group.move();

        ROS_INFO("press 'q' to quit custom mode or 'c' to set another joint state angle");
        std::cin >> ans_c;
      }
      ROS_INFO("Leaving custom mode..");
    }

    else if (inputTest == "x"){
      char ans_c = 'c';
      double off_z, off_y, off_x;
      off_z = 0.900000 - (0.890000-0.6700000);
      off_y = 0.000000;
      off_x = 2.175200;
      while(ans_c != 'q') {
        double x, y, z;

        ROS_INFO("enter x-coordinate");
        std::cin >> x;
        ROS_INFO("enter y-coordinate");
        std::cin >> y;
        ROS_INFO("enter z-coordinate");
        std::cin >> z;

        geometry_msgs::Pose custom_target;
        custom_target.position.x = x + off_x;
        custom_target.position.y = y + off_y;
        custom_target.position.z = z + off_z;

        group.setPoseTarget(custom_target);
        moveit::planning_interface::MoveGroup::Plan custom_plan;
        bool success_ = group.plan(custom_plan);
        ROS_INFO("success = %d", success_);
        group.move();

        /*** get joint values ***/ 
       int N_controllers = sizeof(p)/sizeof(p[0]);
       int k;
       for (k=0; k<N_controllers; k++) {
         ROS_INFO("%d. %s = %f", k, p[k], group.getCurrentJointValues()[k]);  // provide joint values to user
       }  
     }
   }

    else if (inputTest == "i"){
      double off_z, off_y, off_x;
      off_z = 0.900000 - (0.890000-0.6700000);
      off_y = 0.000000;
      off_x = 2.145200;
      
      #define ik_test
      #ifdef ik_test
      while(1) {
        ROS_INFO("ROS IK START!");

        geometry_msgs::Pose target_pose1;
        target_pose1.position.x = off_x;
        target_pose1.position.y = off_y;
        target_pose1.position.z = off_z;

        ROS_INFO("Position tolerance: %f", group.getGoalPositionTolerance());

        group.setPoseTarget(target_pose1);
        moveit::planning_interface::MoveGroup::Plan my_plan_1;
        bool success_1 = group.plan(my_plan_1);
        ROS_INFO("success = %d", success_1);
        group.move();
      
        geometry_msgs::Pose target_pose2;
        target_pose2.position.x = 0.100000 + off_x;
        target_pose2.position.y = off_y;
        target_pose2.position.z = off_z;

        group.setPoseTarget(target_pose2);
        moveit::planning_interface::MoveGroup::Plan my_plan_2;
        bool success_2 = group.plan(my_plan_2);
        ROS_INFO("success = %d", success_2);
        group.move();
        
        geometry_msgs::Pose target_pose3;
        target_pose3.position.x = 0.100000 + off_x;
        target_pose3.position.y = 0.100000 + off_y;
        target_pose3.position.z = 0.000000 + off_z;

        group.setPoseTarget(target_pose3);
        moveit::planning_interface::MoveGroup::Plan my_plan_3;
        bool success_3 = group.plan(my_plan_3);
        ROS_INFO("success = %d", success_3);
        group.move();
        
        geometry_msgs::Pose target_pose4;
        target_pose4.position.x = 0.150000 + off_x;
        target_pose4.position.y = 0.150000 + off_y;
        target_pose4.position.z = 0.000000 + off_z;

        group.setPoseTarget(target_pose3);
        moveit::planning_interface::MoveGroup::Plan my_plan_4;
        bool success_4 = group.plan(my_plan_3);
        ROS_INFO("success = %d", success_4);
        group.move();
        
        ROS_INFO("ROS IK END!");
      #endif
    }
}
   else {
     ROS_INFO("No test mode this time..");
   }
  #endif 

  ROS_INFO("Entering demo mode!"); // communicate status
  while(1){
    ++count; // count loop iteration
    ROS_INFO("loop %ld",count); // communicate loop iteration
    #ifdef joint_angles_specification

      ROS_INFO("Joint angle specification mode!"); // communicate mode

      /*** pass list joint angles to da Vinci  ***/

      moveit::planning_interface::MoveGroup group("gripper"); // setup the MoveGroup class for group gripper
      ROS_INFO("Move to initial state");
      group.setNamedTarget("ready");
      group.move();

      ROS_INFO("set of joint angles 1");
      joints[p[2]] =  0;
      joints[p[1]] =  pi/5;
      joints[p[0]] =  0;
      group.setJointValueTarget(joints);
      group.move();
      
      ROS_INFO("set of joint angles 1");
      joints[p[2]] =  1.2;
      joints[p[1]] =  pi/5;
      joints[p[0]] =  0.0;
      group.setJointValueTarget(joints);
      group.move();
      
      ROS_INFO("set of joint angles 2");
      joints[p[2]] =  0;
      joints[p[1]] =  0;
      joints[p[0]] =  0.09;
      group.setJointValueTarget(joints);
      group.move();
    
      ROS_INFO("set of joint angles 3");
      joints[p[2]] =  0;
      joints[p[1]] =  0;
      joints[p[0]] = -0.09;
      group.setJointValueTarget(joints);
      group.move();
   
      ROS_INFO("set of joint angles 4");
      joints[p[2]] =  -1.2;
      joints[p[1]] =  -pi/5;
      joints[p[0]] =  0;
      group.setJointValueTarget(joints);
      group.move();

      #endif

  }  // while(1)

  ros::waitForShutdown();

} // main

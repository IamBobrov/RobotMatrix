#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <iostream>
#include <cmath>
#include <control_msgs/JointTrajectoryControllerState.h>

const double L_1 = 0.5;
const double L_2 = 1.0;
const double L_3 = 1.5;
double final_point[4][1] = {{0},
                   {0},
                   {0},
                   {1}};

struct Matrix4x4 {
    double data[4][4];
};
struct Matrix4x1 {
    double data[4][1];
};



void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg);

Matrix4x4 multiply_matrix(double A[4][4], double B[4][4]);

Matrix4x1 multiply_matrix_and_vector(double A[4][4], double B[4][1]);

void final_vector(double fi_1, double fi_2, double fi_3);

void demostration(trajectory_msgs::JointTrajectoryPoint &points, ros::Publisher &arm_points);

void control_logic(ros::NodeHandle &nh);



int main(int argc, char** argv){
    ros::init(argc, argv, "robotics_project_1");
    ros::NodeHandle nh;
    control_logic(nh);
    ros::spin();
    return 0;
}









void callback(const control_msgs::JointTrajectoryControllerState::ConstPtr &msg){
    ROS_INFO("Actual joint positions (radians):");
    for (size_t i = 0; i < msg->joint_names.size(); ++i)
    {
        ROS_INFO("%s: %f", msg->joint_names[i].c_str(), msg->actual.positions[i]);
    }
    ros::Duration(1.0).sleep();   
}
Matrix4x4 multiply_matrix(double A[4][4], double B[4][4]) {
    Matrix4x4 result = {0}; 
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                result.data[i][j] += (A[i][k] * B[k][j]);
            }
        }
    }
    return result;
}
Matrix4x1 multiply_matrix_and_vector(double A[4][4], double B[4][1]) {
    Matrix4x1 result = {0}; 
    for (int i = 0; i < 4; ++i) {
        for (int k = 0; k < 1; ++k) {
            result.data[i][k] = 0;
            for (int j = 0; j < 4; ++j) {
                result.data[i][k] += (A[i][j] * B[j][k]);
            }
        }
    }
    return result;
}


void final_vector(double fi_1, double fi_2, double fi_3){
    double rotate_matrix_1_z[4][4] = {{sin(fi_1), -cos(fi_1), 0.0, 0.0},
                                 {cos(fi_1), sin(fi_1), 0.0, 0.0},
                                 {0.0, 0.0, 1.0, 0.0},
                                 {0.0, 0.0, 0.0, 1.0}};
    double translate_matrix_1[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                      {0.0, 1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0, L_1},
                                      {0.0, 0.0, 0.0, 1.0}};
    double rotate_matrix_2_y[4][4] = {{cos(fi_2), 0.0, sin(fi_2), 0.0},
                                      {0.0, 1.0, 0.0, 0.0},
                                      {-sin(fi_2), 0.0, cos(fi_2), 0.0},
                                      {0.0, 0.0, 0.0, 1.0}};
    double translate_matrix_2[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                      {0.0, 1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0, L_2},
                                      {0.0, 0.0, 0.0, 1.0}};
    double rotate_matrix_3_y[4][4] = {{cos(fi_3), 0.0, sin(fi_3), 0.0},
                                 {0.0, 1.0, 0.0, 0.0},
                                 {-sin(fi_3), 0.0, cos(fi_3), 0.0},
                                 {0.0, 0.0, 0.0, 1.0}};
    double translate_matrix_3[4][4] = {{1.0, 0.0, 0.0, 0.0},
                                      {0.0, 1.0, 0.0, 0.0},
                                      {0.0, 0.0, 1.0, L_3},
                                      {0.0, 0.0, 0.0, 1.0}};

    Matrix4x4 matrix_A = multiply_matrix(rotate_matrix_1_z, translate_matrix_1);
    Matrix4x4 matrix_B = multiply_matrix(rotate_matrix_2_y, translate_matrix_2);
    Matrix4x4 matrix_C = multiply_matrix(matrix_A.data, matrix_B.data); 
    Matrix4x4 matrix_D = multiply_matrix(rotate_matrix_3_y, translate_matrix_3);
    Matrix4x4 matrix_E = multiply_matrix(matrix_C.data, matrix_D.data); 

    Matrix4x1 result_vector = multiply_matrix_and_vector(matrix_E.data, final_point);
    
    std::cout << "Final vector:" << std::endl;
    
    std::cout <<"x: "<< result_vector.data[0][0] << std::endl;
    std::cout <<"y: "<< result_vector.data[1][0] << std::endl;
    std::cout <<"z: "<< result_vector.data[2][0] << std::endl;
    std::cout << result_vector.data[3][0] << std::endl;
    
}

void demostration(trajectory_msgs::JointTrajectoryPoint &points, ros::Publisher &arm_points, trajectory_msgs::JointTrajectory &arm_position){
    points.time_from_start = ros::Duration(0.3);
    points.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};

    for(double i = 0; i < (M_PI/2); i += (M_PI/20)){
        points.positions = {0.0, i, 0.0, 0.0, 0.0};
        final_vector(0, i, 0);
        arm_position.points.clear(); 
        arm_position.points.push_back(points);
        arm_points.publish(arm_position);
        ROS_INFO("%.2f", i);
        ros::Rate loop_rate(5); 
        loop_rate.sleep();

    }
    ROS_INFO("2");
    points.positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    points.time_from_start = ros::Duration(1.0);
    arm_position.points.clear(); 
    arm_position.points.push_back(points);
    arm_points.publish(arm_position);
    ros::Rate loop_rate(5); 
    loop_rate.sleep();


    points.time_from_start = ros::Duration(0.5);
    for(double i = 0; i < (M_PI/2); i += (M_PI/20)){
        points.positions = {0.0, 0.0, i, 0.0, 0.0};
        final_vector(0, 0, i);
        ROS_INFO("3");
        ROS_INFO("%.2f", i);
        arm_position.points.clear(); 
        arm_position.points.push_back(points);
        arm_points.publish(arm_position);
        ros::Rate loop_rate(5);  // 1 Hz
        loop_rate.sleep();
    }

    
    for(double i = 0; i < (2*M_PI); i += (M_PI/20)){
        points.positions = {i, 0.0, M_PI/2, 0.0, 0.0};
        final_vector(i, 0, M_PI/2);
        ROS_INFO("4");
        ROS_INFO("%.2f", i);
        arm_position.points.clear(); 
        arm_position.points.push_back(points);
        arm_points.publish(arm_position);
        ros::Rate loop_rate(7);
        loop_rate.sleep();
    }

    points.time_from_start = ros::Duration(0.3);
    for(double i = M_PI/2; i < (M_PI - 0.14) ; i += (M_PI/20)){
        points.positions = {0.0, 0.0, i, 0.0, 0.0};
        final_vector(0, 0, i);
        ROS_INFO("5");
        ROS_INFO("%.2f", i);
        arm_position.points.clear(); 
        arm_position.points.push_back(points);
        arm_points.publish(arm_position);
        ros::Rate loop_rate(7);
        loop_rate.sleep();
    }
    

    points.positions = {0.0, 0.0, 0.0, 0.0, 0.0};
    points.time_from_start = ros::Duration(1.0);
    arm_position.points.clear();
    arm_position.points.push_back(points);
    arm_points.publish(arm_position);
}

void control_logic(ros::NodeHandle &nh){
    ros::Subscriber sub_to_command = nh.subscribe("/arm_controller/state", 10,  callback);
    ros::Publisher arm_points = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 10);
    trajectory_msgs::JointTrajectory arm_position;
    arm_position.joint_names = {"arm_base_joint", "shoulder_joint", "bottom_wrist_joint", "elbow_joint", "top_wrist_joint"};
    trajectory_msgs::JointTrajectoryPoint points;    
    demostration(points, arm_points, arm_position);
       
}
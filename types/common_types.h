
#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <csignal>


namespace types{
    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Vec3d = Eigen::Vector3d;
    using Vec4f = Eigen::Vector4f;
    using Vec4d = Eigen::Vector4d;
    using Vec6f = Eigen::Matrix<float, 6, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    using VecXf = Eigen::VectorXf;
    using VecXd = Eigen::VectorXd;

    using Mat3f = Eigen::Matrix3f;
    using Mat3d = Eigen::Matrix3d;
    using MatXf = Eigen::MatrixXf;
    using MatXd = Eigen::MatrixXd;

    const float gravity = 9.815;

    struct RobotBasicState{
        Vec3f base_rpy;
        Vec4f base_quat;
        Mat3f base_rot_mat;
        Vec3f base_omega;
//        Vec3f flt_base_omega;
        MatXf flt_base_omega_mat;
        Vec3f base_acc;
//        Vec3f flt_base_acc;
        MatXf flt_base_acc_mat;
        VecXf joint_pos;
        VecXf joint_vel;
//        VecXf flt_joint_vel;
        MatXf flt_joint_vel_mat;
        VecXf joint_tau;
    };

    struct RobotAction{
        VecXf goal_joint_pos;
        VecXf goal_joint_vel;
        VecXf kp;
        VecXf kd;
        VecXf tau_ff;

        MatXf ConvertToMat(){
            MatXf res(goal_joint_pos.rows(), 5);
            res.col(0) = kp; res.col(1) = goal_joint_pos;
            res.col(2) = kd; res.col(3) = goal_joint_vel;
            res.col(4) = tau_ff;
            return res;
        }
    };
    
    
    struct UserCommand{
        double time_stamp;
        int safe_control_mode; //0 normal, 1 stand, 2 sit, 3 joint damping
        uint8_t target_mode;
        uint8_t target_gait;  // 1: 复杂地形  2：爬高台 
        float forward_vel_scale;
        float side_vel_scale;
        float turnning_vel_scale;

        bool bipedal_flag=false;
        bool inv_bipedal_flag=false;
        bool keepYawFlag=false;

        bool flipFlag=false;
        bool jumpFlag=false;
        int target_policy=0;

        float targetX=0;
        float targetY=0;
        float targetPitch=0;
        float targetRoll=0;
        float targetHeight=0;

        bool up_shift;          //for quadruped robot to accelerate
        bool down_shift;

        int flip;

        uint8_t locomotion_mode=0;
        bool start_flip=false;
    };

    // struct MotionDataReport{
    //     uint8_t motionstate;
    //     uint8_t gait;
    //     float vel_x, vel_y;
    //     float height;
    // }

    // struct MotionStateFeedback{


    //     MotionStateFeedback(){
            
    //     }


    // };
};

#endif
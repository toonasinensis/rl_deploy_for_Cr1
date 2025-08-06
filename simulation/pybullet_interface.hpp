/**
 * @file pybullet_interface.hpp
 * @brief 
 * @author mazunwang
 * @version 1.0
 * @date 2024-09-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef PYBULLET_SIMULATION_HPP_
#define PYBULLET_SIMULATION_HPP_

#include "robot_interface.h"
#include <iostream>
#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>


namespace interface {
    class PybulletInterface : public RobotInterface {
    private:
        double run_time_ = 0;
        Vec3f omega_body_, rpy_, acc_;
        VecXf joint_pos_, joint_vel_, joint_tau_;
        bool start_thread_flag_ = false;
        std::thread sim_thread_, send_thread_;
    public:
        PybulletInterface(const std::string &name, int dof_num) : RobotInterface(name, dof_num) {
            // run_cnt_ = 0;
            // gravity_ = projected_gravity;
            joint_pos_ = VecXf::Zero(dof_num_);
            joint_tau_ = VecXf::Zero(dof_num_);
            joint_vel_ = VecXf::Zero(dof_num_);

            joint_cmd_ = MatXf::Zero(dof_num_, 5);

            std::cout << name << " is using PyBullet Simulation" << std::endl;
        }

        virtual double GetInterfaceTimeStamp() {
            return run_time_;
        }

        virtual VecXf GetJointPosition() {
            return joint_pos_;
        };

        virtual VecXf GetJointVelocity() {
            return joint_vel_;
        }

        virtual VecXf GetJointTorque() {
            return joint_tau_;
        }

        virtual Vec3f GetImuRpy() {
            return rpy_;
        }

        virtual Vec3f GetImuAcc() {
            return acc_;
        }

        virtual Vec3f GetImuOmega() {
            return omega_body_;
        }

        virtual Vec3f GetImuBodyRpy() {
            return rpy_;
        }

        virtual Vec3f GetImuBodyAcc() {
            return acc_;
        }

        virtual Vec3f GetImuBodyOmega() {
            return omega_body_;
        }

        virtual VecXf GetContactForce() {
            return VecXf::Zero(4);
        }

        virtual VecXf GetMotorTemperture() {
            // return VecXf::Zero(dof_num_);
            return 30. * VecXf::Ones(dof_num_);
        }

        virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
            joint_cmd_ = input;
        }

        virtual void Start() {
            start_thread_flag_ = true;
            sim_thread_ = std::thread(std::bind(&PybulletInterface::ReceiveRobotData, this));
            send_thread_ = std::thread(std::bind(&PybulletInterface::SendJointCmd, this));

            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(0, &cpuset);

            pthread_setaffinity_np(sim_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
            pthread_setaffinity_np(send_thread_.native_handle(), sizeof(cpu_set_t), &cpuset);
        }

        virtual void Stop() {
            // start_flag_ = false;
            // sim_thread_.join();
        }

        void ReceiveRobotData() {
            int tfd;    //定时器描述符
            int efd;    //epoll描述符
            int fds;
            uint64_t value;
            struct epoll_event ev, *evptr;
            struct itimerspec time_intv; //用来存储时间

            tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
            if (tfd == -1) return;

            time_intv.it_value.tv_sec = 0.;
            time_intv.it_value.tv_nsec = 1000 * 1000;
            time_intv.it_interval.tv_sec = 0;
            time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

            timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

            efd = epoll_create1(0); //创建epoll实例
            if (efd == -1) close(tfd);

            evptr = (struct epoll_event *) calloc(1, sizeof(struct epoll_event));
            if (evptr == NULL) {
                close(tfd);
                close(efd);
            }

            ev.data.fd = tfd;
            ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
            epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中


            // 创建UDP套接字
            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                std::cerr << "Error creating socket" << std::endl;
                return;
            }

            // 绑定套接字到端口
            struct sockaddr_in serverAddr;
            serverAddr.sin_family = AF_INET;
            serverAddr.sin_port = htons(30010); // 端口号
            serverAddr.sin_addr.s_addr = INADDR_ANY;
            if (bind(sockfd, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) < 0) {
                std::cerr << "Error binding socket" << std::endl;
                return;
            }

            // 接收数据
            char buffer[1024] = {0};
            int data_length = 1 + 3 + 3 + 3 + dof_num_ * 3;
            float *data = new float[data_length];
            struct sockaddr_in clientAddr;
            socklen_t clientAddrLen = sizeof(clientAddr);
            while (start_thread_flag_) {
                fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
                if (evptr[0].events & EPOLLIN) {
                    int ret = read(evptr->data.fd, &value, sizeof(uint64_t));
                    if (ret == -1) {
                        continue;
                    } else {
                    }
                }
                int recvLen = recvfrom(sockfd, buffer, 1024, 0, (struct sockaddr *) &clientAddr, &clientAddrLen);
                if (recvLen < 0) {
                    std::cerr << "Error receiving data" << std::endl;
                    return;
                }

                std::memcpy(data, buffer, data_length * sizeof(float));
                run_time_ = data[0];
                rpy_ = Eigen::Map<Vec3f>(data + 1, 3);
                acc_ = Eigen::Map<Vec3f>(data + 4, 3);
                omega_body_ = Eigen::Map<Vec3f>(data + 7, 3);
                joint_pos_ = Eigen::Map<VecXf>(data + 10, dof_num_);
                joint_vel_ = Eigen::Map<VecXf>(data + 10 + dof_num_, dof_num_);
                joint_tau_ = Eigen::Map<VecXf>(data + 10 + dof_num_ + dof_num_, dof_num_);

//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | rpy_:" << rpy_.transpose()
//                          << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | acc_:" << acc_.transpose()
//                          << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | omega_body_:"
//                          << omega_body_.transpose() << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | joint_pos_:"
//                          << joint_pos_.transpose() << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | joint_vel_:"
//                          << joint_vel_.transpose() << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | joint_tau_:"
//                          << joint_tau_.transpose() << std::endl;
                // 打印接收到的数据
//                 std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | joint_pos_:" << joint_pos_.transpose() << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | acc_:" << acc_.transpose() << std::endl;
//                std::cout << "Received data: " << std::setprecision(5) << run_time_ << " | omega_body_:" << omega_body_.transpose() << std::endl;

            }

            // 关闭套接字
            close(sockfd);
        }

        void SendJointCmd() {
            int tfd;    //定时器描述符
            int efd;    //epoll描述符
            int fds;
            uint64_t value;
            struct epoll_event ev, *evptr;
            struct itimerspec time_intv; //用来存储时间

            tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
            if (tfd == -1) return;

            time_intv.it_value.tv_sec = 0.;
            time_intv.it_value.tv_nsec = 1000 * 1000;
            time_intv.it_interval.tv_sec = 0;
            time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

            timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

            efd = epoll_create1(0); //创建epoll实例
            if (efd == -1) close(tfd);

            evptr = (struct epoll_event *) calloc(1, sizeof(struct epoll_event));
            if (evptr == NULL) {
                close(tfd);
                close(efd);
            }

            ev.data.fd = tfd;
            ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
            epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中


            int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
            if (sockfd < 0) {
                std::cerr << "Error creating socket" << std::endl;
                return;
            }

            // 设置服务器地址和端口
            struct sockaddr_in serverAddr;
            serverAddr.sin_family = AF_INET;
            serverAddr.sin_port = htons(20001); // 服务器端口号
            serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // 服务器IP地址

            float *data = new float[dof_num_ * 5];
            std::memset(data, 0, sizeof(float) * 5 * dof_num_);

            while (start_thread_flag_) {
                fds = epoll_wait(efd, evptr, 1, -1);    //阻塞监听，直到有事件发生
                if (evptr[0].events & EPOLLIN) {
                    int ret = read(evptr->data.fd, &value, sizeof(uint64_t));
                    if (ret == -1) {
                        continue;
                    } else {
                    }
                }
                // int recvLen = recvfrom(sockfd, buffer, 1024, 0, (struct sockaddr*)&clientAddr, &clientAddrLen);
                // if (recvLen < 0) {
                //     std::cerr << "Error receiving data" << std::endl;
                //     return;
                // }
                // joint_cmd_.setRandom();
                std::memcpy(data, joint_cmd_.data(), sizeof(float) * joint_cmd_.size());
//                std::cout << "kp " <<joint_cmd_.col(0).transpose()<< std::endl;
//                std::cout << "kd " << joint_cmd_.col(2).transpose() << std::endl;
//                std::cout << "joint_pos " << joint_cmd_.col(1).transpose() << std::endl;
//                std::cout << "joint_vel " << joint_cmd_.col(3).transpose() << std::endl;
//                std::cout << "tau_ff " << joint_cmd_.col(4).transpose() << std::endl;

//                std::cout << "cmd: " << joint_cmd_.col(4).transpose() << std::endl;
                // std::cout << "cmd: " << joint_cmd_.size() << std::endl;
                // for(int i=0;i<joint_cmd_.size();++i){
                //     std::cout << data[i] << " ";
                // }
                // std::cout << std::endl;
                // std::cout << "joint_cmd: " << joint_cmd_.transpose() << "\n" << std::endl;

                sendto(sockfd, data, joint_cmd_.size() * sizeof(float), 0, (struct sockaddr *) &serverAddr,
                       sizeof(serverAddr));
            }

            // 关闭套接字
            close(sockfd);
        }
    };

};


#endif
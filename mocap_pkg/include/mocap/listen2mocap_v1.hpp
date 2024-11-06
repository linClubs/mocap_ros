#pragma once
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cstring>
#include <iostream>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <mutex>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include "sphere2hinge.hpp"
#include "readCSV.hpp"
#include "Dataanalysis.hpp"

// #define PI 3.14159265
class VDMocap
{
public:
    VDMocap()
    {
        simMatrix << 0, 1, 0,
            -1, 0, 0,
            0, 0, 1;
    }

    void readLocalData(std::string file)
    {
        using namespace std::chrono_literals;
        auto jntAnima = readCSV(file);

        for (int frmIdx = 0; frmIdx < jntAnima.size(); frmIdx++)
        {
            upJoints = jntAnima[frmIdx];
            std::this_thread::sleep_for(33ms);
        }
    }

    // readlocaldata with timeframe
    // copyed from readLocalData function
    void readLocalDataWithTF(std::string file)
    {
        using namespace std::chrono_literals;
        auto jntAnima = readCSV(file);

        int64_t time_diff = 0;
        int64_t startTime = jntAnima[0][0];
        for (int frmIdx = 0; frmIdx < jntAnima.size(); frmIdx++)
        {

            upJoints.assign(jntAnima[frmIdx].begin() + 1, jntAnima[frmIdx].end());
            std::this_thread::sleep_for(1ms);
            if (frmIdx == 0)
            {
                std::this_thread::sleep_for(32ms);
            }
            else
            {

                time_diff = jntAnima[frmIdx][0] - jntAnima[frmIdx - 1][0];
                std::this_thread::sleep_for(40ms);
            }
        }
    }

    void readOnlineData()
    {
        int sock_data = socket(AF_INET, SOCK_DGRAM, 0);
        int sock_control = socket(AF_INET, SOCK_DGRAM, 0);

        sockaddr_in control_address;
        memset(&control_address, 0, sizeof(control_address));
        control_address.sin_family = AF_INET;
        control_address.sin_addr.s_addr = INADDR_ANY;
        control_address.sin_port = htons(CONTROL_PORT);

        if (bind(sock_control, (struct sockaddr *)&control_address, sizeof(control_address)) < 0)
        {
            std::cerr << "Binding failed" << std::endl;
            return;
        }

        sockaddr_in target_address;
        memset(&target_address, 0, sizeof(target_address));
        target_address.sin_family = AF_INET;
        target_address.sin_port = htons(DATA_PORT);
        inet_pton(AF_INET, TARGET_IP.c_str(), &target_address.sin_addr);

        std::vector<int> target_joint_list = {0, 9, 16, 17, 20, 21, 1, 2, 3, 5, 6, 7};

        // set hand joint list
        std::vector<int> hand_joint_list = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

        Eigen::Vector4d initialGuessR = Eigen::Vector4d(0.1, 0.1, 0.1, 0.1);
        Eigen::Vector4d initialGuessL = Eigen::Vector4d(0.1, 0.1, 0.1, 0.1);
        while (true)
        {
            send_command(sock_data, DATA_REQUEST_HEX, target_address);
            std::vector<uint8_t> data = receive_data(sock_data);
            struct_ReceivedMotionData calculation_data;
            BytestoCalculationData(data.data(), calculation_data);

            int seq = 0;
            for (auto idx : target_joint_list)
            {
                auto quat = calculation_data.quat_nb_body[idx];
                globalRots[seq++] = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
                // std::cout << idx << ":" << quat[0] << quat[1] << quat[2] << quat[3] << " " << std::endl;
            }

            localRots[0] = globalRots[0];
            localRots[1] = globalRots[0].transpose() * globalRots[1];
            localRots[2] = globalRots[1].transpose() * globalRots[2];
            localRots[3] = globalRots[2].transpose() * globalRots[3];
            localRots[4] = globalRots[1].transpose() * globalRots[4];
            localRots[5] = globalRots[4].transpose() * globalRots[5];
            localRots[6] = globalRots[0].transpose() * globalRots[6];
            localRots[7] = globalRots[6].transpose() * globalRots[7];
            localRots[8] = globalRots[7].transpose() * globalRots[8];
            localRots[9] = globalRots[0].transpose() * globalRots[9];
            localRots[10] = globalRots[9].transpose() * globalRots[10];
            localRots[11] = globalRots[10].transpose() * globalRots[11];
            
            // 左右
            upperVecR = simMatrix * localRots[2] * Eigen::Vector3d(1, 0, 0);
            lowerVecR = simMatrix * localRots[2] * localRots[3] * Eigen::Vector3d(1, 0, 0);
            upperVecL = simMatrix * localRots[4] * Eigen::Vector3d(-1, 0, 0);
            lowerVecL = simMatrix * localRots[4] * localRots[5] * Eigen::Vector3d(-1, 0, 0);

            auto [isSucL, resL] = sphere2hingeLeft(upperVecL, lowerVecL, initialGuessL);
            auto [isSucR, resR] = sphere2hingeRight(upperVecR, lowerVecR, initialGuessR);
            
            std::vector<float> _upJoints{0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

            // {
            //     std::lock_guard<std::mutex> lock(jointsMutex);
                if (isSucL)
                {
                    initialGuessL = resL;
                    for (int i = 0; i < 4; i++)
                    {
                        upJoints[i] = resL[i];
                    }
                    _upJoints[3] += PI / 2.0 - 0.2;
                    _upJoints[1] += 0.1;
                }
                else
                {
                    printf("right retarget failed: \n");
                    std::cout << upperVecL.transpose() << "\n"
                              << lowerVecL.transpose() << std::endl;
                }

                if (isSucR)
                {
                    initialGuessR = resR;
                    for (int i = 0; i < 4; i++)
                    {
                        upJoints[i + 4] = resR[i];
                    }
                    _upJoints[7] += PI / 2.0 - 0.2;
                    _upJoints[5] -= 0.2;
                }
                else
                {
                    printf("left retarget failed: \n");
                    std::cout << upperVecR.transpose() << "\n"
                              << lowerVecR.transpose() << std::endl;
                }
            // }
            
            // get left hand joints up position
            seq = 0;
            for (auto idx : hand_joint_list)
            {
                auto quat = calculation_data.quat_nb_lHand[idx];
                globalHandRots[seq++] = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
                // std::cout <<"left hand "<< idx << ":" << quat[0] << quat[1] << quat[2] << quat[3] << " " << std::endl;
            }

            localHandRots[0] = globalHandRots[0];                                    // hand nd
            localHandRots[1] = globalHandRots[0].transpose() * globalHandRots[1];    // thumb       simulation*** need to use for thumb pro 1 and 2
            localHandRots[2] = globalHandRots[1].transpose() * globalHandRots[2];    // thumb 1     simulation*** thumb inter
            localHandRots[3] = globalHandRots[2].transpose() * globalHandRots[3];    // thumb 2     simulation*** thumb distal
            localHandRots[4] = globalHandRots[4];                                    // index
            localHandRots[5] = globalHandRots[4].transpose() * globalHandRots[5];    // index 1
            localHandRots[6] = globalHandRots[5].transpose() * globalHandRots[6];    // index 2
            localHandRots[7] = globalHandRots[6].transpose() * globalHandRots[7];    // index 3
            localHandRots[8] = globalHandRots[8];                                    // middle
            localHandRots[9] = globalHandRots[8].transpose() * globalHandRots[9];    // middle 1
            localHandRots[10] = globalHandRots[9].transpose() * globalHandRots[10];  // middle 2
            localHandRots[11] = globalHandRots[10].transpose() * globalHandRots[11]; // middle 3
            localHandRots[12] = globalHandRots[12];                                  // ring
            localHandRots[13] = globalHandRots[12].transpose() * globalHandRots[13]; // ring 1
            localHandRots[14] = globalHandRots[13].transpose() * globalHandRots[14]; // ring 2
            localHandRots[15] = globalHandRots[14].transpose() * globalHandRots[15]; // ring 3
            localHandRots[16] = globalHandRots[16];                                  // pinky
            localHandRots[17] = globalHandRots[16].transpose() * globalHandRots[17]; // pinky 1
            localHandRots[18] = globalHandRots[17].transpose() * globalHandRots[18]; // pinky 2
            localHandRots[19] = globalHandRots[18].transpose() * globalHandRots[19]; // pinky 3

            // get hand joints up position
            _upJoints[8] = 0; // left hand joint
            // for loop to calculate angles
            Eigen::Vector3d newAxis;
            double newAngle = 0;
            for (int i = 0; i < 20; i++)
            {
                auto [newAxis, newAngle] = rotationMatrixToAxisAngle(localHandRots[i]);
                // float newAngle = newAngleAxis.angle() * newAxis[2];
                handSim[i] = newAngle;
                // std::cout << i << ":" << newAngle << " " << std::endl;
            }

            _upJoints[9] = 0.2;                                 // thumb prox pitch
            _upJoints[10] = clamp(abs_(handSim[1]), -0.1, 0.6); // thumb prox yaw
            _upJoints[11] = clamp(abs_(handSim[2]), 0, 0.8);    // thumb prox inter
            _upJoints[12] = clamp(abs_(handSim[3]), 0, 1.2);    // thumb prox distal

            for (int i = 0; i < 4; i++)
            {
                _upJoints[13 + i * 2] = clamp(abs_(handSim[5 + i * 4]), 0, 1.7); // index inter
                _upJoints[14 + i * 2] = clamp(abs_(handSim[6 + i * 4]), 0, 1.7); // index distal
            }

            // get right hand joints up position
            seq = 0;
            for (auto idx : hand_joint_list)
            {
                auto quat = calculation_data.quat_nb_rHand[idx];
                globalHandRots[seq++] = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).normalized().toRotationMatrix();
                // std::cout << idx << ":" << quat[0] << quat[1] << quat[2] << quat[3] << " " << std::endl;
            }

            localHandRots[0] = globalHandRots[0];                                    // hand nd
            localHandRots[1] = globalHandRots[0].transpose() * globalHandRots[1];    // thumb       simulation*** need to use for thumb pro 1 and 2
            localHandRots[2] = globalHandRots[1].transpose() * globalHandRots[2];    // thumb 1     simulation*** thumb inter
            localHandRots[3] = globalHandRots[2].transpose() * globalHandRots[3];    // thumb 2     simulation*** thumb distal
            localHandRots[4] = globalHandRots[4];                                    // index
            localHandRots[5] = globalHandRots[4].transpose() * globalHandRots[5];    // index 1
            localHandRots[6] = globalHandRots[5].transpose() * globalHandRots[6];    // index 2
            localHandRots[7] = globalHandRots[6].transpose() * globalHandRots[7];    // index 3
            localHandRots[8] = globalHandRots[8];                                    // middle
            localHandRots[9] = globalHandRots[8].transpose() * globalHandRots[9];    // middle 1
            localHandRots[10] = globalHandRots[9].transpose() * globalHandRots[10];  // middle 2
            localHandRots[11] = globalHandRots[10].transpose() * globalHandRots[11]; // middle 3
            localHandRots[12] = globalHandRots[12];                                  // ring
            localHandRots[13] = globalHandRots[12].transpose() * globalHandRots[13]; // ring 1
            localHandRots[14] = globalHandRots[13].transpose() * globalHandRots[14]; // ring 2
            localHandRots[15] = globalHandRots[14].transpose() * globalHandRots[15]; // ring 3
            localHandRots[16] = globalHandRots[16];                                  // pinky
            localHandRots[17] = globalHandRots[16].transpose() * globalHandRots[17]; // pinky 1
            localHandRots[18] = globalHandRots[17].transpose() * globalHandRots[18]; // pinky 2
            localHandRots[19] = globalHandRots[18].transpose() * globalHandRots[19]; // pinky 3

            // get hand joints up position
            _upJoints[21] = 0; // right hand joint

            for (int i = 0; i < 20; i++)
            {
                auto [newAxis, newAngle] = rotationMatrixToAxisAngle(localHandRots[i]);
                // float newAngle = newAngleAxis.angle() * newAxis[2];
                handSim[i] = newAngle;
                // std::cout << "right hand: ";
                // std::cout << i << ":" << newAngle << " " << std::endl;
            }
            _upJoints[22] = 0.2;                            // thumb prox pitch
            _upJoints[23] = clamp((handSim[1]), -0.1, 0.6); // thumb prox yaw
            _upJoints[24] = clamp(handSim[2], 0, 0.8);      // thumb prox inter
            _upJoints[25] = clamp(handSim[3], 0, 1.2);      // thumb prox distal

            for (int i = 0; i < 4; i++)
            {
                _upJoints[26 + i * 2] = clamp(handSim[5 + i * 4], 0, 1.7); // index inter
                _upJoints[27 + i * 2] = clamp(handSim[6 + i * 4], 0, 1.7); // index distal
            }

            {
            std::lock_guard<std::mutex> lock(jointsMutex);
            for(int i = 0; i < _upJoints.size(); ++i)
            {
                upJoints[i] = _upJoints[i];
            }
            }
        }

        send_command(sock_data, DATA_DISCONNECT_HEX, target_address);
        close(sock_control);
        close(sock_data);
        return;
    }

    std::vector<float> getJoints()
    {
        std::lock_guard<std::mutex> lock(jointsMutex);
        return upJoints;
    }

    void getVec(Eigen::Vector3d &upL, Eigen::Vector3d &loL, Eigen::Vector3d &upR, Eigen::Vector3d &loR)
    {
        std::lock_guard<std::mutex> lock(mocapMutex);
        upL = upperVecL;
        loL = lowerVecL;
        upR = upperVecR;
        loR = lowerVecR;
    }

    void setVec(const Eigen::Vector3d &upL, const Eigen::Vector3d &loL, const Eigen::Vector3d &upR, const Eigen::Vector3d &loR)
    {
        std::lock_guard<std::mutex> lock(mocapMutex);
        upperVecL = upL;
        lowerVecL = loL;
        upperVecR = upR;
        lowerVecR = loR;
    }

private:
    std::vector<uint8_t> hex_string_to_bytes(const std::string &hex_str)
    {
        std::vector<uint8_t> bytes;
        for (unsigned int i = 0; i < hex_str.length(); i += 2)
        {
            std::string byteString = hex_str.substr(i, 2);
            uint8_t byte = (uint8_t)strtol(byteString.c_str(), NULL, 16);
            bytes.push_back(byte);
        }
        return bytes;
    }

    void send_command(int sock, const std::string &command, const sockaddr_in &target)
    {
        std::vector<uint8_t> bytes = hex_string_to_bytes(command);
        ssize_t sent = sendto(sock, bytes.data(), bytes.size(), 0, (struct sockaddr *)&target, sizeof(target));
        if (sent == -1)
        {
            std::cerr << "send to failed" << std::endl;
        }
    }

    std::vector<uint8_t> receive_data(int sock)
    {
        std::vector<uint8_t> data(1024);
        sockaddr_in sender;
        socklen_t sender_len = sizeof(sender);
        ssize_t received = recvfrom(sock, data.data(), data.size(), 0, (struct sockaddr *)&sender, &sender_len);
        if (received > 0)
        {
            data.resize(received);
        }
        else
        {
            data.clear();
        }
        return data;
    }
    float clamp(float value, float min, float max)
    {
        if (value < min)
            return min;
        if (value > max)
            return max;
        return value;
    }
    float abs_(float value)
    {
        if (value < 0)
            return value * (-1);
        return value;
    }

    // 自定义 isclose 函数
    bool isclose(double a, double b, double rel_tol = 1e-09, double abs_tol = 0.0)
    {
        return std::abs(a - b) <= std::max(rel_tol * std::max(std::abs(a), std::abs(b)), abs_tol);
    }

    // 自定义函数，将旋转矩阵转换为轴角
    std::pair<Eigen::Vector3d, double> rotationMatrixToAxisAngle(const Eigen::Matrix3d &rotationMatrix)
    {
        // 从旋转矩阵获得四元数
        Eigen::Quaterniond quaternion(rotationMatrix);

        // 提取四元数的系数
        double qw = quaternion.w();
        double qx = quaternion.x();
        double qy = quaternion.y();
        double qz = quaternion.z();

        // 计算旋转角度
        double angle = 2 * std::acos(std::clamp(qw, -1.0, 1.0)); // limit to -1 1

        Eigen::Vector3d axis;
        if (isclose(angle, 0.0))
        {
            axis = Eigen::Vector3d(0, 0, 1);
            angle = 0.0;
        }
        else if (isclose(angle, 2 * M_PI))
        {
            axis = Eigen::Vector3d(0, 0, 1);
            angle = 2 * M_PI;
        }
        else
        {
            // 计算轴
            double s = std::sqrt(1 - qw * qw);
            if (s < 1e-6)
            {
                // 如果s非常小，说明旋转角度接近0或2π
                axis = Eigen::Vector3d(qx, qy, qz);
            }
            else
            {
                axis = Eigen::Vector3d(qx / s, qy / s, qz / s);
            }
        }

        return std::make_pair(axis, angle);
    }

private:
    std::mutex mocapMutex;
    std::mutex jointsMutex;

    Eigen::Matrix3d globalRots[12];
    Eigen::Matrix3d localRots[12];
    Eigen::Vector3d upperVecR, lowerVecR, upperVecL, lowerVecL;
    Eigen::Matrix3d simMatrix;

    // hand rotations
    Eigen::Matrix3d globalHandRots[20];
    Eigen::Matrix3d localHandRots[20];

    const int DATA_PORT = 7000;
    const int CONTROL_PORT = 12347;
    const std::string DATA_REQUEST_HEX = "fa00000b0403a2532352ce3299f432fb30";
    const std::string DATA_DISCONNECT_HEX = "fa000003040ba1fba8";
    const std::string TARGET_IP = "192.168.8.224";

    // upper and hand joints qpos 8 + 1(lefthand) + 12(left fingers) + 1(righthand) + 12(right fingers)
    std::vector<float> upJoints{0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    std::vector<double> handSim{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};
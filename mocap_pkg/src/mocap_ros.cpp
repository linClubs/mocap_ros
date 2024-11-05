#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "mocap/readCSV.hpp"
#include "mocap/listen2mocap_v1.hpp"
#include "mocap/sphere2hinge.hpp"


static VDMocap *mocap;
bool hasVectorChanged(const std::vector<float> &vec1, const std::vector<float> &vec2, float thereshold = 0.00001)
{
    if (vec1.size() != vec2.size())
    {
        throw std::invalid_argument("Vectors must be of equal length");
    }
    float sum_of_square_diff = 0.0;
    for (int i = 0; i < vec1.size(); ++i)
    {
        sum_of_square_diff += (vec1[i] - vec2[i]) * (vec1[i] - vec2[i]);
    }

    return sum_of_square_diff > thereshold;
}

//  save data to csv
void saveVectorToCSV(const std::vector<float> &vec, const std::string &filename)
{
    std::ofstream file(filename, std::ios::app); // Open file in append mode

    auto now = std::chrono::system_clock::now();
    static auto start = now;
    auto duration = now - start;
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();

    file << millis; // Write the timeframe

    for (const auto &val : vec)
    {
        file << "," << val; // Write the vector values
    }

    file << "\n"; // End of line
    file.close();
    // print vector
    printf("\nsaved:");
    for (const auto &val : vec)
    {
        printf("%f ", val);
    }
}


void simDataThread()
{
    using namespace std::chrono_literals;


        // 34 vector 8 + 1(lefthand) + 12(left fingers) + 1(righthand) + 12(right fingers)
    auto upJoints = mocap->getJoints();
    std::vector<float> qpos = std::vector<float>(34, 0.);
    
    // send to simulator pengfei version
    static float coeff = 0;
    // coeff = coeff < 1.0 ? (coeff + 0.01) : 1.0;
    // for (int j = 0; j < 8; j++) {
    //   d->qpos[11 + j] = (1 - coeff) * d->qpos[11 + j] + coeff * upJoints[j]; // slide to mocap result
    // }

    // send to simulator : adding hand joints
    coeff = coeff < 1.0 ? (coeff + 0.01) : 1.0;
    for (int j = 0; j < 8; j++)
    {
        // # 11 : left_shoulder_pitch_joint
        // # 12 : left_shoulder_roll_joint
        // # 13 : left_shoulder_yaw_joint
        // # 14 : left_elbow_joint
        if (j < 4)
        {
            qpos[11 + j] = (1 - coeff) * qpos[11 + j] + coeff * upJoints[j];
        } 
            // slide to mocap result
            // # 28 : right_shoulder_pitch_joint
            // # 29 : right_shoulder_roll_joint
            // # 30 : right_shoulder_yaw_joint
            // # 31 : right_elbow_joint
        else
        {
            qpos[28 + j - 4] = (1 - coeff) * qpos[28 + j - 4] + coeff * upJoints[j];
        }
        // d->qpos[11 + j] = (1 - coeff) * d->qpos[11 + j] + coeff * upJoints[j]; // slide to mocap result
    }
    for (int j = 0; j < 26; j++)
    {
        // left hand and fingers
        if (j < 13)
        {
            qpos[15 + j] = (1 - coeff) * qpos[15 + j] + coeff * upJoints[j + 8];
        }
        // right hand and fingers
        else
        {
            qpos[32 + j - 13] = (1 - coeff) * qpos[32 + j - 13] + coeff * upJoints[j + 8];
        }
    }
    // save data
    
    std::this_thread::sleep_for(10ms); // 25Hz
}

int main(int argc, char const *argv[])
{
    using namespace std::chrono_literals;
    if (argc < 2)
    {
        std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
        exit(-1);
    }
    printf("start \n");
    std::vector<std::thread> myThreads;

    /********************************** START MOCAP THREAD ***************************************/
    mocap = new VDMocap();
    // read online data from motoincapture device;
    myThreads.push_back(std::thread(&VDMocap::readOnlineData, mocap));
    std::this_thread::sleep_for(10ms);

    return 0;
}
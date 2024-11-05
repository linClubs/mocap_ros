
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "readCSV.hpp"
#include "listen2mocap_v1.hpp"
#include "sphere2hinge.hpp"


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


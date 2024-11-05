#pragma once

#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

std::vector<std::vector<float>> readCSV(const std::string &file_name)
{
    std::ifstream file(file_name);
    std::vector<std::vector<float>> data;
    if (!file.is_open())
    {
        std::cerr << "Error opening file!" << std::endl;
        return data;
    }

    std::vector<float> temp;
    std::string line;
    int row = 0;
    while (getline(file, line))
    {
        std::stringstream ss(line);
        std::string cell;
        int col = 0;
        temp.clear();
        while (getline(ss, cell, ','))
        {
            temp.push_back(std::stof(cell));
            col++;
        }
        data.push_back(temp);
        row++;
    }
    file.close();
    return data;
}

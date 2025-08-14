#include "filter_utility_functions.h"

#include "../pch.h"

std::vector<float> readFirFilterFile(const std::string& filePath)
{
    std::ifstream inputFile(filePath);
    if (!inputFile.is_open())
    {
        throw std::runtime_error("Unable to open filter file: " + filePath);
    }

    std::vector<float> filterCoefficients;
    std::string line;
    while (std::getline(inputFile, line))
    {
        std::stringstream lineStream(line);
        std::string token;
        while (std::getline(lineStream, token, ','))
        {
            filterCoefficients.push_back(std::stof(token));
        }
    }
    return filterCoefficients;
}
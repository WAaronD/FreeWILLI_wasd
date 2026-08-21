// test_onnx_integration.cpp
//
// Runs the SAME raw test snippets through FreeWILLI's actual ONNXModel class,
// and writes predictions to cpp_predictions.csv for comparison against
// reference_predictions.csv (Python/onnxruntime). Works for any number of classes --
// CLASS_NAMES order must match load_and_normalize.py / generate_reference_predictions.py.
//
// Vocabulary notes (new to C++):
//   #include "onnx_model.h"   -> pulls in the ONNXModel class declaration (like
//                                a MATLAB function needing its .m file on path)
//   std::ifstream              -> file reading stream (like fopen/fscanf in C,
//                                or fileread/readmatrix in MATLAB)
//   std::vector<float>          -> dynamically-sized array (like a MATLAB vector,
//                                but you must declare the type)
//   std::max_element            -> finds the largest element in a range and returns
//                                an iterator to it (like MATLAB's [~, idx] = max(x))
//
// Build: this file must be compiled as part of the FreeWILLI project (it needs
// onnx_model.h/.cpp, pch.h, and linked ONNX Runtime + Eigen). Add it as an
// additional source/target in FreeWILLI's existing build system (e.g. CMakeLists.txt)
// rather than trying to compile it standalone.

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "onnx_model.h"

const std::string kDataDir = "/media/wasd/Aaron Data 1/training/";
const std::vector<std::string> kClassNames = {"zc", "wbat", "delphinid", "gg", "noise"};

// Reads a CSV file of raw snippets (one row per snippet, 176 comma-separated values)
std::vector<std::vector<float>> loadCsv(const std::string& path)
{
    std::vector<std::vector<float>> rows;
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw std::runtime_error("Could not open file: " + path);
    }

    std::string line;
    while (std::getline(file, line))
    {
        std::vector<float> row;
        std::stringstream ss(line);
        std::string value;
        while (std::getline(ss, value, ','))
        {
            row.push_back(std::stof(value));
        }
        rows.push_back(row);
    }
    return rows;
}

int main()
{
    // ---- 1. Load model + scaler ----
    ONNXModel model(kDataDir + "click_classifier.onnx", kDataDir + "scaler_params.json");

    // ---- 2. Load raw test snippets for every class ----
    std::vector<std::vector<float>> allRows;
    std::vector<int> trueLabels;

    for (size_t classIdx = 0; classIdx < kClassNames.size(); ++classIdx)
    {
        auto rows = loadCsv(kDataDir + "test_" + kClassNames[classIdx] + ".csv");
        std::cout << "test_" << kClassNames[classIdx] << " rows: " << rows.size() << "\n";

        allRows.insert(allRows.end(), rows.begin(), rows.end());
        for (size_t i = 0; i < rows.size(); ++i)
        {
            trueLabels.push_back(static_cast<int>(classIdx));
        }
    }

    // ---- 3. Run inference row by row ----
    std::ofstream out(kDataDir + "cpp_predictions.csv");
    out << "index,true_label,predicted_class";
    for (const auto& name : kClassNames) out << ",prob_" << name;
    out << "\n";

    int correct = 0;
    for (size_t i = 0; i < allRows.size(); ++i)
    {
        std::vector<float> inputCopy = allRows[i];  // runInference normalizes in place
        std::vector<float> output = model.runInference(inputCopy);

        // Generic argmax over N outputs
        size_t bestIdx = std::distance(output.begin(), std::max_element(output.begin(), output.end()));
        int predictedClass = static_cast<int>(bestIdx);
        if (predictedClass == trueLabels[i]) ++correct;

        out << i << "," << trueLabels[i] << "," << predictedClass;
        for (float p : output) out << "," << p;
        out << "\n";
    }
    out.close();

    std::cout << "Saved cpp_predictions.csv (" << allRows.size() << " rows, "
              << kClassNames.size() << " classes)\n";
    std::cout << "C++ pipeline accuracy: "
              << static_cast<float>(correct) / static_cast<float>(allRows.size()) << "\n";

    return 0;
}
// test_onnx_integration.cpp
//
// Runs the SAME raw test snippets (test_ziphius.csv + test_wbat.csv) through
// FreeWILLI's actual ONNXModel class, and writes predictions to cpp_predictions.csv
// for comparison against reference_predictions.csv (Python/onnxruntime).
//
// Vocabulary notes (new to C++):
//   #include "onnx_model.h"   -> pulls in the ONNXModel class declaration (like
//                                a MATLAB function needing its .m file on path)
//   std::ifstream              -> file reading stream (like fopen/fscanf in C,
//                                or fileread/readmatrix in MATLAB)
//   std::vector<float>          -> dynamically-sized array (like a MATLAB vector,
//                                but you must declare the type)
//
// Build: this file must be compiled as part of the FreeWILLI project (it needs
// onnx_model.h/.cpp, pch.h, and linked ONNX Runtime + Eigen). Add it as an
// additional source/target in FreeWILLI's existing build system (e.g. CMakeLists.txt)
// rather than trying to compile it standalone.

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "onnx_model.h"

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
    // ---- 1. Load model + scaler (same paths used elsewhere in FreeWILLI) ----
    ONNXModel model("/media/wasd/Aaron Data 1/training/click_classifier.onnx", "/media/wasd/Aaron Data 1/training/scaler_params.json");

    // ---- 2. Load raw test snippets (same files used in Python) ----
    auto ziphiusRows = loadCsv("/media/wasd/Aaron Data 1/training/test_ziphius.csv");
    auto wbatRows = loadCsv("/media/wasd/Aaron Data 1/training/test_wbat.csv");

    // ---- 3. Concatenate: Ziphius first (label 0), then WBAT (label 1) ----
    // This matches the order used in generate_reference_predictions.py
    std::vector<std::vector<float>> allRows = ziphiusRows;
    allRows.insert(allRows.end(), wbatRows.begin(), wbatRows.end());

    std::vector<int> trueLabels;
    for (size_t i = 0; i < ziphiusRows.size(); ++i) trueLabels.push_back(0);
    for (size_t i = 0; i < wbatRows.size(); ++i) trueLabels.push_back(1);

    // ---- 4. Run inference row by row ----
    std::ofstream out("/media/wasd/Aaron Data 1/training/cpp_predictions.csv");
    out << "index,true_label,predicted_class,prob_ziphius,prob_wbat\n";

    int correct = 0;
    for (size_t i = 0; i < allRows.size(); ++i)
    {
        std::vector<float> inputCopy = allRows[i];  // runInference normalizes in place
        std::vector<float> output = model.runInference(inputCopy);

        // output[0] = prob Ziphius, output[1] = prob WBAT (matches training label order)
        int predictedClass = (output[1] > output[0]) ? 1 : 0;
        if (predictedClass == trueLabels[i]) ++correct;

        out << i << "," << trueLabels[i] << "," << predictedClass << ","
            << output[0] << "," << output[1] << "\n";
    }
    out.close();

    std::cout << "Saved cpp_predictions.csv (" << allRows.size() << " rows)\n";
    std::cout << "C++ pipeline accuracy: "
              << static_cast<float>(correct) / static_cast<float>(allRows.size()) << "\n";

    return 0;
}
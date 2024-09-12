#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>  // Use a JSON library to load JSON data
#include <chrono>

#include "onnx_model.h"

// Function to load validation spectra from a JSON file
std::vector<std::vector<float>> load_val_spectra(const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw std::runtime_error("Error opening val_spectra file.");
    }
    nlohmann::json spectra_json;
    file >> spectra_json;

    return spectra_json["val_spectra"].get<std::vector<std::vector<float>>>();
}

// Function to write the max prediction to a file
void write_max_prediction_to_file(const std::vector<float>& predictions, std::ofstream& output_file) {
    auto max_it = std::max_element(predictions.begin(), predictions.end());
    int max_index = std::distance(predictions.begin(), max_it);
    output_file << max_index << std::endl;
}

int main(int argc, char* argv[]) {
    // Open a file to write max predictions
    std::ofstream output_file("max_predictions2.txt");
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not open file for writing." << std::endl;
        return 1;
    }

    // Initialize model inference
    ONNXModel model_inference("model_quantized_static.onnx", "scaler_params.json");

    // Load validation data
    std::vector<std::vector<float>> val_spectra = load_val_spectra("val_spectra.json");
    if (val_spectra.empty()) {
        std::cerr << "Error: val_spectra data is empty." << std::endl;
        return 1;
    }

    // Process each sample
    for (int sample_idx = 0; sample_idx < val_spectra.size(); ++sample_idx) {
        std::vector<float> input_tensor_values = val_spectra[sample_idx];

        // Normalize the input data
        model_inference.normalize_data(input_tensor_values);

        // Run inference
        std::vector<float> predictions = model_inference.run_inference(input_tensor_values);

        // Write the max prediction to file
        write_max_prediction_to_file(predictions, output_file);
    }

    output_file.close();
    return 0;
}

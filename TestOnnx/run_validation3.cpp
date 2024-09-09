#include <iostream>
#include <fstream>
#include <vector>
#include <onnxruntime_cxx_api.h>
#include <nlohmann/json.hpp>  // Use a JSON library to load JSON data
#include <chrono>

// Function to load scaler parameters (mean and std deviation)
void load_scaler_params(std::vector<float>& mean, std::vector<float>& scale, const std::string& file_path) {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening scaler params file." << std::endl;
        return;
    }
    nlohmann::json scaler_json;
    file >> scaler_json;

    mean = scaler_json["mean"].get<std::vector<float>>();
    scale = scaler_json["scale"].get<std::vector<float>>();
}

// Function to normalize data using mean and std deviation
void normalize_data(std::vector<float>& data, const std::vector<float>& mean, const std::vector<float>& scale) {
    for (size_t i = 0; i < data.size(); ++i) {
        data[i] = (data[i] - mean[i]) / scale[i];
    }
}

// Function to load val_spectra from JSON
auto load_val_spectra(const std::string& file_path) -> std::vector<std::vector<float>> {
    std::ifstream file(file_path);
    if (!file.is_open()) {
        std::cerr << "Error opening val_spectra file." << std::endl;
        return {};
    }
    nlohmann::json spectra_json;
    file >> spectra_json;

    return spectra_json["val_spectra"].get<std::vector<std::vector<float>>>();
}

int main(int argc, char* argv[]) {
    // Initialize ONNX Runtime
    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "TestOnnxRuntime");

    // Set session options
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

    // Specify the model path
    const char* model_path = "model_quantized_static.onnx";

    // Create an inference session
    Ort::Session session(env, model_path, session_options);

    // Print out some information about the model
    Ort::AllocatorWithDefaultOptions allocator;

    // Get input node information
    size_t num_input_nodes = session.GetInputCount();
    std::vector<const char*> input_node_names(num_input_nodes);
    std::vector<Ort::AllocatedStringPtr> input_names;
    input_names.reserve(num_input_nodes);

    std::vector<int64_t> input_node_dims;

    std::cout << "Number of inputs: " << num_input_nodes << std::endl;

    for (size_t i = 0; i < num_input_nodes; i++) {
        // Get the input name
        input_names.emplace_back(session.GetInputNameAllocated(i, allocator));
        input_node_names[i] = input_names.back().get();

        std::cout << "Input " << i << " : name=" << input_node_names[i] << std::endl;

        // Get input node type and shape
        Ort::TypeInfo type_info = session.GetInputTypeInfo(i);
        auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

        ONNXTensorElementDataType type = tensor_info.GetElementType();
        std::cout << "Input " << i << " : type=" << type << std::endl;

        input_node_dims = tensor_info.GetShape();
        std::cout << "Input " << i << " : num_dims=" << input_node_dims.size() << std::endl;

        for (size_t j = 0; j < input_node_dims.size(); j++) {
            std::cout << "Input " << i << " : dim[" << j << "]=" << input_node_dims[j] << std::endl;
        }
    }

    // Load scaler parameters (mean and scale) from JSON
    std::vector<float> mean, scale;
    load_scaler_params(mean, scale, "scaler_params.json");

    // Load the real validation data from JSON
    std::vector<std::vector<float>> val_spectra = load_val_spectra("val_spectra.json");

    // Check that val_spectra data is valid
    if (val_spectra.empty()) {
        std::cerr << "Error: val_spectra data is empty." << std::endl;
        return 1;
    }

    // Loop through the first 10 validation samples and process them one by one
    for (int sample_idx = 0; sample_idx < 10; ++sample_idx) {
        std::cout << "Processing sample " << sample_idx + 1 << "..." << std::endl;

        // Select one sample at a time
        std::vector<float> input_tensor_values = val_spectra[sample_idx];

        // Normalize the real validation data using the scaler parameters
        normalize_data(input_tensor_values, mean, scale);

        // Prepare input tensor shape for a single sample
        std::vector<int64_t> input_shape = input_node_dims;  // Keep the original shape
        input_shape[0] = 1;  // Set batch size to 1

        // Create the input tensor with the single validation sample
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

        // Output node names (assuming one output node)
        const char* output_node_names[] = {"output"};

        // Perform inference on the single validation sample
        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names, 1);

        // Get the output tensor data
        float* output_data = output_tensors[0].GetTensorMutableData<float>();

        // Get the output tensor shape to determine the number of classes
        auto output_tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
        std::vector<int64_t> output_shape = output_tensor_info.GetShape();
        int num_classes = output_shape[1]; // Assuming output shape is [batch_size, num_classes]

        // Print predictions for the current sample
        std::cout << "Predictions (7 probability values for each class) for sample " << sample_idx + 1 << ": [ ";
        for (int j = 0; j < num_classes; ++j) {
            std::cout << output_data[j];
            if (j < num_classes - 1) std::cout << ", ";
        }
        std::cout << " ]" << std::endl;
    }

    return 0;
}


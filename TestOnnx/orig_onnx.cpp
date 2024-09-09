#include <iostream>
#include <chrono>
#include <vector>
#include <onnxruntime_cxx_api.h>

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

    // Initialize the vector with the number of elements and immediately construct the Ort::AllocatedStringPtr objects
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

    // Prepare input tensor data (matching input shape)
    std::vector<float> input_tensor_values(input_node_dims[1], 0.5f);  // Example data
    std::vector<int64_t> input_shape = input_node_dims;

    // Create the input tensor
    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(memory_info, input_tensor_values.data(), input_tensor_values.size(), input_shape.data(), input_shape.size());

    // Output node names (assuming one output node)
    const char* output_node_names[] = {"output"};

    // Run the model (use the same input_node_names vector)
    int N = 100;
    auto startTime = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < N; i++){
        auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names, 1);
    }
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(endTime - startTime);
    std::cout << "Inference time: " << duration.count() << std::endl; 
    auto output_tensors = session.Run(Ort::RunOptions{nullptr}, input_node_names.data(), &input_tensor, 1, output_node_names, 1);

    // Get and print the output tensor shape
    auto output_tensor_info = output_tensors[0].GetTensorTypeAndShapeInfo();
    std::vector<int64_t> output_shape = output_tensor_info.GetShape();

    std::cout << "Output tensor shape: [";
    for (size_t i = 0; i < output_shape.size(); i++) {
        std::cout << output_shape[i];
        if (i < output_shape.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;

    std::cout << "ONNX Runtime test completed successfully." << std::endl;

    return 0;
}

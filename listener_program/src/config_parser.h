#pragma once
#include "pch.h"
struct PipelineStep
{
    std::string type;
    nlohmann::json params;
};

struct FlexibleConfig
{
    nlohmann::json network;
    nlohmann::json global;
    std::vector<PipelineStep> pipelineSteps;
};

class FlexibleConfigParser
{
   public:
    static FlexibleConfig parse(const std::string& jsonFilePath);

   private:
    static nlohmann::json resolveVariables(const nlohmann::json& config, const nlohmann::json& context);
    static nlohmann::json resolveValue(const std::string& str, const nlohmann::json& context);
    static nlohmann::json getVariableValue(const std::string& varPath, const nlohmann::json& context);
};
#include "config_parser.h"

FlexibleConfig FlexibleConfigParser::parse(const std::string& jsonFilePath)
{
    std::ifstream inputFile(jsonFilePath);
    if (!inputFile.is_open())
    {
        throw std::runtime_error("Unable to open JSON file: " + jsonFilePath);
    }

    nlohmann::json jsonConfig;
    inputFile >> jsonConfig;

    FlexibleConfig config;

    // Parse network and global sections
    config.network = jsonConfig.at("network");
    config.global = jsonConfig.at("global");

    // Create context for variable resolution
    nlohmann::json context;
    context["network"] = config.network;
    context["global"] = config.global;

    // Parse pipeline steps
    for (const auto& stepJson : jsonConfig.at("pipeline").at("steps"))
    {
        PipelineStep step;
        step.type = stepJson.at("type").get<std::string>();

        // Resolve variables in params
        step.params = resolveVariables(stepJson.at("params"), context);

        config.pipelineSteps.push_back(step);
    }

    return config;
}

nlohmann::json FlexibleConfigParser::resolveVariables(const nlohmann::json& config, const nlohmann::json& context)
{
    nlohmann::json resolved = config;

    for (auto& [key, value] : resolved.items())
    {
        if (value.is_string())
        {
            std::string strValue = value.get<std::string>();
            resolved[key] = resolveValue(strValue, context);
        }
        else if (value.is_object())
        {
            resolved[key] = resolveVariables(value, context);
        }
        else if (value.is_array())
        {
            for (size_t i = 0; i < value.size(); ++i)
            {
                if (value[i].is_string())
                {
                    resolved[key][i] = resolveValue(value[i].get<std::string>(), context);
                }
                else if (value[i].is_object())
                {
                    resolved[key][i] = resolveVariables(value[i], context);
                }
            }
        }
    }

    return resolved;
}

nlohmann::json FlexibleConfigParser::resolveValue(const std::string& str, const nlohmann::json& context)
{
    std::regex varRegex(R"(\$\{([^}]+)\})");
    std::smatch match;

    // Check if the entire string is a single variable reference
    if (std::regex_match(str, match, varRegex))
    {
        std::string varPath = match[1].str();
        return getVariableValue(varPath, context);
    }

    // Handle string interpolation (mixed text and variables)
    std::string result = str;
    while (std::regex_search(result, match, varRegex))
    {
        std::string varPath = match[1].str();
        nlohmann::json varValue = getVariableValue(varPath, context);

        // Convert to string for interpolation
        std::string replacement;
        if (varValue.is_string())
        {
            replacement = varValue.get<std::string>();
        }
        else if (varValue.is_number())
        {
            replacement = std::to_string(varValue.get<double>());
        }
        else if (varValue.is_boolean())
        {
            replacement = varValue.get<bool>() ? "true" : "false";
        }

        result.replace(match.position(), match.length(), replacement);
    }

    return result;
}

nlohmann::json FlexibleConfigParser::getVariableValue(const std::string& varPath, const nlohmann::json& context)
{
    // Split path by dots and navigate through context
    nlohmann::json value = context;
    std::istringstream pathStream(varPath);
    std::string segment;

    while (std::getline(pathStream, segment, '.'))
    {
        if (value.contains(segment))
        {
            value = value[segment];
        }
        else
        {
            throw std::runtime_error("Variable not found: " + varPath);
        }
    }

    return value;
}
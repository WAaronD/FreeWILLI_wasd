#pragma once
#include "pch.h"
#include "pipeline_variables.h"
#include "socket_variables.h"

auto parseJsonConfig(const std::string& jsonFilePath) -> std::tuple<SocketVariables, PipelineVariables>;

void printMode();

std::string convertTimePointToString(const TimePoint& timePoint);

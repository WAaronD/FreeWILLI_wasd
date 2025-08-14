#pragma once
#include "pch.h"
#include "socket_variables.h"
#include "structs.h"

auto parseJsonConfig(const std::string& jsonFilePath) -> std::tuple<SocketVariables, PipelineVariables>;

void printMode();

std::string convertTimePointToString(const TimePoint& timePoint);

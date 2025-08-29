#pragma once
#include "pch.h"

class SharedDataManager;
class ISocketManager;

void runListenerLoop(SharedDataManager& sess, const std::unique_ptr<ISocketManager>& socketManager, bool verbose);
#pragma once
#include "pch.h"

class SharedDataManager;
class ISocketManager;

void runListenerLoop(SharedDataManager& sess, std::unique_ptr<ISocketManager>& socketManager);
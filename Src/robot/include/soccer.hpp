#pragma once

#include "redbackbots.hpp"

class VisionDebugModule;
// #include "perception/vision/VisionDebuggerInterface.hpp"

// External to denote if running in OffNao mode
extern bool vatNao;

// External to determine shutdown
extern bool attemptingShutdown;

extern VisionDebugModule *vdm;

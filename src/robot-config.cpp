#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain Brain;
digital_out matchloader = digital_out(Brain.ThreeWirePort.G);
digital_out outputblocker = digital_out(Brain.ThreeWirePort.F);
digital_out descorer = digital_out(Brain.ThreeWirePort.A);

// The motor constructor takes motors as (port, ratio, reversed), so for example
// motor LeftFront = motor(PORT1, ratio6_1, false);

// Add your devices below, and don't forget to do the same in robot-config.h:

void vexcodeInit(void)
{
  // nothing to initialize
}
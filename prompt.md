I'm developing a Renode simulation for the RECO board in this rocket avionics repository. So far, the
  C# peripherals for the magnetometer, barometer, and IMU have been completed and are in the sitl/
  peripherals directory. However, integration with the RECO source code at reco/ as well as simulation
  of flight interacting with RECO is not complete. Note that I do not want the actual flight software
  included in this simulation yet, I just want a barebones simulation of flight's interation with RECO.
  I also want real-time data streaming capabilities, using data collected from previous actual tests for
  pressure, magnetic fields, GPS, acceleration, rotation, etc. These will be provided in the RESD
  format, as Renode expects.

  To understand the hardware specification that RECO is running on, read docs/hardware/reco/rev5.md .
  Next, create a plan to use Renode to simulate RECO, taking in sensor data from RESD files with time-
  series data, passing it through the simulated peripheral sensors, and having the actual STM32
  microcontrollers being emulated with the real RECO source code running on them. Put this plan in
  WORKING.md , and as you work, check off implementation details. Clear the plan with me before
  starting.

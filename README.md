Automated Waypoint Navigation code for AeroQuad 32, written by Salem Karani (Project Peregrine lead engineer).
Available for all forms of reuse under MIT license. 
Have fun

Demo Video: https://www.youtube.com/watch?v=IerisuoUIgE

Note:

Look in AeroQuad folder for primary AutoNav code. Be sure to use the GUI to load waypoints before trying the software.
Primary contributions in: HeadingHoldProcessor, GpsNavigator, FlightCommandProcessor, FlightControlProcessor, etc.

Hardware: Baloo stm32 board, OpenLog for logging, cheap 50 cent buzzer and led lights + relay controlled via GPIO upon correct heading + reaching destination.

Key Ideas:

Correcting for heading and path error.

3 part algorithm:

0. Raise to desired altitude and turn on alt. hold
1. Hit the AutoNav Switch (see the main function)
  a. It will use magnetometer to change yaw to face first GPS waypoint
  b. Then will change pitch and move forward to desired waypoint
  c. Upon calculating that it reached the waypoint, will stop pitching and resume altitude hold

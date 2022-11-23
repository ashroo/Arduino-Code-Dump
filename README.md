# Arduino Code Dump
 A repository dedicated for dumping random Arduino Project I have been working on.

This project in no way are completely mine and mine alone. I have taken codes from internet and modified for my use after understanding the full code.
Project Currently include :-
1. Calibration Code for 2 loadcell attached at different pins and thus have 2 project folder of their own.
2. Code for controlling stepper motors using a Microstep driver and an Arduino Nano. I have tried using 2 methods and found using a library easy. So a code was written for controlling 2 motors and can also be found here. The angle is taken as input  and the Stepper motor rotates to that angle. The angle depends on where the Initial position of stepper is at the time it is powered on. Number of steps taken to achieve the angle depends on the microstep set in the Driver physically.
# CCP_Demo
This demo contains CAN Calibration Protocol embedded on GD32F303VCT6

The CCP is based on Vector.com open source code, with paticular embedded platform test and debug.
The A2ltool.exe is downloaded from github, https://github.com/DanielT/a2ltool.git

This project is builded by CMake.

Project contains FOC including Clarke, Park, inv_Park, 7-steps SVPWM, and Current loop, Speed loop using PID controller.

Rotor positon is obtained by Resolver via AD2S1210.
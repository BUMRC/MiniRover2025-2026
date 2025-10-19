# MiniRover2025-2026

🛰️ Mini Rover — Mars Rover Club

This repository is dedicated to the Mini Rover development and testing platform.
As the project progresses, we will use this README to:

MiniRover Team
	To prepare for integrating everything we develop in the real world we want to be able to test it on a minirover while the main rover is still being designed/manufactured. The main tasks for the minirover team are as follows: 
Write a PID controller using an esp32 and motors with an encoder(like the gobuilda motors we have in the lab) to be able to control the speed and direction of the motors. The esp32 should be able to communicate with a process running on the jetson, which will specify over serial or another communication protocol the target speed for the motors which the esp32s should be able to control. 
Try to make PID parameters configurable from the Jetson rather than needing to change the code on the esp32s. 
Small 3d printed rover that can mount either the big jetson or the small jetson, and supports some motors, either the gobuilda motors or other motors of your choice. 

Implement teleop control through ROS for the minirover. I would work with the teleop in simulation team for this part.  


✅ Document our goals 

👤 Ozan (Ozzy) , Quinlan (Quinn) , William

👥 Team & Roles
Name	Role / Focus Area	Status
TBD	Example: Motor Control	Not Started / In Progress / Done
TBD	Example: Communication Layer	Status
TBD	Example: Documentation & Repo Maintenance	Status

This table will be updated as roles are confirmed.

🎯 Goals & Milestones

PID Controller using an ep32 and motors with an econder.(Running on the Jetson)

PID parameters configurable from the Jetson rather than needing to change the code on the esp32s. 

Implement teleop control through ROS for the minirover


🗂 Repository Structure (To Be Defined)

As more modules are added, this section will document folder layout and conventions.

📝 Notes

Updates will be logged here as the project evolves.


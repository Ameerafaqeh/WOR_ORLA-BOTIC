WRO 2025 – Future Engineers: Self-Driving Car Project
1. Introduction
This repository documents our team’s entry for the WRO 2025 Future Engineers – Self-Driving Cars competition.
Our goal is to design, build, and program a fully autonomous vehicle that follows the official rules, completing both the Open Challenge (three laps on a randomized track) and the Obstacle Challenge (three laps plus avoiding randomized pillars and performing a parallel parking maneuver).
This document provides a detailed explanation of our hardware, software, wiring, control algorithms, and testing process. It is written with enough technical depth that another student team, anywhere in the world, could reproduce our work and understand the reasoning behind our design decisions.
________________________________________
2. Bill of Materials (BOM)
The following components were selected to satisfy the official competition rules (max 300×200 mm footprint, ≤ 1.5 kg weight, one driving axle, steering servo, and wired-only communication):
2.1 Chassis
•	Modified 1/18 scale RC car frame, lightweight rigid plastic (ABS/PLA) base plate.
2.2 Drive Motors
•	2× Brushed DC Motors, connected directly to the rear axle (RWD), providing direct power to the rear wheels for movement and basic control.
2.3 Steering & Wheels
•	Steering Servo: 1× standard digital servo, 60° range, mounted on front steering linkage.
•	Wheels: Four rubber wheels (Ø 65 mm) with high-friction tires for stable grip on the competition mats.
2.4 Controllers
•	Single-Board Computer (SBC): Raspberry Pi 4 Model B (4 GB RAM) – runs Linux and performs camera-based perception and AI processing.
•	Microcontroller (MCU): Arduino Uno R4 Minima – handles low-level motor control, wheel encoders, steering servo, and executes commands from Raspberry Pi.
2.5 Motor Driver
•	L298N H-Bridge driver module, providing current to the DC motors.
2.6 Sensors
•	1× USB wide-angle camera (primary perception sensor for color detection and AI analysis).
•	1× MPU6050 IMU for orientation and drift correction.
•	2× Ultrasonic sensors for short-range obstacle detection and parking assist.
•	2× Wheel encoders for odometry and lap counting.
2.7 Power
•	2S Li-Po battery (7.4 V, 2000 mAh) for motors.
•	USB power bank (5 V, 5000 mAh) for Raspberry Pi.
2.8 Miscellaneous
•	Breadboard, jumper wires, buck converter (12 V → 5 V), 3D-printed mounts for camera and sensors, physical Power switch (main battery), and Start button (GPIO input to trigger code start).
________________________________________
3. Mechanical Design
3.1 Dimensions & Weight
•	Dimensions: 280 × 180 × 160 mm (fits within 300 × 200 × 300 mm rule).
•	Weight: 1.35 kg with batteries.
3.2 Drive & Steering Systems
•	Drive system: RWD – 2× Brushed DC Motors connected directly to rear axle.
•	Steering system: Ackermann-style geometry with 1× standard digital servo on front steering linkage.
3.3 Chassis Modifications
•	Modified RC car frame with lightweight rigid plastic base plate.
•	3D-printed camera holder at a 30° downward angle for lane and color detection.
•	CAD files in /cad/.
Summary: The mechanical layout mimics a real car with one steering axle and one powered axle, avoiding forbidden differential drive.
________________________________________
4. Electronics & Wiring
4.1 Main Power
•	Li-Po battery powers DC motors through H-Bridge.
•	Buck converter provides 5 V regulated to Arduino.
4.2 Peripherals
•	Start Button: Connected to Raspberry Pi GPIO, rising-edge trigger to start code.
•	Camera: USB interface to Raspberry Pi.
•	IMU: I2C connection to Raspberry Pi.
•	Ultrasonic Sensors: Connected to Arduino digital pins; readings forwarded to Pi via UART.
•	Encoders: Connected to Arduino interrupt pins for precise speed measurement.
4.3 Communication
•	Pi ↔ Arduino: Serial UART at 115200 baud.
o	Pi sends [steering, speed] commands.
o	Arduino executes them via PWM.
Wiring diagram: /electronics/schematics/.
________________________________________
5. Software Architecture
5.1 High-Level Modules (Raspberry Pi)
•	Perception (Python + OpenCV):
o	Lane Detection: Color thresholding, Canny edge detection, Hough transform.
o	Pillar Detection: HSV segmentation distinguishes red (turn right) and green (turn left) pillars.
o	Parking Spot Recognition: Detects two parallel boundaries to estimate available parking space.
•	Planning:
o	Finite State Machine (FSM): States include Idle, Lap Running, Avoid Obstacle, Parking, Finished.
o	Lap Counter: Increments when vehicle passes start line, using encoder readings and visual markers.
5.2 Low-Level Control (Arduino)
•	Motor Control: DC motor speed via PWM.
•	Steering Control: Updates servo angle based on Pi commands.
•	Safety Stop: Halts vehicle if no command received for >1 second.
5.3 Interfaces
•	Pi → Arduino: Sends [speed, angle] via UART.
•	Arduino → Pi: Returns [encoder ticks, ultrasonic distances] at 20 Hz.
Summary: High-level modules handle perception and decision-making, low-level ensures precise execution and safety.

6. Perception & Control
•	Lane Following:
A Pure Pursuit controller is used for autonomous steering. The camera extracts a target waypoint 40 pixels ahead, and the steering angle is calculated as:
δ=arctan⁡(2L⋅sin⁡(α)d)\delta = \arctan \left(\frac{2L \cdot \sin(\alpha)}{d}\right)δ=arctan(d2L⋅sin(α)) 
where LLL = wheelbase, α\alphaα = heading error, ddd = lookahead distance.
•	Speed Control:
Base speed is ~0.5 m/s, regulated by a PID controller using RPM feedback from the wheel encoders. Speed is reduced near sharp turns or obstacles.
•	Obstacle Handling:
When a red or green pillar is detected, the FSM decides whether to steer left or right. Ultrasonic sensors confirm clearance before passing.
•	Parallel Parking:
Parking is performed in three steps:
1.	Drive forward alongside the parking gap while measuring distance.
2.	Reverse with maximum steering into the spot.
3.	Straighten and adjust until parallel.
Success criteria: vehicle fully inside the parking space, parallel to walls within ≤ 2 cm, without touching boundaries.


7. Testing & Results
•	Simulation: Initial vision pipeline tested in Gazebo + OpenCV synthetic images.
•	Real Track Tests: Practiced with randomized start positions and varying lane widths (600 mm / 1000 mm).
•	Performance Metrics:
o	Open Challenge: Best 3 laps in 1:47, no boundary violations.
o	Obstacle Challenge: Successfully avoided 3 randomized pillars and parked correctly in 5/6 attempts.
•	Reliability:
o	Our stop mechanism is robust (failsafe timeout on Arduino).
o	IMU fusion reduced drift by ~20%.
o	Parking success ~80% across tests.
 8. How to Reproduce
•	Hardware Setup:
1.	Print and assemble the chassis (CAD files in /cad/).
2.	Mount DC motor on the rear axle and servo on the front steering.
3.	Attach Raspberry Pi, camera, and Arduino to the chassis.
4.	Connect wiring according to /electronics/schematics/.
•	Software Setup:
1.	Flash the Arduino firmware (/firmware/arduino_drive.ino).
2.	On Raspberry Pi, install dependencies and run the main program:
3.	cd code/
4.	pip install -r requirements.txt
5.	python main.py
•	Result:
Following these steps should allow the system to operate as described in previous sections, including lane following, obstacle avoidance, and parallel parking.


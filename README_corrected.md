# 5-DOF Robotic Writing Arm

A robotic writing-arm project intended to convert typed text into pen movements and write it on paper. The project brings together mechanical CAD design, a custom KiCad PCB, robot kinematics in MATLAB, simulation in RoboDK, a machine-learning approach to inverse kinematics, Python text-to-motion software, and Arduino firmware.

> **Author:** Tejas  
> **Tools:** Fusion 360, KiCad, MATLAB (Peter Corke Robotics Toolbox), RoboDK, Python, Arduino IDE, and Tinkercad.

**Project demonstration:** [Watch the simulation video](docs/videos/stimulation_vidio_compressed.mp4)

![CAD model of the robotic arm](docs/images/cad_image.png)

> **Implementation note:** This README is written for a 5-DOF arm. The project notes supplied with it currently document only three DH rows, two servo signal pins, and a two-joint serial example. Those details must be completed from the actual CAD model, schematic, and Arduino code before the build can be described as a fully implemented 5-DOF system. The README does not invent the missing joint dimensions or pin assignments.

## Table of Contents

1. [Overview](#1-overview)
2. [System Architecture](#2-system-architecture)
3. [Repository Structure](#3-repository-structure)
4. [Hardware and Bill of Materials](#4-hardware-and-bill-of-materials)
5. [CAD Design](#5-cad-design)
6. [Electronics and KiCad PCB](#6-electronics-and-kicad-pcb)
7. [Kinematics in MATLAB](#7-kinematics-in-matlab)
8. [Simulation in RoboDK](#8-simulation-in-robodk)
9. [Machine-Learning Model](#9-machine-learning-model)
10. [Text to Motion (Python)](#10-text-to-motion-python)
11. [Arduino Firmware](#11-arduino-firmware)
12. [How to Run](#12-how-to-run)

---

## 1. Overview

### Goal

Enter a sentence and have the robotic arm reproduce it on paper.

### Project components

- A 5-DOF arm designed in CAD. The arm joints are described as servo-driven; the exact motor models and final joint count should be confirmed against the hardware.
- A custom PCB designed in KiCad for power regulation, actuator connections, and sensor inputs.
- A MATLAB kinematic model covering Denavit–Hartenberg (DH) parameters, forward kinematics, Jacobian calculations, velocity analysis, and singularity checks.
- A RoboDK station for checking the robot model, workspace, joint limits, and proposed writing movements.
- A machine-learning model based on data generated from the kinematic model. Support Vector Regression (SVR) is suitable when the model predicts continuous joint-angle values.
- Python software to convert text into pen strokes and send motion commands to the Arduino over USB serial.
- Arduino firmware to control the joint actuators and pen mechanism. The supplied project notes also describe an FSR-based pen-pressure feedback loop; confirm that this is implemented in the firmware before treating it as a completed feature.

## 2. System Architecture

```text
Typed sentence
      │
      ▼
Single-stroke font conversion
      │
      ▼
(x, y, pen) stroke points
      │
      ▼
Inverse kinematics / trained regression model
      │
      ▼
Joint targets (q1, q2, q3, q4, q5)
      │
      ▼
Python serial communication over USB
      │
      ▼
Arduino Nano
      ├── Arm joint actuators
      ├── Pen-lift stepper motor and driver
      └── FSR input for pen-pressure feedback (if implemented)
```

![Tinkercad circuit](docs/images/tinkercad_circuit.png)

The pen-lift mechanism is an additional actuator. It should not be counted as one of the arm's five positioning DOF unless the robot's kinematic model explicitly includes it as a robot joint.

## 3. Repository Structure

The structure below matches the folder names shown in the supplied project screenshot:

```text
robotic-arm-5dof/
├── Arduino_code/                 # Arduino firmware (.ino files)
├── cad/                          # Fusion 360 design files and CAD exports
├── docs/
│   ├── images/                   # Project screenshots and diagrams
│   │   ├── cad_image.png
│   │   ├── kicad_circuit.png
│   │   ├── kicad_pcb_layout.png
│   │   ├── kicad_pcb.png
│   │   ├── matlab_calculations.png
│   │   ├── mathlab_image.png
│   │   ├── robodk_stimulation_image.png
│   │   └── tinkercad_circuit.png
│   └── videos/
│       └── stimulation_vidio_compressed.mp4
├── matlab/                       # DH model and kinematics scripts
├── ml/                           # Dataset generation, model training, evaluation
├── pcb/                          # KiCad schematic, PCB layout, and manufacturing files
├── python/                       # Text-to-motion and serial communication scripts
├── tinkercad/                    # Tinkercad circuit designs
└── README.md
```

**Filename note:** The screenshot shows the filenames `mathlab_image.png`, `robodk_stimulation_image.png`, and `stimulation_vidio_compressed.mp4`. They are referenced exactly as they currently appear so the links work. For cleaner naming, consider renaming them to `matlab_image.png`, `robodk_simulation_image.png`, and `simulation_video_compressed.mp4`, respectively, and then updating the paths in this README.

## 4. Hardware and Bill of Materials

| Component | Quantity | Purpose / notes |
|---|---:|---|
| Arduino Nano | 1 | Main controller |
| Servo motors | To be confirmed | Arm-joint actuators; specify the actual quantity, model, torque, and operating voltage |
| 28BYJ-48 stepper motor | 1 | Pen-lift mechanism, as described in the project notes |
| ULN2003 driver board | 1 | Driver for the 28BYJ-48 stepper motor |
| LM2596S-ADJ buck converter | 2 | Separate regulated supplies are described for the Nano logic rail and actuator rail |
| Force-sensitive resistor (FSR) | 1 | Pen-contact / pressure sensing, connected to A0 in the supplied pin map |
| Schottky diode, inductors, capacitors, and resistors | As per schematic | Power protection and regulation; refer to the KiCad BOM for exact part numbers and ratings |
| DC power supply | 1 | The project notes specify a 10–30 V input; verify the rating of the actual converter module and all connected components |
| Screw terminals and connectors | As required | Power, motor, and sensor connections |

![KiCad schematic](docs/images/kicad_circuit.png)

Do not select a power supply or motor based only on this summary table. Check the motor stall current, regulator current and thermal limits, PCB trace widths, connector ratings, and capacitor voltage ratings against the actual hardware.

## 5. CAD Design

The mechanical design was created in **Fusion 360**.

### Design workflow

1. Define the required writing area and the arm's reachable workspace.
2. Model the links, joint brackets, and actuator mounts using the selected motor dimensions.
3. Design the pen holder and its lift mechanism.
4. Assemble the components and check joint travel, clearances, and collisions.
5. Export the parts as STL files for 3D printing and as STEP files for importing into RoboDK.

![CAD isometric view](docs/images/cad_image.png)

## 6. Electronics and KiCad PCB

### 6.1 Circuit description

The supplied circuit notes describe the following arrangement:

- **Power input:** A 10–30 V input, a Schottky protection diode, and a 470 µF input capacitor.
- **Buck converters:** Two LM2596S-ADJ regulators configured for approximately 5 V. The notes specify a 3 kΩ / 1 kΩ feedback divider; verify the resistor positions and measure the output voltage before connecting the loads.
- **Logic supply:** One regulated rail supplies the Arduino Nano. Apply regulated 5 V to the Nano's 5 V pin only after confirming the voltage and polarity.
- **Actuator supply:** The second regulated rail supplies the servos and stepper-driver circuit. Do not power the servos from the Nano's 5 V output.
- **Servo connectors:** Three connections per servo: supply, signal, and ground. A bulk capacitor is shown on the actuator rail.
- **Stepper driver:** A ULN2003 drives the 28BYJ-48. The motor's common connection goes to the motor supply as required by the motor and driver wiring.
- **Force sensor:** The FSR and a 10 kΩ resistor form a voltage divider. The analogue node connects to A0. The intended arrangement in the notes is 5 V → FSR → A0 → 10 kΩ → GND.
- **Grounding:** The Nano, actuator supply, and ULN2003 require a common reference ground. Keep high actuator currents out of narrow logic-ground paths.

### Arduino signal map currently documented

| Signal | Arduino Nano pin |
|---|---:|
| Servo 1 signal | D3 |
| Servo 2 signal | D5 |
| Servo 3–5 signals | Not specified in the supplied notes; add the actual pins from the schematic and firmware |
| ULN2003 IN1–IN4 | D6–D9 |
| FSR analogue output | A0 |

> **Important:** The pin map above documents only two servo outputs. A 5-DOF servo-driven arm needs a defined signal connection and firmware control for every arm joint. Do not assume the remaining pins; record the actual assignments from the KiCad schematic and Arduino sketch.

### 6.2 PCB layout

![KiCad PCB layout](docs/images/kicad_pcb_layout.png)

![KiCad PCB](docs/images/kicad_pcb.png)

### PCB design checks

- Keep the buck converter's high-current switching loop (input capacitor, regulator, inductor, and diode) short.
- Size power traces and connectors for the expected continuous and peak actuator current.
- Keep motor-current return paths away from sensitive analogue sensor paths where practical.
- Use a common ground reference for the Nano, actuator supply, and ULN2003.
- Place 100 nF decoupling capacitors close to the Nano's supply pins.
- Add suitable bulk capacitance near the servo connectors, and verify its voltage and ripple-current ratings.
- Check clearances, copper widths, thermal performance, polarity, and connector pinouts before fabrication.

---

## 7. Kinematics in MATLAB

The kinematic calculations are intended to use **Peter Corke's Robotics Toolbox for MATLAB**.

### 7.1 Denavit–Hartenberg parameters

The project notes currently contain the following three DH rows:

| Joint | θ | d | a | α |
|---|---|---:|---:|---:|
| 1 | θ1 | 30 | 0 | 90° |
| 2 | θ2 | 0 | 30 | 0° |
| 3 | θ3 | 0 | 10 | 0° |

The length units are not stated in the supplied notes. The values above are reproduced as provided; confirm the units and coordinate-frame convention against the CAD model. **Rows for joints 4 and 5 are missing.** Add the correct values before using this as the complete 5-DOF model or importing it into RoboDK.

### 7.2 Standard DH transformation matrix

For a standard DH convention, the homogeneous transformation for a link is:

```text
       | cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)   a cos(θ) |
A_i =  | sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)   a sin(θ) |
       |   0          sin(α)         cos(α)          d    |
       |   0            0              0             1    |
```

Here, `θ` is the joint angle, `d` is the link offset, `a` is the link length, and `α` is the link twist.

![MATLAB transformation matrix calculation](docs/images/matlab_calculations.png)

### 7.3 Plotting and inspecting the arm pose

After creating the robot model `KR` and defining a joint configuration `q`, the model can be visualised with:

```matlab
KR.plot(q);
KR.teach(q);
```

![MATLAB robotic-arm plot](docs/images/mathlab_image.png)

### 7.4 Jacobian and end-effector velocity

```matlab
J = jacob0(KR, q);       % 6-by-N geometric Jacobian
qd = [...].';            % N joint velocities in rad/s; provide one value per joint
V = J * qd;              % [vx; vy; vz; wx; wy; wz]
```

For a 5-DOF model, `q` and `qd` must each contain five joint values, and the model must contain five corresponding joint definitions. The linear velocity components are typically in length-units/second, while the angular velocity components are in radians/second.

### 7.5 Singularity analysis

A non-square Jacobian does not have an ordinary determinant. Use rank and singular values to assess the full Jacobian, and use a suitable Gram matrix for manipulability measures.

```matlab
Jv = J(1:3, :);                  % Translational part of the Jacobian
w_pos = sqrt(det(Jv * Jv.'));    % Position manipulability measure
r = rank(J);                     % Compare with the number of independent joints
s = svd(J);                      % A small minimum singular value indicates a near-singular pose
```

For a 5-DOF robot, `J` is normally 6-by-5. Check whether `rank(J)` is below the model's maximum possible column rank, and monitor the smallest singular value. Symbolic analysis should use appropriate square minors or a rank condition; do not attempt to calculate `det(J)` directly for a 6-by-5 Jacobian.

![MATLAB calculations](docs/images/matlab_calculations.png)

## 8. Simulation in RoboDK

RoboDK can be used to inspect the robot's reachability, joint limits, and possible collisions before testing the physical arm.

### Suggested workflow

1. Export the CAD links from Fusion 360 as STEP or STL files and import them into RoboDK.
2. Build the robot mechanism using the **complete** joint sequence and the same coordinate-frame convention as the MATLAB model.
3. Enter the correct joint limits and define the tool frame at the pen tip.
4. Create a reference frame for the paper and define targets for the writing strokes.
5. Run the motion program and check for unreachable targets, joint-limit violations, and collisions.

The supplied project screenshot shows the following RoboDK image filename:

![RoboDK simulation](docs/images/robodk_stimulation_image.png)

> The supplied DH table has only three rows. Complete and validate the five-joint model before claiming that the RoboDK station uses the same full 5-DOF kinematics as MATLAB.

## 9. Machine-Learning Model

### 9.1 Dataset generation

The intended dataset is generated from the forward-kinematics model. For each valid joint configuration, calculate the corresponding end-effector pose and save the input-output pair to a dataset file.

For an inverse-kinematics predictor, define the feature columns and target columns explicitly. For example, end-effector position (and orientation, if required) can be used as input features, while the joint angles are the regression targets. Keep the units consistent throughout the dataset.

### 9.2 Support Vector Regression (SVR)

The input and target values are continuous numerical quantities, so **Support Vector Regression (SVR)** is a regression approach; an SVM classifier is not the appropriate model for predicting continuous joint angles.

Before using the trained model on the real arm:

- Split the dataset into training and test sets.
- Scale features using parameters fitted only on the training set.
- Evaluate prediction error for every joint, not just the overall score.
- Test predictions against joint limits and reachable workspace constraints.
- Account for the fact that one end-effector position can sometimes correspond to more than one valid joint configuration. Keep the dataset consistent with the intended inverse-kinematics solution branch, or use a method that handles multiple solutions.

## 10. Text to Motion (Python)

The Python application is intended to run on a computer, such as a laptop using VS Code. It converts typed text into pen strokes and sends the corresponding motion commands to the Arduino.

### Processing workflow

1. Read a sentence from the terminal and wrap it into lines that fit the paper.
2. Convert each character into single-stroke line segments using a Hershey font.
3. Scale the strokes to the paper dimensions and interpolate them into short movement segments. The supplied notes specify a step spacing of 0.2 cm.
4. Convert each `(x, y)` point into a valid joint configuration using inverse kinematics or the trained regression model.
5. Send the joint targets and pen state over USB serial, then wait for the acknowledgement from the Arduino.

### Serial protocol

For a 5-DOF arm, the data format must carry five joint values and the pen state. A comma-separated example is:

```text
q1,q2,q3,q4,q5,pen
41.3,62.0,15.0,90.0,0.0,1
```

- `pen = 0`: pen up.
- `pen = 1`: pen down.

The numeric values above are only an example of message formatting, not calibrated angles for the actual robot. Ensure that the Python sender and Arduino parser use exactly the same number of fields, field order, units, and line ending. The two-angle format `q1,q2,pen` in the original notes is not a complete command for a 5-DOF arm.

## 11. Arduino Firmware

The Arduino Nano receives motion commands over serial and controls the connected actuators.

### Intended firmware features

- Smooth joint movement, with a configurable maximum change per control loop.
- Stepper-driven pen lift with a defined travel limit.
- FSR reading on A0 for detecting pen contact or estimating pen pressure.
- Optional feedback control to adjust pen height based on the measured force. The PID feature should only be listed as implemented after it is present and tested in the Arduino sketch.
- A calibration command (the supplied notes specify `F`) to print the current force-sensor reading to the Serial Monitor.

### Tunable parameters

| Setting | Meaning |
|---|---|
| `FORCE_CONTACT` | Sensor reading used to detect pen contact |
| `FORCE_TARGET` | Target sensor reading during writing |
| `Kp`, `Ki`, `Kd` | PID controller gains, if PID control is implemented |
| `MAX_PEN_STEPS` | Safety limit for pen-lift travel |
| `PEN_DIR` | Pen-lift direction (`1` or `-1`) |

The firmware must parse the same serial format generated by the Python program. For a 5-DOF implementation, it must also command all five arm joints, not just two servos.

## 12. How to Run

### 12.1 Requirements

- Arduino IDE.
- Arduino libraries used by the sketch, such as `Servo` and `Stepper` (install only those actually included by the firmware).
- Python 3.10 or newer.
- Python packages used by the scripts: Hershey font package, `pyserial`, `numpy`, and `scikit-learn`.
- MATLAB with Peter Corke's Robotics Toolbox, for the kinematic model.
- RoboDK, for robot simulation.

Install the Python packages from a terminal:

```bash
python -m pip install Hershey-Fonts pyserial numpy scikit-learn
```

If the Hershey font package is not available under that package name in your Python environment, use the installation name documented by the library you actually import in your script.

### 12.2 Hardware and software steps

1. **Assemble the hardware.** Wire the board according to the final schematic. Power the servos and stepper from the properly rated actuator supply, not from the Nano's USB 5 V output. Connect grounds as required by the circuit design.
2. **Upload the firmware.** Open the `.ino` file in `Arduino_code/` in the Arduino IDE, select the Arduino Nano board and the correct serial port, then upload the sketch.
3. **Close the Serial Monitor.** The Python program needs exclusive access to the same serial port.
4. **Find the serial port.** Run this Python snippet:

   ```python
   import serial.tools.list_ports

   for port in serial.tools.list_ports.comports():
       print(port.device, "-", port.description)
   ```

5. **Set the port.** Enter the correct port name in the Python script (for example, `COM5` on Windows or `/dev/ttyUSB0` on Linux; use the actual port detected on your computer).
6. **Run the text-to-motion script** from the repository root:

   ```bash
   python python/write.py
   ```

7. **Enter a test sentence.** Start with a short sentence and verify the pen-up, pen-down, joint directions, workspace limits, and emergency stop procedure before attempting a full page.

---

## Items to complete before calling the project fully 5-DOF

- Add the missing fourth and fifth DH rows and confirm the units and frame convention.
- Confirm the number and model of the arm's servos, and document their torque and voltage requirements.
- Add signal-pin assignments for every arm joint to the schematic, pin map, and firmware.
- Update the Python-to-Arduino serial protocol and parser so they agree on all five joint angles plus the pen state.
- Confirm that the RoboDK robot contains five arm joints and uses the same kinematic model as MATLAB.
- Verify whether the FSR-based PID loop is implemented and tested; otherwise describe it as planned work rather than a completed feature.
- Confirm the actual filename of the Python entry-point script (`python/write.py`) and the actual Arduino sketch filename before following the run instructions.

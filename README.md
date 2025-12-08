# PID Controlled Ping Pong Ball Levitation

## Introduction
  This experiment of the Ping Pong Levitator aims to demonstrate how a PID (Proportional, Integral, and Derivative) controller works and how each of the variables affects the height of the ping pong ball. Using control systems to balance the input with the actual versus the desired output, the creation and use of the levitator serves as a physical display of how these controllers can be utilized in almost every field of engineering. 

## Control Systems

  <img width="485" height="499" alt="image10" src="https://github.com/user-attachments/assets/8f04db30-47b2-401d-a286-e2f66904d382" />
  
  _Figure 1: Diagrams and example circuits of an open-loop and closed-loop control system. All control systems, regardless of type, have the input, controller, actuator, process, and output._

  
  Control systems work to regulate the behavior of a device or process through an input and its corresponding desired output. As can be seen in Figure 1, these systems can be open or closed-looped, but both have the basic components: the input, the controller and the signal it has altered, the plant (also known as the actuator and process combined), and the output from the signal. The controller is the mechanism that dictates the output of the system will look like, using certain criteria to achieve the desired output. The plant is composed of two parts often combined into one: the actuator and the process. The actuator is the device in the system that alters or adjusts the environment in response to the signal from the controller. The process is the component of the system that is ultimately controlled or changed in such a way as to produce an output in a desired form. An example of this can be seen in Figure 1. Where the open- and closed-loop systems differ is that closed-loop systems have a sensor element that measures the actual output and compares it to the desired output, allowing the system to then make adjustments to more accurately achieve the desired output; this is the basis of how the levitator will work.


<img width="461" height="397" alt="image6" src="https://github.com/user-attachments/assets/0156d513-a595-4a3f-8a01-977dfa4559e2" />

_Figure 2: The Flow/Block Diagram of the ping pong ball levitator. The interaction between all of the components, along with what information they transmit to each other, is here as well._

 With the levitator, the system follows the same format as can be seen in Figure 2: the knob of the device is a potentiometer that serves as a voltage divider, determining the desired height (the input), which is then given to the Arduino (the controller).  The Arduino then takes the voltage, uses user-defined criteria to dictate how the Arduino converts the information into a PWM value, and outputs that value to a motor driver that spins the motor of a fan (the actuator). The force generated from the fan then moves the ball in the acrylic tube with an ultrasound sensor (sensor) on top. The sensor finally sends the height (actual output) of the ball back to the Arduino. It compares the selected height (the desired output) from the potentiometer, which then affects the calculations and output of the Arduino, thereby closing the loop. This closed-loop system that the levitator utilizes not only transforms the input signal of one form to be transformed into another form, but also allows for the actual output to affect and to guide, in a sense, the output closer to the desired value.

## PID Controller

<img width="389" height="379" alt="image7" src="https://github.com/user-attachments/assets/c59ce5ee-cf6b-4f16-8037-c22c9a54751c" />

_Figure 2: The output of a control system controller and the types of variables that can affect the output. T<sub>r</sub> is the rise time, or how fast the output reaches the desired level. The overshoot is how much over the desired level the output reaches. Peak time, or T<sub>p</sub>, is how long until the overshoot value returns down to around 1% of the desired level, and the e<sub>ss</sub> is the steady-state error, or how far off the output is from the desired level._



  A PID controller computes a control signal based on three components: proportional, integral, and derivative control. Each of these components work to change the characteristics of the output, as shown in Figure 2. The proportional term produces an output value that is proportional to the current error value. This causes an overall decrease in the rise time and steady-state error, but increases the initial overshoot of the output. The integral term works by staying proportional to both the magnitude and duration of the error. This helps to zero out the steady-state error and lead the output to be a smaller range of values, but also increases the overshoot value. The derivative term responds to the rate of change of the error and multiplies it by an accompanying coefficient, allowing the system to predict its behavior accordingly.  This can decrease the overshoot and stabilize the system, but can potentially amplify measurement in the noise. The components of this controller can be expressed in both the time domain [Eq. 1] and Laplace domain [Eq. 2]. As stated above, the Arduino will serve as the controller, working to balance all of the coefficients as it takes in the desired height and works to move the ball with the characteristics that it is given.

## Hardware
### Components
- 1 x 25k Ω Potentiometer
- 1 x 220 Ω Resistor
- 1 x 12 V DC Fan
- 1 x Acrylic Tube
- 1 x HCSR04 Ultrasonic Sensor
- 1 x Arduino Uno
- 1 x Ping Pong Ball
- 1 x Tiny Nut
- Tape
- 1 x DRV 8871 Motor Driver
- 1 x Nail
- 1 x Kastar 12V 6A Power Supply

  ![image8](https://github.com/user-attachments/assets/012a74a7-59f7-45f7-810a-7084d5a7dbe4)

_Figure 3: An image of the electronics of the levitator. This image shows most of the electronics attached to the wooden stand, and the nail inside the acrylic tube holding the ball._

Using a prebuilt wooden stand, the base of the acrylic tube is attached to the fan. A nail is put through the tube as the platform to hold the ping pong ball above the fan without touching it, as seen in Figure 3. The ping pong ball, with the small nut attached using tape, is dropped inside. The tube is then covered with the ultrasonic sensor on the top and held in place with tape. At the base of the stand, the potentiometer and the resistor are connected as a voltage divider between 5V and GND. The output voltage from the voltage divider is then connected to the analog input pin 5 in the Arduino. The motor driver’s In1,  In2, and GND pins are connected to the digital pins 10 and 11 and the GND pin of the Arduino, with the driver power being connected to the power supply and the motor power being attached to the power and ground of the fan. The ultrasonic sensor’s power is connected to the 5V and GND of the Arduino. The echo pin is connected to the digital input pin 3, and the trigger pin is connected to the digital input pin 2 on the Arduino.


## Goal

The goal of this project is to use PID control to regulate the target height and maintain a ping-pong ball at a target height within a vertical wind tunnel. By managing the coefficients of different aspects of the controller, the ball should be able to reach its intended height quickly and without too much error

## Methodology

The system operates as a closed-loop feedback system. The ultrasonic sensor continuously measures the height of the ping pong ball. The Arduino compares this measured height with the desired setpoint. Based on the resulting error, the PID controller computes a corrective output. A PWM signal is then sent to the motor driver, which adjusts the fan speed. The change in fan speed moves the ball toward the setpoint, and this process repeats continuously to maintain a stable output.

## Software Explanation

  PID Controller V4 implements a PID-based height control system. An ultrasonic sensor measures the position of an object inside a vertical tube, while a PWM-driven DC fan acts as the actuator to regulate height. The desired height setpoint is defined by a potentiometer and scaled into inches. Ultrasonic sensor time of flight data is converted into distance and then into object height inside the tube. The measured height is used directly, without any digital low-pass filtering, as the input to the PID controller. The PID controller compares the raw height measurement with the setpoint and generates a control signal that is constrained to a safe PWM range and inverted to match the fan’s physical behavior. System time, measured height, PID output, and setpoint are continuously logged through the Serial interface for monitoring and analysis.

  PID Controller V5 implements a PID-based height control system using an ultrasonic sensor and a PWM-driven fan. A potentiometer defines the desired height setpoint, while the ultrasonic sensor measures the object’s position inside a vertical tube. Unlike V4, this version applies a low-pass filter to the height measurement before it is used by the PID controller. The filter reduces high-frequency noise and measurement fluctuations inherent to ultrasonic sensing, which can otherwise cause unstable control behavior and excessive fan oscillations. By filtering the measured height, the PID controller primarily responds to actual physical changes rather than sensor noise. The resulting control signal is smoother, more stable, and easier to tune. The PID output is limited, inverted, and applied as a PWM signal to the fan. System time, measured height, PID output, and setpoint are continuously logged over the Serial interface for monitoring and analysis.

## Results

![image9-ezgif com-resize](https://github.com/user-attachments/assets/75edd347-f2f3-47f1-adec-f2e8bb1b7d37)


_Figure 4: A GIF of the incomplete and unoptimized PID controller. While the rise time is pretty low, the overshoot is extreme, and the settling time is too high to even work._

Figure 4 is the result of manually tuning the K<sub>P</sub>, K<sub>I</sub>, and the K<sub>D</sub> values and still encountering some errors within the system. The theory of manually tuning a PID controller is to set everything to zero, and adjust K<sub>P</sub> until it oscillates around the desired level. Half that K<sub>P</sub> values and increase K<sub>I</sub> until the offset is corrected, and increase K<sub>D</sub> until the output reaches the desired value. That value was previously achieved with the values K<sub>P</sub> = 50, K<sub>I</sub> = 3, and K<sub>D</sub> = 10, but system errors and inaccuracies caused those values to not only no longer work, but to be an example of a poorly optimized PID controller.

![image11-ezgif com-resize](https://github.com/user-attachments/assets/002b208c-c737-4310-9778-2a3293591285)

_Figure 5: A GIF of the optimized PID controller with an added low-pass filter. While the system didn't use frequency in any capacity, the low-pass filter was able to not only take care of the steady-state errors from the system, but also helped in making a better controller overall._


## Conclusion

  This project demonstrates the successful application of PID control to regulate the height of a ping pong ball using a closed-loop system. An ultrasonic sensor provided position feedback, while an Arduino-based PID controller adjusted fan speed through PWM control to maintain the desired height. Manual tuning showed that increasing proportional gain improved response speed but increased oscillation, integral control reduced steady-state error, and derivative control improved stability by damping oscillations. Overall, the system achieved stable levitation, proving that PID control is effective in this real-world application.


## Appendix
# Equations
1. $controller(t) = u(t) = K_p e(t) + K_i\int e(t) \mathrm{d}t + K_d\frac{\mathrm{d}e(t)}{\mathrm{d}t}$
2. $Controller(s) = U(s) = K_p + \frac{K_i}{s} + K_ds$





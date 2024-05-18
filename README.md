# Two-Wheeled-Robot-Stability

**BACKGROUND**

A. Inverted Pendulum:

Balancing an inverted pendulum is crucial for stability, control, and efficiency in systems like self-balancing robots. Stability prevents toppling, ensuring consistent performance. Advanced control systems use sensor data to detect deviations and apply precise corrections, optimizing energy use. Efficient balance extends battery life, enhancing functionality and safety by reducing collision risks. Balanced systems minimize accidents, enhancing operational safety and reliability. Managing balance in inverted pendulum systems is essential for stable, controlled, and safe operation in dynamic environments, where precise control and efficient resource use are critical for success.

B. Controller logic:

In self-balancing robots, PID controllers offer simplicity and effectiveness in maintaining stability by adjusting motor speeds in response to deviations from the upright position. However, PID control may not achieve optimal performance due to its limitations in handling complex dynamics. LQR controllers, on the other hand, excel in optimizing control actions for Multiple Input, Multiple Output (MIMO) systems like self-balancing robots, offering superior stability and performance. Using an MPU (Microprocessor Unit) over light sensors enhances control precision and responsiveness. MPU provides real-time data on the robot's orientation, enabling more accurate adjustments to maintain balance compared to light sensors, which may be less reliable in varying lighting conditions. Utilizing these controller logics is crucial for achieving stable and precise control in self-balancing robots, ensuring safe and efficient operation in dynamic environments. 

C. Real-life products:

Self-balancing robots are prominent in real-life applications like Segway personal transporters, enhancing urban mobility. They're also utilized in warehouse logistics for automated material handling, increasing efficiency. Additionally, they find use in educational platforms and research labs, fostering innovation in robotics and control systems.

**METHODS**

A. Nomenclature

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/39f10575-8d1b-4c7a-bc91-166ea2fe894e)

Fig.1 Symbols and Parameters Used 

B. Sensor Readings

In Simulink, our model incorporates Multibody modeling to simulate the dynamic behaviors of mechanical systems comprising rigid and/or flexible bodies connected by joints. Prismatic joints enable movement along the Z-axis, while Revolute joints facilitate rotation. This approach allows for a highly realistic representation of real-world conditions, including gravity and adjustable friction damping. To capture the system's dynamics, we utilize Transfer sensors between the joints. Specifically, we employ a Transfer sensor as a 3-axis Accelerometer to measure linear acceleration and another as a 3-axis Gyroscope to measure angular rate, providing essential sensor values for our simulation.

C. PID

The Proportional-Integral-Derivative (PID) control loop is a continuous modulated control system. As shown in Fig 3, it calculates the error e(t) as the difference between the set point (SP = r(t)) and the measured process variable (PV = y(t)), then adjusts the output u(t) based on PID coefficients (Kp, Ki, Kd) to minimize e(t). By tuning these coefficients, the controller's stability, steady-state error, and oscillations can be adjusted according to specific requirements. Increasing the proportional term (P) in a PID controller can improve stability by reducing the settling time and overshoot. Increasing the integral term (I) reduces steady-state error and helps the system reach the setpoint more accurately. Increasing the derivative term (D) reduces oscillations and improves the system's response to changes. Here  input for the loop in angle(rad) from MPU, the output from the loop is actuator torque which is given to the model motors.

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/dda29c3b-391b-46c3-bd93-8d7595070bdd)

Fig.2 Symbols and Parameters Used 

D. State-Space Modelling

Dynamic systems can be represented as differential equations. From below Equations internal states are connected through matrix A, which defines how the states change over time. Matrix B describes how external inputs affect these states. Matrix C determines how the states are combined to produce the system's output. Matrix D allows inputs to bypass the system and directly influence the output, providing a feedforward path. This representation in state space equations is crucial for understanding and analyzing the behavior of dynamic systems like the inverted pendulum.

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/dfd229ff-e3e8-458a-b39b-817f2e770d8d)

E. LQR

The LQR (Linear Quadratic Regulator) controller computes optimal K values using Q and R parameters. Q = C'*C (reduce for aggressive control) and R = 1 (reduce for slower response). Optimal values are found through trial and error. Linear acceleration and angular rate from MPU are transformed into displacement, velocity, angle, and angular rate, then sent to the LQR block for K multiplication. Gain K is calculated separately using the MATLAB function K = lqr(A, B, Q, R) with the state space model and system parameters. The eigenvalues of Matrix A determine system poles, which are initially positive indicating instability. After controller application, marginal poles suggest marginal stability.

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/e90bfa16-4a25-45da-8539-5dc79820f11a)

Table.1 System Parameters

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/24cdeb37-5760-4daf-8a3f-b18e496999d1)
![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/7432d451-889b-4629-a633-10c206fba71b)

       
(a) LQR model from Simulink (b) LQR controller loop
Fig.3 LQR block

**RESULTS**

The performance of the mobile inverted pendulum robot under different weights on top +0,+0.5kg,+0.9kg as the center of gravity shifts up as seen in the fig 4(b,c,d), and controller types were evaluated through simulations in MATLAB. 

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/3fa96e70-2004-4aa3-9f2f-6a8220cd4674)
(a) No controller  

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/fc5cde8a-b5ab-4c42-ab6f-f458753e5399)
(b) 0 Kg Load	           

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/7aeb674d-df56-48d6-83b2-88c731c368e3)
(c) 0.5 Kg	                     

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/6a4c9619-a3fb-418d-ba7a-772763da2b05)
(d) 0.9 Kg

Fig.4 Cart with no controller (a) and cart with weights (b,c,d) indicating the center of gravity

A. LQR Controller Performance

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/d93e52ef-5fbd-4d75-b2ee-38851a72a2d7)

(a) 0 Kg Load				

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/f5019f23-419a-4589-805a-266eb0c7b655)

(b) 0.5 Kg			

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/7223ae29-fcc0-4a16-a6ba-47210781ed77)

(c) 0.9 Kg

Fig.5 The LQR controller’s Cart displacement and pendulum angle component graph

Figure 5(a) depicts the robot's performance without load using an LQR controller. The pendulum angle (orange curve) and the cart's position (blue curve) indicate rapid stabilization. The robot achieves balance within a second, demonstrating minimal oscillation. The overshoot, rise time, and settling time are all within the desired specifications. In Figure 5(b), the robot is loaded with 0.5 kg. The stabilization process under this condition shows fewer oscillations but a delayed response compared to the no-load scenario. The shift in the center of gravity affects the robot's dynamics, leading to a steady-state error and deviations in rise and settle times from the no-load condition. Figure 5(c) illustrates the scenario with a 0.9 kg load. Here, the robot struggles to stabilize, exhibiting significant oscillations that decrease only gradually over time. This plot indicates unsatisfactory performance, with necessary improvements in settling and rise times for better stability.

B. PID Controller Performance

Figure 6(a) shows the robot's response to no load using a PID controller. The stabilization process takes approximately 5 seconds, with the oscillation curve flattening over time. The metrics of overshoot, rise, and settling times are maintained consistently across tests. Figure 6(b) presents results under a 0.5 kg load with the PID controller. Similar to the LQR scenario, the added mass induces a shift in the center of gravity, influencing the stabilization time and oscillation behavior, albeit with fewer oscillations compared to the LQR controller under similar conditions. Finally, Figure 6(c) examines the robot loaded with 0.9 kg under PID control. The stabilization is delayed, with noticeable oscillations that diminish slowly. The shift in the center of gravity from the no-load and 0.5 kg conditions is evident, affecting overall stability.

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/86374574-f75e-4be8-8a8d-36a2286a5bd4)

(a) 0 Kg Load	

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/1072666d-02f2-4d5f-bf9f-548ab2278aef)

(b) 0.5 Kg	

![image](https://github.com/annadurai-ka/Two-Wheeled-Robot-Stability/assets/156352281/e90f22d1-fa70-415d-92f6-cdc18d35c2cf)

(c) 0.9 Kg
Fig.6 The PID controller’s Cart displacement and pendulum angle component graph

C. Comparative Analysis and Material Considerations

The comparative analysis of LQR and PID controllers under varying load conditions shows that heavier loads significantly affect the robot's stabilization time, leading to increased instability. This insight points out how important it is to choose strong and lightweight materials for the body of the robot to minimize its impact on stability. Furthermore, optimizing the robot's height relative to its center of mass is also very important. While a taller robot would be simpler to balance, a lower center of mass indicates that the wheels can stay below it and keep the robot balanced.

**CONCLUSION**

In conclusion, the work aims to emphasize the importance of selecting appropriate control algorithms and sensor inputs for stabilizing inverted pendulum systems. The simulations demonstrate that heavier loads pose challenges to the system's stability, resulting in longer settling times and increased oscillations. Along with this, the choice of materials for the pendulum is crucial, with lightweight yet strong materials being preferred to optimize stability and minimize settling time. Furthermore, the design of the robot's height and center of mass plays a significant role in achieving balance, necessitating careful consideration during the design phase. Overall, the findings provide valuable insights for designing and optimizing mobile inverted pendulum robots, highlighting the importance of control algorithm selection and system design considerations for achieving stable and efficient operation.

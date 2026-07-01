# Air-Levitation Control System

A complete project showing the process of modeling, simulating, and deploying a feedback control system. The objective is to control the vertical position of a levitating ball within a transparent tube by dynamically regulating the fan's airflow.

---

##  Workflow
1. **Dynamic Modeling:** Used MATLAB for system identification to capture the non-linear aerodynamics of the ball and fan plant.
2. **Simulation & Tuning:** Designed a closed-loop PID controller within Simulink, tuning it to optimize stability, overshoot, and settling time.
3. **Hardware Deployment:** Implemented the tuned controller into an Arduino Uno, reading real-time position data and outputting calculated PWM signals.

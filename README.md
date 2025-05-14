# ⚙️ PID Servo Control for Linear Actuator

This project implements **PID control** for position and velocity regulation of a **DC motor linear actuator** using both **cascaded control** (position + velocity) and standalone **position control**. The system was designed, modeled, and tuned using MATLAB tools for accurate real-world simulation and control.

---

## 🚀 Overview

- **Actuator Specs**:  
  - Max Load: **10 kg**  
  - Max Speed: **65 cm/sec**

- **Control Strategies**:
  - **Position-only PID**
  - **Cascaded PID** (Velocity inner loop + Position outer loop)

- **Goals**:
  - Minimal steady-state error  
  - Fast and stable transient response  
  - Realistic hardware behavior in simulation  

---

## 🛠️ Implementation Details

- **Motor Modeling**:
  - Motor parameters estimated using **MATLAB Parameter Estimation Toolbox**
  - System dynamics simulated in **Simulink**

- **PID Tuning**:
  - Initial manual tuning for baseline performance
  - Final tuning via **control design techniques** like **Root Locus** and system analysis
  - Focused on minimizing overshoot, steady-state error, and rise time

---

## 📂 Features

- Accurate **MATLAB/Simulink model** that closely matches physical actuator behavior
- Tested both **cascaded** and **single-loop** PID configurations
- Reproducible process from modeling to tuning and validation

---

**This repository contains the implementation of a Memristive Hopfield Neural Network for Pseudo-Random Number Generation.**
The project consists of two parts:
**Python Simulation**: A floating-point  used for verification and chaos analysis.

**FPGA Implementation:** A synthesizable Verilog design implemented on the Digilent Nexys 4 DDR board, which involves the Artix-7 FPGA.
This system uses a flux-controlled memristor to realize hyperchaotic behavior and produce high-entropy random bits, which is necessary in cryptographic applications.

**Neural Network Architecture:**
This continuous-time RNN can be described as being made up of:
3 Neurons ($x, y, z$): Modeled by the hyperbolic tangent activation functions.
1 Memristor ($w$): Its nonlinear feedback memory pushes the system from simple stability to hyperchaos. The dynamics are defined by the interaction between the neuron voltages and the magnetic flux of the memristor, solved numerically using the Runge-Kutta 4th Order (RK4) method.

**Applications:**
Cryptography
Secure Communication
Simulation (Monte Carlo methods)
Chaos-based Image Encryption

**FPGA Implementation Details**
The chaotic attractor is implemented in hardware using Xilinx Vivado on the Nexys 4 DDR platform.
**Hardware Specs**
Board: Digilent Nexys 4 DDR
FPGA Chip: Xilinx Artix-7 XC7A100T
Clock Speed: [25 MHz]

Numerical Format: Fixed-Point Arithmetic (IEEE-754 floating point was converted to fixed point for efficient hardware usage).
**Design Flow**
ODE Solver in Hardware: The Runge-Kutta 4 (RK4) or Euler method is implemented using DSP slices for high-speed calculation of differential equations.
Hyperbolic Tangent Approximation: The tanh(x) function is approximated using [ Look-Up Tables (LUTs) ] to reduce resource usage.

Output Interface: The generated random bits are verified via [LEDs].

**Resource Utilization**

LUTs: [56%]
Flip-Flops: [2%]
DSP48: [88%]
IO: [5%]
BUFG: [3%]

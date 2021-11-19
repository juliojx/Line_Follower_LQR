# Line_Follower_LQR
Thesis project for obtaining my bachelor's degree in Physics.  
I designed a simple model of a line follower robot to be controlled by an LQR control.

___

# Table of Contents
1. [Hardware](#Hardware)
2. [Simulation and obtaining the control constants](#Simulation-and-obtaining-the-control-constants)
3. [Third Example](#third-example)
4. [Fourth Example](#fourth-examplehttpwwwfourthexamplecom)


## Hardware
The first step is building the hardware, as I wanted to follow some rules from official contests in robotics, I designed a robot that could be contained in a 20cmx20cm square. Below, you can see the diagram of the project and components.
![Alt text](images/circuit.png?raw=true "Circuit of the prototype, includes power, control and sensor stages")


## Simulation and obtaining the control constants
As the sensors bar has to be centered as the robot goes forward on the line, we can consider the central sensors as a punctual mass in an unidimensional space under the effect of a varying time force. Therefore, we can use the double integrator model, whose equations are given by:

![Alt text](images/equationsDoubleInt.png?raw=true "Equations of a simple double integrator")

Based on the Pontryagin principle we can solve a optimization problem (see http://www.cds.caltech.edu/~murray/books/AM08/pdf/obc09-obc09_03Mar09.pdf for a detailed explanation about the Pontryagin’s maximum principle) by solving the algebraic Riccati equation:

![Alt text](images/RicattiEquations.png?raw=true "Algebraic Riccati equation")

![Alt text](images/RicattiEquationOutput.png?raw=true "Output of Algebraic Riccati equation")

![Alt text](images/RicattiEquationConstant.png?raw=true "Definning the control constant")






## Programming on Arduino


## FOur

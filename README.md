# Line_Follower_LQR
Thesis project for obtaining my bachelor's degree in Physics.  
I designed a simple model of a line follower robot to be controlled by an LQR control.

___

# Table of Contents
1. [Hardware](#Hardware)
2. [Simulation and obtaining the control constants](#Simulation-and-obtaining-the-control-constants)
3. [Programming on Arduino](#Programming-on-Arduino)
4. [Fourth Example](#fourth-examplehttpwwwfourthexamplecom)


## Hardware
The first step is building the hardware, as I wanted to follow some rules from official contests in robotics, I designed a robot that could be contained in a 20cmx20cm square. Below, you can see the diagram of the project and components.
![Alt text](images/circuit.png?raw=true "Circuit of the prototype, includes power, control and sensor stages")


## Simulation and obtaining the control constants
As the sensors bar has to be centered as the robot goes forward on the line, we can consider the central sensors as a punctual mass in an unidimensional space under the effect of a varying time force. Therefore, we can use the double integrator model, whose equations are given by:

![Alt text](images/equationsDoubleInt.png?raw=true "Equations of a simple double integrator")

Based on the Pontryagin principle we can solve a optimization problem by solving the algebraic Riccati equation:

![Alt text](images/RicattiEquations.png?raw=true "Algebraic Riccati equation")

Given a solution, we can set the output as 

![Alt text](images/RicattiEquationOutput.png?raw=true "Output of Algebraic Riccati equation")

and define the constant for the control by feedback

![Alt text](images/RicattiEquationConstant.png?raw=true "Definning the control constant")

A comprehensive explanation of the theory can be found in  http://www.cds.caltech.edu/~murray/books/AM08/pdf/obc09-obc09_03Mar09.pdf or any other classical control theory book.

We can solve the equation in Matlab or Python.

Python solution
```
import numpy as np
import scipy.linalg as linalg
import matplotlib.pyplot as plt



a=np.array([[0,1],[0,0]])
b=np.array([[0],[1]])
q=np.array([[1.5,0],[0,0.5]]) #Weights of Q matrix
r=np.array([[.5]])          #Parameter that weighs ease of rotation, the more distance between wheels, the more weight.
C=np.array([1,0])
D=[0]

ti=0
tf=20
n=500
dt=float(tf-ti)/n
x1_0=0 #Initial conditions
x2_0=0 #Initial conditions
SIN_0=0
x=linalg.solve_continuous_are(a, b, q, r) #Solving Ricatti Equations for our previously set model
#print x

u=-(float(1/r))*np.transpose(b).dot(x) #Getting u

t=np.linspace(ti,tf,n)
y=np.sin(t)
```
So, we can perform a simple simulation with Euler's method
```
#Using Euler method to show the response, as we do not have a closed trayectory is not necesary using an upper grade method
x1= np.zeros([n])
x1[0]=x1_0
x2=np.zeros([n])
x2[0]=x2_0
SIN=np.zeros([n])
SIN[0]=SIN_0
for i in range(1,n):
	SIN[i]=np.sin(t[i-1])
	x1[i]=x1[i-1]+(x2[i-1])*dt+SIN[i]
	x2[i]=x2[i-1]-(x1[i-1]*np.take(x,0))*dt #-kx1
	print np.take(x,0)
#for i in range(1,n):

plt.figure()
plt.subplot(2,2,1)
plt.plot(t, SIN,'r')

plt.subplot(2,2,1)
plt.plot(t, x1,'g')
plt.show()

```
And the response (in green) of the prototype versus the ideal trayectory (in red) will be
![Alt text](images/SimulationIdeal.png?raw=true "Simulation of the response")


## Programming on Arduino

The pseudocodes (In spanish) of each one of the control blocks are presented below
![Alt text](images/PseudicodigoTraccion.png?raw=true "Traction part 1")
![Alt text](images/PseudicodigoTraccion2.png?raw=true "Traction part 2")
![Alt text](images/PseudocodigoBotonPulsador.png?raw=true "Push Button routine")
![Alt text](images/PseudocodigoDeteccionLinea.png?raw=true "Line detection program")
![Alt text](images/PseudocodigoSelector.png?raw=true "Routine selector program")
![Alt text](images/PseudocodigoSelector2.png?raw=true "Routine selector program part 2")
![Alt text](images/PseudocodigoUmbral.png?raw=true "Boundaries setting program")
![Alt text](images/PseudocodigoPrincipal.png?raw=true "Main program")

So, the implementation in Arduino (some filling code is not presented) is presented below:


## FOur

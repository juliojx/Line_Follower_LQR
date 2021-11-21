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

So, the implementation in Arduino (C/C++ language) is presented below:


```
//motor A connected between A01 and A02
//motor B connected between B01 and B02

int VCC=7;
int state=1;
//Variables a utilizar para el botón pulsador
const int pulsador=2;
int contadorBoton=0;
int pulsos=0;
int EstadoPasadoPulsador=0;
int EstadoActualPulsador=0;
unsigned long ActualMillis=0;
unsigned long ActualMillis2=0;
unsigned long UltimoMillis=0;
int linea1=A0;
int linea2=A1;
int linea3=A2;
int linea4=A3;
int linea5=A4;
int linea6=A5;
int linea7=A6;
int linea8=A7;
int cut=550;
float Position=0;
int ContadorAuxiliar=0;
float Time_I;
float Time_F;
float LastPosition;
float Velocity;
int InitialCounter=0;
float TimeInSeconds;

//************Aqui se configuran los motores***************************************************************


int STBY = 10; //standby

//Motor A
int PWMA = 3; //Speed control 
int AIN1 = 9; //Direction
int AIN2 = 8; //Direction

//Motor B
int PWMB = 5; //Speed control
int BIN1 = 11; //Direction
int BIN2 = 12; //Direction

//*********************************************************************

//***************Aqui se configuran los parametros del control LQR***************************************

float k[]={50.729833,0}; //Estos valores los obtenemos de la simulacion hecha en python

float x[]={0,0};
float u;
//******************************************************************************************************






void setup()
{
  Serial.begin(9600);
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(A3,INPUT);
  pinMode(A4,INPUT);
  pinMode(A5,INPUT);
  pinMode(A6,INPUT);
  pinMode(A7,INPUT);
    
}
void loop()
{
  if(InitialCounter==0)
  {
   Time_I=millis();
   ReadPosition();
   DetectPositionWhiteSurfaceMedium();
   LastPosition=Position;
   InitialCounter=1;
  } 
  
  ReadPosition();
  DetectPositionWhiteSurfaceMedium();
  
  if(Position>LastPosition)
  {
    if(Position<0)
    {
       Time_F=millis();
   // Serial.print("Tiempo inicial "); Serial.print(Time_I);Serial.print("Tiempo final "); Serial.println(Time_F);
    TimeInSeconds=(Time_F-Time_I)*0.001;
    Velocity=0.0135/TimeInSeconds;
    //Serial.println(Velocity);
    Time_I=Time_F;
    ReadPosition();
    DetectPositionWhiteSurfaceMedium();
    LastPosition=Position;
    u=(k[0]*Position+k[1]*Velocity); //I removed the minus sing to turn the motors correctly
    Derecha();
    //Serial.print("The position is "); Serial.print(Position,4); Serial.print("And the Velocity is "); Serial.println(Velocity,8);
    Serial.println(u,8);
    }

    if(Position>=0)
    {
    Time_F=millis();
   // Serial.print("Tiempo inicial "); Serial.print(Time_I);Serial.print("Tiempo final "); Serial.println(Time_F);
    TimeInSeconds=(Time_F-Time_I)*0.001;
    Velocity=0.0135/TimeInSeconds;
    //Serial.println(Velocity);
    Time_I=Time_F;
    ReadPosition();
    DetectPositionWhiteSurfaceMedium();
    LastPosition=Position;
    u=(k[0]*Position+k[1]*Velocity); //I removed the minus sing to turn the motors correctly
    Izquierda();
    //Serial.print("The position is "); Serial.print(Position,4); Serial.print("And the Velocity is "); Serial.println(Velocity,8);
    Serial.println(u,8);
    }
  
 }
  
  if(Position<LastPosition)
  {
 
   if(Position<0)
    {
       Time_F=millis();
   // Serial.print("Tiempo inicial "); Serial.print(Time_I);Serial.print("Tiempo final "); Serial.println(Time_F);
    TimeInSeconds=(Time_F-Time_I)*0.001;
    Velocity=0.0135/TimeInSeconds;
    //Serial.println(Velocity);
    Time_I=Time_F;
    ReadPosition();
    DetectPositionWhiteSurfaceMedium();
    LastPosition=Position;
    u=(k[0]*Position+k[1]*Velocity); //I removed the minus sing to turn the motors correctly
    Derecha();
    //Serial.print("The position is "); Serial.print(Position,4); Serial.print("And the Velocity is "); Serial.println(Velocity,8);
    Serial.println(u,8);
    }

    if(Position>=0)
    {
    Time_F=millis();
   // Serial.print("Tiempo inicial "); Serial.print(Time_I);Serial.print("Tiempo final "); Serial.println(Time_F);
    TimeInSeconds=(Time_F-Time_I)*0.001;
    Velocity=0.0135/TimeInSeconds;
    //Serial.println(Velocity);
    Time_I=Time_F;
    ReadPosition();
    DetectPositionWhiteSurfaceMedium();
    LastPosition=Position;
    u=(k[0]*Position+k[1]*Velocity); //I removed the minus sing to turn the motors correctly
    Izquierda(); //era izquierda
    //Serial.print("The position is "); Serial.print(Position,4); Serial.print("And the Velocity is "); Serial.println(Velocity,8);
    Serial.println(u,8);
    }
  }
 
  
  
  //PrintSensorLevels();
  //Serial.print("The position is "); Serial.print(Position,4); Serial.print("And the Velocity is "); Serial.println(Velocity,8);
  //Now, the control in this step will be

}










void ReadPosition()
{
  linea1=analogRead(A0);
  linea2=analogRead(A1);
  linea3=analogRead(A2);
  linea4=analogRead(A3);
  linea5=analogRead(A4);
  linea6=analogRead(A5);
  linea7=analogRead(A6);
  linea8=analogRead(A7);
}
void DetectPositionWhiteSurface()
{
  if(linea1<=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-3.0*0.0135;
   ContadorAuxiliar=-300;
  
  }
  
  if(linea1>=cut&&linea2<=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-2.0*0.0135;
   ContadorAuxiliar=-200;
  }
  if(linea1>=cut&&linea2>=cut&&linea3<=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-1.0*0.0135;
   ContadorAuxiliar=-100;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4<=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
    Position=-0.5;
   
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5<=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=0.5;
  }
   if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4<=cut&&linea5<=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=0;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6<=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=1.0*0.0135;
   ContadorAuxiliar=100;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7<=cut&&linea8>=cut)
  {
   Position=2.0*0.0135;
   ContadorAuxiliar=200;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8<=cut)
  {
   Position=3.0*0.0135;
   ContadorAuxiliar=300;  
  }



  //AL SALIR DE LA LINEA GUARDA EL ULTIMO LUGAR Y DICE SI ES POSICION -4 o 4
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut&&ContadorAuxiliar==-300)
   {
    Position=-4.0*0.0135;
   }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut&&ContadorAuxiliar==300)
   {
    Position=4.0*0.0135;
   }
  
  
}


void DetectPositionWhiteSurfaceMedium()
{
  if(linea1<=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-3.5*0.0135;
   ContadorAuxiliar=-350;
  
  }
  if(linea1<=cut&&linea2<=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-3.0*0.0135;
   ContadorAuxiliar=-300;
  
  }
  if(linea1>=cut&&linea2<=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-2.5*0.0135;
   ContadorAuxiliar=-250;
  }
  if(linea1>=cut&&linea2<=cut&&linea3<=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-2.0*0.0135;
   ContadorAuxiliar=-200;
  }
  
  if(linea1>=cut&&linea2>=cut&&linea3<=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-1.5*0.0135;
   ContadorAuxiliar=-150;
  }
  if(linea1>=cut&&linea2>=cut&&linea3<=cut&&linea4<=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=-1.0*0.0135;
   ContadorAuxiliar=-100;
  }
 
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4<=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
    Position=-0.5*0.0135;
    ContadorAuxiliar=-50;
   
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4<=cut&&linea5<=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=0;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5<=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=0.5*0.0135;
   ContadorAuxiliar=50;

  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5<=cut&&linea6<=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=1.0*0.0135;
   ContadorAuxiliar=100;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6<=cut&&linea7>=cut&&linea8>=cut)
  {
   Position=1.5*0.0135;
   ContadorAuxiliar=150;

  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6<=cut&&linea7<=cut&&linea8>=cut)
  {
   Position=2.0*0.0135;
   ContadorAuxiliar=200;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7<=cut&&linea8>=cut)
  {
   Position=2.5*0.0135;
   ContadorAuxiliar=250;
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=3.0*0.0135;
   ContadorAuxiliar=300;  
  }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8<=cut)
  {
   Position=3.5*0.0135;
   ContadorAuxiliar=350;  
  }



  //AL SALIR DE LA LINEA GUARDA EL ULTIMO LUGAR Y DICE SI ES POSICION -4 o 4
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut&&ContadorAuxiliar==-350)
   {
    Position=-4.0*0.0135;
   }
  if(linea1>=cut&&linea2>=cut&&linea3>=cut&&linea4>=cut&&linea5>=cut&&linea6>=cut&&linea7>=cut&&linea8>=cut&&ContadorAuxiliar==350)
   {
    Position=4.0*0.0135;
   }
  
  
}

void DetectPositionBlackSurface()
{
  if(linea1>=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=-3.0*0.0135;
   ContadorAuxiliar=-300;
   
   if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut&&ContadorAuxiliar==-300)
   {
    Position=-4.0*0.0135;
   }
  
  }
  if(linea1<=cut&&linea2>=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=-2.0*0.0135;
   ContadorAuxiliar=-200;
  }
  if(linea1<=cut&&linea2<=cut&&linea3>=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=-1.0*0.0135;
   ContadorAuxiliar=-100;
  }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4>=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut)
  {
    Position=0.0;
   
  }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5>=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=0;
  }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6>=cut&&linea7<=cut&&linea8<=cut)
  {
   Position=1.0*0.0135;
   ContadorAuxiliar=100;
  }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7>=cut&&linea8<=cut)
  {
   Position=2.0*0.0135;
   ContadorAuxiliar=200;
  }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8>=cut)
  {
   Position=3.0*0.0135;
   ContadorAuxiliar=300;  

   if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut&&ContadorAuxiliar==300)
   {
    Position=4.0*0.0135;
   }
   
  }
  
  //AL SALIR DE LA LINEA GUARDA EL ULTIMO LUGAR Y DICE SI ES POSICION -4 o 4
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8<=cut&&ContadorAuxiliar==-300)
   {
    Position=-4.0*0.0135;
   }
  if(linea1<=cut&&linea2<=cut&&linea3<=cut&&linea4<=cut&&linea5<=cut&&linea6<=cut&&linea7<=cut&&linea8>=cut&&ContadorAuxiliar==300)
   {
    Position=4.0*0.0135;
   }
  
}


void ComputeVelocity()
{
  Velocity=((float(Time_F)-float(Time_I)))*.001;


  return Velocity;
}
void PrintSensorLevels()
{
  Serial.print(linea1); Serial.print("  ");
  Serial.print(linea2); Serial.print("  ");
  Serial.print(linea3); Serial.print("  ");
  Serial.print(linea4); Serial.print("  ");
  Serial.print(linea5); Serial.print("  ");
  Serial.print(linea6); Serial.print("  ");
  Serial.print(linea7); Serial.print("  ");
  Serial.print(linea8); Serial.println("  ");
  
  }






void move(int motor, int speed, int direction){
//Move specific motor at speed and direction
//motor: 0 for B 1 for A
//speed: 0 is off, and 255 is full speed
//direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if(direction == 1){
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if(motor == 1){
    digitalWrite(AIN1, inPin1);
    digitalWrite(AIN2, inPin2);
    analogWrite(PWMA, speed);
  }else{
    digitalWrite(BIN1, inPin1);
    digitalWrite(BIN2, inPin2);
    analogWrite(PWMB, speed);
  }

}

void stop(){
//enable standby  
  digitalWrite(STBY, LOW); 
}

void Recto()
{
  move(1, 200, 0); //motor 1, full speed, atrás
  move(2, 200, 1); //motor 2, full speed, adelante

}
void Derecha()
{
  move(1, 128, 0); //motor 1, full speed, atrás
  move(2, 128.0+46.0*u, 1); //motor 2, full speed, adelante

}
void Izquierda()
{
  move(1, 128.0-u*46.0, 0); //motor 1, full speed, atrás
  move(2, 128, 1); //motor 2, full speed, adelante

}


```

## FOur

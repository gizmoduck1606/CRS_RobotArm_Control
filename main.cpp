#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"
// These two offsets are only used in the main file user_CRSRobot.c
// You just need to create them here and find the correct offset
// and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.45;//offset of motor angle 2 -0.45 better
float offset_Enc3_rad = 0.225; //offset of motor angle 3
// Your global varialbes.
long mycount = 0;
#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;
#pragma DATA_SECTION(whattoprint2, ".my_vars")
float whattoprint2 = 0.0;
#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];
#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];
long arrayindex = 0;
float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;
// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;
float x=0,y=0,z = 0;
float q1=0,q2=0,q3=0;
float r = 0;
float s =0;
float th1m=0;
float th2m=0;
float th3m=0;
float th1dh=0,th2dh=0,th3dh=0;
float gamma;
float rp; //rprime
float alpha ;
float beta ;
float th3p;
float th3d ;//lab2 code start from next line
float Theta1_old = 0;
float Omega1_old1 = 0;
float Omega1_old2 = 0;
float Omega1 = 0;//joint 2 starts from below
float Theta2_old = 0;
float Omega2_old1 = 0;
float Omega2_old2 = 0;
float Omega2 = 0;//joint 3 start from below
float Theta3_old = 0;
float Omega3_old1 = 0;
float Omega3_old2 = 0;
float Omega3 = 0;
float Kp1 = 20;
float Kp2 = 20;
float Kp3 = 20;
float Kd1 = 1.75;
float Kd2 = 1.75;
float Kd3 = 1.75;
float Ki1 = 3;
float Ki2 = 3;
float Ki3 = 3;
float theta1d=0;
float theta2d=0;
float theta3d=0;
float Ik1 = 0;
float ek1 = 0;
float ek1old = 0;
float Ik2 = 0;
float ek2 = 0;
float ek2old = 0;
float Ik3 = 0;
float ek3 = 0;
float ek3old = 0;
float threshold=0.1;
float t=0;
float td=0;
float d_alpha1=0;
float d_alpha2=0;
float d_alpha3=0;
float d_omega1=0;
float d_omega2=0;
float d_omega3=0;
float Tau1=0;
float Tau2=0;
float Tau3=0;
float xa=0;
float ya=0;
float za=0;
float mm1;
float mm2;
float mm3;
float v1p=0.223;//Viscous coefficient for motor 1 positive direction
float c1p=0.35;//coulomb coefficient for motro 1 positive direction
float v1n=0.24;
float c1n=-.28;
float v2p=0.22;//Viscous coefficient for motor 2 positive direction
float c2p=0.476;//coulomb coefficient for motro 2 positive direction
float v2n=0.34;
float c2n=-.3;
float v3p=0.25;//Viscous coefficient for motor 3 positive direction
float c3p=0.3;//coulomb coefficient for motro 3 positive direction
float v3n=0.213;
float c3n=-.55;
float atheta2=0;
float KP2=1000;
float KD2=50;
float atheta3=0;
float KP3=1000;
float KD3=50;
float p1,p2,p3,p4,p5;
float theta2m,theta3m;
float sintheta2,costheta2,sintheta3,costheta3, sinth3minusth2,costh3minusth2;
// lab 4
float cosq1 = 0;
float sinq1 = 0;
float cosq2 = 0;
float sinq2 = 0;
float cosq3 = 0;
float sinq3 = 0;
float JT_11 = 0;
float JT_12 = 0;
float JT_13 = 0;
float JT_21 = 0;
float JT_22 = 0;
float JT_23 = 0;
float JT_31 = 0;
float JT_32 = 0;
float JT_33 = 0;
float cosz = 0;
float sinz = 0;
float cosx = 0;
float sinx = 0;
float cosy = 0;
float siny = 0;
float thetaz = 0;
float thetax = 0;
float thetay = 0;
float R11 = 0;
float R12 = 0;
float R13 = 0;
float R21 = 0;
float R22 = 0;
float R23 = 0;
float R31 = 0;
float R32 = 0;
float R33 = 0;
float RT11 = 0;
float RT12 = 0;
float RT13 = 0;
float RT21 = 0;
float RT22 = 0;
float RT23 = 0;
float RT31 = 0;
float RT32 = 0;
float RT33 = 0;
float xdot=0;
float xold=0;
float xdot_old1=0;
float xdot_old2=0;
float ydot=0;
float yold=0;
float ydot_old1=0;
float ydot_old2=0;
float zdot=0;
float zold=0;
float zdot_old1=0;
float zdot_old2=0;
float FX=0,FY=0,FZ=0;
float KPX=0.5;
float KPY=0.5;
float KPZ=0.5;
float KDX=0.025;//0.025;
float KDY=0.025;//0.025;
float KDZ=0.025;//0.025;
float xd = 10;
float yd = 0;
float zd = 20;
float xdotd=0;
float ydotd=0;
float zdotd=0;
float JTR_11=0;
float JTR_12=0;
float JTR_13=0;
float JTR_21=0;
float JTR_22=0;
float JTR_23=0;
float JTR_31=0;
float JTR_32=0;
float JTR_33=0;
float KPRT_11=0;
float KPRT_21=0;
float KPRT_31=0;
float KDRT_11=0;
float KDRT_21=0;
float KDRT_31=0;
float xi=10;
float yi=0;
float zi=20;
float xf=10;
float yf=0;
float zf=20;
float ttotal = 0;
float speed=0;
float delta_x=0;
float delta_y=0;
float delta_z=0;
float tstart=0;
float tfinal=0;
float temp = 0;
float tt1=0;
float tt2=0;
float tt3=0;
float tt4=0;
float tt5=0;
float tt6=0;
float vel=0;
//float tfinal2=0;
// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int
error) {
td=t/1000.0;
//Motor torque limitation(Max: 5 Min: -5)
// save past states
if ((mycount%50)==0) {
theta1array[arrayindex] = theta1motor;
if (arrayindex >= 100) {
arrayindex = 0;
} else {
arrayindex++;
}
}
if ((mycount%1000)==0) {
if (whattoprint > 0.5) {
serial_printf(&SerialA, "I love robotics\n\r");
} else {
printtheta1motor = theta1motor; //motor angle 1
printtheta2motor = theta2motor; //motor angle 2
printtheta3motor = theta3motor; //motor angle 3
SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many
floats.
}
GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box
}
// Rotation zxy and its Transpose
// Jacobian Transpose in motor angle
cosq1 = cos(theta1motor);
sinq1 = sin(theta1motor);
cosq2 = cos(theta2motor);
sinq2 = sin(theta2motor);
cosq3 = cos(theta3motor);
sinq3 = sin(theta3motor);
JT_11 = -10*sinq1*(cosq3 + sinq2);
JT_12 = 10*cosq1*(cosq3 + sinq2);
JT_13 = 0;
JT_21 = 10*cosq1*(cosq2 - sinq3);
JT_22 = 10*sinq1*(cosq2 - sinq3);
JT_23 = -10*(cosq3 + sinq2);
JT_31 = -10*cosq1*sinq3;
JT_32 = -10*sinq1*sinq3;
JT_33 = -10*cosq3;
// forward kinematics lab 4 defined in motor angle
x = 10*cosq1*(cosq3+sinq2);
y = 10*sinq1*(cosq3+sinq2);
z = 10*(1+cosq2-sinq3);
// forward kinmatics in D-H angle
// q1 = theta1motor; // motor angle 1 to D-H angle conversion
// q2 = theta2motor-PI/2; // motor angle 2 to D-H angle conversion
// q3 = -theta2motor+theta3motor+PI/2; //// motor angle 3 to D-H angle conversion
//
//
// xa = 20 *cos(q1)* cos(q2 + q3/2)*cos(q3/2); // equations are obtained from
mathematica
// //forward kinematics x,y,z position expressed in D-H angle
// ya = 20* cos(q2 + q3/2)*cos(q3/2)*sin(q1);
// za = -10*(-1 + sin(q2) + sin(q2 + q3));
////
//
// // This function is called every 1 ms
// joint 1
Omega1 = (theta1motor - Theta1_old)/0.001;
Omega1 = (Omega1 + Omega1_old1 + Omega1_old2)/3.0;
Theta1_old = theta1motor;
Omega1_old2 = Omega1_old1;
Omega1_old1 = Omega1;
//joint 2
Omega2 = (theta2motor - Theta2_old)/0.001;
Omega2 = (Omega2 + Omega2_old1 + Omega2_old2)/3.0;
Theta2_old = theta2motor;
Omega2_old2 = Omega2_old1;
Omega2_old1 = Omega2;
//joint 3
Omega3 = (theta3motor - Theta3_old)/0.001;
Omega3 = (Omega3 + Omega3_old1 + Omega3_old2)/3.0;
Theta3_old = theta3motor;
Omega3_old2 = Omega3_old1;
Omega3_old1 = Omega3;
//// x dot
xdot = (x - xold)/0.001;
xdot = (xdot + xdot_old1 + xdot_old2)/3.0;
xold = x;
xdot_old2 = xdot_old1;
xdot_old1 = xdot;
// //y dot
ydot = (y - yold)/0.001;
ydot = (ydot + ydot_old1 + ydot_old2)/3.0;
yold = y;
ydot_old2 = ydot_old1;
ydot_old1 = ydot;
// //z dot
zdot = (z - zold)/0.001;
zdot = (zdot + zdot_old1 + zdot_old2)/3.0;
zold = z;
zdot_old2 = zdot_old1;
zdot_old1 = zdot;
//////Error integral
// ek1 = theta1d-theta1motor ;
// Ik1 = Ik1 + (ek1+ek1old)/2*0.001;
// ek1old = ek1;
//
// ek2 = theta2d-theta2motor ;
// Ik2 = Ik2 + (ek2+ek2old)/2*0.001;
// ek2old = ek2;
//
// ek3 = theta3d-theta3motor ;
// Ik3 = Ik3 + (ek3+ek3old)/2*0.001;
// ek3old = ek3;
// if (fabs(ek1)>threshold){
// Ik1=0;
// }
// if (fabs(ek2)>threshold){
// Ik2=0;
// }
// if (fabs(ek3)>threshold){
// Ik3=0;
// }
////Torque control with forward feedback PID
////*tau1 = Kp1*(theta1d-theta1motor)+Kd1*(d_omega1-Omega1)+Ki1*Ik1+0.0167*d_alpha1;
//torque at motor 1
////*tau2 = Kp2*(theta2d-theta2motor)+Kd2*(d_omega2-Omega2)+Ki2*Ik2+0.03*d_alpha2;
//torque at motor 2
////*tau3 = Kp3*(theta3d-theta3motor)+Kd3*(d_omega3-Omega3)+Ki3*Ik3+0.0128*d_alpha3;
//torque at motor 3
//
////Torque control with forward feedback PD
////*tau1 = Kp1*(theta1d-theta1motor)+Kd1*(d_omega1-Omega1)+0.0167*d_alpha1; //torque
at motor 1
////*tau2 = Kp2*(theta2d-theta2motor)+Kd2*(d_omega2-Omega2)+0.03*d_alpha2; //torque at
motor 2
////*tau3 = Kp3*(theta3d-theta3motor)+Kd3*(d_omega3-Omega3)+0.0128*d_alpha3; //torque
at motor 3
//
////Torque control without forward feedback
////mm1 = theta1motor;
////mm2 = theta2motor;
////mm3 = theta3motor;
////*tau1 = Kp1*(theta1d-theta1motor)-Kd1*Omega1+Ki1*Ik1; //torque at motor 1
////*tau2 = Kp2*(theta2d-theta2motor)-Kd2*Omega2+Ki2*Ik2; //torque at motor 2
////*tau3 = Kp3*(theta3d-theta3motor)-Kd3*Omega3+Ki3*Ik3; //torque at motor 3
//
////Control without integral control
////*tau1 = Kp1*(theta1d-theta1motor)-Kd1*Omega1; //torque at motor 1
////*tau2 = Kp2*(theta2d-theta2motor)-Kd2*Omega2; //torque at motor 2
////*tau3 = Kp3*(theta3d-theta3motor)-Kd3*Omega3; //torque at motor 3
////*tau1=0;
////*tau2=0;
////*tau3=0;
//
////Lab 3 Inverse Dynamics control
//*tau1 = Kp1*(theta1d-theta1motor)+Kd1*(d_omega1-Omega1)+0.0167*d_alpha1; //torque at
motor 1
//*tau2 = p1*atheta2+(-p3*sinth3minusth2*atheta3)+(-
p3*costh3minusth2*Omega3*Omega3)+(-p4*9.8*sintheta2);
//*tau3 = (-
p3*sinth3minusth2*atheta2)+p2*atheta3+(p3*costh3minusth2*Omega2*Omega2)+(-
p5*9.8*costheta2);
// TASK SPACE CONTROL
//
// FX = KPX*(xd-x)+KDX*(xdotd-xdot);
// FY = KPY*(yd-y)+KDY*(ydotd-ydot);
// FZ = KPZ*(zd-z)+KDZ*(zdotd-zdot);
////
// *tau1 = JT_11*FX+JT_12*FY+JT_13*FZ;
// *tau2 = JT_21*FX+JT_22*FY+JT_23*FZ;
// *tau3 = JT_31*FX+JT_32*FY+JT_33*FZ;
//
tt1 = 1.5;//from home to motor zero
tt2 = 2;//motor zero to hole top
tt3 = 0.5;//moving down
tt4 = 1;//stay inside hole for a long time
tt5 = 0.5;//moving up
tt6 = 2;//moving around obstacle
if(td<=0.1){//moving to zero spending 1.5
xi =x;
yi =y;
zi =z;
xf =10;
yf =0;
zf =20;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.3;
}
if (td > 0.1 && td<= 0.7 ) { //moving to the hole top 2
 tstart=0.1;
 xi =10;
yi =0;
zi=20;
xf =0.8;
yf =13.314;
zf=10;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
 ttotal = 0.7;
}
if (td > 0.7 && td<= 0.9 ) { //moving to the hole top 2
 tstart=0.7;
 xi =0.8;
yi =13.314;
zi=10;
xf =0.8;
yf =13.314;
zf=8;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
 ttotal = 0.2;
}
if(td>0.9 && td<=1.3){//moving down the hole 3
tstart=0.9;
xi =0.8;
yi =13.314;
zi=8;
xf =0.8;
yf =13.314;
zf=5.5;
KPX = 0.05;
KPY = 0.05;
KPZ = 1.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.4;
}
//
if (td>1.3 && td<=2){ //staying in the hole 1
 tstart=1.3;
xi =x;
yi =y;
zi =z;
xf =x;
yf =y;
zf=z;
KPX = 0.05;
KPY = 0.05;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal= 0.7;
}
if(td>2 && td<=2.2){//moving up the hole 3
tstart=2;
xi =0.8;
yi =13.314;
zi=5.5;
xf =0.8;
yf =13.314;
zf=10;
KPX = 0.05;
KPY = 0.05;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.2;
}
if(td>2.2 && td<=2.7 ){//moving to the obstacle 2
tstart=2.2;
xi =0.8;
yi =13.314;
zi=10;
xf =11.26;
yf =4.25;
zf=8.28;
KPX = 1;
KPY = 1;
KPZ = 1;
KDX = 0.05;
KDY = 0.05;
KDZ = 0.05;
ttotal=0.5;
}
if(td>2.7 && td<=2.9 ){//moving towards zigzag
tstart=2.7;
xi =11.26;
yi =4.25;
zi=8.3;
xf =14.9;
yf =4.25;
zf=8.3;
KPX = 1;
KPY = 1;
KPZ = 1;
KDX = 0.05;
KDY = 0.05;
KDZ = 0.05;
ttotal=0.2;
}
if(td>2.9 && td<=3 ){//moving to the zig zag entrance point 1
tstart=2.9;
xi =15;
yi =4.25;
zi=8.3;
xf =15.08;
yf =3.76;
zf=8.3;
KPX = 1;
KPY = 1;
KPZ = 1;
KDX = 0.025;
KDY = 0.005;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3 && td<=3.2 ){//moving to point 2
tstart=3;
xi =15.08;
yi =3.76;
zi=8.3;
xf =16.13;
yf =2.25;
zf=8.3;
thetaz= -53.13*PI/180;
KPX = 0.5;
KPY = 0.01;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.005;
KDZ = 0.025;
ttotal=0.2;
}
if(td>3.2 && td<=3.3 ){//moving to point 3
tstart=3.2;
xi =16.13;
yi =2.25;
zi=8.3;
xf =15.43;
yf =1.85;//1.75 to 1.85
zf=8.3;
thetaz= -143.13*PI/180;
KPX = 1.4;
KPY = 0.01;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.004;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3.3 && td<=3.4){//moving to point 4a
tstart=3.3;
xi =15.43;
yi =1.85;
zi= 8.3;
xf =13.07;
yf =2.04;
zf=8.3;
 thetaz= -195*PI/180;
KPX = 1.2;
KPY = 0.01;
KPZ = 1.2;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal= 0.1;
}
if(td>3.4 && td<=3.5){//moving to point 4b
tstart=3.4;
xi =13.07;
yi =2.04;
zi= 8.3;
xf =12.67;
yf =1.29;
zf=8.3;
 thetaz= -150*PI/180;
KPX = 1.2;
KPY = 0.01;
KPZ = 1.2;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal= 0.1;
}
if(td>3.5 && td<=3.6 ){//moving to point 5
tstart=3.5;
xi =12.67;
yi =1.29;
zi=8.3;
xf =12.5;
yf =1.28;
zf=8.3;
thetaz= -105*PI/180;
KPX = 1.2;
KPY = 0.01;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3.6 && td<=3.7){//moving to point 6
tstart=3.6;
xi =12.50;
yi =1.28;
zi=8.3;
xf =15.62;
yf =-2.08;
zf=8.3;
thetaz= -53.13*PI/180;
KPX = 1.2;
KPY = 0.05;
KPZ = 1.2;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3.7 && td<=3.8){//moving to point before egg
tstart=3.7;
xi =15.486;
yi =-1.682;
zi=8.3;
xf =14.803;
 yf =-2.521;
 zf=16.584;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3.8 && td<= 3.9 ){//Vertically above the egg
tstart=3.8;
xi =14.803;
 yi =-2.521;
 zi=16.584;
 xf =14.84;
 yf =-5;
zf=16.584;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if(td>3.9 && td<=4 ){//Touching the egg
tstart=3.9;
xi =14.84;
 yi =-5;
 zi= 16.584;
 xf =14.84;
 yf =-5;
zf=10;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if(td>4 && td<=4.5 ){//Pressing the egg down
tstart=4;
 xi =14.84;
 yi =-5;
zi=14.32;
xf =14.84;
 yf =-5;
 zf=13.42;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.5;
}
if(td>4.5 && td<=5){//Hold the egg down
tstart=4.5;
 xi =14.84;
 yi =-5;
zi=13.42;
xf =14.84;
 yf =-5;
 zf=13.42;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.5;
}
if(td>5 && td<=5.1 ){//Moving back the egg
tstart=5;
 xi =14.84;
 yi =-5;
zi=13.42;
xf =14.84;
 yf =-5;
 zf=14.32;
KPX = 0.5;
KPY = 0.5;
KPZ = 0.5;
KDX = 0.025;
KDY = 0.025;
KDZ = 0.025;
ttotal=0.1;
}
if (td>5.1 && td<=5.2){//moving to motor zero
tstart= 5.1;
xi =14.84;
 yi =-5;
 zi=14.32;
 xf =10;
 yf =0;
 zf=20;
 ttotal= 0.2;
}
if (td>5.2){
tstart=5.2;
xi =10;
yi =0;
 zi=20;
 xf =10;
 yf =0;
 zf=20;
 ttotal=5;
}
delta_x= xf-xi;
delta_y= yf-yi;
delta_z= zf-zi;
xd = delta_x*((td-tstart)/ttotal)+xi;
yd = delta_y*((td-tstart)/ttotal)+yi;
zd = delta_z*((td-tstart)/ttotal)+zi;
cosz = cos(thetaz);
sinz = sin(thetaz);
cosx = cos(thetax);
sinx = sin(thetax);
cosy = cos(thetay);
siny = sin(thetay);
RT11 = R11 = cosz*cosy-sinz*sinx*siny;
RT21 = R12 = -sinz*cosx;
RT31 = R13 = cosz*siny+sinz*sinx*cosy;
RT12 = R21 = sinz*cosy+cosz*sinx*siny;
RT22 = R22 = cosz*cosx;
RT32 = R23 = sinz*siny-cosz*sinx*cosy;
RT13 = R31 = -cosx*siny;
RT23 = R32 = sinx;
RT33 = R33 = cosx*cosy;
 JTR_11=JT_11*R11+JT_12*R21+JT_13*R31;
 JTR_12=JT_11*R12+JT_12*R22+JT_13*R32;
JTR_13=JT_11*R13+JT_12*R23+JT_13*R33;
JTR_21=JT_21*R11+JT_22*R21+JT_23*R31;
JTR_22=JT_21*R12+JT_22*R22+JT_23*R32;
JTR_23=JT_21*R13+JT_22*R23+JT_23*R33;
JTR_31=JT_31*R11+JT_32*R21+JT_33*R31;
JTR_32=JT_31*R12+JT_32*R22+JT_33*R32;
JTR_33=JT_31*R13+JT_32*R23+JT_33*R33;
KPRT_11= KPX*RT11*(xd-x)+KPX*RT12*(yd-y)+KPX*RT13*(zd-z);
KPRT_21= KPY*RT21*(xd-x)+KPY*RT22*(yd-y)+KPY*RT23*(zd-z);
KPRT_31= KPZ*RT31*(xd-x)+KPZ*RT32*(yd-y)+KPZ*RT33*(zd-z);
KDRT_11= KDX*RT11*(xdotd-xdot)+KDX*RT12*(ydotd-ydot)+KDX*RT13*(zdotd-zdot);
KDRT_21=KDY*RT21*(xdotd-xdot)+KDY*RT22*(ydotd-ydot)+KDY*RT23*(zdotd-zdot);
KDRT_31= KDZ*RT31*(xdotd-xdot)+KDZ*RT32*(ydotd-ydot)+KDZ*RT33*(zdotd-zdot);
//
// Task space and Impedance control
 *tau1= JTR_11*(KPRT_11+KDRT_11)+JTR_12*(KPRT_21+KDRT_21)+JTR_13*(KPRT_31+KDRT_31);
 *tau2= JTR_21*(KPRT_11+KDRT_11)+JTR_22*(KPRT_21+KDRT_21)+JTR_23*(KPRT_31+KDRT_31);
*tau3=
JTR_31*(KPRT_11+KDRT_11)+JTR_32*(KPRT_21+KDRT_21)+JTR_33*(KPRT_31+KDRT_31);
//friction compensation
if (Omega1 > 0.1) {
*tau1 = *tau1 + 0.8*(v1p*Omega1 + c1p) ;
} else if (Omega1 < -0.1) {
*tau1 = *tau1+0.8*(v1n*Omega1 + c1n);
} else {
*tau1 = *tau1+0.8*3.6*Omega1;
}
if (Omega2 > 0.05) {
*tau2 = *tau2 + 0.8*(v2p*Omega2 + c2p) ;
} else if (Omega2 < -0.05) {
*tau2 = *tau2+0.8*(v2n*Omega2 + c2n);
} else {
*tau2 = *tau2+0.8*3.6*Omega2;
}
if (Omega3 > 0.05) {
*tau3 = *tau3 + 0.8*(v3p*Omega3 + c3p) ;
} else if (Omega3 < -0.05) {
*tau3 = *tau3+0.8*(v3n*Omega3 + c3n);
} else {
*tau3 = *tau3+3.6*Omega3;
}
//saturation
if (*tau1>5){
*tau1 = 5;
}
if (*tau1<-5){
*tau1 = -5;
}
if (*tau2>5){
*tau2 = 5;
}
if (*tau2<-5){
*tau2 = -5;
}
if (*tau3>5){
*tau3 = 5;
}
if (*tau3<-5){
*tau3 = -5;
}
Tau1=*tau1;
Tau2=*tau2;
Tau3=*tau3;
Simulink_PlotVar1 = theta1motor;
Simulink_PlotVar2 = theta2motor;
Simulink_PlotVar3 = theta3motor;
Simulink_PlotVar4 = theta1d;
mycount++;
t=t+1;
//if (td>tfinal2){
//t=0;
//}
// Reset for defined trajectory
// if(t>=2*PI*1000){
// t=0;
// }
//
}
void printing(void){
serial_printf(&SerialA, " x:%f y:%f z:%f tau1:%f tau2:%f tau3:%f \n\r", x,y,z,Tau1,Tau2,Tau3);
}

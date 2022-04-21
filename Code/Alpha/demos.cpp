#include "demos.hpp"
// #include "helpers.hpp"
#include <stdio.h>

float whiteboard(int GR, float k, float c, float a){
  
  // clarify this with alex
  float trig_matrix1, t1= trig_func();
  float trig_matrix2, t2= trig_func();


  float e_e1=ee_pos(trig_matrix1,a,t1);
  float e_e2=ee_pos(trig_matrix2,a,t2);
  //only add since we are manually doing o-drive data, all this code breaks if dt = 0
  e_e2[3]=e_e2[3]+1;
  //remove above
  float dist=normal_dist(e_e2);
  float v_wtime= vel_ee(e_e1,e_e2); 
  float v= v_wtime[0:3];
  float F = k*dist - c*v;
  float motor_torque =  jacobian_torque(trig_matrix2,a,F,GR);
  return motor_torque;
}

float inertia(int GR,float a, float m){
  float trig_matrix1,t = trig_func()
  float trig_matrix2,t = trig_func()
  float trig_matrix3,t = trig_func()

  // modify pull to store 3 e_e positions at once
  float e_e1=ee_pos(trig_matrix1,a,t)
  float e_e2=ee_pos(trig_matrix2,a,t)
  float e_e3=ee_pos(trig_matrix3,a,t)
  // have to add to mock-up the odrive pull data:
  e_e2[3]=e_e2[3]+1;
  e_e3[3]= e_e3[3]+1;

  float accel=accel_ee(e_e1,e_e2,e_e3)
  float F_go [3]= {0,0,m*-9.81}
  float F= -(m*accel[0:3] - F_go)
  float motor_torque =  jacobian_torque(trig_matrix3,a,F,GR)
  return(motor_torque)
}

void past_wall(int GR, float k, float c, float a){
     // clarify this with alex
  float trig_matrix1, t1= trig_func();
  float trig_matrix2, t2= trig_func();


  float e_e1=ee_pos(trig_matrix1,a,t1);
  float e_e2=ee_pos(trig_matrix2,a,t2);
  //only add since we are manually doing o-drive data, all this code breaks if dt = 0
  e_e2[3]=e_e2[3]+1;
  //remove above
  float dist=normal_dist(e_e2);

  if (dist<0) {
      print("You are outside of the wall");
  } 
  else{
      print("You are at or inside the wall");
  }
  printf('Distance from wall:%f',dist)


}
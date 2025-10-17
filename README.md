这两套代码几乎一样，但是区别在于底层的电机不同。希望日后有人能够开发一个一件切换的代码。我这边先暂时写出两者区别，方便后人参考

在主ino代码里面。11行，#include "Sig_Motor_Control.h"   和#include "Motor_Control_Tmotor.h"

在initial_Sig_motor函数，整个替换

在loop函数下发阶段，sig
    if (ctl_mode == 1){
        sig_m1.sig_mit_ctl_cmd(0.0,0.0,10.0,0.01,M1_torque_command);
        sig_m2.sig_mit_ctl_cmd(0.0,0.0,10.0,0.01,M2_torque_command);
        receive_mit_ctl_feedback();
    }else{
        for (int i=0;i<4;++i) receive_torque_ctl_feedback();
        sig_m1.sig_torque_cmd(M1_torque_command);
        sig_m2.sig_torque_cmd(M2_torque_command);
    }

和tmotor
    sig_m1.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M1_torque_command);
    sig_m2.send_cmd(0.0f, 0.0f, 0.0f, 0.0f, (float)M2_torque_command);

tmotor的实测扭矩：
// 实测扭矩（Nm）
float M1_torque_meas = 0.0f;
float M2_torque_meas = 0.0f;

void receive_tm_feedback()
{
  CAN_message_t mr;
  while (Can3.read(mr)) {
    sig_m1.unpack_reply(mr);
    sig_m2.unpack_reply(mr);
  }

  // 同步实测扭矩（Nm）
  M1_torque_meas = sig_m1.torque;
  M2_torque_meas = sig_m2.torque;

  // 如果你要相对位姿：
  // float pos1_rel = sig_m1.pos - initial_pos_1;
  // float pos2_rel = sig_m2.pos - initial_pos_2;
}


sig motor的多与函数：
float Sig_torque_control(float force_des, float dt_force_des, float force_t, float dt_force_t, float kp, float kd, float tau_ff)  
{
  float tor_cmd = 0;   

  tor_cmd = kp * (force_des - force_t) + kd * (dt_force_des - dt_force_t) + tau_ff; 

  return tor_cmd;   
}  

float Sig_motion_control(float pos_des, float vel_des, float pos_t, float vel_t, float kp, float kd, float tau_ff)  
{
  float pos_ctl_cmd = 0;   

  pos_ctl_cmd = kp * (pos_des - pos_t) + kd * (vel_des - vel_t) + tau_ff; 

  return pos_ctl_cmd;   
}  

void receive_mit_ctl_feedback() {
  if (Can3.read(msgR)) {
    Can3.read(msgR);  

    if (msgR.id == 0x008)     
    {
      if (msgR.buf[0] == 0x000)      
      {
        sig_m1.unpack_reply(msgR, initial_pos_1);      
      } 
    } 
  }
}   

const uint16_t ID_M1_POSVEL = (0x002<<5) | 0x009;  // 0x049
const uint16_t ID_M1_TORQUE = (0x002<<5) | 0x01C;  // 0x05C
const uint16_t ID_M2_POSVEL = (0x001<<5) | 0x009;  // 0x029
const uint16_t ID_M2_TORQUE = (0x001<<5) | 0x01C;  // 0x03C
const uint16_t ID_M1_IQ = (0x002<<5) | 0x014;  // 0x064
const uint16_t ID_M2_IQ = (0x001<<5) | 0x014;  // 0x034
#define KT_1  0.43f   
#define KT_2  0.43f   
void receive_torque_ctl_feedback()
{
    while (Can3.read(msgR)) {
        switch (msgR.id) {
            case ID_M1_POSVEL:
                sig_m1.unpack_pos_vel(msgR, initial_pos_1);
                break;
            case ID_M1_IQ: {
                float iq = *(float *)&msgR.buf[4];      
                sig_m1.torque = iq * KT_1;
                tau_t_1 = sig_m1.torque;              
                break;
            }

            case ID_M2_POSVEL:
                sig_m2.unpack_pos_vel(msgR, initial_pos_2);
                break;
            case ID_M2_IQ: {
                float iq = *(float *)&msgR.buf[4];
                sig_m2.torque = iq * KT_2;
                tau_t_2 = sig_m2.torque;           
                break;
            }
        }
    }
}

其他应该没有太多要改变的地方。

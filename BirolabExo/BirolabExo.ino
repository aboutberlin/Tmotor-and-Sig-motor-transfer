//To make this work: Turn on motors. Load to Teensy while subject standing still. When done (data updating in Serial Monitor), run Python code ISRA_Main.py.
//Once that one is running (data updating in Command Window), start running. To change peak intensity, change lines 85-86 in the Python code.
#include "Serial_Com.h"
#include "IMU_Adapter.h"
#include <Arduino.h>
#include "MovingAverage.h"
#include <math.h>  
#include <iomanip> 
#include <cstring>  
#include <FlexCAN_T4.h>   
#include "Sig_Motor_Control.h"   

#define LOG_FILENAME "2024-12-17-Hip-Walking_Powered_06.csv" 

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;   

// *** for RingBuf *** //
#include "SdFat.h"
#include "RingBuf.h" 

// Use Teensy SDIO
#define SD_CONFIG SdioConfig(FIFO_SDIO)

// Interval between points for 25 ksps.
#define LOG_INTERVAL_USEC 2000 // 500 Hz

// Size to log 10 byte lines at 25 kHz for more than ten minutes.
// #define LOG_FILE_SIZE 10*25000*600  // Size to log 10 byte lines at 25 kHz for more than ten minutes = 150,000,000 bytes.
// #define LOG_FILE_SIZE 100 * 500 * 80 // Size to log 10 byte lines at 500 Hz for more than 80 seconds =  bytes.
#define LOG_FILE_SIZE 220 * 500 * 600 // Size to log 10 byte lines at 500 Hz for more than 80 seconds = 66,000,000 bytes.

// Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400*512 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
// #define RING_BUF_CAPACITY 400 * 512 * 10 / 50 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.
#define RING_BUF_CAPACITY 250 * 500 * 1 // Space to hold more than 800 ms of data for 10 byte lines at 25 ksps.

// #define LOG_FILENAME "Walking.csv"
//#define LOG_FILENAME "0407_Powered_Jennifer_Walking_bd0.04_0.00_kd_1_0.6_0_0_0_0_newtau.csv"
#define LOG_FILENAME "2022-05-24-Weibo-Walking_Powered_06.csv"
#include "sdlogger.h"
SdLogger logger(BUILTIN_SDCARD, F("walking_log_"), F(".csv"));



SdFs sd;
FsFile file;  
const float DEG2RAD = PI / 180.0f;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;    

// ads1292r torque_sensor1;                                      //Create torque sensor object see ads1292r.h

// Data logging
int isLogging = 0;   
int L_load_cell_torque = 0;  
int R_load_cell_torque = 0;    

/*Filter*/
MovingAverage LTAVx(12);    
MovingAverage RTAVx(12);     
float f_LTAVx = 0;  
float f_RTAVx = 0;  

CAN_message_t msgR;   
int CAN_ID = 3;  

int Sig_Motor_ID_1 = 1;     
int Sig_Motor_ID_2 = 0;       

double torque_command = 0;  
double velocity_command = 0;  
double position_command = 0;  

double M1_torque_command = 0;  
double M2_torque_command = 0;   

double RL_torque_command_1 = 0.0;    
double RL_torque_command_2 = 0.0;   

double MAX_torque_command = 8;   
double MIN_torque_command = -8;    

int LimitInf = -18;    
int LimitSup = 18;    

float p_des = 0;   
float v_des = 0;    
float kp = 0;   
float kd = 0;   
float t_ff = 0;   

/*MOTOR*/    
float initial_pos_1 = 0;       
float initial_pos_2 = 0;       

Motor_Control_Tmotor sig_m1(0x002, CAN_ID);   
Motor_Control_Tmotor sig_m2(0x001, CAN_ID);   
/*MOTOR*/  

/*Isra Serial Class Setup*/  
Serial_Com Serial_Com;   

/*Sensors Setup*/ 
IMU_Adapter imu;

/*Serial Send*/  
size_t Send_Length = 11; 
char Send[11] = { 0x31, 0x32, 0x32, 0x33, 0x33,
                  0x30, 0x31, 0x32, 0x33, 0x33,
                  0x33 };   

/*iMU SEND*/
uint16_t L_IMUX_int = 0x00;  
uint16_t R_IMUX_int = 0x00;   

uint16_t L_IMUV_int = 0x00;
uint16_t R_IMUV_int = 0x00; 

uint16_t L_CMD_int16 = 0x7fff;  
float L_CMD_serial   = 0.0;  

uint16_t R_CMD_int16 = 0x7fff;
float R_CMD_serial   = 0.0;

float IMUX_float = 0;   
float IMU11 = 0;   
float IMU22 = 0;   
float IMU33 = 0;  
float IMU44 = 0;  

/* Time control*/
// unsigned long Delta_T1 = 35;  //Looks like increasing this improves stability, but mkaes the torque less smooth
// unsigned long t_i, t_pr1;
// unsigned long beginning = 0;
double t;   
double next_t;    
double delta_t;    

//***For managing the Controller and Bluetooth rate
unsigned long t_0 = 0;
// double cyclespersec_ctrl = 28;  // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ctrl = 100;    // [Hz] teensy controller sample rate (Maximum frequency: 1000 Hz due to Can Bus) Controller must be faster than ble
double cyclespersec_ble  = 20;     // [Hz] Bluetooth sending data frequency 
unsigned long current_time = 0;
unsigned long previous_time = 0;                                           // used to control the controller sample rate.
unsigned long previous_time_ble = 0;                                       // used to control the Bluetooth communication frequency
unsigned long Tinterval_ctrl_micros = (unsigned long)(1000000 / cyclespersec_ctrl); // used to control the teensy controller frequency
unsigned long Tinterval_ble_micros  = (unsigned long)(1000000 / cyclespersec_ble);  // used to control the Bluetooth communication frequency
//**********************************

//***Data sent via bluetooth
char datalength_ble = 32;      // Bluetooth Data Length (32)
char data_ble[60] = {0};       // Data array for bluetooth data sending:  Teensy->RS232->Adafruit Feather nRF52840 Express(peripheral)->bluetooth->Adafruit Feather nRF52840 Express(central)->usb->computer
uint8_t data_rs232_rx[60] = {0};   // <-- char → uint8_t

int L_leg_IMU_angle = 0;       
int R_leg_IMU_angle = 0;  
int L_motor_torque  = 0;   
int R_motor_torque  = 0;  
int L_motor_torque_desired = 0;    
int R_motor_torque_desired = 0;   
int t_teensy = 0;  
int M_Selected = 0;  
int CtrlMode_Selected = 0;  

int GUI_force_cmd     = 0;  
int GUI_stiffness_cmd = 0;  
int GUI_damping_cmd   = 0;  
int GUI_assistive_ratio_cmd = 0;     

int GUI_pos_ampl_cmd    = 0;       
int GUI_pos_fre_cmd     = 0;       
int GUI_force_ampl_cmd  = 10;        
int GUI_force_fre_cmd   = 10;       

double GUI_force = 1.0;    
double GUI_K_p   = 1.0;    
double GUI_K_d   = 0.1;    

double assistive_ratio = 0.08;   

int L_pos_int_d = 0;     
int L_pos_int_a = 0;     
int L_vel_int_d = 0;       
int L_vel_int_a = 0;      
 
int R_pos_int_d = 0;     
int R_pos_int_a = 0;       
int R_vel_int_d = 0;         
int R_vel_int_a = 0;       
//**************************

double cmd_ampl = 1.0;        
double cmd_fre  = 1.0;      
float pos_ampl  = 0.0;      
float pos_fre   = 0.5;      

float l_pos_des = 0.0;    
float l_vel_des = 0.0;    
float r_pos_des = 0.0;     
float r_vel_des = 0.0;     

float ref_force_ampl = 0.2;    
float ref_force_fre = 0.5;    

float l_ref_tau    = 0.0;   
float l_ref_tau_dt = 0.0;    
float r_ref_tau    = 0.0;     
float r_ref_tau_dt = 0.0;    

float l_leg_angle    = 0.0;  
float r_leg_angle    = 0.0;  
float l_leg_velocity = 0.0;   
float r_leg_velocity = 0.0;  

//***Impedance Control Test***//  
float tau_imp = 0.0;      
float kp_imp = 1.0;     
float kd_imp = 0.01 * kp_imp;         
//***Impedance Control Test***//    

//***Torque Control Test */
float dt = 0.01;    

float tau_t_1 = 0.0;  
float tau_t_1_last = 0.0;    
float tau_t_2 = 0.0;    
float tau_t_2_last = 0.0;   

float tau_dt_1 = 0.0;    
float tau_dt_2 = 0.0;     

float tau_ff_1 = 0.0;      
float tau_ff_2 = 0.0;   

//*** Motor Mode Set ***//   
int ctl_method = 1;    // 0 for using RL controller, 1 for using other normal controller  
int ctl_mode = 0;      // 0 for torque control, 1 for mit control    
int ctl_type = 0;      // 0 for motion, 1 for force tracking, 2 for direct torque   

int sensor_type = 0;   // 0 for using IMU, 1 for using encoder   
int l_ctl_dir = 1;      //确实是左脚，1是向上
int r_ctl_dir = 1;     //确实是右脚，1是向上
float torque_cmd_scale = 20.0;   
//*** Motor Mode Set ***//    

int doi          = 0;   
int currentpoint = 0;    
int delayindex   = 0;    
// double Rescaling_gain    = 0.01; // 1.6 for max 4 Nm

double LTx_filtered      = 0;
double LTx_filtered_last = 0;
double RTx_filtered      = 0;
double RTx_filtered_last = 0;
double torque_filtered   = 0;
double RLTx              = 0;
double RLTx_filtered     = 0;
double RLTx_filtered_last = 0;  // ✅ 这个变量你还没定义
double RLTx_delay[100]   = {};
double torque_delay[100] = {};  

int L_motor_torque_command = 0;
int R_motor_torque_command = 0;

double Rescaling_gain   = 5;
double Flex_Assist_gain = 2.5;
double Ext_Assist_gain  = 2.5;

/* ======= ① 新增全局参数 ======= */
int8_t phase_offset_L = 0;   // << 你可以开 BLE 命令实时改
int8_t phase_offset_R = 0;
/* ============================== */


// double Rescaling_gain    = 0;
// double Flex_Assist_gain  = 0;
// double Ext_Assist_gain   = 0;
double Assist_delay_gain = 0;  

double S_torque_command_left = 0.0;    
double S_torque_command_right = 0.0;   
volatile uint8_t imu_init_ok = 0;    // 0=未通过, 1=通过（见下一节的自检逻辑）





/* ---------- 新增：标定 / 死区 / 滞环 ---------- */
float L0 = 0.0f, R0 = 0.0f;          // 开机零点
uint32_t nCal = 0;                   // 累计样本数
const float DEAD_TOR = 0.03f;        // 死区 (rad) 约 2°
const float CROSS_DEG = 20.0f;        // 滞环翻边阈值 (deg)
int8_t side = 0;                     // 当前步相位 -1/+1

// 把角度 low-pass 打包成函数，方便以后调系数
inline double lowpass(double x, double &y)
{
    y = 0.9 * y + 0.1 * x;
    return y;
}

// ≈8 byte 结构：0xAA 0x55  len  cmd  AngleX_L  AngleX_H  crc_L  crc_H
const uint8_t PKT_LEN = 8;

bool readPkt(HardwareSerial& port, float &angleDeg){
    static uint8_t buf[PKT_LEN];
    static uint8_t idx = 0;

    while (port.available()){
        uint8_t b = port.read();
        if (idx==0 && b!=0xAA)      continue;         // 找帧头
        if (idx==1 && b!=0x55){ idx=0; continue; }

        buf[idx++] = b;
        if (idx < PKT_LEN) continue;

        idx = 0;                                     // 准备下次

        /* —— 可选 CRC 校验 —— */

        int16_t raw = (buf[4] | (buf[5]<<8));        // little-endian
        angleDeg = raw * 0.01f;                      // 固件默认 0.01°
        return true;
    }
    return false;
}



//// setup can and motors ////
void setup() {
  delay(3000);   

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial5.begin(115200);    // BLE 串口
  Serial_Com.INIT();
  delay(300);


  memset(RLTx_delay,   0, sizeof(RLTx_delay));
  memset(torque_delay, 0, sizeof(torque_delay));


  initial_CAN();
  Serial.println("[OK] CAN bus setup");
  imu.INIT(); 
  Serial5.setTimeout(5);

  initial_Sig_motor();   
  Serial.println("initial_Sig_motor bus setup done...");  

  delay(2000);     
  imu.INIT_MEAN();    // 自动锁定 offset
  Serial.println("[OK] IMUSetup done");
  imu_init_ok = 1;    // 简单认为成功（INIT_MEAN 里会有 warn 打印）



  if (logger.begin()) {
    Serial.print(F("SD Logging file: "));
    Serial.println(logger.filename());
    logger.println(F("Time,imu_RTx,imu_LTx,RLTx_delay,torque_delay,"
                     "tau_raw_L,tau_raw_R,S_torque_command_left,"
                     "S_torque_command_right,M1_torque_command,"
                     "M2_torque_command,Rescaling_gain,Flex_Assist_gain,"
                     "Ext_Assist_gain,Assist_delay_gain"));
    logger.flush();
  } else {
    Serial.println(F("SD card init or file create failed!"));
  }

  t_0 = micros();    
}  

//// initial sig motor //// 
void initial_Sig_motor() {  
  // sig_m1.error_clear();    
  // delay(200); 

  // sig_m1.reboot();  
  // delay(200);  

  // sig_m1.sig_motor_reset();   
  // sig_m2.sig_motor_reset();   
  // delay(1000);  

  // // delay(1000);  
  // sig_m1.sig_encoder_reset();    
  // sig_m2.sig_encoder_reset();    
  // delay(10000);   

  /////////// set control mode /////////  
  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_mode_start();      
    sig_m2.sig_mit_ctl_mode_start();     
  } 
  else
  {
    sig_m1.sig_torque_ctl_mode_start();    
    delay(200);   
    sig_m2.sig_torque_ctl_mode_start();        
  } 
  delay(200);  
  
  sig_m1.sig_motor_start();    
  sig_m1.request_pos_vel();    
  delay(500);   

  sig_m2.sig_motor_start();    
  sig_m2.request_pos_vel();     
  delay(500);    

  if (ctl_mode == 1)  
  {
    sig_m1.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);     
    sig_m2.sig_mit_ctl_cmd(0.0, 0.0, 0.0, 0.0, 0.01);          
    receive_mit_ctl_feedback();     
  }
  else{
    // sig_m1.request_torque();   
    sig_m1.sig_torque_cmd(0.01);    
    delay(200);    
    // sig_m2.request_torque();   
    sig_m2.sig_torque_cmd(0.01);      
    delay(200);   
  } 

  for (int i =0; i < 1000; i++)
  {
    receive_torque_ctl_feedback();     
  }
  delay(1000);   

  initial_pos_1 = sig_m1.pos;     
  initial_pos_2 = sig_m2.pos;     

  delay(500);  

  /////// command initial setting ///////
  M1_torque_command = 0.0;         
  M2_torque_command = 0.0;         
}


// ==== Params (可由 BLE 覆盖) ====
volatile float  alpha_tau     = 0.85f;   // 一阶低通
volatile float  torque_rate   = 150.0f;  // Nm/s
volatile float max_torque_cfg = 0.0f;  // Nm，默认 15，通过 BLE 接收覆盖


// ---- Gate 参数（默认与仿真一致，可 BLE 改）----
volatile float  gate_k       = 8.0f;   // tanh 的 k
volatile float  gate_p_on    = 2.5;   // power-on 阈值 (τ·ω 的阈)
volatile uint8_t gate_lead_ms = 50;    // ★ 默认 20ms（可由 BLE 改）

// ---- Gate 输入预测需要的状态（每腿各一个）----
static float xL_prev = 0.0f;
static float xR_prev = 0.0f;


float tau_raw_L = 0.0f, tau_raw_R = 0.0f;

float tau_cmd_L_filt = 0.0f, tau_cmd_R_filt = 0.0f;
float M1_prev = 0.0f, M2_prev = 0.0f;

inline float clip_torque(float t){
  float max_abs = fabsf(max_torque_cfg);              // 允许 GUI 传 负数也能容错成绝对值
  return (t >  max_abs) ?  max_abs :
         (t < -max_abs) ? -max_abs : t;
}

// 0.5*(tanh(k*(x - p_on)) + 1)  带阈值的软门
inline float smooth_gate_p(float x, float k, float p_on){
  return 0.5f * (tanhf(k * (x - p_on)) + 1.0f);
}

// 一阶线性预测：用当前斜率估计 lead_s 后的 x
inline float lead_predict(float x, float x_prev, float lead_ms, float Ts){
  const float lead_s = 0.001f * lead_ms;      // ms -> s
  const float dx     = (x - x_prev) / Ts;     // 斜率
  return x + lead_s * dx;                     // 预测 x(t+lead_s)
}


inline float slew(float target, float prev, float rate, float Ts){
    float diff = target - prev, maxDiff = rate * Ts;
    if (fabs(diff) > maxDiff) target = prev + copysignf(maxDiff, diff);
    return target;
}

/****************************************/
// ===== Extension 复制/回放配置（按你要求的默认值）=====
static const bool  use_ext_copy   = true;   // 开关
static const int   ext_delay_ms   = 700;    // 延迟（ms）
static const float ext_gain       = 0.4f;   // 幅值增益
// 源信号选择：true=LPF 后的 S（与 Python 的 ext_src="lpf" 一致）；false=门后未滤波 target
static const bool  ext_from_lpf   = true;   // 本实现采用 LPF 后

// ===== 环形缓冲（仅存“屈相(负值)”）=====
#define EXT_BUF_LEN 400  // 足够覆盖更大的延迟（例如 Ts=10ms 时可覆盖4s）
static float flexL_hist[EXT_BUF_LEN] = {0.0f};
static float flexR_hist[EXT_BUF_LEN] = {0.0f};
static int   ext_i = 0; // 写指针（随周期递增）
/****************************************/

// === 自适应时间参数（按步态周期）===
volatile float lead_frac         = 0.06f;  // 门控提前比例: lead_ms = lead_frac * T_gait_ms
volatile float ext_delay_frac_L  = 0.35f;  // 左腿 extension 延迟比例
volatile float ext_delay_frac_R  = 0.35f;  // 右腿 extension 延迟比例

volatile bool  ext_enable_L = true;        // 只在左腿复制 extension
volatile bool  ext_enable_R = true;       // 右腿默认不复制

// 安全部分：周期范围与平滑
const float  TGAIT_MIN_MS   = 350.0f;      // 最小步态周期（快走/小跑）
const float  TGAIT_MAX_MS   = 1800.0f;     // 最大步态周期（慢走）
const float  TGAIT_SMOOTH_A = 0.35f;       // 周期 EMA 平滑系数

// 零交越滞回（防抖动）
const float  RL_HYST_DEG    = 3.0f;        // 过±3°才认作跨越

// 全局步态周期估计
static float    T_gait_ms   = 1000.0f;     // 初值 1s
static uint8_t  gait_inited = 0;
static float    RL_prev     = 0.0f;
static uint32_t t_last_cross= 0;

int8_t flex_sign_L = -1; // 左腿屈相抓负半波
int8_t flex_sign_R = +1; // 右腿屈相抓正半波


// —— 小工具：更新步态周期（基于 RLTx 的带滞回“负→正”过零）——
inline void update_gait_period(float RL_now_deg, uint32_t now_ms){
  // 触发条件：上帧 ≤ -h，本帧 ≥ +h，视为“负→正”跨越
  if (RL_prev <= -RL_HYST_DEG && RL_now_deg >= RL_HYST_DEG){
    if (t_last_cross != 0){
      float dt = (float)(now_ms - t_last_cross);
      if (dt >= TGAIT_MIN_MS && dt <= TGAIT_MAX_MS){
        if (!gait_inited){ T_gait_ms = dt; gait_inited = 1; }
        else { T_gait_ms = (1.0f - TGAIT_SMOOTH_A) * T_gait_ms + TGAIT_SMOOTH_A * dt; }
      }
    }
    t_last_cross = now_ms;
  }
  RL_prev = RL_now_deg;
}
void loop()
{
  imu.READ();
  Serial_Com.READ2();

  current_time = micros() - t_0;
  t = current_time / 1e6f;
  // === 简单失效/恢复判定 ===
  if (fabs(imu.RTx) > 80.0f || fabs(imu.LTx) > 80.0f) {
    if (imu_init_ok) {
      Serial.println("[WARN] IMU角度超过 ±80°，关闭辅助");
    }
    imu_init_ok = 0;
  } else {
    if (!imu_init_ok) {
      Serial.println("[OK] IMU角度回到安全范围，恢复辅助");
    }
    imu_init_ok = 1;
  }

  if (current_time - previous_time > Tinterval_ctrl_micros) {

    const float Ts = Tinterval_ctrl_micros / 1e6f;   // 控制周期

    /* BLE */
    if (current_time - previous_time_ble > Tinterval_ble_micros){
        Receive_ble_Data();
        Transmit_ble_Data();
        previous_time_ble = current_time;
    }


    /* --- 滤波角度 --- */
    RLTx = imu.RTx - imu.LTx;

    LTx_filtered_last = LTx_filtered;
    LTx_filtered      = 0.9f * LTx_filtered_last + 0.1f * imu.LTx;

    RTx_filtered_last = RTx_filtered;
    RTx_filtered      = 0.9f * RTx_filtered_last + 0.1f * imu.RTx;

    /* --- 角速度(°/s) ---> rad/s 更物理，不过比例因子无关宏旨 --- */
    const float LTx_vel = (LTx_filtered - LTx_filtered_last) * (PI/180.0f) / Ts;
    const float RTx_vel = (RTx_filtered - RTx_filtered_last) * (PI/180.0f) / Ts;

    /* --- 缓冲与延迟 --- */
    RLTx_filtered      = RTx_filtered - LTx_filtered;
    update_gait_period(RLTx_filtered, (uint32_t)(current_time / 1000));

    RLTx_delay[doi]    = RLTx_filtered;
    torque_filtered    = sinf(RTx_filtered*PI/180.0f) - sinf(LTx_filtered*PI/180.0f);
    torque_delay[doi]  = torque_filtered;


        /* ---- (A) 统一基准延迟 ---- */
    delayindex = doi - Assist_delay_gain;
    if (delayindex < 0)              delayindex += 100;
    else if (delayindex >= 100)      delayindex -= 100;

    /* ---- (B) 各腿再补相位 ---- */
    int idx_L = (delayindex + phase_offset_L + 100) % 100;
    int idx_R = (delayindex + phase_offset_R + 100) % 100;

    doi = (doi + 1) % 100;

    /* ---------- 原始对称扭矩 ---------- */
    tau_raw_L = tau_raw_R = 0.0f;

    if (fabs(RLTx_delay[idx_L]) <= 120.0f){
        tau_raw_L = (RLTx_delay[idx_L] >= 0 ?
                    -Rescaling_gain * Ext_Assist_gain :
                    +Rescaling_gain * Ext_Assist_gain ) * torque_delay[idx_L];
    }
    if (fabs(RLTx_delay[idx_R]) <= 120.0f){
        tau_raw_R = (RLTx_delay[idx_R] >= 0 ?
                    +Rescaling_gain * Flex_Assist_gain :
                    -Rescaling_gain * Flex_Assist_gain ) * torque_delay[idx_R];
    }

    /* --- 每腿功率门控 --- */
    // 原始功率 (注意右腿速度取负)
    const float xL_raw = tau_raw_L * LTx_vel;
    const float xR_raw = tau_raw_R * (-RTx_vel);

    // 预测超前（20ms 默认，可BLE覆盖）
    float lead_ms_dyn = gait_inited ? (lead_frac * T_gait_ms) : (float)gate_lead_ms;
    // 安全夹紧
    if (lead_ms_dyn < 5.0f)  lead_ms_dyn = 5.0f;
    if (lead_ms_dyn > 120.0f) lead_ms_dyn = 120.0f;
    
    const float xL_pred = lead_predict(xL_raw, xL_prev, lead_ms_dyn, Ts);
    const float xR_pred = lead_predict(xR_raw, xR_prev, lead_ms_dyn, Ts);
    
    // 更新“上一帧”状态（注意是 raw，用于斜率）
    xL_prev = xL_raw;
    xR_prev = xR_raw;

    // 带阈值的平滑门控
    const float gate_L = smooth_gate_p(xL_pred, gate_k, gate_p_on);
    const float gate_R = smooth_gate_p(xR_pred, gate_k, gate_p_on);

    // 门后扭矩
    S_torque_command_left  = tau_raw_L * gate_L;
    S_torque_command_right = tau_raw_R * gate_R;

    /* --- 一阶低通 --- */
    tau_cmd_L_filt = alpha_tau * tau_cmd_L_filt + (1 - alpha_tau) * S_torque_command_left;
    tau_cmd_R_filt = alpha_tau * tau_cmd_R_filt + (1 - alpha_tau) * S_torque_command_right;
    S_torque_command_left  = tau_cmd_L_filt;
    S_torque_command_right = tau_cmd_R_filt;

    // ---- 选择 extension 的复制源 ----
    float S_src_L = tau_cmd_L_filt;  // 按你之前的“lpf”源
    float S_src_R = tau_cmd_R_filt;

    // 仅保留“屈相（负值）”
    // float flexL_now = (S_src_L < 0.0f) ? S_src_L : 0.0f;
    // float flexR_now = (S_src_R < 0.0f) ? S_src_R : 0.0f;
    auto keep_if_flex = [](float S, int8_t sign){ return ((sign > 0) ? (S > 0.0f) : (S < 0.0f)) ? S : 0.0f; };
    float flexL_now = keep_if_flex(S_src_L, flex_sign_L);
    float flexR_now = keep_if_flex(S_src_R, flex_sign_R);
    
    // 写入环形缓冲（只存 flexion 负值）
    int w = ext_i % EXT_BUF_LEN;
    flexL_hist[w] = flexL_now;
    flexR_hist[w] = flexR_now;

    // —— 计算“自适应延迟”的样本数 ——
    // 若尚未估到步态周期，就退回你原来的固定 ms
    float ext_ms_L = gait_inited ? (ext_delay_frac_L * T_gait_ms) : (float)ext_delay_ms;
    float ext_ms_R = gait_inited ? (ext_delay_frac_R * T_gait_ms) : (float)ext_delay_ms;

    // 安全夹紧，避免极端值
    auto clampf = [](float x, float a, float b){ return (x<a)?a:((x>b)?b:x); };
    ext_ms_L = clampf(ext_ms_L, 30.0f, 1200.0f);
    ext_ms_R = clampf(ext_ms_R, 30.0f, 1200.0f);

    // 计算样本延迟数：N = round(ext_ms / Ts)
    int extN_L = (int)lrintf((ext_ms_L / 1000.0f) / Ts);
    int extN_R = (int)lrintf((ext_ms_R / 1000.0f) / Ts);
    if (extN_L >= EXT_BUF_LEN) extN_L = EXT_BUF_LEN - 1;
    if (extN_R >= EXT_BUF_LEN) extN_R = EXT_BUF_LEN - 1;
    if (extN_L < 0) extN_L = 0;
    if (extN_R < 0) extN_R = 0;

    // 读出延迟后的 flexion（复制为 extension = 取相反号 × 增益）
    int rL = ext_i - extN_L;
    int rR = ext_i - extN_R;
    // 做正模
    if (rL < 0) rL += ((-rL / EXT_BUF_LEN) + 1) * EXT_BUF_LEN;
    if (rR < 0) rR += ((-rR / EXT_BUF_LEN) + 1) * EXT_BUF_LEN;
    rL %= EXT_BUF_LEN;  rR %= EXT_BUF_LEN;

    float S_L_ext = 0.0f, S_R_ext = 0.0f;
    if (use_ext_copy) {
      if (ext_enable_L) S_L_ext = -ext_gain * flexL_hist[rL];
      if (ext_enable_R) S_R_ext = -ext_gain * flexR_hist[rR];
    }

    // 叠加得到最终 S
    S_torque_command_left  = S_src_L + S_L_ext;
    S_torque_command_right = S_src_R + S_R_ext;

    // 推进指针
    ext_i++;




    /* --- 斜率限幅 --- */
    M1_torque_command = S_torque_command_right * r_ctl_dir;
    M2_torque_command = S_torque_command_left  * l_ctl_dir;
    M1_torque_command = slew(M1_torque_command, M1_prev, torque_rate, Ts);
    M2_torque_command = slew(M2_torque_command, M2_prev, torque_rate, Ts);
    M1_prev = M1_torque_command;
    M2_prev = M2_torque_command;

    /* --- 饱和 --- */
    M1_torque_command = clip_torque(M1_torque_command);
    M2_torque_command = clip_torque(M2_torque_command);
    // if (!imu_init_ok) {
    //   M1_torque_command = 0;
    //   M2_torque_command = 0;
    // }

    /* --- 下发 --- */
    if (ctl_mode == 1){
        sig_m1.sig_mit_ctl_cmd(0.0,0.0,10.0,0.01,M1_torque_command);
        sig_m2.sig_mit_ctl_cmd(0.0,0.0,10.0,0.01,M2_torque_command);
        receive_mit_ctl_feedback();
    }else{
        for (int i=0;i<4;++i) receive_torque_ctl_feedback();
        sig_m1.sig_torque_cmd(M1_torque_command);
        sig_m2.sig_torque_cmd(M2_torque_command);
    }

    previous_time = current_time;
    // == 日志写入（100Hz）==
    if (logger.isOpen()) {
      logger.printf("%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        current_time / 1000,   // Time in ms
        imu.RTx,
        imu.LTx,
        RLTx_delay[doi],        // 当前帧
        torque_delay[doi],      // 当前帧
        tau_raw_L,
        tau_raw_R,
        S_torque_command_left,
        S_torque_command_right,
        M1_torque_command,
        M2_torque_command,
        Rescaling_gain,
        Flex_Assist_gain,
        Ext_Assist_gain,
        Assist_delay_gain
      );
      // logger.flush(); // 可每N次flush，或1s flush一次以保护SD卡寿命
    }
    static int log_flush_count = 0;
    if (++log_flush_count >= 10) {
      logger.flush();
      log_flush_count = 0;
    }
  }

#if 0
  Serial.printf("gateL=%.2f gateR=%.2f | τL %.2f τR %.2f\r\n",
                smooth_gate(tau_raw_L * (LTx_filtered - LTx_filtered_last)*(PI/180.0f)/(Tinterval_ctrl_micros/1e6f)),
                smooth_gate(tau_raw_R * (RTx_filtered - RTx_filtered_last)*(PI/180.0f)/(Tinterval_ctrl_micros/1e6f)),
                M2_torque_command, M1_torque_command);
#endif
}







void IMUSetup() {
  imu.INIT();   
  delay(1500);     
  // imu.INIT_MEAN();     
}  




void initial_CAN() {
  Can3.begin();
  // Can3.setBaudRate(1000000);  
  Can3.setBaudRate(1000000);  
  delay(400);  
  Serial.println("Can bus setup done...");  
  delay(200);  
}   

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



void SendIMUSerial()
{
  L_IMUX_int = Serial_Com.float_to_uint(l_leg_angle, -180, 180, 16);     
  R_IMUX_int = Serial_Com.float_to_uint(r_leg_angle, -180, 180, 16);     

  L_IMUV_int = Serial_Com.float_to_uint(l_leg_velocity, -800, 800, 16);     
  R_IMUV_int = Serial_Com.float_to_uint(r_leg_velocity, -800, 800, 16);         
  
  Send[0] = 0x31;  
  Send[1] = 0x32;  
  Send[2] = L_IMUX_int >> 8;
  Send[3] = L_IMUX_int & 0xFF;  
  Send[4] = R_IMUX_int >> 8;  
  Send[5] = R_IMUX_int & 0xFF;  
  Send[6] = L_IMUV_int >> 8;  
  Send[7] = L_IMUV_int & 0xFF;  
  Send[8] = R_IMUV_int >> 8;  
  Send[9] = R_IMUV_int & 0xFF;  
  Send[10] = 0x33;   
}


void Wait(unsigned long delay_control) {
  unsigned long Time_start = micros();
  unsigned long Time_Delta = delay_control;
  unsigned long Time_Control = 0;

  do {
    Time_Control = micros() - Time_start;
  } while (Time_Control < Time_Delta);
}

bool findFrameHeader()
{
    static uint8_t h[3]={0};
    while (Serial5.available()) {
        h[0]=h[1];  h[1]=h[2];  h[2]=Serial5.read();
        if(h[0]==165 && h[1]==90 && h[2]==20) return true;
    }
    return false;
}

void Receive_ble_Data()
{
    while (findFrameHeader()) {
        if (Serial5.available() < 17)
            return;

        Serial5.readBytes(data_rs232_rx, 17);

        Rescaling_gain    = int16_t(data_rs232_rx[0] | data_rs232_rx[1] << 8) / 100.0f;
        Flex_Assist_gain  = int16_t(data_rs232_rx[2] | data_rs232_rx[3] << 8) / 100.0f;
        Ext_Assist_gain   = int16_t(data_rs232_rx[4] | data_rs232_rx[5] << 8) / 100.0f;

        Rescaling_gain    = constrain(Rescaling_gain,    0.0f, 10.0f);
        Flex_Assist_gain  = constrain(Flex_Assist_gain,  0.0f, 10.0f);
        Ext_Assist_gain   = constrain(Ext_Assist_gain,   0.0f, 10.0f);

        Assist_delay_gain = data_rs232_rx[6];
        if (Assist_delay_gain > 99) Assist_delay_gain = 99;

        phase_offset_L = int8_t(data_rs232_rx[7]);
        phase_offset_R = int8_t(data_rs232_rx[8]);

        int16_t k100   = int16_t(data_rs232_rx[9]  | data_rs232_rx[10] << 8);
        int16_t pon100 = int16_t(data_rs232_rx[11] | data_rs232_rx[12] << 8);
        uint8_t leadms = data_rs232_rx[13];

        gate_k        = constrain(k100   / 100.0f, 0.1f, 20.0f);
        gate_p_on     = pon100 / 100.0f;
        gate_lead_ms  = (leadms > 200) ? 200 : leadms;

        // === 新增：max_torque (Nm) from GUI ===
        int16_t mt100 = int16_t(data_rs232_rx[14] | data_rs232_rx[15] << 8);
        float   mt    = mt100 / 100.0f;
        // 合理限幅（根据你硬件能力自行调整上限）
        max_torque_cfg = constrain(mt, 2.0f, 15.0f);

        Serial.printf("BLE: Res=%.2f Flex=%.2f Ext=%.2f | Assist=%u phaseL=%d phaseR=%d | k=%.2f p_on=%.2f lead=%ums | maxT=%.2fNm\n",
                      Rescaling_gain, Flex_Assist_gain, Ext_Assist_gain,
                      Assist_delay_gain, phase_offset_L, phase_offset_R,
                      gate_k, gate_p_on, gate_lead_ms, max_torque_cfg);
    }
}





void Transmit_ble_Data() {
  t_teensy        = t * 100;
  L_leg_IMU_angle = imu.LTx * 100;
  R_leg_IMU_angle = imu.RTx * 100;
  L_motor_torque  = sig_m1.torque * 100;
  R_motor_torque  = sig_m2.torque * 100;
  L_motor_torque_command = M1_torque_command *100;
  R_motor_torque_command = M2_torque_command *100;

  data_ble[0]  = 165;
  data_ble[1]  = 90;
  data_ble[2]  = datalength_ble;
  data_ble[3]  = t_teensy;
  data_ble[4]  = t_teensy >> 8;
  data_ble[5]  = L_leg_IMU_angle;
  data_ble[6]  = L_leg_IMU_angle >> 8;
  data_ble[7]  = R_leg_IMU_angle;
  data_ble[8]  = R_leg_IMU_angle >> 8;
  data_ble[9]  = L_load_cell_torque;
  data_ble[10] = L_load_cell_torque >> 8;  
  data_ble[11] = R_load_cell_torque;  
  data_ble[12] = R_load_cell_torque >> 8;  
  data_ble[13] = L_motor_torque_command;
  data_ble[14] = L_motor_torque_command >> 8;
  data_ble[15] = R_motor_torque_command;
  data_ble[16] = R_motor_torque_command >> 8;

  // === 新增：IMU 初始化状态 + 当前 max_torque ===
  data_ble[17] = imu_init_ok;   // 0 or 1

  int16_t mt100 = (int16_t)(max_torque_cfg * 100.0f);
  data_ble[18] = (uint8_t)(mt100 & 0xFF);
  data_ble[19] = (uint8_t)((mt100 >> 8) & 0xFF);

  // 其余保持 0（占位）
  data_ble[20] = 0;
  data_ble[21] = 0;
  data_ble[22] = 0;
  data_ble[23] = 0;
  data_ble[24] = 0;
  data_ble[25] = 0;
  data_ble[26] = 0;
  data_ble[27] = 0;
  data_ble[28] = 0;

  Serial5.write(data_ble, datalength_ble);
  if (Serial.availableForWrite() > 64) dumpBleTx();
}


/* ---------- 调试打印 ---------- */
void dumpBleRx() {
  Serial.print(F("[BLE-RX] k="));
  Serial.print(Rescaling_gain, 2);
  Serial.print(F(" Flex="));
  Serial.print(Flex_Assist_gain, 2);
  Serial.print(F(" Ext="));
  Serial.print(Ext_Assist_gain, 2);
  Serial.print(F(" delay="));
  Serial.println(Assist_delay_gain);
  Serial.printf("τL_cmd %.2f  τR_cmd %.2f\r\n",
    M2_torque_command, M1_torque_command);
}

void dumpBleTx() {
  Serial.print(F("[BLE-TX] t="));
  Serial.print(t, 3);
  Serial.print(F("  Lθ="));
  Serial.print(imu.LTx, 1);
  Serial.print(F("  Rθ="));
  Serial.print(imu.RTx, 1);
  Serial.print(F("  τL_cmd="));
  Serial.print(M2_torque_command, 2);
  Serial.print(F("  τR_cmd="));
  Serial.println(M1_torque_command, 2);
}


double derivative(double dt, double derivative_prev[], double *actual_in_ptr, double *prev_in_ptr){
  int i;
  double diff = 0.0, diff_sum = 0.0;
  if (dt != 0.0){
    for (i = 0; i < 3; i++){
      diff_sum += derivative_prev[i];
    }
    diff = (diff_sum + (*actual_in_ptr - *prev_in_ptr) / dt) / (i + 1);
  } else 
    diff = derivative_prev[3];
  return diff;
}

void print_Data_Ivan() {
  Serial.print(t);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(sig_m1.torque);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  //Serial.print(sig_m2.torque);
  Serial.print(" ");
  Serial.println(" ");
}  

void print_Data_Jimmy() {
  Serial.print(t);
  Serial.print("Please give "); 
  // Serial.print(GUI_K);    
  // Serial.print(" ");   
  // Serial.print(imu.LTAVx);  
  // Serial.print(" ");
  // Serial.print(sig_m1.torque); 
  // Serial.print(" ");
  // Serial.print(sig_m1.pos);  
  // Serial.print(" ");
  // Serial.print(sig_m1.spe);    
  // Serial.print(" ");
  // Serial.print(L_CMD_serial);  
  //Serial.print(sig_m2.torque);  
  Serial.print(" "); 
  Serial.println(" ");  
}

void print_Data() {
  //  Serial.print(-20);
  //  Serial.print(" ");
  //  Serial.print(20);
  //  Serial.print(" ");  
  //  Serial.print(imu.RTx);
  //  Serial.print(" ");
  //  Serial.print(f_RTAVx);
  //  Serial.print(" ");  
  //  Serial.print(IMU22);
  //  Serial.print(" ");
  //  Serial.print( t_i / 1000 );
  //  Serial.print(" ");
  //  Serial.print(imu.RTx / 5);
  //  Serial.print(" ");
  Serial.print(imu.RTAVx / 10);
  Serial.print(" ");
  //  Serial.print(M1_torque_command);
  //  Serial.print(" ");
  //Serial.print(R_CMD_serial);//Received by Python from serial usb (commanded by the NN)
  //Serial.print(" ");

  // Serial.print(imu.LTx / 5);
  // Serial.print(" ");
  Serial.print(imu.LTAVx / 10);
  Serial.print(" ");
  Serial.print(sig_m2.torque);//Why is the sign opposite to m1?
  Serial.print(" ");  
  //  Serial.print(-M2_torque_command); // The one we send to the motor after R-CMD-SERIAL IS RECEIVED. Should be same as R_CMD_serial, unless saturation
  //  Serial.print(" ");
  //Serial.print(-sig_m2.torque);  //Feedback torque from the motor (estimated with current)
  Serial.print(M2_torque_command);
  Serial.print(" ");

  Serial.print(sig_m1.torque);
  Serial.print(" ");
  Serial.print(M1_torque_command);
  Serial.print(" ");
  Serial.print(LimitInf);
  Serial.print(" ");
  Serial.print(LimitSup);
  Serial.print(" ");   

  Serial.println(" ");
}

void print_Data_IMU() {
  Serial.print(-180);
  Serial.print(" ");
  Serial.print(180);
  Serial.print(" ");
  //  Serial.print(IMU22);
  Serial.print(" ");
  Serial.print(imu.LTx);
  Serial.print(" ");
  Serial.print(imu.LTAVx);
  Serial.print(" ");
  Serial.print(imu.RTx);
  Serial.print(" ");
  Serial.print(imu.RTAVx);
  Serial.println(" ");
}

void print_Data_Received() {
  Serial.print(20);
  Serial.print(" ");
  Serial.print(-20);
  Serial.print(" ");
  Serial.print(L_CMD_serial);
  Serial.print(" ");
  Serial.print(R_CMD_serial);
  Serial.print(" ");
  Serial.println(" ");  
}

void print_data_motor() {
  //  double v1 = 90;
  //  double v2 = -v1;
  //  Serial.print(v1);
  //  Serial.print("   ");
  //Serial.print(current_time);
  Serial.print(" ; ");
  Serial.print(" M1_tor ; "); //M1 is left, M2 is right
  Serial.print(sig_m1.torque);    
  Serial.print(" ; M1_cmd ; ");   
  Serial.print(M1_torque_command);   
  Serial.print(" ; M2_tor ; ");  
  Serial.print(sig_m2.torque);  
  Serial.print(" ; M2_cmd ; ");   
  Serial.print(M2_torque_command);
  Serial.print(" ; M1_pos ; ");
  Serial.print(sig_m1.pos);
  Serial.println(" ;  ");
}  



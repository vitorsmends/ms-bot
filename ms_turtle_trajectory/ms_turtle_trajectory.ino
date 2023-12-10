#include <Dynamixel2Arduino.h>

int motor_speed[] = {0,0};


// Please modify it to suit your hardware.
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using DynamixelShield
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL soft_serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_DUE) // When using DynamixelShield
  #define DXL_SERIAL   Serial
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_SAM_ZERO) // When using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL SerialUSB
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#elif defined(ARDUINO_OpenCM904) // When using official ROBOTIS board with DXL circuit.
  #define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
#elif defined(ARDUINO_OpenCR) // When using official ROBOTIS board with DXL circuit.
  // For OpenCR, there is a DXL Power Enable pin, so you must initialize and control it.
  // Reference link : https://github.com/ROBOTIS-GIT/OpenCR/blob/master/arduino/opencr_arduino/opencr/libraries/DynamixelSDK/src/dynamixel_sdk/port_handler_arduino.cpp#L78
  #define DXL_SERIAL   Serial3
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 84; // OpenCR Board's DIR PIN.    
#elif defined(ARDUINO_OpenRB)  // When using OpenRB-150
  //OpenRB does not require the DIR control pin.
  #define DXL_SERIAL Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = -1;
#else // Other boards when using DynamixelShield
  #define DXL_SERIAL   Serial1
  #define DEBUG_SERIAL Serial
  const int DXL_DIR_PIN = 2; // DYNAMIXEL Shield DIR PIN
#endif


const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];

const uint16_t SR_START_ADDR = 126; // Starting Data Addr, Can differ Depending on what address to access
const uint16_t SR_ADDR_LEN = 10; // Data Length (2+4+4), Can differ depending on how many address to access. 
const uint16_t SW_START_ADDR = 104; 
const uint16_t SW_ADDR_LEN = 4;
typedef struct sr_data{
  int16_t present_current;
  int32_t present_velocity;
  int32_t present_position;
} __attribute__((packed)) sr_data_t;
typedef struct sw_data{
  int32_t goal_velocity;
} __attribute__((packed)) sw_data_t;

static uint32_t try_count = 0;
uint8_t i, recv_cnt;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  Serial1.begin(19200);
  uint8_t i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(115200);
  dxl.begin(1000000);
  dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  for(i=0; i<DXL_ID_CNT; i++){
    dxl.torqueOff(DXL_ID_LIST[i]);
    dxl.setOperatingMode(DXL_ID_LIST[i], OP_VELOCITY);
  }
  dxl.torqueOn(BROADCAST_ID);

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos.packet.p_buf = user_pkt_buf;
  sr_infos.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos.packet.is_completed = false;
  sr_infos.addr = SR_START_ADDR;
  sr_infos.addr_length = SR_ADDR_LEN;
  sr_infos.p_xels = info_xels_sr;
  sr_infos.xel_count = 0;  

  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sr[i].id = DXL_ID_LIST[i];
    info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
    sr_infos.xel_count++;
  }
  sr_infos.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos.packet.p_buf = nullptr;
  sw_infos.packet.is_completed = false;
  sw_infos.addr = SW_START_ADDR;
  sw_infos.addr_length = SW_ADDR_LEN;
  sw_infos.p_xels = info_xels_sw;
  sw_infos.xel_count = 0;

  sw_data[0].goal_velocity = 0;
  sw_data[1].goal_velocity = 0;
  
  for(i=0; i<DXL_ID_CNT; i++){
    info_xels_sw[i].id = DXL_ID_LIST[i];
    info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_velocity;
    sw_infos.xel_count++;
  }
  sw_infos.is_info_changed = true;
}

void loop() {
    if (Serial1.available() > 0) {
      // state machine
      if (Serial1.read() == 1){
        square(); 
      }
      else if (Serial1.read() == 2){
        circle();
      }
      else{
        motor_stop(0.001);
      }
    }
}

void square(){
  move_foward(); 
  motor_stop(1);
  move_left();
  motor_stop(1);
}

void circle(){
  move_left();
}

void move_foward(){
  motor_speed[0] = 50;
  motor_speed[1] = 50;
  set_motor_speed(motor_speed,3);
}

void move_left(){
  motor_speed[0] = 25;
  motor_speed[1] = -25;
  set_motor_speed(motor_speed, 1);
}

void motor_stop(int stop_time){
  motor_speed[0] = 0;
  motor_speed[1] = 0;
  set_motor_speed(motor_speed, stop_time);
}
void set_motor_speed(int *motor_speed, int motor_time){
  
  for(i = 0; i < DXL_ID_CNT; i++){
    sw_data[i].goal_velocity = motor_speed[i];
    if(sw_data[i].goal_velocity > 50){
      sw_data[i].goal_velocity = 0;
    }
  }
  sw_infos.is_info_changed = true;

  feedback();
  delay(motor_time*1000);
}

// -----------------------------------------------------------
void feedback(){
  DEBUG_SERIAL.print("\n>>>>>> Sync Instruction Test : ");
  DEBUG_SERIAL.println(try_count++);
  if(dxl.syncWrite(&sw_infos) == true){
    DEBUG_SERIAL.println("[SyncWrite] Success");
    for(i=0; i<sw_infos.xel_count; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.println(sw_infos.p_xels[i].id);
      DEBUG_SERIAL.print("\t Goal Velocity: ");DEBUG_SERIAL.println(sw_data[i].goal_velocity);
    }
  }else{
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.print(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println();

//  delay(250);

  recv_cnt = dxl.syncRead(&sr_infos);
  if(recv_cnt > 0){
    DEBUG_SERIAL.print("[SyncRead] Success, Received ID Count: ");
    DEBUG_SERIAL.println(recv_cnt);
    for(i=0; i<recv_cnt; i++){
      DEBUG_SERIAL.print("  ID: ");DEBUG_SERIAL.print(sr_infos.p_xels[i].id);
      DEBUG_SERIAL.print(", Error: ");DEBUG_SERIAL.println(sr_infos.p_xels[i].error);
      DEBUG_SERIAL.print("\t Present Current: ");DEBUG_SERIAL.println(sr_data[i].present_current);
      DEBUG_SERIAL.print("\t Present Velocity: ");DEBUG_SERIAL.println(sr_data[i].present_velocity);
      DEBUG_SERIAL.print("\t Present Position: ");DEBUG_SERIAL.println(sr_data[i].present_position);
    }
  }else{
    DEBUG_SERIAL.print("[SyncRead] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  DEBUG_SERIAL.println("=======================================================");

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
//  delay(750);
}

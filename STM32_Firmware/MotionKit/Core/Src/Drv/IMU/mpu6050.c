#include <math.h>
#include "mpu6050.h"

#define CHIP_RESET_DELAY        3
#define FIFO_RESET_DELAY        3
#define FIFO_MAX_PACKET_SIZE    12
#define DMP_FIRMWARE_CHUNK_SIZE 16
#define SENSOR_INIT_DELAY       50
#define BYPASS_SETTING_DELAY    3
#define SELF_TEST_TRIES         2

static uint32_t test_gyro_sens    = 32768/250;
static uint32_t test_accel_sens   = 32768/16;
static uint8_t test_reg_rate_div  = 0;
static uint8_t test_reg_lpf       = 1;
static uint8_t test_reg_gyro_fsr  = 0;
static uint8_t test_reg_accel_fsr = 0x18;
static uint16_t test_wait_ms      = 50;
static uint8_t packet_thresh      = 5;
static float min_dps              = 10.0f;
static float max_dps              = 105.0f;
static float max_gyro_var         = 0.14f;
static float min_gravity          = 0.3f;
static float max_gravity          = 0.95f;
static float max_accel_var        = 0.14f;

const int8_t MPU6050_ORIENTATION_DEFAULT[9] = {1,0,0,
                                               0,1,0,
                                               0,0,1};

void (*MPU6050_Log)(const char* message) = NULL;

#define LOG MPU6050_Log

/*============================Self Definable============================*/

__weak MPU6050_State MPU6050_Init(MPU6050* target, MPU6050_DMP* dmp){
  if(target->addr!=0x68&&target->addr!=0x69) target->addr = MPU6050_I2C_ADDR;
  target->addr<<=1;
  if(dmp!=NULL) target->dmp = dmp;
  else target->dmp = (MPU6050_DMP*)malloc(sizeof(MPU6050_DMP)*1);

  uint8_t data[6],rev;
  //reset device
  data[0] = MPU6050_BIT_RESET;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,data,1)!=HAL_OK) {
    if(LOG!=NULL) LOG("MPU_INIT: mpu reset fail\n");
    return MPU6050_TRANS_FAIL;
  }
  osDelay(CHIP_RESET_DELAY*1000);
  //wakeup device
  data[0] = 0x00;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,data,1)!=HAL_OK) {
    if(LOG!=NULL) LOG("MPU_INIT: mpu wakeup fail\n");
    return MPU6050_TRANS_FAIL;
  }
  //check product revision
  if(MPU6050_Read(target->addr,MPU6050_REG_ACCEL_OFFS,data,6)!=HAL_OK) {
    if(LOG!=NULL) LOG("MPU_INIT: read accel_offset fail\n");
    return MPU6050_TRANS_FAIL;
  }
  rev = ((data[5]&0x01)<<2)|((data[3]&0x01)<<1)|(data[1]&0x01);
  if(rev!=0){
    if(rev==1) target->accel_half = 1;
    else if(rev==2) target->accel_half = 0;
    else {
      if(LOG!=NULL) LOG("MPU_INIT: unsupported product\n");
      return MPU6050_INCORRECT;
    }
  }else{
    if(MPU6050_Read(target->addr,MPU6050_REG_PRODUCT_ID,data,1)!=HAL_OK) {
      if(LOG!=NULL) LOG("MPU_INIT: read prod_id fail\n");
      return MPU6050_TRANS_FAIL;
    }
    rev = data[0]&0x0F;
    if(rev==0) {
      if(LOG!=NULL) LOG("MPU_INIT: incompatible device\n");
      return MPU6050_INCORRECT;
    }else if(rev==4) target->accel_half = 1;
    else target->accel_half = 0;
  }
  //pre-load data
  target->sensors = 0xFF;
  target->gyro_fsr = 0xFFFF;
  target->accel_fsr = 0xFF;
  target->lpf = 0xFFFF;
  target->sample_rate = 0xFFFF;
  target->fifo_sensors = 0xFF;
  target->bypass_en = 0xFF;
  target->clock_source = MPU6050_CLK_PLL;
  target->active_low_int = 1;
  target->int_latched = 0;
  target->lp_accel_mode = 0;
  target->dmp_en = 0;
  target->dmp_loaded = 0;
  target->dmp_sample_rate = 0x0000;

  MPU6050_State status;
  //gyroscope fsr = 2000dps
  status = MPU6050_SetGyroFSR(target,2000);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set gyroscope fsr error\n");return status;}
  //accelerator fsr = 2g
  status = MPU6050_SetAccelFSR(target,2);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set accelerator fsr error\n");return status;}
  //low-pass filter = 42Hz
  status = MPU6050_SetLPF(target,42);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set low-pass filter error\n");return status;}
  //chip sample rate = 50Hz
  status = MPU6050_SetSampleRate(target,50);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set sample rate error\n");return status;}
  //Disable all FIFO
  status = MPU6050_SetConfigFIFO(target,0x00);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: config fifo error\n");return status;}
  //Disable bypass mode
  status = MPU6050_SetBypassMode(target,0);
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set bypass mode error\n");return status;}
  //Launch XYZ accelerator and XYZ gyroscope
  status = MPU6050_SetSensors(target,0x00);//or (MPU6050_SENS_ACCEL|MPU6050_SENS_GYRO_ALL)
  if(status!=MPU6050_ACCEPTED) {if(LOG!=NULL) LOG("MPU_INIT: set sensors error\n");return status;}
  if(LOG!=NULL) LOG("MPU_INIT: passed\n");
  return MPU6050_ACCEPTED;
}

__weak MPU6050_State MPU6050_Calibrate(MPU6050* target,uint8_t load_gyro,uint8_t load_accel){
  uint8_t result;
  int32_t gyro[3],accel[3];
  MPU6050_State status = MPU6050_SelfTest(target,gyro,accel,&result);
  if(status!=MPU6050_ACCEPTED) return status;
  if(result!=0x03) return MPU6050_INCORRECT;


  float gyro_sens=0;
  if(load_gyro){
    status = MPU6050_CalcGyroSens(target,&gyro_sens);
    if(status!=MPU6050_ACCEPTED) return status;
  }
  gyro[0] = (int32_t)((float)gyro[0]*gyro_sens);
  gyro[1] = (int32_t)((float)gyro[1]*gyro_sens);
  gyro[2] = (int32_t)((float)gyro[2]*gyro_sens);
  status = MPU6050_DMP_SetGyroBias(target,gyro);
  if(status!=MPU6050_ACCEPTED) return status;

  float accel_sens=0;
  if(load_accel){
    status = MPU6050_CalcAccelSens(target,&accel_sens);
    if(status!=MPU6050_ACCEPTED) return status;
  }
  accel[0] = (int32_t)((float)accel[0]*accel_sens);
  accel[1] = (int32_t)((float)accel[1]*accel_sens);
  accel[2] = (int32_t)((float)accel[2]*accel_sens);
  status = MPU6050_DMP_SetAccelBias(target,accel);
  if(status!=MPU6050_ACCEPTED) return status;

  return MPU6050_ACCEPTED;
}

void MPU6050_SetLog(void (*func)(const char* message)){
  MPU6050_Log = func;
}

/*===========================Configuration API==========================*/

MPU6050_State MPU6050_SetSensors(MPU6050* target,uint8_t sensors){
  uint8_t data;
  if(sensors&MPU6050_SENS_GYRO_ALL) data = MPU6050_CLK_PLL;
  else if(sensors) data = MPU6050_CLK_INTERNAL;
  else data = MPU6050_BIT_SLEEP;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,&data,1)!=HAL_OK){
    target->sensors = 0x00;
    return MPU6050_TRANS_FAIL;
  }

  data=0x00;
  if(!(sensors&MPU6050_SENS_GYRO_X)) data|=MPU6050_BIT_STBY_XG;
  if(!(sensors&MPU6050_SENS_GYRO_Y)) data|=MPU6050_BIT_STBY_YG;
  if(!(sensors&MPU6050_SENS_GYRO_Z)) data|=MPU6050_BIT_STBY_ZG;
  if(!(sensors&MPU6050_SENS_ACCEL )) data|=MPU6050_BIT_STBY_XYZA;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT2,&data,1)!=HAL_OK){
    target->sensors = 0x00;
    return MPU6050_TRANS_FAIL;
  }


  if(sensors&&(sensors!=MPU6050_SENS_ACCEL)){
    MPU6050_State status = MPU6050_SetIntLatched(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
  }
  target->sensors = sensors;
  target->lp_accel_mode = 0;
  osDelay(SENSOR_INIT_DELAY*1000);
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetBypassMode(MPU6050* target,uint8_t enable){
  if(target->bypass_en==enable) return MPU6050_ACCEPTED;
  uint8_t tmp;

  if(enable){
    if(MPU6050_Read(target->addr,MPU6050_REG_USER_CTRL,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    tmp &= ~MPU6050_BIT_AUX_IF_EN;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    osDelay(BYPASS_SETTING_DELAY*1000);

    tmp = MPU6050_BIT_BYPASS_EN;
    if(target->active_low_int) tmp |= MPU6050_BIT_ACTL;
    if(target->int_latched) tmp |= (MPU6050_BIT_LATCH_EN|MPU6050_BIT_ANY_RD_CLR);
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_PIN_CFG,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
  }else{
    if(MPU6050_Read(target->addr,MPU6050_REG_USER_CTRL,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    if(target->sensors&MPU6050_SENS_COMPASS) tmp |= MPU6050_BIT_AUX_IF_EN;
    else tmp &= ~MPU6050_BIT_AUX_IF_EN;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    osDelay(BYPASS_SETTING_DELAY*1000);

    tmp = target->active_low_int?MPU6050_BIT_ACTL:0x00;
    if(target->int_latched) tmp |= (MPU6050_BIT_LATCH_EN|MPU6050_BIT_ANY_RD_CLR);
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_PIN_CFG,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
  }

  target->bypass_en = enable;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetIntEnable(MPU6050* target,uint8_t enable){
  uint8_t tmp;
  if(target->dmp_en){
    tmp = enable?MPU6050_BIT_DMP_INT_EN:0x00;
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    target->int_en = tmp;
  }else{
    if(!target->sensors) return MPU6050_ERROR;
    if(enable&&target->int_en) return MPU6050_ACCEPTED;
    tmp = enable?MPU6050_BIT_DATA_RDY_EN:0x00;
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    target->int_en = tmp;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetIntLevel(MPU6050* target,uint8_t active_low){
  target->active_low_int = active_low;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetIntLatched(MPU6050* target,uint8_t enable){
  if(target->int_latched==enable) return MPU6050_ACCEPTED;
  uint8_t tmp;

  if(enable) tmp = MPU6050_BIT_LATCH_EN | MPU6050_BIT_ANY_RD_CLR;
  else tmp = 0;
  if(target->bypass_en) tmp |= MPU6050_BIT_BYPASS_EN;
  if(target->active_low_int) tmp |= MPU6050_BIT_ACTL;
  if(MPU6050_Write(target->addr,MPU6050_REG_INT_PIN_CFG,&tmp,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->int_latched = enable;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetLpAccelMode(MPU6050* target,uint8_t rate){
  if(rate>40) return MPU6050_INCORRECT;
  uint8_t tmp[2];
  MPU6050_State status;
  if(rate==0){
    status = MPU6050_SetIntLatched(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
    tmp[0] = 0;
    tmp[1] = MPU6050_BIT_STBY_XYZG;
    if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,tmp,2)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    target->lp_accel_mode = 0;
    return MPU6050_ACCEPTED;
  }

  status = MPU6050_SetIntLatched(target,1);
  if(status!=MPU6050_ACCEPTED) return status;
  tmp[0] = MPU6050_BIT_LPA_CYCLE;
  if(rate==1){
    tmp[1] = MPU6050_LPA_1_25HZ;
    status = MPU6050_SetLPF(target,5);
  }else if(rate<=5){
    tmp[1] = MPU6050_LPA_5HZ;
    status = MPU6050_SetLPF(target,5);
  }else if(rate<=20){
    tmp[1] = MPU6050_LPA_20HZ;
    status = MPU6050_SetLPF(target,10);
  }else{
    tmp[1] = MPU6050_LPA_40HZ;
    status = MPU6050_SetLPF(target,20);
  }
  if(status!=MPU6050_ACCEPTED) return status;
  tmp[1] = (tmp[1]<<6)|MPU6050_BIT_STBY_XYZG;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,tmp,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;

  target->sensors = MPU6050_SENS_ACCEL;
  target->lp_accel_mode = 1;
  target->clock_source = MPU6050_CLK_INTERNAL;
  status = MPU6050_SetConfigFIFO(target,0x00);
  return status;
}

MPU6050_State MPU6050_SetStateDMP(MPU6050* target,uint8_t enable){
  if(target->dmp_en==enable) return MPU6050_ACCEPTED;
  uint8_t tmp;
  MPU6050_State status;

  if(enable){
    if(!target->dmp_loaded) return MPU6050_INCORRECT;
    status = MPU6050_SetIntEnable(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_SetBypassMode(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_SetSampleRate(target,target->dmp_sample_rate);
    if(status!=MPU6050_ACCEPTED) return status;
    tmp=0x00;
    if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    target->dmp_en = 1;
    status = MPU6050_SetIntEnable(target,1);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_ResetFIFO(target);
  }else{
    status = MPU6050_SetIntEnable(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
    tmp = target->fifo_sensors;
    if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,&tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    target->dmp_en = 0;
    status = MPU6050_ResetFIFO(target);
  }

  return status;
}

MPU6050_State MPU6050_GetStateDMP(MPU6050* target,uint8_t* enable){
  enable[0] = target->dmp_en;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetConfigFIFO(MPU6050*target, uint8_t sensor){
  if(!target->sensors) return MPU6050_ERROR;
  if(target->dmp_en) return MPU6050_ACCEPTED;
  uint8_t prev;
  MPU6050_State res,status;
  sensor &= ~MPU6050_SENS_COMPASS;

  prev = target->fifo_sensors;
  target->fifo_sensors = sensor&target->sensors;
  if(target->fifo_sensors!=sensor) res = MPU6050_ERROR;
  else res = MPU6050_ACCEPTED;

  if(sensor||target->lp_accel_mode) status = MPU6050_SetIntEnable(target,1);
  else status = MPU6050_SetIntEnable(target,0);
  if(status!=MPU6050_ACCEPTED) return status;

  if(sensor&& MPU6050_ResetFIFO(target)!=MPU6050_ACCEPTED){
    target->fifo_sensors = prev;
    return MPU6050_ERROR;
  }
  return res;
}

MPU6050_State MPU6050_GetConfigFIFO(MPU6050* target,uint8_t* sensor){
  sensor[0] = target-> fifo_sensors;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetLPF(MPU6050* target,uint16_t lpf){
  uint8_t data;
  if(!target->sensors) return MPU6050_ERROR;

  if(lpf==0) data = MPU6050_FILTER_256HZ_NOLPF2;
  else if(lpf>=188) data = MPU6050_FILTER_188HZ;
  else if(lpf>=98) data = MPU6050_FILTER_98HZ;
  else if(lpf>=42) data = MPU6050_FILTER_42HZ;
  else if(lpf>=20) data = MPU6050_FILTER_20HZ;
  else if(lpf>=10) data = MPU6050_FILTER_10HZ;
  else data = MPU6050_FILTER_5HZ;

  if(target->lpf==data) return MPU6050_ACCEPTED;
  if(MPU6050_Write(target->addr,MPU6050_REG_LPF,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->lpf = data;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetLPF(MPU6050* target,uint16_t* lpf){
  switch(target->lpf){
    case MPU6050_FILTER_188HZ : lpf[0] = 188;
                                break;
    case MPU6050_FILTER_98HZ  : lpf[0] = 98;
                                break;
    case MPU6050_FILTER_42HZ  : lpf[0] = 42;
                                break;
    case MPU6050_FILTER_20HZ  : lpf[0] = 20;
                                break;
    case MPU6050_FILTER_10HZ  : lpf[0] = 10;
                                break;
    case MPU6050_FILTER_5HZ   : lpf[0] = 5;
                                break;
    case MPU6050_FILTER_2100HZ_NOLPF:
    case MPU6050_FILTER_256HZ_NOLPF2:
    default : lpf[0] = 0;break;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetSampleRate(MPU6050* target,uint16_t rate){
  if(!target->sensors) return MPU6050_ERROR;
  if(target->dmp_en) return MPU6050_INCORRECT;

  uint8_t data;
  MPU6050_State status;
  if(target->lp_accel_mode){
      if(rate&&rate<=40){
        status = MPU6050_SetLpAccelMode(target,rate);
        return status;
      }
    status = MPU6050_SetLpAccelMode(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
  }
  if(rate<4) rate=4;
  if(rate>1000) rate=1000;
  data = 1000/rate-1;
  if(MPU6050_Write(target->addr,MPU6050_REG_RATE_DIV,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->sample_rate = rate;
  status = MPU6050_SetLPF(target,target->sample_rate>>1);
  return status;
}

MPU6050_State MPU6050_GetSampleRate(MPU6050* target,uint16_t* rate){
  if(target->dmp_en) return MPU6050_INCORRECT;
  rate[0] = target->sample_rate;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetGyroFSR(MPU6050* target,uint16_t fsr){
  if(!target->sensors) return MPU6050_ERROR;

  uint8_t data;
  switch(fsr){
    case 250  : data = MPU6050_FSR_250DPS<<3;
                break;
    case 500  : data = MPU6050_FSR_500DPS<<3;
                break;
    case 1000 : data = MPU6050_FSR_1000DPS<<3;
                break;
    case 2000 : data = MPU6050_FSR_2000DPS<<3;
                break;
    default : return MPU6050_INCORRECT;
  }

  if(target->gyro_fsr==(data>>3)) return MPU6050_ACCEPTED;
  if(MPU6050_Write(target->addr,MPU6050_REG_GYRO_CFG,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->gyro_fsr = data>>3;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetGyroFSR(MPU6050* target,uint16_t* fsr){
  switch(target->gyro_fsr){
    case MPU6050_FSR_250DPS  :  fsr[0] = 250;
                                break;
    case MPU6050_FSR_500DPS  :  fsr[0] = 500;
                                break;
    case MPU6050_FSR_1000DPS :  fsr[0] = 1000;
                                break;
    case MPU6050_FSR_2000DPS :  fsr[0] = 2000;
                                break;
    default : fsr[0]=0;return MPU6050_INCORRECT;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_SetAccelFSR(MPU6050* target,uint8_t fsr){
  if(!target->sensors) return MPU6050_ERROR;

  uint8_t data;
  switch(fsr){
    case 2  : data = MPU6050_FSR_2G<<3;
              break;
    case 4  : data = MPU6050_FSR_4G<<3;
              break;
    case 8  : data = MPU6050_FSR_8G<<3;
              break;
    case 16 : data = MPU6050_FSR_16G<<3;
              break;
    default : return MPU6050_INCORRECT;
  }

  if(target->accel_fsr==(data>>3)) return MPU6050_ACCEPTED;
  if(MPU6050_Write(target->addr,MPU6050_REG_ACCEL_CFG,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->accel_fsr = data>>3;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetAccelFSR(MPU6050* target,uint8_t* fsr){
  switch(target->accel_fsr){
    case MPU6050_FSR_2G  :  fsr[0] = 2;
                            break;
    case MPU6050_FSR_4G  :  fsr[0] = 4;
                            break;
    case MPU6050_FSR_8G  :  fsr[0] = 8;
                            break;
    case MPU6050_FSR_16G :  fsr[0] = 16;
                            break;
    default : fsr[0]=0;return MPU6050_INCORRECT;
  }
  if(target->lp_accel_mode) fsr[0]<<=1;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_CalcGyroSens(MPU6050* target,float* scale){
  uint16_t fsr;
  if(MPU6050_GetGyroFSR(target,&fsr)!=MPU6050_ACCEPTED||fsr==0)
    return MPU6050_ERROR;
  scale[0] = 32768.0f/(float)fsr;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_CalcAccelSens(MPU6050* target,float* scale){
  uint8_t fsr;
  if(MPU6050_GetAccelFSR(target,&fsr)!=MPU6050_ACCEPTED||fsr==0)
    return MPU6050_ERROR;
  scale[0] = 32768.0f/(float)fsr;
  return MPU6050_ACCEPTED;
}

/*===========================Data Operate API===========================*/

MPU6050_State MPU6050_GetRawGyro(MPU6050* target,int16_t* data){
  uint8_t raw[6];
  if(!(target->sensors&MPU6050_SENS_GYRO_ALL))
    return MPU6050_ERROR;
  if(MPU6050_Read(target->addr,MPU6050_REG_RAW_GYRO,raw,6)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  data[0] = (int16_t)((raw[0]<<8)|raw[1]);
  data[1] = (int16_t)((raw[2]<<8)|raw[3]);
  data[2] = (int16_t)((raw[4]<<8)|raw[5]);
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetRawAccel(MPU6050* target,int16_t* data){
  uint8_t raw[6];
  if(!(target->sensors&MPU6050_SENS_ACCEL))
    return MPU6050_ERROR;
  if(MPU6050_Read(target->addr,MPU6050_REG_RAW_ACCEL,raw,6)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  data[0] = (int16_t)((raw[0]<<8)|raw[1]);
  data[1] = (int16_t)((raw[2]<<8)|raw[3]);
  data[2] = (int16_t)((raw[4]<<8)|raw[5]);
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetRawTemp(MPU6050* target,int16_t* data){
  uint8_t raw[2];
  if(!target->sensors) return MPU6050_ERROR;
  if(MPU6050_Read(target->addr,MPU6050_REG_RAW_TEMP,raw,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  data[0] = (int16_t)((raw[0]<<8)|raw[1]);
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_CalculateTemp(MPU6050* target,float* temp){
  int16_t data;
  MPU6050_State status = MPU6050_GetRawTemp(target,&data);
  if(status!=MPU6050_ACCEPTED) return status;
  temp[0] = MPU6050_TEMP_OFFSET + (float)data/MPU6050_TEMP_SENS;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_ReadFIFO(MPU6050* target,int16_t* gyro,int16_t* accel,uint8_t* sensors,uint8_t* more){
  if(target->dmp_en) return MPU6050_INCORRECT;
  if(!target->sensors) return MPU6050_ERROR;
  if(!target->fifo_sensors) return MPU6050_ERROR;

  sensors[0]=0;
  uint8_t data[FIFO_MAX_PACKET_SIZE];
  uint8_t packet_size=0;
  uint16_t fifo_count,index=0;

  if(target->fifo_sensors&MPU6050_SENS_GYRO_X) packet_size+=2;
  if(target->fifo_sensors&MPU6050_SENS_GYRO_Y) packet_size+=2;
  if(target->fifo_sensors&MPU6050_SENS_GYRO_Z) packet_size+=2;
  if(target->fifo_sensors&MPU6050_SENS_ACCEL)  packet_size+=6;
  if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_COUNT,data,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  fifo_count = (data[0]<<8)|data[1];

  if(fifo_count<packet_size) {
    more[0]=0;
    return MPU6050_INCORRECT;
  }
  if(fifo_count>(target->max_fifo>>1)){
    if(MPU6050_Read(target->addr,MPU6050_REG_INT_ST,data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    if(data[0]&MPU6050_BIT_FIFO_OVERFLOW){
      MPU6050_ResetFIFO(target);
      return MPU6050_ERROR;
    }
  }

  if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_RW,data,packet_size)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  more[0] = fifo_count/packet_size-1;

  if(index!=packet_size && target->fifo_sensors&MPU6050_SENS_ACCEL){
    accel[0] = (int16_t)((data[index+0]<<8)|data[index+1]);
    accel[1] = (int16_t)((data[index+2]<<8)|data[index+3]);
    accel[2] = (int16_t)((data[index+4]<<8)|data[index+5]);
    sensors[0] |= MPU6050_SENS_ACCEL;
    index+=6;
  }
  if(index!=packet_size && target->fifo_sensors&MPU6050_SENS_GYRO_X){
    gyro[0] = (int16_t)((data[index+0]<<8)|data[index+1]);
    sensors[0] |= MPU6050_SENS_GYRO_X;
    index+=2;
  }
  if(index!=packet_size && target->fifo_sensors&MPU6050_SENS_GYRO_Y){
    gyro[1] = (int16_t)((data[index+0]<<8)|data[index+1]);
    sensors[0] |= MPU6050_SENS_GYRO_Y;
    index+=2;
  }
  if(index!=packet_size && target->fifo_sensors&MPU6050_SENS_GYRO_Z){
    gyro[2] = (int16_t)((data[index+0]<<8)|data[index+1]);
    sensors[0] |= MPU6050_SENS_GYRO_Z;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_StreamFIFO(MPU6050* target, uint16_t length,uint8_t* data,uint8_t* more){
  if(!target->dmp_en){
    if(LOG!=NULL) LOG("MPU: [Stream]dmp not launched\n");
    return MPU6050_INCORRECT;
  }
  if(!target->sensors) {
    if(LOG!=NULL) LOG("MPU: [Stream]sensor not open\n");
    return MPU6050_ERROR;
  }

  uint8_t tmp[2];
  uint16_t fifo_count;
  if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_COUNT,tmp,2)!=HAL_OK) {
    if(LOG!=NULL) LOG("MPU: [Stream]read count fail\n");
    return MPU6050_TRANS_FAIL;
  }
  fifo_count = (tmp[0]<<8)|tmp[1];
  if(fifo_count<length){
    more[0]=0;
    if(LOG!=NULL) LOG("MPU: [Stream]data not ready\n");
    return MPU6050_INCORRECT;
  }
  if(fifo_count>(target->max_fifo>>1)){
    if(MPU6050_Read(target->addr,MPU6050_REG_INT_ST,tmp,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    if(tmp[0]&MPU6050_BIT_FIFO_OVERFLOW){
      if(LOG!=NULL) LOG("MPU: [Stream]fifo overflow\n");
      MPU6050_ResetFIFO(target);
      return MPU6050_ERROR;
    }
  }

  if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_RW,data,length)!=HAL_OK){
    if(LOG!=NULL) LOG("MPU: [Stream]read data fail\n");
    return MPU6050_TRANS_FAIL;
  }
  more[0] = fifo_count/length-1;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_ResetFIFO(MPU6050* target){
  if(!target->sensors) return MPU6050_ERROR;

  uint8_t data = 0x00;
  if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&data,1)!=HAL_OK)
    return MPU6050_TRANS_FAIL;

  if(target->dmp_en){
    data = MPU6050_BIT_FIFO_RST|MPU6050_BIT_DMP_RST;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    osDelay(FIFO_RESET_DELAY*1000);

    data = MPU6050_BIT_DMP_EN|MPU6050_BIT_FIFO_EN;
    if(target->sensors&MPU6050_SENS_COMPASS) data |= MPU6050_BIT_AUX_IF_EN;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;

    data = (target->int_en?MPU6050_REG_INT_EN:0x00);
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;

    data = 0x00;
    if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
  }else{
    data = MPU6050_BIT_FIFO_RST;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;

    if(target->bypass_en||!(target->sensors&MPU6050_SENS_COMPASS)) data = MPU6050_BIT_FIFO_EN;
    else data = MPU6050_BIT_FIFO_EN|MPU6050_BIT_AUX_IF_EN;
    if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    osDelay(FIFO_RESET_DELAY*1000);

    data = (target->int_en?MPU6050_BIT_DATA_RDY_EN:0x00);
    if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,&data,1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,&(target->fifo_sensors),1)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_ReadWriteMem(MPU6050* target, uint16_t mem_addr,uint16_t length,uint8_t* data,uint8_t read){
  uint8_t tmp[2];
  if(length==0||data==NULL) return MPU6050_ERROR;

  tmp[0] = (uint8_t)(mem_addr>>8);
  tmp[1] = (uint8_t)(mem_addr&0xFF);
  if(tmp[1]+length>MPU6050_BANK_SIZE) return MPU6050_INCORRECT;

  if(MPU6050_Write(target->addr,MPU6050_REG_BANK_SEL,tmp,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  HAL_StatusTypeDef (*func)(uint8_t,uint8_t,uint8_t*,uint16_t);
  func = (read?MPU6050_Read:MPU6050_Write);
  if(func(target->addr,MPU6050_REG_MEM_RW,data,length)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_ProgramDMP(MPU6050* target,uint8_t*firmware,uint16_t length,uint16_t start_addr,uint16_t sample_rate){
  if(target->dmp_loaded) return MPU6050_INCORRECT;
  if(length==0||firmware==NULL) return MPU6050_ERROR;

  uint8_t chunk[DMP_FIRMWARE_CHUNK_SIZE],tmp[2];
  uint16_t current_size;
  MPU6050_State status;
  for(uint16_t addr=0;addr<length;addr+=current_size){
    current_size = min(DMP_FIRMWARE_CHUNK_SIZE,length-addr);

    status = MPU6050_ReadWriteMem(target,addr,current_size,firmware+addr,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_ReadWriteMem(target,addr,current_size,chunk,1);
    if(status!=MPU6050_ACCEPTED) return status;

    if(memcmp(firmware+addr,chunk,current_size)!=0) return MPU6050_ERROR;
  }

  tmp[0] = start_addr>>8;
  tmp[1] = start_addr&0xFF;
  if(MPU6050_Write(target->addr,MPU6050_REG_PRGM_START,tmp,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  target->dmp_loaded = 1;
  target->dmp_sample_rate = sample_rate;
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_ReadRegister(MPU6050* target,uint8_t reg,uint8_t* data){
  if(reg==MPU6050_REG_FIFO_RW||reg==MPU6050_REG_MEM_RW) return MPU6050_INCORRECT;
  if(reg>=MPU6050_REG_NUMBER) return MPU6050_INCORRECT;
  if(MPU6050_Read(target->addr,reg,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  return MPU6050_ACCEPTED;
}

uint8_t MPU6050_RegHealthyCheck(MPU6050* target){
  uint8_t data;
  for(uint8_t reg=0;reg<MPU6050_REG_NUMBER;reg++){
    if(reg==MPU6050_REG_FIFO_RW||reg==MPU6050_REG_MEM_RW) continue;
    if(MPU6050_Read(target->addr,reg,&data,1)!=HAL_OK)
      return 0;
  }
  return 1;
}

MPU6050_State MPU6050_GetIntState(MPU6050* target, uint16_t* status){
  uint8_t tmp[2];
  if(!target->sensors) return MPU6050_ERROR;
  if(MPU6050_Read(target->addr,MPU6050_REG_INT_ST,tmp,2)!=HAL_OK)
    return MPU6050_TRANS_FAIL;
  status[0] = ((uint16_t)tmp[0]<<8)|(uint16_t)tmp[1];
  return MPU6050_ACCEPTED;
}

uint16_t MPU6050_Row2Scale(const int8_t* row){
  uint16_t res;
  if(row[0]>0) res=0;
  else if(row[0]<0) res=4;
  else if(row[1]>0) res=1;
  else if(row[1]<0) res=5;
  else if(row[2]>0) res=2;
  else if(row[2]<0) res=6;
  else res=7;//error
  return res;
}

uint16_t MPU6050_Matrix2Scalar(const int8_t* mtx){
  uint16_t scalar=0x00;
  scalar |= MPU6050_Row2Scale(mtx);
  scalar |= MPU6050_Row2Scale(mtx+3)<<3;
  scalar |= MPU6050_Row2Scale(mtx+6)<<6;
  return scalar;
}

/*============================Self-Test API=============================*/

MPU6050_State MPU6050_SelfTest(MPU6050* target,int32_t* gyro,int32_t* accel,uint8_t* result){
#if SELF_TEST_TRIES>255
#error Too Much Tries of Self-Test!
#endif
  int32_t gyro_st[3],accel_st[3];
  uint8_t gyro_error,accel_error,dmp_was_on;
  uint8_t gyro_result,accel_result;
  uint8_t try,limit = SELF_TEST_TRIES;
  uint8_t accel_fsr,fifo_sensors,sensors_on;
  uint16_t gyro_fsr,sample_rate,lpf;
  MPU6050_State status;

  if(target->dmp_en){
    status = MPU6050_SetStateDMP(target,0);
    if(status!=MPU6050_ACCEPTED) return status;
    dmp_was_on = 1;
  }else dmp_was_on = 0;

  status = MPU6050_GetGyroFSR(target,&gyro_fsr);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_GetAccelFSR(target,&accel_fsr);
  if(status!=MPU6050_ACCEPTED) return status;
  MPU6050_GetLPF(target,&lpf);
  status = MPU6050_GetSampleRate(target,&sample_rate);
  if(status!=MPU6050_ACCEPTED) return status;
  MPU6050_GetConfigFIFO(target,&fifo_sensors);
  sensors_on = target->sensors;

  for(try=0;try<limit;try++)
    if(MPU6050_GetChipBiases(target,gyro,accel,0)==MPU6050_ACCEPTED) break;
  if(try==limit){
    result[0]=0;
    goto RESTORE;
  }
  for(try=0;try<limit;try++)
    if(MPU6050_GetChipBiases(target,gyro_st,accel_st,1)==MPU6050_ACCEPTED) break;
  if(try==limit){
    result[0]=0;
    goto RESTORE;
  }
  accel_error = (MPU6050_AccelTest(target,accel,accel_st,&accel_result)!=MPU6050_ACCEPTED);
  gyro_error  = (MPU6050_GyroTest (target,gyro ,gyro_st ,&gyro_result )!=MPU6050_ACCEPTED);
  result[0] = 0;
  if(gyro_error ==0) result[0] |= 0x01;
  if(accel_error==0) result[0] |= 0x02;

  RESTORE:
  target->gyro_fsr = 0xFF;
  target->accel_fsr = 0xFF;
  target->lpf = 0xFFFF;
  target->sample_rate = 0xFFFF;
  target->sensors = 0xFF;
  target->fifo_sensors = 0xFF;
  target->clock_source = MPU6050_CLK_PLL;
  status = MPU6050_SetGyroFSR(target,gyro_fsr);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_SetAccelFSR(target,accel_fsr);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_SetLPF(target,lpf);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_SetSampleRate(target,sample_rate);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_SetSensors(target,sensors_on);
  if(status!=MPU6050_ACCEPTED) return status;
  status = MPU6050_SetConfigFIFO(target,fifo_sensors);
  if(dmp_was_on){
    status = MPU6050_SetStateDMP(target,1);
    if(status!=MPU6050_ACCEPTED) return status;
  }

  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetChipBiases(MPU6050* target,int32_t* gyro,int32_t* accel,uint8_t hw_test){
  uint8_t data[FIFO_MAX_PACKET_SIZE],packet_count;
  uint16_t fifo_count;
  //reset device first
  data[0] = 0x01;
  data[1] = 0x00;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,data,2)!=HAL_OK) return MPU6050_TRANS_FAIL;
  osDelay(2*CHIP_RESET_DELAY*1000);
  //flush registers
  data[0] = 0x00;
  if(MPU6050_Write(target->addr,MPU6050_REG_INT_EN,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_PWR_MGMT1,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_I2C_MASTER,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  //reset fifo&dmp
  data[0] = MPU6050_BIT_FIFO_RST|MPU6050_BIT_DMP_RST;
  if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  osDelay(2*FIFO_RESET_DELAY*10000);
  //set LPF
  data[0] = test_reg_lpf;
  if(MPU6050_Write(target->addr,MPU6050_REG_LPF,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  //set Chip Rate
  data[0] = test_reg_rate_div;
  if(MPU6050_Write(target->addr,MPU6050_REG_RATE_DIV,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  //set gyro_fsr
  data[0] = test_reg_gyro_fsr;
  if(hw_test) data[0] |= 0xE0;
  if(MPU6050_Write(target->addr,MPU6050_REG_GYRO_CFG,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  //set accel_fsr
  data[0] = test_reg_accel_fsr;
  if(hw_test) data[0] |= 0xE0;
  if(MPU6050_Write(target->addr,MPU6050_REG_ACCEL_CFG,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  //hardware test delay
  if(hw_test) osDelay(2*CHIP_RESET_DELAY*1000);
  //Fill FIFO for test_wait_ms milliseconds
  data[0] = MPU6050_BIT_FIFO_EN;
  if(MPU6050_Write(target->addr,MPU6050_REG_USER_CTRL,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  data[0] = MPU6050_SENS_GYRO_ALL|MPU6050_SENS_ACCEL;
  if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  osDelay(test_wait_ms*1000);
  data[0] = 0x00;
  if(MPU6050_Write(target->addr,MPU6050_REG_FIFO_EN,data,1)!=HAL_OK) return MPU6050_TRANS_FAIL;
  if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_COUNT,data,2)!=HAL_OK) return MPU6050_TRANS_FAIL;

  fifo_count = (data[0]<<8)|data[1];
  packet_count = fifo_count/FIFO_MAX_PACKET_SIZE;
  memset(gyro,0,sizeof(int32_t)*3);
  memset(accel,0,sizeof(int32_t)*3);

  for(uint8_t i=0;i<packet_count;i++){
    int16_t accel_cur[3],gyro_cur[3];
    if(MPU6050_Read(target->addr,MPU6050_REG_FIFO_RW,data,FIFO_MAX_PACKET_SIZE)!=HAL_OK)
      return MPU6050_TRANS_FAIL;
    accel_cur[0] = (int16_t)((data[0]<<8)|data[1]);
    accel_cur[1] = (int16_t)((data[2]<<8)|data[3]);
    accel_cur[2] = (int16_t)((data[4]<<8)|data[5]);
    accel[0] += (int32_t)accel_cur[0];
    accel[1] += (int32_t)accel_cur[1];
    accel[2] += (int32_t)accel_cur[2];
    gyro_cur[0] = (int16_t)((data[6]<<8)|data[7]);
    gyro_cur[1] = (int16_t)((data[8]<<8)|data[9]);
    gyro_cur[2] = (int16_t)((data[10]<<8)|data[11]);
    gyro[0] += (int32_t)gyro_cur[0];
    gyro[1] += (int32_t)gyro_cur[1];
    gyro[2] += (int32_t)gyro_cur[2];
  }
  gyro[0] = (int32_t)((int64_t)gyro[0]<<16/test_gyro_sens/packet_count);
  gyro[1] = (int32_t)((int64_t)gyro[1]<<16/test_gyro_sens/packet_count);
  gyro[2] = (int32_t)((int64_t)gyro[2]<<16/test_gyro_sens/packet_count);
  accel[0] = (int32_t)((int64_t)accel[0]<<16/test_accel_sens/packet_count);
  accel[1] = (int32_t)((int64_t)accel[1]<<16/test_accel_sens/packet_count);
  accel[2] = (int32_t)((int64_t)accel[2]<<16/test_accel_sens/packet_count);
  //process gravity
  if(accel[2]>0) accel[2]-=65536;
  else accel[2]+=65536;

  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GyroTest(MPU6050* target,int32_t* bias_regular,int32_t* bias_st,uint8_t* result){
  uint8_t tmp[3];
  float st_shift,st_shit_cust,st_shift_var;
  result[0] = 0;

  if(MPU6050_Read(target->addr,0x0D,tmp,3)!=HAL_OK){
    result[0] = 0x07;
    return MPU6050_TRANS_FAIL;
  }
  tmp[0] &= 0x1F;tmp[1] &= 0x1F;tmp[2] &= 0x1F;

  for(uint8_t i=0;i<3;i++){
    st_shit_cust = (float)labs(bias_regular[i]-bias_st[i])/65536.0f;
    if(tmp[i]){
      st_shift = 3275.0f/(float)test_gyro_sens;
      while(--tmp[i]) st_shift*=1.046f;
      st_shift_var = st_shit_cust/st_shift-1.0f;
      if(fabsf(st_shift_var)>max_gyro_var) result[0] |= 1<<i;
    }
    else if(st_shit_cust<min_dps||st_shit_cust>max_dps) result[0] |= 1<<i;
  }

  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_AccelTest(MPU6050* target,int32_t* bias_regular,int32_t* bias_st,uint8_t* result){
  uint8_t tmp[3];
  float st_shift[3],st_shift_cust,st_shit_var;

  result[0] = 0;
  if(MPU6050_GetAccelShift(target,st_shift)!=MPU6050_ACCEPTED){
    result[0] = 0x07;
    return MPU6050_TRANS_FAIL;
  }
  for(uint8_t i=0;i<3;i++){
    st_shift_cust = (float)labs(bias_regular[i]-bias_st[i])/65536.0f;
    if(st_shift[i]){
      st_shit_var = st_shift_cust/st_shift[i]-1.0f;
      if(fabsf(st_shit_var)>max_accel_var) result[0] |= 1<<i;
    }
    else if(st_shift_cust<min_gravity||st_shift_cust>max_gravity) result[0] |= 1<<i;
  }
  return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_GetAccelShift(MPU6050* target,float* st_shift){
  uint8_t tmp[4],shift_codes[3];
  if(MPU6050_Read(target->addr,0x0D,tmp,4)!=HAL_OK)
    return MPU6050_TRANS_FAIL;

  shift_codes[0] = (tmp[0]&0xE0)>>3 | (tmp[3]&0x30)>>4;
  shift_codes[1] = (tmp[1]&0xE0)>>3 | (tmp[3]&0x0C)>>2;
  shift_codes[2] = (tmp[2]&0xE0)>>3 | (tmp[3]&0x03);

  for(uint8_t i=0;i<3;i++){
    if(!shift_codes[i]){
      st_shift[i] = 0.0f;
      continue;
    }
    st_shift[i] = 0.34f;
    while(--shift_codes[i]) st_shift[i] *= 1.034f;
  }

  return MPU6050_ACCEPTED;
}

////


#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)

//#define DMP_CODE_SIZE           (3062)
//
//static uint8_t dmp_memory[DMP_CODE_SIZE]={
//        /* bank # 0 */
//        0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
//        0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
//        0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
//        0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
//        0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
//        0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
//        0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
//        0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
//        0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
//        0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
//        0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
//        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
//        0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
//        0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
//        0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        /* bank # 1 */
//        0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
//        0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
//        0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
//        0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
//        0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
//        0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
//        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
//        0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
//        0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
//        0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
//        /* bank # 2 */
//        0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
//        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
//        0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
//        0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
//        0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
//        0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        /* bank # 3 */
//        0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
//        0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//        0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
//        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//
//        /* bank # 4 */
//        0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
//        0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
//        0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
//        0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
//        0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
//        0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
//        0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
//        0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
//        0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
//        0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
//        0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
//        0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
//        0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
//        0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
//        0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
//        0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
//        /* bank # 5 */
//        0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
//        0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
//        0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
//        0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
//        0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
//        0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
//        0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
//        0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
//        0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
//        0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
//        0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
//        0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
//        0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
//        0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
//        0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
//        0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
//        /* bank # 6 */
//        0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
//        0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
//        0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
//        0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
//        0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
//        0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
//        0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
//        0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
//        0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
//        0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
//        0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
//        0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
//        0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
//        0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
//        0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
//        0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
//        /* bank # 7 */
//        0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
//        0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
//        0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
//        0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
//        0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
//        0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
//        0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
//        0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
//        0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
//        0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
//        0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
//        0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
//        0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
//        0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
//        0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
//        0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
//        /* bank # 8 */
//        0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
//        0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
//        0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
//        0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
//        0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
//        0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
//        0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
//        0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
//        0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
//        0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
//        0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
//        0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
//        0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
//        0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
//        0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
//        0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
//        /* bank # 9 */
//        0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
//        0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
//        0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
//        0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
//        0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
//        0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
//        0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
//        0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
//        0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
//        0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
//        0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
//        0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
//        0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
//        0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
//        0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
//        0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
//        /* bank # 10 */
//        0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
//        0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
//        0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
//        0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
//        0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
//        0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
//        0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
//        0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
//        0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
//        0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
//        0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
//        0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
//        0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
//        0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
//        0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
//        0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
//        /* bank # 11 */
//        0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
//        0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
//        0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
//        0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
//        0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
//        0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
//        0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
//        0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
//        0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
//        0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
//        0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
//        0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
//        0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
//        0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
//        0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
//        0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
//};

static uint16_t sStartAddress = 0x0400;

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define MPU6050_DMP_FEATURE_SEND_ANY_GYRO   (MPU6050_DMP_FEATURE_SEND_RAW_GYRO | MPU6050_DMP_FEATURE_SEND_CAL_GYRO)

#define MAX_PACKET_LENGTH   (32)

#define DMP_SAMPLE_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)

#define FIFO_CORRUPTION_CHECK
#ifdef FIFO_CORRUPTION_CHECK
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif

#define LOG MPU6050_Log

/*=========================Main Functions========================*/

__weak MPU6050_State MPU6050_DMP_Init(MPU6050* target,MPU6050_DMP* dmp,uint8_t* error_step){
    error_step[0] = 0;
    MPU6050_State status;
    status = MPU6050_Init(target,dmp);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: mpu init fail\n");error_step[0]=1;return status;}
    status = MPU6050_SetSensors(target,MPU6050_SENS_GYRO_ALL|MPU6050_SENS_ACCEL);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: ser sensor fail\n");error_step[0]=2;return status;}
    status = MPU6050_SetConfigFIFO(target,MPU6050_SENS_GYRO_ALL|MPU6050_SENS_ACCEL);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: config FIFO fail\n");error_step[0]=3;return status;}
    status = MPU6050_SetSampleRate(target,MPU6050_DEFAULT_HZ);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: set sample rate fail\n");error_step[0]=4;return status;}
    status = MPU6050_DMP_LoadFirmware(target);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: load firmware fail\n");error_step[0]=5;return status;}
    status = MPU6050_DMP_SetOrientation(target, MPU6050_Matrix2Scalar(MPU6050_ORIENTATION_DEFAULT));
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: config orientation fail\n");error_step[0]=6;return status;}
    status = MPU6050_DMP_SetFeature(target,MPU6050_DMP_FEATURE_6X_LP_QUAT|MPU6050_DMP_FEATURE_TAP|MPU6050_DMP_FEATURE_ANDROID_ORIENT|
                                               MPU6050_DMP_FEATURE_SEND_RAW_ACCEL|MPU6050_DMP_FEATURE_SEND_CAL_GYRO|MPU6050_DMP_FEATURE_GYRO_CAL);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: config feature fail\n");error_step[0]=7;return status;}
    status = MPU6050_DMP_SetRateFIFO(target,MPU6050_DEFAULT_HZ);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: set fifo rate fail\n");error_step[0]=8;return status;}
    status = MPU6050_Calibrate(target,1,1);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: mpu calibration fail\n");error_step[0]=9;return status;}
    status = MPU6050_SetStateDMP(target,1);
    if(status!=MPU6050_ACCEPTED){if(LOG!=NULL) LOG("DMP_INIT: launch dmp fail\n");error_step[0]=10;return status;}
    if(LOG!=NULL) LOG("DMP_INIT: passed\n");
    return MPU6050_ACCEPTED;
}

__weak MPU6050_State MPU6050_DMP_Calculate(MPU6050* target,float* pitch,float* roll,float* yaw,int16_t* gyro_raw,int16_t* accel_raw){
    float q0,q1,q2,q3;
    uint16_t sensors;
    uint8_t more;
    int32_t quat[4];
    MPU6050_State status = MPU6050_DMP_ReadFIFO(target,gyro_raw,accel_raw,quat,&sensors,&more);
    if(status!=MPU6050_ACCEPTED) {
        if(LOG!=NULL) LOG("DMP: [calculate]read fifo error\n");
        return status;
    }
    if(sensors&MPU6050_DMP_SENSE_WXYZ_QUAT){
        q0 = (float)quat[0]/Quaternion30;
        q1 = (float)quat[1]/Quaternion30;
        q2 = (float)quat[2]/Quaternion30;
        q3 = (float)quat[3]/Quaternion30;

        *pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2)*57.3f;
        *roll  = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1)*57.3f;
        *yaw   = atan2f(2.0f * q1 * q2 + 2.0f * q0 * q3, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3)*57.3f;
    }else {
        if(LOG!=NULL) LOG("DMP: [calculate]sensor type error\n");
        return MPU6050_ERROR;
    }
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_ReadFIFO(MPU6050* target,int16_t* gyro,int16_t* accel,int32_t* quaternion,uint16_t* sensors,uint8_t* more){
    uint8_t fifo_data[MAX_PACKET_LENGTH];
    uint8_t index=0;
    sensors[0] = 0;
    MPU6050_State status = MPU6050_StreamFIFO(target,target->dmp->packet_length,fifo_data,more);
    if(status!=MPU6050_ACCEPTED) {
        if(LOG!=NULL) LOG("DMP: [FIFO] read fifo stream error\n");
        return status;
    }

    //quaternions decode
    if(target->dmp->feature_mask&(MPU6050_DMP_FEATURE_LP_QUAT|MPU6050_DMP_FEATURE_6X_LP_QUAT)){
        quaternion[0] = ((int32_t)fifo_data[0])<<24  | ((int32_t)fifo_data[1])<<16  | ((int32_t)fifo_data[2])<<8  | fifo_data[3];
        quaternion[1] = ((int32_t)fifo_data[4])<<24  | ((int32_t)fifo_data[5])<<16  | ((int32_t)fifo_data[6])<<8  | fifo_data[7];
        quaternion[2] = ((int32_t)fifo_data[8])<<24  | ((int32_t)fifo_data[9])<<16  | ((int32_t)fifo_data[10])<<8 | fifo_data[11];
        quaternion[3] = ((int32_t)fifo_data[12])<<24 | ((int32_t)fifo_data[13])<<16 | ((int32_t)fifo_data[14])<<8 | fifo_data[15];
        index+=16;
        sensors[0] |= MPU6050_DMP_SENSE_WXYZ_QUAT;
#ifndef MPU6050_DISABLE_Q14
        //quaternion corruption check by q14
        int32_t quat_q14[4],quat_mag_sq;
        quat_q14[0] = quaternion[0]>>16;
        quat_q14[1] = quaternion[1]>>16;
        quat_q14[2] = quaternion[2]>>16;
        quat_q14[3] = quaternion[3]>>16;
        quat_mag_sq = quat_q14[0]*quat_q14[0]+quat_q14[1]*quat_q14[1]+quat_q14[2]*quat_q14[2]+quat_q14[3]*quat_q14[3];
        if(quat_mag_sq>QUAT_MAG_SQ_MAX||quat_mag_sq<QUAT_MAG_SQ_MIN){
            MPU6050_ResetFIFO(target);
            sensors[0]=0;
            if(LOG!=NULL) LOG("DMP: [FIFO] quaternion check failed\n");
            return MPU6050_ERROR;
        }
#endif
    }
    //accelerator raw-data decode
    if(target->dmp->feature_mask & MPU6050_DMP_FEATURE_SEND_RAW_ACCEL){
        accel[0] = (int16_t)(fifo_data[index+0]<<8|fifo_data[index+1]);
        accel[1] = (int16_t)(fifo_data[index+2]<<8|fifo_data[index+3]);
        accel[2] = (int16_t)(fifo_data[index+4]<<8|fifo_data[index+5]);
        index+=6;
        sensors[0] |= MPU6050_SENS_ACCEL;
    }
    //gyroscope data decode
    if(target->dmp->feature_mask & MPU6050_DMP_FEATURE_SEND_ANY_GYRO){
        gyro[0] = (int16_t)(fifo_data[index+0]<<8|fifo_data[index+1]);
        gyro[1] = (int16_t)(fifo_data[index+2]<<8|fifo_data[index+3]);
        gyro[2] = (int16_t)(fifo_data[index+4]<<8|fifo_data[index+5]);
        index+=6;
        sensors[0] |= MPU6050_SENS_GYRO_ALL;
    }
    //decode gesture
    if(target->dmp->feature_mask&(MPU6050_DMP_FEATURE_TAP|MPU6050_DMP_FEATURE_ANDROID_ORIENT))
        if(target->dmp->tap_cb!=NULL||target->dmp->android_orient_cb!=NULL) MPU6050_DMP_DecodeGesture(target,fifo_data+index);
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_DecodeGesture(MPU6050* target,const uint8_t* gesture){
    uint8_t tap,android_orient;
    android_orient = gesture[3]&0xC0;
    tap = 0x3F & gesture[3];
    if(gesture[1]&INT_SRC_TAP){
        uint8_t direction,count;
        direction = tap>>3;
        count = (tap%8)+1;
        if(target->dmp->tap_cb!=NULL) target->dmp->tap_cb(count,direction);
    }
    if(gesture[1]&INT_SRC_ANDROID_ORIENT){
        if(target->dmp->android_orient_cb!=NULL) target->dmp->android_orient_cb(android_orient>>6);
    }
    return MPU6050_ACCEPTED;
}

/*===========================Setup API===========================*/

MPU6050_State MPU6050_DMP_LoadFirmware(MPU6050* target){
    return MPU6050_ProgramDMP(target,dmp_memory,DMP_CODE_SIZE,sStartAddress,DMP_SAMPLE_RATE);
}

MPU6050_State MPU6050_DMP_SetRateFIFO(MPU6050* target,uint16_t rate){
    uint8_t regs_end[12]={DINAFE,DINAF2,DINAAB,0xC4,DINAAA,DINAF1,
                          DINADF,DINADF,0xBB,0xAF,DINADF,DINADF};
    uint16_t div;
    uint8_t tmp[8];
    if(rate>DMP_SAMPLE_RATE) return MPU6050_INCORRECT;
    div = DMP_SAMPLE_RATE/rate-1;
    tmp[0] = (uint8_t)((div>>8)&0xFF);
    tmp[1] = (uint8_t)(div&0xFF);
    MPU6050_State status;
    status = MPU6050_ReadWriteMem(target,D_0_22,2,tmp,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_ReadWriteMem(target,CFG_6,12,regs_end,0);
    if(status!=MPU6050_ACCEPTED) return status;

    if(target->dmp==NULL) return MPU6050_ERROR;
    target->dmp->fifo_rate = rate;
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_GetRateFIFO(MPU6050* target,uint16_t* rate){
    if(target->dmp==NULL) return MPU6050_ERROR;
    rate[0] = target->dmp->fifo_rate;
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetFeature(MPU6050* target,uint16_t mask){
    uint8_t tmp[10];
    MPU6050_State status;

    //set integration scale factor
    tmp[0] = (uint8_t)((GYRO_SF >> 24) & 0xFF);
    tmp[1] = (uint8_t)((GYRO_SF >> 16) & 0xFF);
    tmp[2] = (uint8_t)((GYRO_SF >> 8 ) & 0xFF);
    tmp[3] = (uint8_t)(GYRO_SF & 0xFF);
    status = MPU6050_ReadWriteMem(target,D_0_104,4,tmp,0);
    if(status!=MPU6050_ACCEPTED) return status;

    //send sensor data to FIFO
    tmp[0] = 0xA3;
    if(mask & MPU6050_DMP_FEATURE_SEND_RAW_ACCEL){
        tmp[1] = 0xC0;tmp[2] = 0xC8;tmp[3] = 0xC2;
    }else{
        tmp[1] = 0xA3;tmp[2] = 0xA3;tmp[3] = 0xA3;
    }
    if(mask & MPU6050_DMP_FEATURE_SEND_ANY_GYRO){
        tmp[4] = 0xC4;tmp[5] = 0xCC;tmp[6] = 0xC6;
    }else{
        tmp[4] = 0xA3;tmp[5] = 0xA3;tmp[6] = 0xA3;
    }
    tmp[7]=0xA3;tmp[8]=0xA3;tmp[8]=0xA3;
    status = MPU6050_ReadWriteMem(target,CFG_15,10,tmp,0);
    if(status!=MPU6050_ACCEPTED) return status;

    //send gesture data to FIFO
    if(mask & (MPU6050_DMP_FEATURE_TAP|MPU6050_DMP_FEATURE_ANDROID_ORIENT))
        tmp[0] = DINA20;
    else tmp[0] = 0xD8;
    status = MPU6050_ReadWriteMem(target,CFG_27,1,tmp,0);
    if(status!=MPU6050_ACCEPTED) return status;

    if(mask&MPU6050_DMP_FEATURE_GYRO_CAL) MPU6050_DMP_GyroCalibrate(target,1);
    else MPU6050_DMP_GyroCalibrate(target,0);

    if(mask&MPU6050_DMP_FEATURE_SEND_ANY_GYRO){
        if(mask&MPU6050_DMP_FEATURE_SEND_CAL_GYRO){
            tmp[0] = 0xB2;tmp[1] = 0x8B;tmp[2] = 0xB6;tmp[3] = 0x9B;
        }else{
            tmp[0] = DINAC0;tmp[1] = DINA80;tmp[2] = DINAC2;tmp[3] = DINA90;
        }
        status = MPU6050_ReadWriteMem(target,CFG_GYRO_RAW_DATA,4,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }

    if(mask&MPU6050_DMP_FEATURE_TAP){
        tmp[0] = 0xF8;
        status = MPU6050_ReadWriteMem(target,CFG_20,1,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetTapThresh(target,MPU6050_DMP_TAP_XYZ,250);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetTapAxes(target,MPU6050_DMP_TAP_XYZ);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetTapCount(target,1);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetTapTime(target,100);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetTapTimeMulti(target,500);
        if(status!=MPU6050_ACCEPTED) return status;

        status = MPU6050_DMP_SetShakeRejectThresh(target,GYRO_SF,200);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetShakeRejectTime(target,40);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_DMP_SetShakeRejectTimeout(target,10);
        if(status!=MPU6050_ACCEPTED) return status;
    }else{
        tmp[0] = 0xD8;
        status = MPU6050_ReadWriteMem(target,CFG_20,1,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }

    if(mask&MPU6050_DMP_FEATURE_ANDROID_ORIENT) tmp[0] = 0xD9;
    else tmp[0] = 0xD8;
    status = MPU6050_ReadWriteMem(target,CFG_ANDROID_ORIENT_INT,1,tmp,0);
    if(status!=MPU6050_ACCEPTED) return status;

    if(mask&MPU6050_DMP_FEATURE_LP_QUAT){
        status = MPU6050_DMP_EnableQuaternions3(target,1);
        if(status!=MPU6050_ACCEPTED) return status;
    }else{
        status = MPU6050_DMP_EnableQuaternions3(target,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }
    if(mask&MPU6050_DMP_FEATURE_6X_LP_QUAT){
        status = MPU6050_DMP_EnableQuaternions6(target,1);
        if(status!=MPU6050_ACCEPTED) return status;
    }else{
        status = MPU6050_DMP_EnableQuaternions6(target,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }

    if(target->dmp==NULL) return MPU6050_ERROR;
    target->dmp->feature_mask = mask|MPU6050_DMP_FEATURE_PEDOMETER;
    status = MPU6050_ResetFIFO(target);
    if(status!=MPU6050_ACCEPTED) return status;

    target->dmp->packet_length = 0;
    if(mask&MPU6050_DMP_FEATURE_SEND_RAW_ACCEL)
        target->dmp->packet_length += 6;
    if(mask&MPU6050_DMP_FEATURE_SEND_ANY_GYRO)
        target->dmp->packet_length += 6;
    if(mask&(MPU6050_DMP_FEATURE_LP_QUAT|MPU6050_DMP_FEATURE_6X_LP_QUAT))
        target->dmp->packet_length += 16;
    if(mask&(MPU6050_DMP_FEATURE_TAP|MPU6050_DMP_FEATURE_ANDROID_ORIENT))
        target->dmp->packet_length += 4;

    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_GetFeature(MPU6050* target,uint16_t* mask){
    mask[0] = target->dmp->feature_mask;
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetIntMode(MPU6050* target,uint8_t mode){
    uint8_t regs_continuous[11] = {0xd8, 0xb1, 0xb9, 0xf3, 0x8b,
                                   0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
    uint8_t regs_gesture[11] = {0xda, 0xb1, 0xb9, 0xf3, 0x8b,
                                0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};

    switch(mode){
        case MPU6050_DMP_INT_CONTINUOUS:
            return MPU6050_ReadWriteMem(target,CFG_FIFO_ON_EVENT,11,regs_continuous,0);
        case MPU6050_DMP_INT_GESTURE:
            return MPU6050_ReadWriteMem(target,CFG_FIFO_ON_EVENT,11,regs_gesture,0);
        default:
            return MPU6050_ERROR;
    }
}

MPU6050_State MPU6050_DMP_SetOrientation(MPU6050* target,uint16_t orient){
    MPU6050_State status;
    uint8_t gyro_regs[3],accel_regs[3];
    uint8_t gyro_axes[3] = {DINA4C, DINACD, DINA6C};
    uint8_t accel_axes[3] = {DINA0C, DINAC9, DINA2C};
    uint8_t gyro_sign[3] = {DINA36, DINA56, DINA76};
    uint8_t accel_sign[3] = {DINA26, DINA46, DINA66};

    gyro_regs[0] = gyro_axes[orient & 3];
    gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
    gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
    accel_regs[0] = accel_axes[orient & 3];
    accel_regs[1] = accel_axes[(orient >> 3) & 3];
    accel_regs[2] = accel_axes[(orient >> 6) & 3];

    status = MPU6050_ReadWriteMem(target,FCFG_1,3,gyro_regs,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_ReadWriteMem(target,FCFG_2,3,accel_regs,0);
    if(status!=MPU6050_ACCEPTED) return status;

    memcpy(gyro_regs, gyro_sign, 3);
    memcpy(accel_regs, accel_sign, 3);
    if (orient & 4) {
        gyro_regs[0] |= 1;
        accel_regs[0] |= 1;
    }
    if (orient & 0x20) {
        gyro_regs[1] |= 1;
        accel_regs[1] |= 1;
    }
    if (orient & 0x100) {
        gyro_regs[2] |= 1;
        accel_regs[2] |= 1;
    }

    status = MPU6050_ReadWriteMem(target,FCFG_3,3,gyro_regs,0);
    if(status!=MPU6050_ACCEPTED) return status;
    status = MPU6050_ReadWriteMem(target,FCFG_7,3,accel_regs,0);
    if(status!=MPU6050_ACCEPTED) return status;

    target->dmp->orient = orient;
    return 0;
}

MPU6050_State MPU6050_DMP_GyroCalibrate(MPU6050* target,uint8_t enable){
    if(enable){
        uint8_t regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
        return MPU6050_ReadWriteMem(target,CFG_MOTION_BIAS,9,regs,0);
    }else{
        uint8_t regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
        return MPU6050_ReadWriteMem(target,CFG_MOTION_BIAS,9,regs,0);
    }
}

MPU6050_State MPU6050_DMP_SetGyroBias(MPU6050* target,const int32_t* bias){
    int32_t gyro_bias_body[3];
    uint8_t regs[4];
    MPU6050_State status;

    gyro_bias_body[0] = bias[target->dmp->orient&3];
    if(target->dmp->orient&0x04) gyro_bias_body[0]*=-1;
    gyro_bias_body[1] = bias[(target->dmp->orient>>3)&3];
    if(target->dmp->orient&0x20) gyro_bias_body[1]*=-1;
    gyro_bias_body[2] = bias[(target->dmp->orient>>6)&3];
    if(target->dmp->orient&0x100) gyro_bias_body[2]*=-1;

    gyro_bias_body[0] = (int32_t)(((uint64_t)gyro_bias_body[0]*GYRO_SF)>>30);
    gyro_bias_body[1] = (int32_t)(((uint64_t)gyro_bias_body[1]*GYRO_SF)>>30);
    gyro_bias_body[2] = (int32_t)(((uint64_t)gyro_bias_body[2]*GYRO_SF)>>30);

    regs[0] = (uint8_t)((gyro_bias_body[0]>>24)&0xFF);
    regs[1] = (uint8_t)((gyro_bias_body[0]>>16)&0xFF);
    regs[2] = (uint8_t)((gyro_bias_body[0]>>8)&0xFF);
    regs[3] = (uint8_t)(gyro_bias_body[0]&0xFF);
    status = MPU6050_ReadWriteMem(target,D_EXT_GYRO_BIAS_X,4,regs,0);
    if(status!=MPU6050_ACCEPTED) return status;

    regs[0] = (uint8_t)((gyro_bias_body[1]>>24)&0xFF);
    regs[1] = (uint8_t)((gyro_bias_body[1]>>16)&0xFF);
    regs[2] = (uint8_t)((gyro_bias_body[1]>>8)&0xFF);
    regs[3] = (uint8_t)(gyro_bias_body[1]&0xFF);
    status = MPU6050_ReadWriteMem(target,D_EXT_GYRO_BIAS_Y,4,regs,0);
    if(status!=MPU6050_ACCEPTED) return status;

    regs[0] = (uint8_t)((gyro_bias_body[2]>>24)&0xFF);
    regs[1] = (uint8_t)((gyro_bias_body[2]>>16)&0xFF);
    regs[2] = (uint8_t)((gyro_bias_body[2]>>8)&0xFF);
    regs[3] = (uint8_t)(gyro_bias_body[2]&0xFF);
    status = MPU6050_ReadWriteMem(target,D_EXT_GYRO_BIAS_Z,4,regs,0);
    if(status!=MPU6050_ACCEPTED) return status;

    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetAccelBias(MPU6050* target,const int32_t* bias){
    int32_t accel_bias_body[3];
    uint8_t regs[12];
    int64_t accel_sf;
    float accel_sens;
    MPU6050_State status;

    status = MPU6050_CalcAccelSens(target,&accel_sens);
    if(status!=MPU6050_ACCEPTED) return status;
    accel_sf = (int64_t)accel_sens<<15;

    accel_bias_body[0] = bias[target->dmp->orient&3];
    if(target->dmp->orient&0x04) accel_bias_body[0]*=-1;
    accel_bias_body[1] = bias[(target->dmp->orient>>3)&3];
    if(target->dmp->orient&0x20) accel_bias_body[1]*=-1;
    accel_bias_body[2] = bias[(target->dmp->orient>>6)&3];
    if(target->dmp->orient&0x100) accel_bias_body[2]*=-1;

    accel_bias_body[0]=(int32_t)(((int64_t)accel_bias_body[0]*accel_sf)>>30);
    accel_bias_body[1]=(int32_t)(((int64_t)accel_bias_body[1]*accel_sf)>>30);
    accel_bias_body[2]=(int32_t)(((int64_t)accel_bias_body[2]*accel_sf)>>30);

    regs[0]  =(uint8_t)((accel_bias_body[0]>>24)&0xFF);
    regs[1]  =(uint8_t)((accel_bias_body[0]>>16)&0xFF);
    regs[2]  =(uint8_t)((accel_bias_body[0]>>8)&0xFF);
    regs[3]  =(uint8_t)(accel_bias_body[0]&0xFF);
    regs[4]  =(uint8_t)((accel_bias_body[1]>>24)&0xFF);
    regs[5]  =(uint8_t)((accel_bias_body[1]>>16)&0xFF);
    regs[6]  =(uint8_t)((accel_bias_body[1]>>8)&0xFF);
    regs[7]  =(uint8_t)(accel_bias_body[1]&0xFF);
    regs[8]  =(uint8_t)((accel_bias_body[2]>>24)&0xFF);
    regs[9]  =(uint8_t)((accel_bias_body[2]>>16)&0xFF);
    regs[10] =(uint8_t)((accel_bias_body[2]>>8)&0xFF);
    regs[11] =(uint8_t)(accel_bias_body[2]&0xFF);

    return MPU6050_ReadWriteMem(target,D_ACCEL_BIAS,12,regs,0);
}

/*========================Tap & Shake API========================*/

MPU6050_State MPU6050_DMP_SetTapThresh(MPU6050* target,uint8_t axis,uint16_t thresh){
    uint8_t tmp[4],accel_fsr;
    float scaled_thresh;
    uint16_t dmp_thresh_1,dmp_thresh_2;
    if(!(axis&MPU6050_DMP_TAP_XYZ)||thresh>1600) return MPU6050_INCORRECT;
    scaled_thresh = (float)thresh/DMP_SAMPLE_RATE;
    MPU6050_State status;

    status = MPU6050_GetAccelFSR(target,&accel_fsr);
    if(status!=MPU6050_ACCEPTED) return status;
    switch(accel_fsr){
        case 2  : dmp_thresh_1 = (uint16_t)(scaled_thresh*16384);
            dmp_thresh_2 = (uint16_t)(scaled_thresh*12288);
            break;
        case 4  : dmp_thresh_1 = (uint16_t)(scaled_thresh*8192);
            dmp_thresh_2 = (uint16_t)(scaled_thresh*6144);
            break;
        case 8  : dmp_thresh_1 = (uint16_t)(scaled_thresh*4096);
            dmp_thresh_2 = (uint16_t)(scaled_thresh*3072);
            break;
        case 16 : dmp_thresh_1 = (uint16_t)(scaled_thresh*2048);
            dmp_thresh_2 = (uint16_t)(scaled_thresh*1536);
            break;
        default : return MPU6050_ERROR;
    }
    tmp[0] = (uint8_t)(dmp_thresh_1>>8);
    tmp[1] = (uint8_t)(dmp_thresh_1&0xFF);
    tmp[2] = (uint8_t)(dmp_thresh_2>>8);
    tmp[3] = (uint8_t)(dmp_thresh_2&0xFF);

    if(axis&MPU6050_DMP_TAP_X){
        status = MPU6050_ReadWriteMem(target,DMP_TAP_THX,2,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_ReadWriteMem(target,D_1_36,2,tmp+2,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }
    if(axis&MPU6050_DMP_TAP_Y){
        status = MPU6050_ReadWriteMem(target,DMP_TAP_THY,2,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_ReadWriteMem(target,D_1_40,2,tmp+2,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }
    if(axis&MPU6050_DMP_TAP_Z){
        status = MPU6050_ReadWriteMem(target,DMP_TAP_THZ,2,tmp,0);
        if(status!=MPU6050_ACCEPTED) return status;
        status = MPU6050_ReadWriteMem(target,D_1_44,2,tmp+2,0);
        if(status!=MPU6050_ACCEPTED) return status;
    }
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetTapAxes(MPU6050* target,uint8_t axis){
    uint8_t tmp=0;
    if(axis&MPU6050_DMP_TAP_X) tmp|=0x30;
    if(axis&MPU6050_DMP_TAP_Y) tmp|=0x0C;
    if(axis&MPU6050_DMP_TAP_Z) tmp|=0x03;
    return MPU6050_ReadWriteMem(target,D_1_72,1,&tmp,0);
}

MPU6050_State MPU6050_DMP_SetTapCount(MPU6050* target,uint8_t min_taps){
    uint8_t tmp;
    if(min_taps<1) min_taps=1;
    if(min_taps>4) min_taps=4;
    tmp = min_taps-1;
    return MPU6050_ReadWriteMem(target,D_1_79,1,&tmp,0);
}

MPU6050_State MPU6050_DMP_SetTapTime(MPU6050* target,uint16_t time){
    uint16_t dmp_time = time/(1000/DMP_SAMPLE_RATE);
    uint8_t tmp[2];
    tmp[0] = (uint8_t)(dmp_time>>8);
    tmp[1] = (uint8_t)(dmp_time&0xFF);
    return MPU6050_ReadWriteMem(target,DMP_TAPW_MIN,2,tmp,0);
}

MPU6050_State MPU6050_DMP_SetTapTimeMulti(MPU6050* target,uint16_t time){
    uint16_t dmp_time = time/(1000/DMP_SAMPLE_RATE);
    uint8_t tmp[2];
    tmp[0] = (uint8_t)(dmp_time>>8);
    tmp[1] = (uint8_t)(dmp_time&0xFF);
    return MPU6050_ReadWriteMem(target,D_1_128,2,tmp,0);
}

MPU6050_State MPU6050_DMP_SetShakeRejectThresh(MPU6050* target,int32_t sf,uint16_t thresh){
    uint8_t tmp[4];
    int32_t thresh_scaled = sf/1000*thresh;
    tmp[0] = (uint8_t)((thresh_scaled>>24)&0xFF);
    tmp[1] = (uint8_t)((thresh_scaled>>16)&0xFF);
    tmp[2] = (uint8_t)((thresh_scaled>>8 )&0xFF);
    tmp[3] = (uint8_t)(thresh_scaled&0xFF);
    return MPU6050_ReadWriteMem(target,D_1_92,4,tmp,0);
}

MPU6050_State MPU6050_DMP_SetShakeRejectTime(MPU6050* target,uint16_t time){
    uint8_t tmp[2];
    time /= (1000/DMP_SAMPLE_RATE);
    tmp[0] = time>>8;
    tmp[1] = time&0xFF;
    return MPU6050_ReadWriteMem(target,D_1_90,2,tmp,0);
}

MPU6050_State MPU6050_DMP_SetShakeRejectTimeout(MPU6050* target,uint16_t time){
    uint8_t tmp[2];
    time /= (1000/DMP_SAMPLE_RATE);
    tmp[0] = time>>8;
    tmp[1] = time&0xFF;
    return MPU6050_ReadWriteMem(target,D_1_88,2,tmp,0);
}

/*========================Quaternions API========================*/

MPU6050_State MPU6050_DMP_EnableQuaternions3(MPU6050* target,uint8_t enable){
    uint8_t regs[4];
    if(enable){
        regs[0] = DINBC0;regs[1] = DINBC2;regs[2] = DINBC4;regs[3] = DINBC6;
    }else memset(regs,0x8B,sizeof(regs));
    MPU6050_State status = MPU6050_ReadWriteMem(target,CFG_LP_QUAT,4,regs,0);
    if(status!=MPU6050_ACCEPTED) return status;
    return MPU6050_ResetFIFO(target);
}

MPU6050_State MPU6050_DMP_EnableQuaternions6(MPU6050* target,uint8_t enable){
    uint8_t regs[4];
    if(enable){
        regs[0] = DINA20;regs[1] = DINA28;regs[2] = DINA30;regs[3] = DINA38;
    }else memset(regs,0xA3,sizeof(regs));
    MPU6050_State status = MPU6050_ReadWriteMem(target,CFG_8,4,regs,0);
    if(status!=MPU6050_ACCEPTED) return status;
    return MPU6050_ResetFIFO(target);
}

/*========================Pedometer API==========================*/

MPU6050_State MPU6050_DMP_GetStep(MPU6050* target,uint32_t* count){
    uint8_t tmp[4];
    MPU6050_State status;
    status = MPU6050_ReadWriteMem(target,D_PEDSTD_STEPCTR,4,tmp,1);
    if(status!=HAL_OK) return status;
    count[0] = ((uint32_t)tmp[0]<<24)|((uint32_t)tmp[1]<<16)|((uint32_t)tmp[2]<<8)|(uint32_t)tmp[3];
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetStep(MPU6050* target,uint32_t count){
    uint8_t tmp[4];
    tmp[0] = (uint8_t)((count>>24)&0xFF);
    tmp[1] = (uint8_t)((count>>16)&0xFF);
    tmp[2] = (uint8_t)((count>>8)&0xFF);
    tmp[3] = (uint8_t)(count&0xFF);
    return MPU6050_ReadWriteMem(target,D_PEDSTD_STEPCTR,4,tmp,0);
}

MPU6050_State MPU6050_DMP_GetWalkTime(MPU6050* target,uint32_t* time){
    //time is ms
    uint8_t tmp[4];
    MPU6050_State status;
    status = MPU6050_ReadWriteMem(target,D_PEDSTD_TIMECTR,4,tmp,1);
    if(status!=HAL_OK) return status;
    time[0] = ((uint32_t)tmp[0]<<24)|((uint32_t)tmp[1]<<16)|((uint32_t)tmp[2]<<8)|(uint32_t)tmp[3];
    time[0]*=20;
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetWalkTime(MPU6050* target,uint32_t time){
    uint8_t tmp[4];
    tmp[0] = (uint8_t)((time>>24)&0xFF);
    tmp[1] = (uint8_t)((time>>16)&0xFF);
    tmp[2] = (uint8_t)((time>>8)&0xFF);
    tmp[3] = (uint8_t)(time&0xFF);
    return MPU6050_ReadWriteMem(target,D_PEDSTD_TIMECTR,4,tmp,0);
}

/*=======================Callback Settings=======================*/

MPU6050_State MPU6050_DMP_SetOrientCallback(MPU6050* target,void (*func)(uint8_t)){
    target->dmp->android_orient_cb = func;
    return MPU6050_ACCEPTED;
}

MPU6050_State MPU6050_DMP_SetTapCallback(MPU6050* target,void (*func)(uint8_t,uint8_t)){
    target->dmp->tap_cb = func;
    return MPU6050_ACCEPTED;
}


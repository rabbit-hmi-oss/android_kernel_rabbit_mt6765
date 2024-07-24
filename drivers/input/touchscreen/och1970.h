//OCH1970 Driver v.1.2

//#include <stdint.h>  //----------------optional

#ifndef OCH1970_DRIVER_H
#define OCH1970_DRIVER_H


struct OCH1970_data {
	struct i2c_device *client;
	struct input_dev *input_dev;
	struct delayed_work nvt_fwu_work;
	uint8_t x_num;
	uint8_t y_num;
	uint16_t abs_x_max;
	uint16_t abs_y_max;
	uint8_t max_touch_num;
	uint8_t max_button_num;

};

//operation mode define
enum OCH1970_OPERATION_MODE
{
    OCH1970_OP_STANDBY_MODE,
    OCH1970_OP_SINGLE_MODE,
    OCH1970_OP_CONTINUOUS_MODE_1_0p5Hz,
    OCH1970_OP_CONTINUOUS_MODE_2_1Hz,
    OCH1970_OP_CONTINUOUS_MODE_3_2Hz,
    OCH1970_OP_CONTINUOUS_MODE_4_20Hz,
    OCH1970_OP_CONTINUOUS_MODE_5_40Hz,
    OCH1970_OP_CONTINUOUS_MODE_6_100Hz,
    OCH1970_OP_CONTINUOUS_MODE_7_500Hz,
};

enum OCH1970_SDR_MODE
{
	OCH1970_LOW_NOISE_MODE,
	OCH1970_LOW_POWER_MODE,
};

enum OCH1970_SMR_MODE
{
	OCH1970_HIGH_SENSE_MODE,
	OCH1970_HIGH_RANGE_MODE,
};

//bit operation
#define bit7    (0x01<<7)
#define bit6    (0x01<<6)
#define bit5    (0x01<<5)
#define bit4    (0x01<<4)
#define bit3    (0x01<<3)
#define bit2    (0x01<<2)
#define bit1    (0x01<<1)
#define bit0    (0x01<<0)

#define OCH_DISABLE          0
#define OCH_ENABLE           1

//register address of the OCH1970
#define OCH1970_REG_WIA 0x00	   //company id & device id,4bytes
#define OCH1970_REG_STATUS1	0x10   //status data,2bytes
#define OCH1970_REG_DATAX 0x11	   //X data [15:0]
#define OCH1970_REG_DATAY 0x12     //Y data [15:0]
#define OCH1970_REG_DATAX_Y 0x13   //X data [15:0] & Y data [15:0]
#define OCH1970_REG_DATAZ 0x14
#define OCH1970_REG_DATAX_Z 0x15
#define OCH1970_REG_DATAY_Z 0x16
#define OCH1970_REG_DATAX_Y_Z 0x17

#define OCH1970_REG_STATUS2 0x18    //same with STATUS1
#define OCH1970_REG_DATAXH 0x19	    //X data [15:8] only 8bits
#define OCH1970_REG_DATAYH 0x1A     //Y data [15:8] only 8bits
#define OCH1970_REG_DATAXH_YH 0x1B  //X data [15:8] & Y data [15:8]
#define OCH1970_REG_DATAZH 0x1C
#define OCH1970_REG_DATAXH_ZH 0x1D
#define OCH1970_REG_DATAYH_ZH 0x1E
#define OCH1970_REG_DATAXH_YH_ZH 0x1F

#define OCH1970_REG_CNTL1 0X20      //CNTL1 config interrupt
#define OCH1970_REG_CNTL2 0x21      //CNTL2 config other things
#define OCH1970_REG_THRE_X1 0x22    //BOP&BRP setting
#define OCH1970_REG_THRE_X2 0x23
#define OCH1970_REG_THRE_Y1 0X24
#define OCH1970_REG_THRE_Y2 0X25
#define OCH1970_REG_THRE_Z1 0X26
#define OCH1970_REG_THRE_Z2 0X27

#define OCH1970_REG_SRST 0x30       //soft reset

//specific constant values
#define OCH1970_SLA 0x0D //8 bits address,if you use 7 bits address, the address is 0x0D;
#define OCH1970_VALUE_STANDBY_MODE 0x00
#define OCH1970_VALUE_SINGLE_MODE 0x01
#define OCH1970_VALUE_CONTINUOUS_MODE_1_0p5Hz 0x02
#define OCH1970_VALUE_CONTINUOUS_MODE_2_1Hz 0x04
#define OCH1970_VALUE_CONTINUOUS_MODE_3_2Hz 0x06
#define OCH1970_VALUE_CONTINUOUS_MODE_4_20Hz 0x08
#define OCH1970_VALUE_CONTINUOUS_MODE_5_40Hz 0x0A
#define OCH1970_VALUE_CONTINUOUS_MODE_6_100Hz 0x0C
#define OCH1970_VALUE_CONTINUOUS_MODE_7_500Hz 0x0E


//===========================function declear===============================

//set operation mode
void OCH1970_SetOperationMode(enum OCH1970_OPERATION_MODE value);    //value should be OCH1970_OPERATION_MODE type,10 diff values
void OCH1970_SetSDRMode(enum OCH1970_SDR_MODE value);        //2 diff values: OCH1970_LOW_NOISE_MODE or OCH1970_LOW_POWER_MODE
void OCH1970_SetSMRMode(enum OCH1970_SMR_MODE value);        //2 diff values: OCH1970_HIGH_SENSE_MODE or OCH1970_HIGH_RANGE_MODE

#if 0
//set threshold(BOP&BRP) of x/y/z axis
void OCH1970_SetThre_X1(uint8_t *value);    //value: 4 bytes (BOPX1_high,BOPX1_low,BRPX1_high,BRPX1_low)
void OCH1970_SetThre_X2(uint8_t *value);    //value: 4 bytes (BOPX2_high,BOPX2_low,BRPX2_high,BRPX2_low)
void OCH1970_SetThre_Y1(uint8_t *value);    //value: 4 bytes (BOPY1_high,BOPY1_low,BRPY1_high,BRPY1_low)
void OCH1970_SetThre_Y2(uint8_t *value);    //value: 4 bytes (BOPY2_high,BOPY2_low,BRPY2_high,BRPY2_low)
void OCH1970_SetThre_Z1(uint8_t *value);    //value: 4 bytes (BOPZ1_high,BOPZ1_low,BRPZ1_high,BRPZ1_low)
void OCH1970_SetThre_Z2(uint8_t *value);    //value: 4 bytes (BOPZ2_high,BOPZ2_low,BRPZ2_high,BRPZ2_low)

//OD-INT pin output signal setting
void OCH1970_EN_ERRXY(uint8_t value);   //value: OCH_DISABLE or OCH_ENABLE
void OCH1970_EN_SWZ2(uint8_t value);
void OCH1970_EN_SWZ1(uint8_t value);
void OCH1970_EN_SWY2(uint8_t value);
void OCH1970_EN_SWY1(uint8_t value);
void OCH1970_EN_SWX2(uint8_t value);
void OCH1970_EN_SWX1(uint8_t value);
void OCH1970_EN_DRDY(uint8_t value);
void OCH1970_EN_ERRADC(uint8_t value);
void OCH1970_ODINTEN(uint8_t value);

//initial function
void OCH1970_Init(OCH1970_OPERATION_MODE value1,OCH1970_SDR_MODE value2,OCH1970_SMR_MODE value3);

//soft reset function
void OCH1970_SRST();

//get data function
void OCH1970_GetXYZ_HLdata(uint8_t *value);     //value: 6 bytes    Xhigh&low + Yhigh&low + Zhigh&low
void OCH1970_GetXYZ_HighData(uint8_t *value);   //value: 3 bytes    Xhigh + Yhigh + Zhigh
void OCH1970_GetX_HLdata(uint8_t *value);       //value: 2 bytes    Xhigh&low
void OCH1970_GetY_HLdata(uint8_t *value);       //value: 2 bytes    Yhigh&low
void OCH1970_GetZ_HLdata(uint8_t *value);       //value: 2 bytes    Zhigh&low
void OCH1970_GetStatus(uint8_t *value);         //value: 10 bytes  ERRXY,SWZ2,SWZ1,SWY2,SWY1,SWX2,SWX1,DRDY,DOR,ERRADC
uint8_t OCH1970_CheckDRDY();                    //return 1 bytes,1:data is ready; 0:data isn't ready
void OCH1970_CheckWIA(uint8_t *value);          //value: 2 bytes, check who i am

void OCH1970_MergeHighLowData(uint8_t *predata, int16_t *postdata,uint8_t length_pre);  //postdata=1/2length of predata
//merge high8bits and low8bits, unsigned int_8 ---> signed int_16
#endif
#endif
#include <stdio.h>
#include <stdint.h>
#include <termios.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>


const float Imin = 2.5;
/**
 * PI control variables
*/
typedef struct __pid __pid;

struct __pid
{
    void (*init)(__pid*,float,float,float);
    float (*calculatePI)(__pid *);
    float Kp;  
    float Ki;
    float error_integral;
    float previous_error;
    float error;
    float error_derivative;
    float output;
    float setPoint;
    float currentState;
    float dt;
};

__pid controlCC = {};
__pid controlCV = {};

/**
 * CAN-BUS variables
*/
typedef struct 
{
    uint8_t Data[8];
    uint16_t Length;
    uint32_t ID;
}CAN_msg_typedef;


CAN_msg_typedef Can_tx;
CAN_msg_typedef Can_rx;

/**
 * charging stages
*/
typedef enum
{
    STAGE_IDLE = 0,
    STAGE_CC,
    STAGE_CV,
}__chargingStage;

__chargingStage stage = STAGE_IDLE;
/**
 * CAN Messages ID
*/
enum
{
    CANBUS_HEARTBEAT = 0x701,
    CANBUS_CHARGE_EN = 0x201,
    CANBUS_CHARGE_STATUS = 0x181,
};

typedef struct 
{
    float Vref;
    float Iref;
    uint8_t chargeEn;
}__bmsRequest;

__bmsRequest bmsData = {.chargeEn = 0};
/**
 * can read write functions
*/

void CAN_write(CAN_msg_typedef *msg);
bool CAN_read(CAN_msg_typedef *msg); //return true if there is received msg


/**
 * PI control functions
*/
void initializePI(__pid *pidControl,float Kp,float Ki,float dt);
float PI_control(__pid *pidControl);
float getCurrentFeedback();
float getVoltageFeedback();

void Initialization(void);
float control_routine(void);
void main_state_machine(void);

void CAN_write_handler(uint32_t id,uint8_t *data,uint8_t size);
void CAN_read_handler(void);
void network_management(void);
 typedef struct 
 {
    uint32_t EPWM1_INT;
 }__PieVectTable;
 __PieVectTable PieVectTable;

int main()
{
    printf("%s : initialize CCCV \n",__func__);
    clock_t prevHeartBeat = 0;
    Initialization();
    PieVectTable.EPWM1_INT = 0;
    while(1)
    {
        // sleep(0.01);
        main_state_machine();
        PieVectTable.EPWM1_INT = control_routine();
        network_management();
    }
    return 0;
}

void initializePI(__pid *pidControl,float Kp,float Ki,float dt)
{
    printf("%s : Kp = %f Ki = %f dt = %f \n",__func__,Kp,Ki,dt);
    pidControl->init = initializePI;
    pidControl->calculatePI = PI_control;
    pidControl->Kp = Kp;  
    pidControl->Ki = Ki;
    pidControl->error_integral = 0.0;
    pidControl->previous_error = 0.0;
    pidControl->error = 0.0;
    pidControl->error_derivative = 0.0;
    pidControl->output = 0.0;
    pidControl->setPoint = 0.0;
    pidControl->currentState = 0.0;
    pidControl->dt = dt;
}

float PI_control(__pid *pidControl)
{

    pidControl->error = pidControl->setPoint - pidControl->currentState;

    pidControl->output = pidControl->Kp * pidControl->error + pidControl->Ki * pidControl->error_integral;

    pidControl->error_integral += pidControl->error * pidControl->dt;

    return pidControl->output;
}

float getCurrentFeedback()
{
    return 0.0;
}
float getVoltageFeedback()
{
    return 0.0;
}


void Initialization(void)
{
//initialize your variables here
    controlCC.init = initializePI;
    controlCC.calculatePI = PI_control;
    controlCV.init = initializePI;
    controlCV.calculatePI = PI_control;
    controlCC.init(&controlCC,0.1,0.1,0.01);
    controlCV.init(&controlCV,0.1,0.1,0.01);
}
float control_routine(void)
{
//run the control algorithm here
float  output = 0.0;
static clock_t prevControl = 0;
if((double)(clock() - prevControl) / CLOCKS_PER_SEC>= 0.01)
{
    switch(stage)
    {
        case STAGE_IDLE:
            //todo:: add stop charging
            break;
        case STAGE_CC:
            controlCC.setPoint = bmsData.Iref;
            output = controlCC.calculatePI(&controlCC);
            break;
        case STAGE_CV:
            controlCV.setPoint = bmsData.Vref;
            output = controlCV.calculatePI(&controlCV);
            break;
    } 
}
}
void main_state_machine(void)
{
    controlCC.currentState = getCurrentFeedback();
    controlCV.currentState = getVoltageFeedback();
    if(stage == STAGE_IDLE)
    {
        if(bmsData.chargeEn == 0x01)
        {
            stage =  STAGE_CC;
        }
    }
    if(stage == STAGE_CC)
    {
        if(controlCV.currentState >= bmsData.Vref)
        {
            stage =  STAGE_CV;
        }
    }
    if(stage == STAGE_CV)
    {
       if(controlCC.currentState <= Imin)
        {
            stage =  STAGE_IDLE;
            bmsData.chargeEn = 0;
        }
    }

}

void CAN_write_handler(uint32_t id,uint8_t *data,uint8_t size)
{
    Can_tx.ID = id,
    Can_tx.Length =size;
    for(int i=0;i<size;i++)
        Can_tx.Data[i] = data[i];
    CAN_write(&Can_tx);
}
void CAN_read_handler(void)
{
    if(CAN_read(&Can_rx))
    {
        switch (Can_rx.ID)
        {
        case CANBUS_CHARGE_EN:
            bmsData.Vref = (*(uint16_t *)(Can_rx.Data+0))/10;
            bmsData.Iref = (*(uint16_t *)(Can_rx.Data+2))/10;
            bmsData.chargeEn = Can_rx.Data[4];
            break;
        }
    }
}
void sendChargeStatus(float voltage,float current,uint8_t chargingStatus)
{
    uint8_t canBuff[8] = "";
    *(uint16_t *)canBuff = (uint16_t)(voltage*10);
    *(uint16_t *)(canBuff+2) = (uint16_t)(current *10);
    canBuff[4] = chargingStatus;
    
    
    CAN_write_handler(CANBUS_CHARGE_STATUS,canBuff,4);
}

void network_management(void)
{
    static clock_t prevHeartBeat = 0;
    static clock_t prevChargingStatus = 0;
    static uint8_t initialize = 0;

    if((double)(clock() - prevHeartBeat) / CLOCKS_PER_SEC>= 1)
    {
        prevHeartBeat = clock();
        uint8_t chargeStage = 0;
        if(!initialize)
        {
            initialize = 1;
            chargeStage = 0;
        }
        else
        {
            chargeStage = (stage == STAGE_IDLE) ? 0x01 : 0x02;
        }
        CAN_write_handler(CANBUS_HEARTBEAT,(uint8_t *)chargeStage,1);
    }
    
    if((double)(clock() - prevChargingStatus)/CLOCKS_PER_SEC >= 0.2 && bmsData.chargeEn)
    {
        prevChargingStatus = clock();
        sendChargeStatus(controlCV.currentState,controlCC.currentState,stage == STAGE_IDLE ? 0 : 1);
    }
    CAN_read_handler();

}

void CAN_write(CAN_msg_typedef *msg)
{

}
bool CAN_read(CAN_msg_typedef *msg)
{

}

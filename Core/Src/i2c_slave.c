/*

 *
 *  Created on: Dec 8, 2024
 *      Author: Bailey
 */


#include "main.h"
#include "i2c_slave.h"

uint8_t dataToSend[5] = { 'H', 'e', 'l', 'l', 'o' };

//i2c command flag
volatile uint8_t i2c_flag = I2C_FLAG_RESET;
//power saving mode flag
volatile uint8_t pwr_flag = PWR_NOR;

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
//extern uint8_t testnumber = 'c';

// Recieve buffer
uint8_t RxData[RxSIZE];

// Track # bytes sent/recieved 
uint8_t rxCount = 0;
uint8_t txCount = 0;

int firstByteRecieved = false; 

int countAddr = 0;   // # times AddrCallback is called
int countRxCplt = 0; // # times rxCplt is called
int countError = 0;  // # times error is called

uint8_t count = 0;

uint8_t i2c_flag_getter()
{
	return i2c_flag;
}

void i2c_flag_reset()
{
	i2c_flag = I2C_FLAG_RESET;
}


uint8_t pwr_flag_getter()
{
	return pwr_flag;
}

void pwr_flag_setter(uint8_t flag)
{
	pwr_flag = flag;
}

void process_data(void)
{
    switch(RxData[0])
    {
        case I2C_CMD_RESET:
            HAL_NVIC_SystemReset();
            break;
        case I2C_CMD_START:
        	i2c_flag = I2C_FLAG_SET;
            break;
        case I2C_CMD_PWRSAV: // turn off the 5V supply for the testing ICs
            //HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE);
        	turn_off_5v_plane();
        	pwr_flag_setter(PWR_SAV);
        	HAL_TIM_Base_Stop_IT(&htim2);
            break;
        case I2C_CMD_PWRNOR:
        	turn_on_5v_plane();
        	pwr_flag_setter(PWR_NOR);
        	HAL_TIM_Base_Start_IT(&htim2);
        	break;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // master is sending us data
	{
        rxCount = 0;
        countAddr++;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, (RxData + rxCount), 1, I2C_FIRST_FRAME);
	}
	else
	{
        txCount = 0;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, dataToSend, 1, I2C_FIRST_FRAME);
	}
}


//TODO: implement transmitting the information collected in the SD card back to OBC
//2 types of info: 1. data collected 2. error happened
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_ResumeTick();
    txCount++;
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (dataToSend + txCount), 1, I2C_NEXT_FRAME);

}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_ResumeTick();
    rxCount++;
    if (rxCount < RxSIZE)
    {
        if (rxCount == RxSIZE - 1)
        {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, (RxData + rxCount), 1, I2C_LAST_FRAME);
        }
        else
        {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, (RxData + rxCount), 1, I2C_NEXT_FRAME);
        }
    }

    if (rxCount == RxSIZE)
    {
        process_data();
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    countError++; 
    uint32_t errorCode = HAL_I2C_GetError(hi2c);

    if (errorCode == 4) // AF (ack failure): master stopped sending at less than RxSIZE
    {
        process_data();
    }


	HAL_I2C_EnableListen_IT(hi2c);
}

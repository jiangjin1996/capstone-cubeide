/*
 * i2c_slave.h
 *
 *  Created on: Nov 28, 2024
 *      Author: jiang35j
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#include "main.h"

#define false 0
#define true 1

#define RxSIZE	6

#define I2C_FLAG_RESET  0
#define I2C_FLAG_SET    1

#define PWR_SAV         1
#define PWR_NOR         0


#define I2C_CMD_RESET   97 // Reset the payload board 
#define I2C_CMD_STOP    98 // Stop all routines on the payload board 
#define I2C_CMD_START   99 // Forced start of single testing routine
#define I2C_CMD_NORMAL 100 // read sensors, write to sd card
#define I2C_CMD_PWRSAV 101 // Power saving mode, no routines till OBC says normal mode. 
#define I2C_CMD_PWRNOR 102 // Normal power mode
// TODO: add read commands

//this function let the main.c to read the i2c flag
uint8_t i2c_flag_getter();

//this function let the main.c to reset the i2c flag
void i2c_flag_reset();

uint8_t pwr_flag_getter();
void pwr_flag_setter(uint8_t flag);

//Start listening again after the ISR finished
extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c);

//address matching and transmit direction
extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);

//Receive call back function, which also parses the commands
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);

//TODO: finish Transmit function

//error call back function
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void increment();


#endif /* INC_I2C_SLAVE_H_ */

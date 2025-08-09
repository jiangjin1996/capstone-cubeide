/*
 * i2c_queue.h
 *
 *  Created on: Jul 20, 2025
 *      Author: Keerat Singh Tanwar
 */

#ifndef INC_I2C_QUEUE_H_
#define INC_I2C_QUEUE_H_

#define I2C_CMD_QUEUE_SIZE 10
#include <stdint.h>

static uint8_t i2c_cmd_queue[I2C_CMD_QUEUE_SIZE];
static uint8_t i2c_cmd_head = 0;
static uint8_t i2c_cmd_tail = 0;

void enqueue_i2c_cmd(uint8_t);

// Dequeue a command
uint8_t dequeue_i2c_cmd();

// Check if queue has any pending commands
uint8_t is_i2c_cmd_pending();



#endif /* INC_I2C_QUEUE_H_ */

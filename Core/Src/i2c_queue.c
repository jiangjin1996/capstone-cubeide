/*
 * i2c_queue.c
 *
 *  Created on: Jul 20, 2025
 *      Author: Keerat Singh Tanwar
 */


// Enqueue a command
void enqueue_i2c_cmd(uint8_t cmd) {
    uint8_t next = (i2c_cmd_head + 1) % I2C_CMD_QUEUE_SIZE;
    if (next != i2c_cmd_tail) {  // Check for overflow
        i2c_cmd_queue[i2c_cmd_head] = cmd;
        i2c_cmd_head = next;
        printf("I2C command added to queue!\r\n");
    } else {
        // Queue full — optional: log or handle overflow
        printf("I2C command queue full!\r\n");
    }
}

// Dequeue a command
uint8_t dequeue_i2c_cmd() {
    if (i2c_cmd_head == i2c_cmd_tail) {
        return 0;  // Empty queue
    }
    uint8_t cmd = i2c_cmd_queue[i2c_cmd_tail];
    i2c_cmd_tail = (i2c_cmd_tail + 1) % I2C_CMD_QUEUE_SIZE;
    return cmd;
}

// Check if queue has any pending commands
uint8_t is_i2c_cmd_pending() {
    return (i2c_cmd_head != i2c_cmd_tail);
}

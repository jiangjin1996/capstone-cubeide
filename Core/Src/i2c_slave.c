/*

 *
 *  Created on: Dec 8, 2024
 *      Author: Bailey
 */


#include "main.h"
#include "i2c_slave.h"
#include "sdcard.h"

uint8_t dataToSend[5] = { 'H', 'e', 'l', 'l', 'o' };
static const uint8_t busyMsg[] = "BUSY";

//i2c command flag
//try remove the static decoration if you find the flag cannot be changed by the setter function
static volatile uint8_t i2c_flag = I2C_FLAG_RESET;
//power saving mode flag
//try remove the static decoration if you find the flag cannot be changed by the setter function
static volatile uint8_t pwr_flag = PWR_NOR;
//Check whether the transmit buffer is empty
static volatile uint8_t tx_buf_empty_flag = BUF_NOT_EMPTY;

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
//extern uint8_t testnumber = 'c';

extern BYTE working_buff[TxSIZE];
extern FIL fil;
extern FRESULT fres; //Result after operations

// Receive buffer
uint8_t RxData[RxSIZE];
// Transmit buffer. Static because it will be accessed in the main.c
//maximum size is defined by the TxSIZE
static uint8_t TxBuffer[TxSIZE];

//dynamic size of the buffer
static uint16_t buf_size = 0;

// Track # bytes sent/received
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

uint8_t get_latest_s_file(char *outName, size_t outSize) {
    DIR dir;
    FILINFO fno;
    UINT idx;
    UINT maxIdx = 0;
    uint8_t found = false;

    // Open the root directory (or change "/" to your subfolder)
    if (f_opendir(&dir, "/") != FR_OK) {
        return false;
    }

    // Enumerate all entries
    for (;;) {
        if (f_readdir(&dir, &fno) != FR_OK || fno.fname[0] == 0) {
            break;  // error or end of dir
        }
        // Skip directories
        if (fno.fattrib & AM_DIR) {
            continue;
        }
        // Try to parse names like "S_<number>.CSV"
        if (sscanf(fno.fname, "S_%u.CSV", &idx) == 1) {
            if (!found || idx > maxIdx) {
                maxIdx = idx;
                found = true;
            }
        }
    }
    f_closedir(&dir);

    if (!found) {
        return false;
    }
    // Build the filename into outName
    snprintf(outName, outSize, "S_%u.CSV", maxIdx);
    return true;
}

/*
void load_buf()
{
	TCHAR *line;
	uint32_t bufIdx = 0;

	mount_sdcard();
	print_sdcard_stats();
	open_sdcard_file_read("S_0.CSV");

	// 2) Skip the header
	line = f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil);
	if (!line) {
		printf("Error: cannot read header (%d)\r\n", (int)fres);
		goto done;
	}

	// 3) Read & append each line
	while ((line = f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil)) != NULL) {
		uint16_t len = strlen((char*)working_buff);
		if (bufIdx + len > TxSIZE) {
			printf("Warning: TxBuffer overflow at %lu bytes\r\n", bufIdx);
			break;
		}
		// copy the full line (including trailing '\n')
		memcpy(&TxBuffer[bufIdx], working_buff, len);
		bufIdx += len;
	}
	buf_size = bufIdx;
	printf("Loaded %lu bytes into TxBuffer\r\n", bufIdx);

	// 4) Check for I/O error vs EOF
	if (f_error(&fil)) {
		printf("Read error (%d)\r\n", (int)fres);
	}

	done:
	// 5) Clean up

	close_sdcard_file();
	unmount_sdcard();
}
*/

//TODO this function only reads the data files. To read the error logs we need a different function
//TODO add time stamps to the files
//void load_buf(void) {
//    TCHAR *line, *tok;
//    uint16_t values[MAX_VALUES];
//    uint32_t valCount = 0, rowCount = 0;
//
//    mount_sdcard();
//    open_sdcard_file_read("S_0.CSV");
//
//    // 1) Skip header
//    f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil);
//
//    // 2) Read up to ROWS_TO_SEND rows
//    while (rowCount < ROWS_TO_SEND
//        && (line = f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil))
//        != NULL)
//    {
//        // tokenize the line on commas
//        tok = strtok((char*)line, ",");
//        uint32_t fld = 0;
//        while (tok && fld < FIELDS_PER_ROW) {
//            if (valCount < MAX_VALUES) {
//                // parse and store as uint16_t
//                values[valCount++] = (uint16_t)atoi(tok);
//            }
//            fld++;
//            tok = strtok(NULL, ",");
//        }
//        rowCount++;
//    }
//
//    // 3) Pack into TxBuffer as little‑endian words
//    for (uint32_t i = 0; i < valCount; i++) {
//        TxBuffer[2*i    ] = (uint8_t)(values[i] & 0xFF);
//        TxBuffer[2*i + 1] = (uint8_t)(values[i] >> 8);
//    }
//    buf_size = valCount * 2;  // total bytes to send
//
//
//    close_sdcard_file();
//    unmount_sdcard();
//}

void load_buf(void) {
    char     filename[LATEST_NAME_MAX];
    uint16_t values[MAX_VALUES];
    uint32_t valCount, offset;

    mount_sdcard();

    // 1) Determine latest file
    if (!get_latest_s_file(filename, sizeof(filename))) {
        printf("No S_*.CSV files found!\r\n");
        return;
    }
    printf("Loading latest file: %s\r\n", filename);

    // 2) Open file
    open_sdcard_file_read(filename);

    // 3) Parse CSV rows into 'values'
    valCount = parse_csv_rows(values);

    // 4) Pack numeric values into TxBuffer
    offset = pack_values(values, valCount, TxBuffer);

    // 5) Append file timestamp and get total size
    buf_size = offset + append_file_timestamp(filename, TxBuffer, offset);

    // 6) Cleanup
    close_sdcard_file();
    unmount_sdcard();
}

//------------------------------------------------------------------------------
// Read up to ROWS_TO_SEND lines (skipping header) and parse into values[]
//------------------------------------------------------------------------------
static uint32_t parse_csv_rows(uint16_t *values) {
    TCHAR   *line;
    char    *tok;
    uint32_t rowCount = 0, valCount = 0;

    // Skip header
    line = f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil);
    if (!line) {
        printf("Error skipping header (%d)\r\n", (int)fres);
        return 0;
    }
    // Read rows
    while (rowCount < ROWS_TO_SEND &&
           (line = f_gets((TCHAR*)working_buff, MAX_LINE_LEN, &fil)) != NULL) {
        tok = strtok((char*)line, ",");
        for (uint32_t fld = 0; tok && fld < FIELDS_PER_ROW; fld++) {
            if (valCount < MAX_VALUES) {
                values[valCount++] = (uint16_t)atoi(tok);
            }
            tok = strtok(NULL, ",");
        }
        rowCount++;
    }
    return valCount;
}

//------------------------------------------------------------------------------
// Pack 'count' uint16 values into 'buffer' (little-endian). Returns byte count.
//------------------------------------------------------------------------------
static uint32_t pack_values(const uint16_t *values, uint32_t count, uint8_t *buffer) {
    uint32_t offset = 0;
    for (uint32_t i = 0; i < count; i++) {
        buffer[offset++] = (uint8_t)(values[i] & 0xFF);
        buffer[offset++] = (uint8_t)(values[i] >> 8);
    }
    return offset;
}

//------------------------------------------------------------------------------
// Append ASCII timestamp from file metadata; returns length of appended text
//------------------------------------------------------------------------------
static uint32_t append_file_timestamp(const char *filename, uint8_t *buffer, uint32_t offset) {
    FILINFO finfo;
    char    tsbuf[32];
    uint32_t len = 0;

    if (f_stat(filename, &finfo) == FR_OK) {
        uint16_t fdate = finfo.fdate;
        uint16_t ftime = finfo.ftime;
        uint16_t year  = ((fdate >> 9) & 0x7F) + 1980;
        uint8_t  month = (fdate >> 5) & 0x0F;
        uint8_t  day   = fdate & 0x1F;
        uint8_t  hour  = (ftime >> 11) & 0x1F;
        uint8_t  minute= (ftime >> 5) & 0x3F;
        uint8_t  second= (ftime & 0x1F) * 2;

        int tslen = snprintf(tsbuf, sizeof(tsbuf),
            "TS:%04u-%02u-%02u %02u:%02u:%02u\r\n",
            year, month, day, hour, minute, second
        );
        if (tslen > 0 && offset + tslen <= TxSIZE) {
            memcpy(&buffer[offset], tsbuf, tslen);
            len = tslen;
        }
    } else {
        printf("f_stat failed (%d)\r\n", (int)fres);
    }
    return len;
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
        	turn_off_5v_plane();
        	pwr_flag_setter(PWR_SAV);
        	HAL_TIM_Base_Stop_IT(&htim2);
            break;
        case I2C_CMD_PWRNOR:
        	turn_on_5v_plane();
        	pwr_flag_setter(PWR_NOR);
        	HAL_TIM_Base_Start_IT(&htim2);
        	break;
        	//TODO add the transmit commands
        case I2C_CMD_SEND_DATA:
        	i2c_flag = I2C_FLAG_READ_DATA;
        	break;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
	HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
	if (busy_flag_getter()) {
		// We’re busy—immediately NACK or send a “BUSY” packet
		// Here we choose to send back a 4-byte ASCII “BUSY”
		HAL_I2C_Slave_Seq_Transmit_IT(
			hi2c,
			(uint8_t*)busyMsg,
			sizeof(busyMsg)-1,
			I2C_FIRST_FRAME
		);
		return;
	}
	if (TransferDirection == I2C_DIRECTION_TRANSMIT) // master is sending us data
	{
        rxCount = 0;
        countAddr++;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, (RxData + rxCount), 1, I2C_FIRST_FRAME);
	}
	else
	{
        txCount = 0;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, TxBuffer, 1, I2C_FIRST_FRAME);
	}
}


//TODO: implement transmitting the information collected in the SD card back to OBC
//2 types of info: 1. data collected, 2. error log
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    txCount++;
    if(txCount < buf_size){
    	HAL_I2C_Slave_Seq_Transmit_IT(hi2c, (TxBuffer + txCount), 1, I2C_NEXT_FRAME);
    	//i2c_flag = I2C_FLAG_READ_DATA;
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	process_data();
	/*
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
    */
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

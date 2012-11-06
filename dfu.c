/* *****************************************************************************
 * The MIT License
 *
 * Copyright (c) 2012 Silicon Laboratories LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ****************************************************************************/

// See Silabs AN533 for Serial DFU standard.

#include "hardware.h"

// Communication Interface
void Comm_Init(void);
uint32_t Comm_Receive(uint8_t *rx_buff, uint32_t length);
uint32_t Comm_Transmit(uint8_t *tx_buff, uint32_t length);

// Firmware Image File Manager
uint32_t Get_Block_Size(void);
uint32_t Init_Dnload(uint8_t *buffer, uint32_t length);
uint32_t Cont_Dnload(uint8_t *buffer, uint32_t length);
uint32_t Finish_Dnload(void);
uint32_t Init_Upload(uint8_t *buffer, uint32_t length);
uint32_t Cont_Upload(uint8_t *buffer, uint32_t length);

// Flash Control Interface
uint32_t Flash_Read(uint8_t *buffer, uint32_t address, uint32_t length);
uint32_t Flash_Write(uint8_t *buffer, uint32_t address, uint32_t length);
uint32_t Flash_Erase(uint32_t address, uint32_t num_sectors);
uint32_t Get_Sector_Size(void);


//**************************************************************************************************
//                                  Communication Interface
//**************************************************************************************************
// Description: Initializes communication interface.
void Comm_Init(void)
{
    return;
}

// Description: Blocking function. Receives up to <length> bytes over comm interface and stores to
//              <rx_buff>.
// Return: Status code indicating validity of data and number of bytes received.
uint32_t Comm_Receive(uint8_t *rx_buff, uint32_t length)
{
    return 0;
}

// Description: Blocking function transmits <length> bytes from <tx_buff> over comm interface and
//              waits for acknowledgment.
// Return: Status code, success/failure.
uint32_t Comm_Transmit(uint8_t *tx_buff, uint32_t length)
{
    return 0;
}

//**************************************************************************************************
//                              Firmware Image File Manager
//**************************************************************************************************
// Description: Returns block size of program memory. Typically flash sector size.
uint32_t Get_Block_Size(void)
{
    return 0;
}

// Description: Verifies the first block of firmware image file and erases app code space.
// Returns: Status code, success/failure.
uint32_t Init_Dnload(uint8_t *buffer, uint32_t length)
{
    return 0;
}

// Description: Writes a block of firmware image file to app space. Blocks must be sent consecutively.
// Return: Status code with the number of bytes written.
uint32_t Cont_Dnload(uint8_t *buffer, uint32_t length)
{
    return 0;
}

// Description: Returns status code, success/failure.
uint32_t Finish_Dnload(void)
{
    return 0;
}

// Description: Fills <buffer> with first block of firmware image.
// Return: Status code indicating number of valid bytes in <buffer>
uint32_t Init_Upload(uint8_t *buffer, uint32_t length)
{
    return 0;
}

// Description: Copies a block from app space into <buffer>. Blocks must be sent consecutively.
// Return: Status code with number of bytes written into <buffer>. Zero indicates upload is complete.
uint32_t Cont_Upload(uint8_t *buffer, uint32_t length)
{
    return 0;
}

//**************************************************************************************************
//                              Flash Control Interface
//**************************************************************************************************
// Description: Copies <length> bytes from flash starting at <address> into <buffer>.
// Return: Number of bytes read.
uint32_t Flash_Read(uint8_t *buffer, uint32_t address, uint32_t length)
{

    return 0;
}

// Description: Copies <length> bytes from buffer to flash starting at <address>.
// Return: Number of bytes written.
uint32_t Flash_Write(uint8_t *buffer, uint32_t address, uint32_t length)
{

    return 0;
}

// Description: Erases <num_sectors> in flash starting at <address>.
// Return: Number of sectors erased.
uint32_t Flash_Erase(uint32_t address, uint32_t num_sectors)
{

    return 0;
}

// Description: Returns number of bytes in each sector.
uint32_t Get_Sector_Size(void)
{

    return 0;
}

//**************************************************************************************************
//                                      Miscellaneous
//**************************************************************************************************
uint32_t Image_Valid(void)
{
    return 1;
}

//**************************************************************************************************
//                                      State Machine
//**************************************************************************************************
typedef enum command_num {
    command_dnload =        1,
    command_upload =        2,
    command_get_status =    3,
    command_clr_status =    4,
    command_get_state =     5,
    command_abort =         6,
    command_reset =         7
} command_num;

typedef enum state_num {
    exit_dfu =              0,
    state_idle =            2,
    state_dnload_sync =     3,
    state_dnbusy =          4,
    state_dnload_idle =     5,
    state_mani_sync =       6,
    state_upld_idle =       9,
    state_error =           10
} state_num;

// Description: Idle state.
state_num State_Idle(void)
{
    state_num state;
    command_num cmd;


    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            break;
        case command_dnload:
            return state_dnload_sync;
        case command_upload:
            return state_upld_idle;
        case command_reset:
            return exit_dfu;
        default:
            return state_error;
        }

    }
    return state;
}

// Description: Synchronize firmware downloads.
state_num State_Dnload_Sync(void)
{
    state_num state;
    command_num cmd;

    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            return 1 ? state_dnbusy : state_dnload_idle;
        default:
            return state_error;
        }

    }
    return state;
}

// Description: Device is updating program memory. Can't receive communication in this state.
state_num State_DnBusy(void)
{
    int32_t cntr = 20000;
    while (cntr--);
    return state_dnload_sync;
}

// Description: Syncs firmware downloads. Indicates block complete.
state_num State_Dnload_Idle(void)
{
    state_num state;
    command_num cmd;

    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            break;
        case command_dnload:
            return 1 ? state_dnload_sync : state_mani_sync;
        default:
            return state_error;
        }

    }
    return state;
}

// Description: Provides current state information to master programmer.
state_num State_Manifest_Sync(void)
{
    state_num state;
    command_num cmd;

    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            return state_idle;
        default:
            return state_error;
        }

    }
    return state;
}

// Description: Uploads firmware image from target MCU.
state_num State_Upload_Idle(void)
{
    state_num state;
    command_num cmd;

    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            break;
        case command_upload:
            if (1) {
                continue;
            }
            return state_idle;
        default:
            return state_error;
        }

    }
    return state;
}

// Description: Used to indicate error condition.
state_num State_Error(void)
{
    state_num state;
    command_num cmd;

    cmd = command_reset;

    while (1) {

        switch (cmd) {
        case command_get_state:
            break;
        case command_get_status:
            break;
        case command_clr_status:
            return state_idle;
        default:
            break;
        }

    }
    return state;
}

state_num (*g_states[])(void) =
            {0,             0,                  State_Idle,             State_Dnload_Sync,
            State_DnBusy,   State_Dnload_Idle,  State_Manifest_Sync,    0,
            0,              State_Upload_Idle,  State_Error};

//g_states[2] = {State_Idle, State_Dnload_Sync};
void Dfu_State_Mchn(void)
{
    state_num current_state;
    current_state = Image_Valid() ? state_idle : state_error;

    do {
        current_state = g_states[current_state]();
    } while(current_state);

    // Return control to bootloader
    return;
}

/*
 * i2c_download_protocol.h
 *
 *  Created on: Oct 6, 2023
 *      Author: a5123412
 */

#ifndef DOWNLOADER_I2C_DOWNLOAD_PROTOCOL_H_
#define DOWNLOADER_I2C_DOWNLOAD_PROTOCOL_H_

#include "Config_IICA0.h"

/***********************************************************************************************************************
 * Macro definitions
 **********************************************************************************************************************/
/* Command / Response relative data size */
#define ONE_BYTE                        ( 0x01 )
#define TWO_BYTE                        ( 0x02 )
#define THREE_BYTE                      ( 0x03 )
#define DATA_STATUS_RESPONSE_BYTES      ( 0x07 )  // for status OK, status ERR

/* Command Definition whichever consumed here */
#define DATA_STATUS_OK_RESPONSE_SIZE    ( 0x07 )
#define DEVICE_INQUIRY_CMD_SIZE         ( 0x06 )
#define ID_AUTH_CMD_SIZE                ( 0x16 )
#define SIGNATURE_REQUEST_CMD_SIZE      ( 0x06 )
#define SIGNATURE_DATA_SIZE             ( 0x19 )
#define MEMORY_AREA_INFO_CMD_SIZE       ( 0x07 )
#define MEMORY_AREA_INFO_DATA_SIZE      ( 0x1B )
#define ERASE_CMD_SIZE                  ( 0x0E )
#define ERASE_DATA_SIZE                 ( 0x07 )
#define WRITE_CMD_SIZE                  ( 0x0E )
#define WRITE_DATA_STATUS_RESP_SIZE     ( 0x07 )
#define WRITE_DATA_HEADER_SIZE          ( 0x04 )
#define READ_CMD_SIZE                   ( 0x0E )
#define READ_DATA_STATUS_RESP_SIZE      ( 0x07 )
#define JUMP_APP_CMD_SIZE               ( 0x08 )
#define JUMP_BOOT_CMD_SIZE              ( 0x06 )
#define JUMP_DATA_STATUS_RESP_SIZE      ( 0x07 )

#define BOOT_MODE_CHECK_CMD             ( 0x55 )
#define BOOT_MODE_ACK                   ( 0xC3 )
#define INVALID_CMD_BYTE                ( 0xFF )
#define INQUIRY_CMD_BYTE                ( 0x00 )
#define ERASE_CMD_BYTE                  ( 0x12 )
#define WRITE_CMD_BYTE                  ( 0x13 )
#define READ_CMD_BYTE                   ( 0x15 )
#define ID_AUTHENTICATION_CMD_BYTE      ( 0x30 )
#define SIGNATURE_CMD_BYTE              ( 0x3A )
#define MEMORY_AREA_INFO_CMD_BYTE       ( 0x3B )
#define SWTICH_APP_CMD_BYTE             ( 0x3C )
#define SWTICH_BOOT_CMD_BYTE            ( 0x3D )

#define COMMAND_PACKET_START            ( 0x01 )
#define DATA_PACKET_START               ( 0x81 )
#define END_OF_PACKET                   ( 0x03 )
#define INQUIRY_RESP_ERROR_CODE         ( 0x80 )

#define MAXIMUM_RECV_PACKET_SIZE        ( 262 )                     // SOD + LNH + LNL + RES + DAT(1-256bytes) + SUM + ETX
#define MAXIMUM_RESP_PACKET_SIZE        MAXIMUM_RECV_PACKET_SIZE

#define STATUS_CODE_OK                  ( 0x00 )
#define STATUS_CODE_UNSUPPORTED_CMD     ( 0xC0 )
#define STATUS_CODE_PACKET_ERROR        ( 0xC1 )
#define STATUS_CODE_CHECKSUM_ERROR      ( 0xC2 )
#define STATUS_CODE_FLOW_ERROR          ( 0xC3 )
#define STATUS_CODE_ADDRESS_ERROR       ( 0xD0 )
#define STATUS_CODE_ERASE_ERROR         ( 0xE1 )
#define STATUS_CODE_WRITE_ERROR         ( 0xE2 )
#define STATUS_CODE_SEQUENCER_ERROR     ( 0xE7 )

/* iic downloader operations event ids */
#define ERR_I2C_ABORTED_EVT             ( 1 << 0 )
#define ERR_I2C_RX_TRANSFER_EVT         ( 1 << 1 )
#define ERR_I2C_TX_TRANSFER_EVT         ( 1 << 2 )
#define USER_AREA_START_ERASE_EVT       ( 1 << 3 )
#define IMAGE_START_TRANSFER_EVT        ( 1 << 4 )
#define IMAGE_TRANSFER_EVT              ( 1 << 5 )
#define MEMORY_READ_START_EVT           ( 1 << 6 )
#define MEMORY_READ_NEXT_EVT            ( 1 << 7 )
#define JUMP_TO_APP                     ( 1 << 8 )

/***********************************************************************************************************************
 * Typedef definitions
 **********************************************************************************************************************/
typedef enum e_if_resp_data_type
{
    NONE_RESP = 0,
    BOOT_MODE_ACK_RESPOND = 1,
    INQUIRY_RESPOND = 2,
    ERASE_MEMORY = 3,
    WRITE_MEMORY_START = 4,
    WRITE_MEMORY_NEXT = 5,
    READ_MEMORY_START = 6,
    READ_MEMORY_NEXT = 7,
    SIGNATURE_RESPOND = 8,
    AREA_INFO_RESPOND = 9,
    SWITCH_APP = 10,
    SWITCH_BOOT = 11,
    PACKET_ERROR = 12,
    CHECKSUM_ERROR = 13,
    ADDRESS_ERROR = 14,
    WRITE_ERROR = 15,
    FLOW_ERROR = 16,
    SEQUENCER_ERROR = 17,
    UNSUPPORT_ERROR = 18,
    AUTH_RESPOND = 19,
}if_resp_data_type_t;

typedef enum e_operation_phase
{
    COMMUNICATION_SETTING_PHASE = 0,
    AUTHENTICATION_PHASE = 1,
    COMMAND_ACCEPTABLE_PHASE = 2,
}operation_phase_t;

typedef struct st_i2c_downloader_ctrl
{
    uint8_t                     * p_rxbuff;
    uint8_t                     * p_txbuff;
    uint16_t volatile             rxbyteCnt;
    uint16_t volatile             txbyteCnt;
    uint8_t volatile              recv_cmd;
    uint8_t volatile              area_num;
    if_resp_data_type_t volatile  resp_type;
    operation_phase_t             downloader_operation_phase;
    uint16_t                      image_version;
    uint16_t volatile             write_data_length;
    uint16_t volatile             read_data_length;
}i2c_downloader_ctrl_t;

/***********************************************************************************************************************
 * Public Function Prototypes
 **********************************************************************************************************************/
//fsp_err_t init_iic_interface(i2c_downloader_ctrl_t * p_downloader_ctrl);
//fsp_err_t deinit_iic_interface(void);
//int target_recv_data_size_set(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl);


#endif /* DOWNLOADER_I2C_DOWNLOAD_PROTOCOL_H_ */

/*_____ I N C L U D E S ____________________________________________________*/
#include <stdio.h>
#include <string.h>

#include "r_smc_entry.h"
#include "platform.h"

#include "misc_config.h"
#include "custom_func.h"

#include "i2c_download_protocol.h"
/*_____ D E C L A R A T I O N S ____________________________________________*/

volatile volatile struct flag_32bit flag_PROJ_CTL;
#define FLAG_PROJ_TIMER_PERIOD_1000MS                 	(flag_PROJ_CTL.bit0)
#define FLAG_PROJ_TRIG_BTN1                       	    (flag_PROJ_CTL.bit1)
#define FLAG_PROJ_TRIG_BTN2                 	        (flag_PROJ_CTL.bit2)
#define FLAG_PROJ_REVERSE3                    		    (flag_PROJ_CTL.bit3)
#define FLAG_PROJ_REVERSE4                              (flag_PROJ_CTL.bit4)
#define FLAG_PROJ_REVERSE5                              (flag_PROJ_CTL.bit5)
#define FLAG_PROJ_REVERSE6                              (flag_PROJ_CTL.bit6)
#define FLAG_PROJ_REVERSE7                              (flag_PROJ_CTL.bit7)


#define FLAG_PROJ_TRIG_1                                (flag_PROJ_CTL.bit8)
#define FLAG_PROJ_TRIG_2                                (flag_PROJ_CTL.bit9)
#define FLAG_PROJ_TRIG_3                                (flag_PROJ_CTL.bit10)
#define FLAG_PROJ_TRIG_4                                (flag_PROJ_CTL.bit11)
#define FLAG_PROJ_TRIG_5                                (flag_PROJ_CTL.bit12)
#define FLAG_PROJ_REVERSE13                             (flag_PROJ_CTL.bit13)
#define FLAG_PROJ_REVERSE14                             (flag_PROJ_CTL.bit14)
#define FLAG_PROJ_REVERSE15                             (flag_PROJ_CTL.bit15)

/*_____ D E F I N I T I O N S ______________________________________________*/

volatile unsigned long counter_tick = 0;
volatile unsigned long btn_counter_tick = 0;

#define BTN_PRESSED_LONG                                (2500)

#define SW						                        P13_bit.no7

#define TARGET_DEVICE_NOA                               ( 1 )       // Number of recordable areas (user area in code flash, data flash)
#define TARGET_DEVICE_TYPE                              ( 0x02 )    // 0x02(RA2/RA4), 0x03(RA6)
#define BOOT_FIRMWARE_MAJOR_VERSION                     ( 1 )
#define BOOT_FIRMWARE_MINOR_VERSION                     ( 0 )
#define MAX_CODE_FLASH_WRITE_DATA_SIZE                  (256)

#define MAX_FLASH_READ_DATA_SIZE                        (256)
#define FLASH_END_BLOCK			                        256ul
#define	FLASH_AREA				                        0x5000  // app code addresss
#define FLASH_BYTE_PER_BLOCK	                        1024ul
#define FLASH_END_ADDRESS		                        ((FLASH_END_BLOCK*FLASH_BYTE_PER_BLOCK)-1-FLASH_BYTE_PER_BLOCK)
#define FLASH_AP_SIZE		                            (FLASH_END_BLOCK*FLASH_BYTE_PER_BLOCK-FLASH_AREA-4-FLASH_BYTE_PER_BLOCK)

//programming relative data
uint32_t volatile   flash_memory_start_address = 0;
uint32_t volatile   flash_memory_end_address = 0;
uint32_t volatile   erase_area_start_address = 0;
uint32_t volatile   erase_area_end_address = 0;
// uint32_t volatile   read_area_start_address = 0;
// uint32_t volatile   read_area_end_address = 0;
uint32_t volatile   flash_target_memory_size = 0;
uint32_t volatile   flash_memory_destination_address = 0;
volatile bool g_flash_operation_progressing = false;
volatile bool g_app_required_update = false;
volatile bool slv_write_start_flag = false;

//I2C relative data
uint8_t DataBuf[MAXIMUM_RECV_PACKET_SIZE] = {0};
uint8_t ResponseBuf[MAXIMUM_RESP_PACKET_SIZE] = {0};
i2c_downloader_ctrl_t i2c_downloader_ctrl;
volatile uint8_t 	downloader_iica0_slave_status_flag;     /* iica0 slave flag */
volatile uint16_t 	downloader_iica0_rx_cnt;                /* iica0 receive data count */
volatile uint16_t 	downloader_iica0_tx_cnt;                /* iica0 send data count */

const uint8_t device_auth_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, ID_AUTHENTICATION_CMD_BYTE, STATUS_CODE_OK, 0xCE, END_OF_PACKET};
const uint8_t device_inquiry_data_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, INQUIRY_CMD_BYTE, STATUS_CODE_OK, 0xFE, END_OF_PACKET};
const uint8_t device_memory_erase_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, ERASE_CMD_BYTE, STATUS_CODE_OK, 0xEC, END_OF_PACKET};
const uint8_t device_memory_write_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, WRITE_CMD_BYTE, STATUS_CODE_OK, 0xEB, END_OF_PACKET};
const uint8_t device_memory_read_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, READ_CMD_BYTE, STATUS_CODE_OK, 0xE9, END_OF_PACKET};
const uint8_t device_memory_read_packet_error[7] = {DATA_PACKET_START, 0x00, 0x02, 0x95, STATUS_CODE_PACKET_ERROR, 0xA8, END_OF_PACKET};
const uint8_t device_memory_read_checksum_error[7] = {DATA_PACKET_START, 0x00, 0x02, 0x95, STATUS_CODE_CHECKSUM_ERROR, 0xA7, END_OF_PACKET};
const uint8_t device_memory_switch_app_ok[7] = {DATA_PACKET_START, 0x00, 0x02, SWTICH_APP_CMD_BYTE, STATUS_CODE_OK, 0xC2, END_OF_PACKET};

const uint8_t device_part_number[13] = "RL78_F24_FPB";

uint8_t			status;

// void ErrorHandler(uint8_t);
// uint8_t WriteCodeFlash(uint32_t u32_start_addr, uint8_t * u8_write_data, uint32_t u32_write_data_len);

void I2C_Downloader_routine(void);
static void transmit_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl);
static void received_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl);
static bool checksum_verification(uint8_t const * p_input, uint16_t len);
static uint8_t checksum_calculate(uint8_t const * p_input, uint16_t data_len);
static void assemble_error_data_packet(uint8_t * txdata, uint8_t associated_cmd, uint8_t error_code);
static void encode16bit(uint32_t input_data, uint8_t * output_array);
static uint16_t decode16bit(uint8_t * data);
// static uint32_t decode32bit(uint8_t * data);
// static bool Is_target_address_valid(uint8_t *data);

void __near I2C_Downloader_routine_IRQ(void);

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

unsigned long btn_get_tick(void)
{
	return (btn_counter_tick);
}

void btn_set_tick(unsigned long t)
{
	btn_counter_tick = t;
}

void btn_tick_counter(void)
{
	btn_counter_tick++;
    if (btn_get_tick() >= 60000)
    {
        btn_set_tick(0);
    }
}

unsigned long get_tick(void)
{
	return (counter_tick);
}

void set_tick(unsigned long t)
{
	counter_tick = t;
}

void tick_counter(void)
{
	counter_tick++;
    if (get_tick() >= 60000)
    {
        set_tick(0);
    }
}

void delay_ms(unsigned long ms)
{
	#if 1
    unsigned long tickstart = get_tick();
    unsigned long wait = ms;
	unsigned long tmp = 0;
	
    while (1)
    {
		if (get_tick() > tickstart)	// tickstart = 59000 , tick_counter = 60000
		{
			tmp = get_tick() - tickstart;
		}
		else // tickstart = 59000 , tick_counter = 2048
		{
			tmp = 60000 -  tickstart + get_tick();
		}		
		
		if (tmp > wait)
			break;
    }
	
	#else
	TIMER_Delay(TIMER0, 1000*ms);
	#endif
}



static int target_recv_data_size_set(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
	int 	status = 0;
	uint8_t start_byte = p_i2c_downloader_ctrl->p_rxbuff[0]; //First byte is Generic Code or the header byte

    switch(p_i2c_downloader_ctrl->recv_cmd)
    {
    	case ID_AUTHENTICATION_CMD_BYTE:
    	{
    		p_i2c_downloader_ctrl->rxbyteCnt = ID_AUTH_CMD_SIZE;
    		break;
    	}

		case INQUIRY_CMD_BYTE:
		{
			p_i2c_downloader_ctrl->rxbyteCnt = DEVICE_INQUIRY_CMD_SIZE;
			break;
		}

        case ERASE_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = ERASE_CMD_SIZE;
            break;
        }

        case SIGNATURE_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = SIGNATURE_REQUEST_CMD_SIZE;
            break;
        }

        case MEMORY_AREA_INFO_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = MEMORY_AREA_INFO_CMD_SIZE;
            break;
        }

        case SWTICH_APP_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = JUMP_APP_CMD_SIZE;
            break;
        }

        case WRITE_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = WRITE_CMD_SIZE;
            }
            else if(start_byte == DATA_PACKET_START)
            {
            	p_i2c_downloader_ctrl->write_data_length = (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1); // Length of "RES+DATA"
            	p_i2c_downloader_ctrl->rxbyteCnt = p_i2c_downloader_ctrl->write_data_length + WRITE_DATA_HEADER_SIZE + 2;
            }
            break;
        }

        case READ_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = READ_CMD_SIZE;
            }
            else if(start_byte == DATA_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = READ_DATA_STATUS_RESP_SIZE;
            }
            break;
        }

        default:
        	status = -1;
        	break;
    }

    return status;
}

static void received_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
    uint8_t     start_byte = p_i2c_downloader_ctrl->p_rxbuff[0]; //First byte is Generic Code or the header byte
    uint8_t     end_byte = p_i2c_downloader_ctrl->p_rxbuff[p_i2c_downloader_ctrl->rxbyteCnt - 1]; //End of packet
    // uint8_t     i = 0;

    p_i2c_downloader_ctrl->recv_cmd = p_i2c_downloader_ctrl->p_rxbuff[3];

    /* check the ETX to make sure we receive a complete frame */
    if(end_byte != END_OF_PACKET)
    {
        p_i2c_downloader_ctrl->resp_type = PACKET_ERROR;
        return;
    }

    /* check the SUM to make sure the integrity of received frame */
    if(false == checksum_verification(&p_i2c_downloader_ctrl->p_rxbuff[0], p_i2c_downloader_ctrl->rxbyteCnt))
    {
        p_i2c_downloader_ctrl->resp_type = CHECKSUM_ERROR;
        return;
    }

    if(AUTHENTICATION_PHASE == p_i2c_downloader_ctrl->downloader_operation_phase)
    {
        if((start_byte == COMMAND_PACKET_START) && (p_i2c_downloader_ctrl->recv_cmd == INQUIRY_CMD_BYTE) &&
           (DEVICE_INQUIRY_CMD_SIZE == p_i2c_downloader_ctrl->rxbyteCnt))
        {
            p_i2c_downloader_ctrl->resp_type = INQUIRY_RESPOND;
            p_i2c_downloader_ctrl->downloader_operation_phase = COMMAND_ACCEPTABLE_PHASE;
        }
        else
        {
            p_i2c_downloader_ctrl->resp_type = FLOW_ERROR;
        }
        return;
    }

    /* check whether this is a Command packet or a Data packet */
    switch(p_i2c_downloader_ctrl->recv_cmd)
    {
        case INQUIRY_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->resp_type = INQUIRY_RESPOND;
            break;
        }

        case ERASE_CMD_BYTE:
        {
            // if(false == Is_target_address_valid(&p_i2c_downloader_ctrl->p_rxbuff[4]))
            // {
            //     p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
            //     return;
            // }

            // erase_area_start_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[6]);
            // erase_area_end_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[10]);

            // erase_area_start_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
            // erase_area_end_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[8]);

            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[7]& 0xFF;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[6]<< 8;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[5]<< 16;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[4]<< 24;

            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[11]& 0xFF;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[10]<< 8;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[9]<< 16;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[8]<< 24;               

            // printf_tiny("(slv)received_handler erase_area_start:0x%04X," ,erase_area_start_address>>16);
            // printf_tiny("0x%04X\r\n" ,erase_area_start_address&0xFFFF);
            // printf_tiny("(slv)received_handler erase_area_end:0x%04X," ,erase_area_end_address>>16);
            // printf_tiny("0x%04X\r\n" ,erase_area_end_address&0xFFFF);
            // for( i = 0 ; i < 15; i++)
            // {
            //     printf_tiny("(slv)received_handler erase_area_end_address:(%d)" ,i);
            //     printf_tiny("0x%02X\r\n" ,p_i2c_downloader_ctrl->p_rxbuff[i]);
            // }

            p_i2c_downloader_ctrl->resp_type = ERASE_MEMORY;
            g_flash_operation_progressing = true;
            break;
        }

        case WRITE_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
                /* receive command packet of write transfer */
                // if(false == Is_target_address_valid(&p_i2c_downloader_ctrl->p_rxbuff[4]))
                // {
                //     p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
                //     return;
                // }

                // flash_memory_start_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[6]);
                // flash_memory_end_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[10]);

                // flash_memory_start_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
                // flash_memory_end_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[8]);

                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[7]& 0xFF;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[6]<< 8;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[5]<< 16;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[4]<< 24;

                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[11]& 0xFF;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[10]<< 8;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[9]<< 16;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[8]<< 24;

                // printf_tiny("(slv)received_handler flash_memory_start:0x%04X," ,flash_memory_start_address>>16);
                // printf_tiny("0x%04X\r\n" ,flash_memory_start_address&0xFFFF);
                
                // printf_tiny("(slv)received_handler flash_memory_end:0x%04X," ,flash_memory_end_address>>16);
                // printf_tiny("0x%04X\r\n" ,flash_memory_end_address&0xFFFF);

                flash_memory_destination_address = flash_memory_start_address;
                flash_target_memory_size = flash_memory_end_address - flash_memory_start_address + 1;
                p_i2c_downloader_ctrl->resp_type = WRITE_MEMORY_START;
                g_flash_operation_progressing = true;
            }
            else if(start_byte == DATA_PACKET_START)
            {
                if(!g_flash_operation_progressing)
                {
                    p_i2c_downloader_ctrl->resp_type = SEQUENCER_ERROR;
                    return;
                }

                /* receive data packet of write transfer */
                p_i2c_downloader_ctrl->write_data_length = (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1); // Length of "RES+DATA"
                p_i2c_downloader_ctrl->resp_type = WRITE_MEMORY_NEXT;
                slv_write_start_flag = true;                
            }
            break;
        }

        case SIGNATURE_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->resp_type = SIGNATURE_RESPOND;
            break;
        }

        case MEMORY_AREA_INFO_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->area_num = p_i2c_downloader_ctrl->p_rxbuff[4];
            if(p_i2c_downloader_ctrl->area_num > TARGET_DEVICE_NOA)
            {
                p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
            }
            else
            {
                p_i2c_downloader_ctrl->resp_type = AREA_INFO_RESPOND;
            }
            break;
        }

        case SWTICH_APP_CMD_BYTE:
        {
            if(TWO_BYTE != (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1))
            {
                p_i2c_downloader_ctrl->resp_type = PACKET_ERROR;
            }
            else
            {
                p_i2c_downloader_ctrl->image_version = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
                p_i2c_downloader_ctrl->resp_type = SWITCH_APP;
            }
            break;
        }

        default:
            p_i2c_downloader_ctrl->resp_type = UNSUPPORT_ERROR;
            break;
    }
}

static void transmit_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
    switch((uint8_t) p_i2c_downloader_ctrl->resp_type)
    {
        case BOOT_MODE_ACK_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = BOOT_MODE_ACK;
            p_i2c_downloader_ctrl->txbyteCnt = 0x01;
            break;
        }

        case AUTH_RESPOND:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_auth_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case INQUIRY_RESPOND:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_inquiry_data_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case ERASE_MEMORY:
        {
            if(!g_flash_operation_progressing)
            {
                /* flash erase operation is completed */
                memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_erase_status_ok, DATA_STATUS_RESPONSE_BYTES);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            else
            {
                /* flash erase operation is not completed */
                assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_ERASE_ERROR);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
                /* cancel the flash operation */
                g_flash_operation_progressing = false;
            }
            break;
        }

        case WRITE_MEMORY_START:
        {
            if(g_flash_operation_progressing)
            {
                /* flash programming setup is ready */
                memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_write_status_ok, DATA_STATUS_RESPONSE_BYTES);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            else
            {
                /* flash programming operation does not start properly */
                assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_PACKET_ERROR);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            break;
        }

        case WRITE_MEMORY_NEXT:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_write_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case SIGNATURE_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = DATA_PACKET_START;                 //SOD
            p_i2c_downloader_ctrl->p_txbuff[1] = 0x00;                              //LNH
            p_i2c_downloader_ctrl->p_txbuff[2] = 0x14;                              //LNL
            p_i2c_downloader_ctrl->p_txbuff[3] = p_i2c_downloader_ctrl->recv_cmd;   //RES
            p_i2c_downloader_ctrl->p_txbuff[4] = TARGET_DEVICE_NOA;                 //NOA, number of recordable areas
            p_i2c_downloader_ctrl->p_txbuff[5] = TARGET_DEVICE_TYPE;               //TYP
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[6], &device_part_number[0], 13);
            p_i2c_downloader_ctrl->p_txbuff[19] = BOOT_FIRMWARE_MAJOR_VERSION;      //BFV Major
            p_i2c_downloader_ctrl->p_txbuff[20] = BOOT_FIRMWARE_MINOR_VERSION;      //BFV Minor
            p_i2c_downloader_ctrl->p_txbuff[21] = 1;   //AFV Major
            p_i2c_downloader_ctrl->p_txbuff[22] = 0;   //AFV Minor
            p_i2c_downloader_ctrl->p_txbuff[23] = checksum_calculate(p_i2c_downloader_ctrl->p_txbuff, 25);//SUM
            p_i2c_downloader_ctrl->p_txbuff[24] = END_OF_PACKET;
            p_i2c_downloader_ctrl->txbyteCnt = 25;
            break;
        }

        case AREA_INFO_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = DATA_PACKET_START;                 //SOD
            p_i2c_downloader_ctrl->p_txbuff[1] = 0x00;                              //LNH
            p_i2c_downloader_ctrl->p_txbuff[2] = 0x16;                              //LNL
            p_i2c_downloader_ctrl->p_txbuff[3] = p_i2c_downloader_ctrl->recv_cmd;   //RES
            p_i2c_downloader_ctrl->p_txbuff[4] = p_i2c_downloader_ctrl->area_num;   //KOA
            //SAD (Start Address) => 0x5000 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[5] = 0;
			p_i2c_downloader_ctrl->p_txbuff[6] = 0;
			// p_i2c_downloader_ctrl->p_txbuff[7] = (uint8_t)(FLASH_AREA>>8);
			// p_i2c_downloader_ctrl->p_txbuff[8] = (uint8_t)(FLASH_AREA&0xFF);
			encode16bit(FLASH_AREA, &p_i2c_downloader_ctrl->p_txbuff[7]);
			//EAD (End Address) => 0x3FFFF only needs 3 bytes
            #if 1
			p_i2c_downloader_ctrl->p_txbuff[9] = 0;
			// p_i2c_downloader_ctrl->p_txbuff[10] = (uint8_t)(FLASH_END_ADDRESS>>16);
			// p_i2c_downloader_ctrl->p_txbuff[11] = (uint8_t)(FLASH_END_ADDRESS>>8);
			// p_i2c_downloader_ctrl->p_txbuff[12] = (uint8_t)(FLASH_END_ADDRESS&0xFF);
			encode16bit(FLASH_END_ADDRESS, &p_i2c_downloader_ctrl->p_txbuff[10]);
            #else
			p_i2c_downloader_ctrl->p_txbuff[9] = 0;
			p_i2c_downloader_ctrl->p_txbuff[10] = 0;
			encode16bit(FLASH_END_ADDRESS, &p_i2c_downloader_ctrl->p_txbuff[11]);
            #endif
			//EAU (Erase Access Unit) => 0x400 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[13] = 0;
			p_i2c_downloader_ctrl->p_txbuff[14] = 0;
			encode16bit(FLASH_BYTE_PER_BLOCK, &p_i2c_downloader_ctrl->p_txbuff[15]);
			//MWAU (Max Write Access Unit) => 256 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[17] = 0;
			p_i2c_downloader_ctrl->p_txbuff[18] = 0;
			encode16bit(MAX_CODE_FLASH_WRITE_DATA_SIZE, &p_i2c_downloader_ctrl->p_txbuff[19]);
			//MRAU (Max Read Access Unit) => 256 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[21] = 0;
			p_i2c_downloader_ctrl->p_txbuff[22] = 0;
            encode16bit(MAX_FLASH_READ_DATA_SIZE, &p_i2c_downloader_ctrl->p_txbuff[23]);
            p_i2c_downloader_ctrl->p_txbuff[25] = checksum_calculate(p_i2c_downloader_ctrl->p_txbuff, 27);//SUM
            p_i2c_downloader_ctrl->p_txbuff[26] = END_OF_PACKET;
            p_i2c_downloader_ctrl->txbyteCnt = 27;
            break;
        }

        case SWITCH_APP:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_switch_app_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            g_app_required_update = false;
            break;
        }

        case WRITE_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_WRITE_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case PACKET_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_PACKET_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case CHECKSUM_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_CHECKSUM_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case ADDRESS_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_ADDRESS_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case FLOW_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_FLOW_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case SEQUENCER_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_SEQUENCER_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case UNSUPPORT_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_UNSUPPORTED_CMD);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }
    }
}

void R_Config_IICA0_Create_UserInit(void)
{
    /* Start user code for user init. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

#pragma interrupt r_Config_IICA0_interrupt(vect=INTIICA0)
void __near r_Config_IICA0_interrupt(void)
{
    if (0U == (IICS0 & _80_IICA_STATUS_MASTER))
    {            
        I2C_Downloader_routine_IRQ();   // r_Config_IICA0_slave_handler();
    }
}

void __near I2C_Downloader_routine_IRQ(void)
{
    //if STOP condition is received
    if (1U == SPD0)
    {
        //Disable STOP condition detection/interrupt generation
        SPIE0 = 0U;
        WREL0 = 1U;
        downloader_iica0_slave_status_flag = 0U;

        downloader_iica0_tx_cnt = 0U;
        //Start I2C operation
        IICE0 = 1U;
    }
    else
    {
        //for I2C repeated start (restart)
        if(1U == STD0){
            downloader_iica0_slave_status_flag = 0;
        }

        if (0U == (downloader_iica0_slave_status_flag & _80_IICA_ADDRESS_COMPLETE))
        {
            //Address match
            if (1U == COI0)
            {
                //Enable STOP condition detection/interrupt generation
                SPIE0 = 1U;
                downloader_iica0_slave_status_flag |= _80_IICA_ADDRESS_COMPLETE;

                //I2C slave transmit requested
                if (1U == TRC0)
                {
                    //Change interrupt generation from 8 th to 9 th falling edge
                    WTIM0 = 1U;

                    transmit_handler(&i2c_downloader_ctrl);
                    i2c_downloader_ctrl.resp_type = NONE_RESP;
                    //if there is something to send
                    if (i2c_downloader_ctrl.txbyteCnt != 0U)
                    {
                        IICA0 = i2c_downloader_ctrl.p_txbuff[downloader_iica0_tx_cnt];
                        downloader_iica0_tx_cnt++;
                        i2c_downloader_ctrl.txbyteCnt -= downloader_iica0_tx_cnt;
                    }
                    else
                    {
                        #if 1 //Send dummy data?
                        IICA0 = 0;
                        #endif
                        //release clock stretching
                        WREL0 = 1U;
                    }
                }
                //I2C slave receive requested
                else
                {
                    // receive data portion of Master Write Slave Read is started from here
                    downloader_iica0_rx_cnt = 0;
                    ACKE0 = 1U; //Enable ACK
                    WTIM0 = 0U; //Change interrupt generation from 9 th to 8 th falling edge for slave read data
                    WREL0 = 1U; //release clock stretching
                }
            }
        }
        else
        {
            //I2C slave transmit requested
            if (1U == TRC0)
            {
                //ACK was not detected
                if (0U == ACKD0)
                {
                    //release clock stretching
                    WREL0 = 1U;
                }
                else
                {
                    if (i2c_downloader_ctrl.txbyteCnt != 0U)
                    {
                        IICA0 = i2c_downloader_ctrl.p_txbuff[downloader_iica0_tx_cnt];
                        downloader_iica0_tx_cnt++;
                        i2c_downloader_ctrl.txbyteCnt -= downloader_iica0_tx_cnt;
                    }
                    else
                    {
                        //exit from communication mode
                        LREL0 = 1U;
                    }
                }
            }
            //I2C slave receive requested
            else
            {
                i2c_downloader_ctrl.p_rxbuff[downloader_iica0_rx_cnt] = IICA0;
                if(COMMUNICATION_SETTING_PHASE == i2c_downloader_ctrl.downloader_operation_phase)
                {
                    if(i2c_downloader_ctrl.p_rxbuff[0] == BOOT_MODE_CHECK_CMD)
                    {
                        i2c_downloader_ctrl.resp_type = BOOT_MODE_ACK_RESPOND;
                        i2c_downloader_ctrl.downloader_operation_phase = AUTHENTICATION_PHASE;
                    }

                    WTIM0 = 1U; //Change interrupt generation from 8 th to 9 th falling edge for slave write data
                    WREL0 = 1U; //release clock stretching
                }
                else
                {
                    downloader_iica0_rx_cnt++;
                    if(downloader_iica0_rx_cnt > 3)
                    {
                        i2c_downloader_ctrl.recv_cmd = i2c_downloader_ctrl.p_rxbuff[3];
                        target_recv_data_size_set(&i2c_downloader_ctrl);
                    }
                    else if(downloader_iica0_rx_cnt > MAXIMUM_RECV_PACKET_SIZE)
                    {
                        downloader_iica0_rx_cnt = 0;
                    }

                    if((INVALID_CMD_BYTE != i2c_downloader_ctrl.recv_cmd) &&
                        (downloader_iica0_rx_cnt == i2c_downloader_ctrl.rxbyteCnt))
                    {
                        received_handler(&i2c_downloader_ctrl);
                        WTIM0 = 1U; //Change interrupt generation from 8 th to 9 th falling edge for slave write data
                    }

                    WREL0 = 1U; //release clock stretching
                }
            }
        }
    }
}

static void encode16bit(uint32_t input_data, uint8_t * output_array)
{
    if (input_data > 0xFFFF)
    {
        output_array[2] = (uint8_t)(input_data & 0x00ff);
        output_array[1] = (uint8_t)((input_data>>8) & 0x00ff);
        output_array[0] = (uint8_t)((input_data>>16) & 0x00ff);
    }
    else
    {
        output_array[1] = (uint8_t)(input_data & 0x00ff);
        output_array[0] = (uint8_t)((input_data>>8) & 0x00ff);
    }
}

static uint16_t decode16bit(uint8_t * data)
{
    return (uint16_t)( (data[1]) | (data[0] << 8) );
}

// static uint32_t decode32bit(uint8_t * data)
// {
//     return (uint32_t)( (data[3]) | (data[2] << 8) | (data[1] << 16) | (data[0] << 24) );
// }

static bool checksum_verification(uint8_t const * p_input, uint16_t frame_len)
{
    uint8_t wCRC = 0;
    uint16_t index = 1;
    uint16_t computed_length = frame_len - 2; // remove SOH, ETX bytes

    while (computed_length--)
    {
        wCRC += p_input[index];
        index++;
    }

    if (wCRC == 0)
    {
        return true;
    }

    return  false;
}

static uint8_t checksum_calculate(uint8_t const * p_input, uint16_t frame_len)
{
    uint8_t sum = 0;
    uint16_t index = 1;
    uint16_t computed_length = frame_len - 3; // remove SOD, SUM, ETX bytes

    // SUM = 0x00 - (LNH + LNL + RES + Data)
    while (computed_length--)
    {
        sum += p_input[index];
        index++;
    }

    return (uint8_t)(0x00 - sum);
}

static void assemble_error_data_packet(uint8_t * txdata, uint8_t associated_cmd, uint8_t error_code)
{
    txdata[0] = DATA_PACKET_START;          //SOD
    txdata[1] = 0x00;                       //LNH
    txdata[2] = 0x02;                       //LNL
    txdata[3] = (associated_cmd | 0x80);    //RES
    txdata[4] = error_code;                 //STS
    txdata[5] = checksum_calculate(txdata, DATA_STATUS_RESPONSE_BYTES); //SUM
    txdata[6] = END_OF_PACKET;
}

// static bool Is_target_address_valid(uint8_t *data)
// {
// 	uint16_t target_sad = (uint16_t)( (data[3]) | (data[2] << 8) );
// 	uint16_t target_ead = (uint16_t)( (data[7]) | (data[6] << 8) );

//     if(target_sad > target_ead)
//     {
//         /* The start address is bigger than the end address */
//         return false;
//     }

//     if ((target_sad >= FLASH_AREA) && (target_sad < FLASH_END_ADDRESS)
//             && (target_ead > FLASH_END_ADDRESS))
//     {
//         /* The start address is in user code flash area, but end address is not */
//         return false;
//     }

//     if ((target_sad >= FLASH_AREA) && (target_sad < FLASH_END_ADDRESS)
//             && (target_ead > FLASH_END_ADDRESS))
//     {
//         /* The start address is in user data flash area, but end address is not */
//         return false;
//     }

//     return true;
// }

void I2C_slv_init(void)
{
	i2c_downloader_ctrl.resp_type = NONE_RESP;
	i2c_downloader_ctrl.recv_cmd = INVALID_CMD_BYTE;
	i2c_downloader_ctrl.txbyteCnt = 0;
	i2c_downloader_ctrl.rxbyteCnt = 0;
	i2c_downloader_ctrl.p_txbuff = &ResponseBuf[0];
	i2c_downloader_ctrl.p_rxbuff = &DataBuf[0];
	i2c_downloader_ctrl.image_version = 0;
	i2c_downloader_ctrl.area_num = 0;
	i2c_downloader_ctrl.write_data_length = 0;
	i2c_downloader_ctrl.downloader_operation_phase = COMMUNICATION_SETTING_PHASE;

	downloader_iica0_tx_cnt = 0U;
	downloader_iica0_slave_status_flag = 0U;

}


void I2C_slv_polling(void)
{
	if(FLAG_PROJ_TRIG_3)
	{
        FLAG_PROJ_TRIG_3 = 0;
        printf_tiny("(slv)button pressed\r\n");

	    g_app_required_update = true;
	}


    if(g_app_required_update)
    {                   
        if(ERASE_MEMORY == i2c_downloader_ctrl.resp_type)
        {
            // EraseCodeFlash();
            g_flash_operation_progressing = false;                
            // printf_tiny("(slv)I2C transfer:ERASE_MEMORY bypass\r\n");
        }

        if((WRITE_MEMORY_NEXT == i2c_downloader_ctrl.resp_type) &&
            (slv_write_start_flag == true))
        {   
            // printf_tiny("(slv)I2C transfer:WRITE_MEMORY_NEXT\r\n");
                slv_write_start_flag = false;

            if ((flash_memory_destination_address >= flash_memory_start_address) &&
                (flash_memory_destination_address < flash_memory_end_address))
            {

                #if 1   // debug
                printf_tiny("(slv)I2C transfer write! 0x%04X," ,flash_memory_destination_address>>16);
                printf_tiny("0x%04X," ,flash_memory_destination_address&0xFFFF);
                // printf_tiny("0x%02X\r\n" ,i2c_downloader_ctrl.write_data_length);
                printf_tiny("[0x%02X" ,DataBuf[4+0]);
                printf_tiny(",0x%02X" ,DataBuf[4+1]);
                printf_tiny(",0x%02X" ,DataBuf[4+2]);
                printf_tiny(",0x%02X]," ,DataBuf[4+3]);
                printf_tiny("[0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-4]);
                printf_tiny(",0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-3]);
                printf_tiny(",0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-2]);
                printf_tiny(",0x%02X]\r\n" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-1]);
                #endif
                
                if (1)	//by pass flash programming 
                {
                    /* if programming routine passed, then increase the flash address */
                    flash_memory_destination_address += i2c_downloader_ctrl.write_data_length;
                    flash_target_memory_size -= i2c_downloader_ctrl.write_data_length;

                    if(flash_memory_destination_address == (flash_memory_end_address+1))
                    {
                        /* end flash programming */
                        g_flash_operation_progressing = false;

                        printf_tiny("(slv)I2C upgrade finish : Perform RST...\r\n");
                        const unsigned char ILLEGAL_ACCESS_ON = 0x80;
                        IAWCTL |= ILLEGAL_ACCESS_ON;            // switch IAWEN on (default off)
                        *(__far volatile char *)0x00000 = 0x00; //write illegal address 0x00000(RESET VECTOR)
                    }
                }
                else
                {
                    i2c_downloader_ctrl.resp_type = WRITE_ERROR;
                    printf_tiny("(slv)I2C transfer:WRITE_ERROR\r\n");

                }
            }
            else
            {
                i2c_downloader_ctrl.resp_type = PACKET_ERROR;
                printf_tiny("(slv)I2C transfer:PACKET_ERROR\r\n");
            }
        }

        if(SWITCH_APP == i2c_downloader_ctrl.resp_type)
        {                
            printf_tiny("(slv)I2C transfer:SWITCH_APP\r\n");
        }
    }
    // else
    // {        
    //     printf_tiny("(slv)g_app_required_update = 0,break\r\n");
    //     // break;
    // }
}

void Timer_1ms_IRQ(void)
{
    tick_counter();

    if ((get_tick() % 1000) == 0)
    {
        FLAG_PROJ_TIMER_PERIOD_1000MS = 1;
    }

    if ((get_tick() % 50) == 0)
    {

    }	

    Button_Process_long_counter();
}


/*
    F24 target board
    LED1 connected to P66, LED2 connected to P67
*/
void LED_Toggle(void)
{
    // PIN_WRITE(6,6) = ~PIN_READ(6,6);
    // PIN_WRITE(6,7) = ~PIN_READ(6,7);
    P6_bit.no6 = ~P6_bit.no6;
    P6_bit.no7 = ~P6_bit.no7;
}

void loop(void)
{
	static unsigned long LOG1 = 0;

    if (!g_app_required_update)
    {
        if (FLAG_PROJ_TIMER_PERIOD_1000MS)
        {
            FLAG_PROJ_TIMER_PERIOD_1000MS = 0;

            printf("log(timer):%4d\r\n",LOG1++);
            LED_Toggle();             
        }
    }

    Button_Process_in_polling();

    I2C_slv_polling();
}


// F24 EVB , P137/INTP0 , set both edge 
void Button_Process_long_counter(void)
{
    if (FLAG_PROJ_TRIG_BTN2)
    {
        btn_tick_counter();
    }
    else
    {
        btn_set_tick(0);
    }
}

void Button_Process_in_polling(void)
{
    static unsigned char cnt = 0;

    if (FLAG_PROJ_TRIG_BTN1)
    {
        FLAG_PROJ_TRIG_BTN1 = 0;
        printf("BTN pressed(%d)\r\n",cnt);

        if (cnt == 0)   //set both edge  , BTN pressed
        {
            FLAG_PROJ_TRIG_BTN2 = 1;
        }
        else if (cnt == 1)  //set both edge  , BTN released
        {
            FLAG_PROJ_TRIG_BTN2 = 0;
        }

        cnt = (cnt >= 1) ? (0) : (cnt+1) ;
    }

    if ((FLAG_PROJ_TRIG_BTN2 == 1) && 
        (btn_get_tick() > BTN_PRESSED_LONG))
    {         
        printf("BTN pressed LONG\r\n");
        btn_set_tick(0);
        FLAG_PROJ_TRIG_BTN2 = 0;
    }
}

// F24 EVB , P137/INTP0
void Button_Process_in_IRQ(void)    
{
    FLAG_PROJ_TRIG_BTN1 = 1;
}

void UARTx_Process(unsigned char rxbuf)
{    
    if (rxbuf > 0x7F)
    {
        printf("invalid command\r\n");
    }
    else
    {
        printf("press:%c(0x%02X)\r\n" , rxbuf,rxbuf);   // %c :  C99 libraries.
        switch(rxbuf)
        {
            case '1':
                FLAG_PROJ_TRIG_1 = 1;
                break;
            case '2':
                FLAG_PROJ_TRIG_2 = 1;
                break;
            case '3':
                FLAG_PROJ_TRIG_3 = 1;
                break;
            case '4':
                FLAG_PROJ_TRIG_4 = 1;
                break;
            case '5':
                FLAG_PROJ_TRIG_5 = 1;
                break;

            case 'X':
            case 'x':
                RL78_soft_reset(7);
                break;
            case 'Z':
            case 'z':
                RL78_soft_reset(1);
                break;
        }
    }
}

/*
    Reset Control Flag Register (RESF) 
    BIT7 : TRAP
    BIT6 : 0
    BIT5 : 0
    BIT4 : WDCLRF
    BIT3 : 0
    BIT2 : 0
    BIT1 : IAWRF
    BIT0 : LVIRF
*/
void check_reset_source(void)
{
    /*
        Internal reset request by execution of illegal instruction
        0  Internal reset request is not generated, or the RESF register is cleared. 
        1  Internal reset request is generated. 
    */
    uint8_t src = RESF;
    printf("Reset Source <0x%08X>\r\n", src);

    #if 1   //DEBUG , list reset source
    if (src & BIT0)
    {
        printf("0)voltage detector (LVD)\r\n");       
    }
    if (src & BIT1)
    {
        printf("1)illegal-memory access\r\n");       
    }
    if (src & BIT2)
    {
        printf("2)EMPTY\r\n");       
    }
    if (src & BIT3)
    {
        printf("3)EMPTY\r\n");       
    }
    if (src & BIT4)
    {
        printf("4)watchdog timer (WDT) or clock monitor\r\n");       
    }
    if (src & BIT5)
    {
        printf("5)EMPTY\r\n");       
    }
    if (src & BIT6)
    {
        printf("6)EMPTY\r\n");       
    }
    if (src & BIT7)
    {
        printf("7)execution of illegal instruction\r\n");       
    }
    #endif

}

/*
    7:Internal reset by execution of illegal instruction
    1:Internal reset by illegal-memory access
*/
//perform sofware reset
void _reset_by_illegal_instruction(void)
{
    static const unsigned char illegal_Instruction = 0xFF;
    void (*dummy) (void) = (void (*)(void))&illegal_Instruction;
    dummy();
}
void _reset_by_illegal_memory_access(void)
{
    #if 1
    const unsigned char ILLEGAL_ACCESS_ON = 0x80;
    IAWCTL |= ILLEGAL_ACCESS_ON;            // switch IAWEN on (default off)
    *(__far volatile char *)0x00000 = 0x00; //write illegal address 0x00000(RESET VECTOR)
    #else
    signed char __far* a;                   // Create a far-Pointer
    IAWCTL |= _80_CGC_ILLEGAL_ACCESS_ON;    // switch IAWEN on (default off)
    a = (signed char __far*) 0x0000;        // Point to 0x000000 (FLASH-ROM area)
    *a = 0;
    #endif
}

void RL78_soft_reset(unsigned char flag)
{
    switch(flag)
    {
        case 7: // do not use under debug mode
            _reset_by_illegal_instruction();        
            break;
        case 1:
            _reset_by_illegal_memory_access();
            break;
    }
}

// retarget printf
int __far putchar(int c)
{
    // F24 , UART0
    STMK0 = 1U;    /* disable INTST0 interrupt */
    SDR00L = (unsigned char)c;
    while(STIF0 == 0)
    {

    }
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    return c;
}

void hardware_init(void)
{
    // const unsigned char indicator[] = "hardware_init";
    BSP_EI();
    R_Config_UART0_Start();         // UART , P15 , P16
    R_Config_TAU0_1_Start();        // TIMER
    R_Config_INTC_INTP0_Start();    // BUTTON , P137 
    
    I2C_slv_init();

    // check_reset_source();
    printf("%s finish\r\n\r\n",__func__);
}

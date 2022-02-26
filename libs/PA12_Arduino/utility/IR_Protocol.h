/*
 * irp2.h
 *
 */
#ifndef IRP2_H_
#define IRP2_H_
#ifdef __cplusplus
extern "C" {
#endif
#include "IR_Protocol_format.h"


#define IRP_RX_BUF_SIZE 255  //0x3FF
#define IRP_PARAMETER_BUF_SIZE 128


typedef struct irp_dev {
/*
	gpio_dev *tx_port;      *< Maple pin's GPIO device
	gpio_dev *rx_port;      *< Maple pin's GPIO device
	gpio_dev *dir_port;      *< Maple pin's GPIO device

	uint8 tx_pin;
	uint8 rx_pin;
	uint8 dir_pin;*/

	int write_pointer;
	int read_pointer;
	int data_buffer[IRP_RX_BUF_SIZE];
	//voidFuncPtrUart handlers;
} irp_dev;
/*
extern irp_dev *IRP_DEV1;
extern irp_dev *IRP_DEV2;
extern irp_dev *IRP_DEV3;

void irpInterrupt1(byte data);
void irpInterrupt2(byte data);
void irpInterrupt3(byte data);
*/
unsigned long irp_get_baudrate(int baudnum);
//unsigned short update_crc_ir(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size); 

void PrintBuffer(uint8_t *bpPrintBuffer, uint8_t bLength);

#ifdef __cplusplus
}
#endif
#endif /* IRP2_H_ */

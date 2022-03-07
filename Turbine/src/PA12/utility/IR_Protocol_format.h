
#ifndef IRP_CONSTANTS_H_
#define IRP_CONSTANTS_H_


#define IRP_BROADCAST_ID		(254) /* 0xFE */

/*
 * Instruction command
 * */

/*
#define CMD_PING           0xF1
#define CMD_READ           0xF2
#define CMD_WRITE          0xF3
#define CMD_REG_WRITE      0xF4
#define CMD_ACTION         0xF5
#define CMD_RESET          0xF6
#define CMD_RESTART  	    0xF8
#define CMD_FACTORY_MODE   0xF9
#define CMD_SYNC_WRITE     0x73

*/
enum IRP_INSTRUCTION{ 
	CMD_PING           = 0xF1,
	CMD_READ           = 0xF2,
	CMD_WRITE          = 0xF3,
	CMD_REG_WRITE      = 0xF4,
	CMD_ACTION         = 0xF5,
	CMD_FACTORY_RESET  = 0xF6,
	CMD_REBOOT         = 0xF8,
	CMD_SYSTEM_WRITE   = 0xF9,   // 0x0D
	CMD_STATUS         = 0x55,
	CMD_SYNC_READ      = 0x82,
	CMD_SYNC_WRITE     = 0x73,  // 0x83
	CMD_BULK_READ      = 0x92,
	CMD_BULK_WRITE     = 0x93
};


/*
 * defines error message for protocol 1.0
 * */
#define ERRBIT_VOLTAGE		(1)
#define ERRBIT_ANGLE		(2)
#define ERRBIT_OVERHEAT		(4)
#define ERRBIT_RANGE		(8)
#define ERRBIT_CHECKSUM		(16)
#define ERRBIT_OVERLOAD		(32)
#define ERRBIT_INSTRUCTION	(64)

/*
 * defines error message for protocol 2.0
 * */
#define ERRBIT_RESULT_FAIL	(1)
#define ERRBIT_CMD_ERROR	(2)
#define ERRBIT_CRC			(4)
#define ERRBIT_DATA_RANGE	(8)
#define ERRBIT_DATA_LENGTH	(16)
#define ERRBIT_DATA_LIMIT	(32)
#define ERRBIT_ACCESS		(64)

/*
 * defines message of communication
 * */
#define	COMM_TXSUCCESS		(0)
#define COMM_RXSUCCESS		(1)
#define COMM_TXFAIL			(2)
#define COMM_RXFAIL			(3)
#define COMM_TXERROR		(4)
#define COMM_RXWAITING		(5)
#define COMM_RXTIMEOUT		(6)
#define COMM_RXCORRUPT		(7)

/* timing defines */
#define RX_TIMEOUT_COUNT2		(1600L) //(1000L) //porting
#define NANO_TIME_DELAY			(12000)
//#define RX_TIMEOUT_COUNT1  	(RX_TIMEOUT_COUNT2*90L)// -> 110msec  before Ver 1.11e
#define RX_TIMEOUT_COUNT1  		(RX_TIMEOUT_COUNT2*128L)//  -> 156msec for ID 110 safe access after Ver 1.11f //porting ydh


/*
 *
 * 
 * 
 * */


#define IRP_MODEL_NUMBER_L                    0x00                
#define IRP_MODEL_NUMBER_H                    0x01                
#define IRP_VERSION_OF_FIRMWARE               0x02                
#define IRP_ACTUATOR_ID                       0x03                
#define IRP_ACTUATOR_BAUDRATE                 0x04                
#define IRP_RETURN_DELAY_TIME                 0x05                
#define IRP_CW_ANGLE_LIMIT_L                  0x06                
#define IRP_CW_ANGLE_LIMIT_H                  0x07                
#define IRP_CCW_ANGLE_LIMIT_L                 0x08                
#define IRP_CCW_ANGLE_LIMIT_H                 0x09                
#define IRP_LIMIT_TEMPERATURE                 0x0b                
#define IRP_DOWN_LIMIT_VOLTAGE                0x0c                
#define IRP_UP_LIMIT_VOLTAGE                  0x0d                
#define IRP_MAX_TORQUE_L                      0x0e                
#define IRP_MAX_TORQUE_H                      0x0f                
#define IRP_STATUS_RETURN_LEVEL               0x10                
#define IRP_ALARM_LED                         0x11                
#define IRP_ALARM_SHUTDOWN                    0x12                
#define IRP_RESOLUTION_DIVIDER                0x16 
#define IRP_DOWN_CALIBRATION_L                0x18                
#define IRP_DOWN_CALIBRATION_H                0x19                
#define IRP_UP_CALIBRATION_L                  0x1a                
#define IRP_UP_CALIBRATION_H                  0x1b                
#define IRP_CALIBRATION_DIFFER_L              0x1c                
#define IRP_CALIBRATION_DIFFER_H              0x1d           
#define IRP_ACC_RATE                          0x21               
#define IRP_DEC_RATE                          0x22
#define IRP_MAX_ACC_TIME                      0x23                
#define IRP_MAX_DEC_TIME                      0x24
#define IRP_D_GAIN                            0x25                
#define IRP_I_GAIN                            0x26
#define IRP_P_GAIN                            0x27                
#define IRP_PW_SHORT_STROKE_L                 0x28
#define IRP_PW_SHORT_STROKE_H                 0x29                
#define IRP_PW_LONG_STROKE_L                  0x2A
#define IRP_PW_LONG_STROKE_H                  0x2B
#define IRP_PW_MID_STROKE_L                   0x2C
#define IRP_PW_MID_STROKE_H                   0x2D 
#define IRP_PULSE_INERTIA_RATE                0x2E 
#define IRP_PULSE_INERTIA_RANGE_L             0x2F
#define IRP_PULSE_INERTIA_RANGE_H             0x30
#define IRP_PULSE_MODE_DISABLE                0x31
#define IRP_CENTER_DIFFER_L                   0x32                
#define IRP_CENTER_DIFFER_H                   0x33           
#define IRP_PUNCH_INIT_L                      0x34               
#define IRP_PUNCH_INIT_H                      0x35 

#define IRP_TORQUE_ENABLE                     0x80                
#define IRP_LED_COLOR                         0x81                
#define IRP_CW_COMPLIANCE_MARGIN              0x82                
#define IRP_CCW_COMPLIANCE_MARGIN             0x83                
#define IRP_CW_COMPLIANCE_SLOPE               0x84                
#define IRP_CCW_COMPLIANCE_SLOPE              0x85                
#define IRP_GOAL_POSITION_L                   0x86                
#define IRP_GOAL_POSITION_H                   0x87                
#define IRP_MOVING_SPEED_L                    0x88                
#define IRP_MOVING_SPEED_H                    0x89                
#define IRP_TORQUE_LIMIT_L                    0x8A                
#define IRP_TORQUE_LIMIT_H                    0x8B                
#define IRP_PRESENT_POSITION_L                0x8C                
#define IRP_PRESENT_POSITION_H                0x8D                
#define IRP_PRESENT_SPEED_L                   0x8E                
#define IRP_PRESENT_SPEED_H                   0x8F                
#define IRP_PRESENT_LOAD_L                    0x90                
#define IRP_PRESENT_LOAD_H                    0x91                
#define IRP_PRESENT_VOLTAGE                   0x92                
#define IRP_PRESENT_TEMPERATURE               0x93                
#define IRP_REGISTERED_INSTRUCTION            0x94                
#define IRP_RESERVED_45                       0x95                
#define IRP_MOVING                            0x96                
#define IRP_LOCK                              0x97                
#define IRP_PUNCH_L                           0x98                
#define IRP_PUNCH_H                           0x99                
#define IRP_CURRENT_L                         0xAC
#define IRP_CURRENT_H                         0xAD
#define IRP_TORGUE_CONTROL_MODE_ENABLE        0xAE
#define IRP_GOAL_TORQUE_L                     0xAF
#define IRP_GOAL_TORQUE_H                     0xB0
#define IRP_GOAL_ACCELERATION                 0xB1

/*Half Duflex UART ENABLE PIN ----------------------------------------------------------------------------------------------*/

//#define PORT_TXRX_DIRECTION	GPIOC
//#define PIN_TXRX_DIRECTION		4


///////////////// utility for value ///////////////////////////
#define IRP_MAKEint(a, b)      ((unsigned short)(((unsigned char)(((unsigned long)(a)) & 0xff)) | ((unsigned short)((unsigned char)(((unsigned long)(b)) & 0xff))) << 8))
#define IRP_MAKEDint(a, b)     ((unsigned long)(((unsigned short)(((unsigned long)(a)) & 0xffff)) | ((unsigned long)((unsigned short)(((unsigned long)(b)) & 0xffff))) << 16))
#define IRP_LOint(l)           ((unsigned short)(((unsigned long)(l)) & 0xffff))
#define IRP_HIint(l)           ((unsigned short)((((unsigned long)(l)) >> 16) & 0xffff))
#define IRP_LOBYTE(w)           ((unsigned char)(((unsigned short)(w)) & 0xff))
#define IRP_HIBYTE(w)           ((unsigned char)((((unsigned short)(w)) >> 8) & 0xff))

#define IRP_PACKET_TYPE1	1 //IRP protocol 1.0 packet type
#define IRP_PACKET_TYPE2	2 //IRP protocol 2.0 packet type



#endif /* IRP_CONSTANTS_H_ */

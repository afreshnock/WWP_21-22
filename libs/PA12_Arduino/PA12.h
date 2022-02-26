/*
 * PA12.h
 *
 *      Author: Progressive Automations
 */

#ifndef PA12_H_
#define PA12_H_

#include "utility/IR_Protocol.h"
#include "HardwareSerial.h"
#include <SoftwareSerial.h>

/*
typedef struct data {
    int             iID;
    int				iAddr;
    int             iLength;
    int             iError;
    int  			iData[8];
} BulkData, *PBulkData;

*/
enum stroke_type{
		Short =0,
		Lowest =0,
		Highest=1,
		Long=1,
		Center=2,
		Middle =2
	};
enum level_type{
	Low =0,
	High,	
};
enum gain{
	dGain,
	iGain,
	pGain
};	
enum RGB{
	off=0,
	RED=2,
	GREEN=4,
    BLUE =8
};

class PA12 {
public:
	PA12(HardwareSerial  *dev_serial, int DirectionPin);
	PA12(SoftwareSerial  *dev_serial, int DirectionPin);
	PA12(HardwareSerial  *dev_serial, int DirectionPin,bool TxLevel);
	PA12(SoftwareSerial  *dev_serial, int DirectionPin,bool TxLevel);
	virtual ~PA12();

	/////////// Device control methods /////////////
	void begin(int buad);
	int  ping(int  bID);
	int readRaw(void);
	int available(void);
	void writeRaw(int value);
	int getError(int errbit);

	int txRxPacket(int bID, int bInst, int bTxParaLen);
	int txPacket(int bID, int bInstruction, int bParameterLength);
	int rxPacket(int bRxLength);
	int  getResult(void);
	int getTxRxStatus(void);
	void setPacketType(int ver);
	int getPacketType(void);
	//// High communication methods ////////
	int readByte(int bID, int bAddress);
	int writeByte(int bID, int bAddress, int bData);
	int readint(int bID, int bAddress);
	int writeint(int bID, int bAddress, short wData);
	//int writeDint( int bID, int wAddress,  unsigned long value );
	//unsigned long readDint( int bID, int wAddress );

	int setPosition(int bID, int Position, int Speed);
	int syncWrite(int start_addr, int data_length, int *param, int param_length); // int(16bit) syncwrite() for IRP

	/////// Methods for making a packet ////////
	void setTxPacketId( int id );
	void setTxPacketInstruction( int instruction );
	void setTxPacketParameter( int index, int value );
	void setTxPacketLength( int length );
	int txrxPacket(void);
	int getRxPacketParameter( int index );
	int getRxPacketLength(void);

	//Easy Functions for IRP
	
	int getModelNumber(int bID);
	int Version(int bID);
	void ServoID(int bID, int new_ID);
	int ServoID(int bID);
	void BaudRate(int bID, int baud_num);
	int BaudRate(int bID);
	void returnDelayTime(int bID, int time);
	int returnDelayTime(int bID);
	
	// Stroke Limit	
	void StrokeLimit(int bID,int dir, int position);
	int StrokeLimit(int bID,int dir);
	void ShortStrokeLimit(int bID, int position);
	int ShortStrokeLimit(int bID);	
	void LongStrokeLimit(int bID, int position);
	int LongStrokeLimit(int bID);
	// maxTemperature	
	void maxTemperature(int bID, int temp);
	int maxTemperature(int bID);
	// Limit Voltage	
	void limitVolt(int bID,int level, int value);
	int limitVolt(int bID,int level);
	void minVolt(int bID, int value);
	int minVolt(int bID);	
	void maxVolt(int bID, int value);
	int maxVolt(int bID);
	
	void maxForce(int bID, int value);
	int maxForce(int bID);		
	void returnMode(int bID, int mode);
	int returnMode(int bID);
	void alarmLed(int bID, int value);
	int alarmLed(int bID);	
	void alarmShutdown(int bID,int option);
	int alarmShutdown(int bID);
	void resolutionFactor(int bID,int value);
	int resolutionFactor(int bID);	
	
	int CalStroke(int bID, int dir);
	void CalStroke(int bID,int dir, int value);		
	
	void ThirdPartyProgramInterface(int bID,int value);
	int ThirdPartyProgramInterface(int bID);
	void ThirdPartyProgramVersion(int bID,int value);
	int ThirdPartyProgramVersion(int bID);
	
	void pidGain(int bID,int num, int value);
	int pidGain(int bID,int num);
	
	void StrokePulseWidth(int bID,int num, int value);
	int StrokePulseWidth(int bID,int num);
	
	void CenterDiffernce(int bID,int value);
	int CenterDiffernce(int bID);
	void PunchInitial(int bID,int value);
	int PunchInitial(int bID);		
	void forceEnable(int bID,int value);
	int forceEnable(int bID);
	void ledOn(int bID,int value);
	int ledOn(int bID);	
	void complianceMargin(int bID,int dir, int value);
	int complianceMargin(int bID,int dir);
	void goalPosition(int bID,int value);
	int goalPosition(int bID);
	void movingSpeed(int bID,int value);
	int movingSpeed(int bID);
	void forceLimit(int bID,int value);
	int forceLimit(int bID);	
	int presentPosition(int bID);
	int presentSpeed(int bID);
	int presentLoad(int bID);
	int presentVolt(int bID);
	int presentTemperature(int bID);
	int receiveData(int bID);
	int Moving(int bID);
	void Lock(int bID,int value);
	int Lock(int bID);
	void Punch(int bID,int value);
	int Punch(int bID);
	
	void initPacket(int bID, int bInst);
	void pushByte(int value);
	int flushPacket(void);
	void pushParam(byte value);
	void pushParam(int value);
	
	int fast_txRxPacket(int bID, int bInst, int bTxParaLen);
	int fast_rxPacket(int bRxLength);
    int quick_presentPosition(int bID);
	/*
	 * Utility methods for PA12
	 */

	/*int getLowByte( int wData ); //can be replaced by IRP_LOBYTE(w)
	int getHighByte( int wData );//can be replaced by IRP_HIBYTE(w)
	int makeint( int lowint, int highint ); //can be replaced by IRP_MAKEint(w)*/

private:
void printBuffer(int *bpPrintBuffer, int bLength);
int Dummy(int tmp);
	void uDelay(int uTime);
	void nDelay(int nTime);
	void irpTxEnable(void);
	void irpTxDisable(void);
	void clearBuffer(void);
	int checkPacketType(void);
	int PA12_Serial_Type;
	Stream  *PA12_Serial;
	//HardwareSerial  *PA12_Serial;
	//SoftwareSerial  *PA12_Serial;
	int PA12_DirPin;
	bool PA12_DirPin_Level_Tx;
	bool PA12_DirPin_Level_Rx;
	int mRxBuffer[IRP_RX_BUF_SIZE];
	int mTxBuffer[IRP_RX_BUF_SIZE];
	int mParamBuffer[IRP_PARAMETER_BUF_SIZE];
	int mBusUsed;
	int mRxLength;

	// additions to return proper COMM_* status
	int mIRPtxrxStatus;
	// additions to permit non-default Status Return Level settings without returning errors
	int gbIRPStatusReturnLevel;
	// additions to adjust number of txrx attempts
	int gbIRPNumberTxRxAttempts;

	int mPacketType;  

	int mPktIdIndex;
	int mPktLengthIndex;
	int mPktInstIndex;
	int mPktErrorIndex;
	//int mRxLengthOffset;

	int mbLengthForPacketMaking;
	int mbIDForPacketMaking;
	int mbInstructionForPacketMaking;
	int mCommStatus;

	int SmartDelayFlag;
};


#endif /* PA12_H_ */

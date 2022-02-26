/*
 * PA12.cpp
 *
 */

#include "Arduino.h"
#include "PA12.h"

#include <SoftwareSerial.h>

PA12::PA12(HardwareSerial  *dev_serial, int DirectionPin) {
PA12_Serial_Type=0;
	PA12_Serial=dev_serial;
	PA12_DirPin=DirectionPin;
	PA12_DirPin_Level_Tx = HIGH;//tx
	PA12_DirPin_Level_Rx = LOW;
}



PA12::PA12(SoftwareSerial  *dev_serial, int DirectionPin) {
  //SoftwareSerial mySerial(receivePin, transmitPin); // RX, TX
  PA12_Serial_Type=1;
  PA12_Serial=dev_serial;
  PA12_DirPin=DirectionPin;
  PA12_DirPin_Level_Tx = HIGH;//tx
  PA12_DirPin_Level_Rx = LOW;
}

PA12::PA12(HardwareSerial  *dev_serial, int DirectionPin,bool TxLevel) {
	PA12_Serial_Type=0;
	PA12_Serial=dev_serial;
	PA12_DirPin=DirectionPin;
	PA12_DirPin_Level_Tx = TxLevel;//tx
	PA12_DirPin_Level_Rx = !TxLevel;
}



PA12::PA12(SoftwareSerial  *dev_serial, int DirectionPin,bool TxLevel) {
	//SoftwareSerial mySerial(receivePin, transmitPin); // RX, TX
	PA12_Serial_Type=1;
	PA12_Serial=dev_serial;
	PA12_DirPin=DirectionPin;
	PA12_DirPin_Level_Tx = TxLevel;//tx
	PA12_DirPin_Level_Rx = !TxLevel;
}

PA12::~PA12() {
	// TODO Auto-generated destructor stub
}
void PA12::begin(int baud){

	pinMode(PA12_DirPin, OUTPUT);
	if(PA12_Serial_Type)
	((SoftwareSerial *)PA12_Serial)->begin(irp_get_baudrate(baud));
	else
	((HardwareSerial *)PA12_Serial)->begin(irp_get_baudrate(baud));
	/*
	 while (!(*PA12_Serial)) {
		 ; // wait for serial port to connect. Needed for native USB port only
	 }
	 */
	digitalWrite( PA12_DirPin, PA12_DirPin_Level_Tx);// TX Enable

	mIRPtxrxStatus = 0;
	mBusUsed = 0;// only 1 when tx/rx is operated
	SmartDelayFlag=1;



	this->setPacketType(IRP_PACKET_TYPE1);

//	this->clearBuffer();
	
}


int PA12::readRaw(void){
int temp=0;
if (PA12_Serial->available()) {
	temp = PA12_Serial->read();
	}
	return temp;
}
void PA12::writeRaw(int value){

PA12_Serial->write(value);
}

/*
 * @brief : if data coming from irp bus, returns 1, or if not, returns 0.
 *
 */
int PA12::available(void){

		return PA12_Serial->available();
}
void PA12::irpTxEnable(void){

	digitalWrite( PA12_DirPin, PA12_DirPin_Level_Tx );// RX Disable

}
void PA12::irpTxDisable(void){

	digitalWrite( PA12_DirPin, PA12_DirPin_Level_Rx );// RX Enable

}

void PA12::clearBuffer(void){
	while((this->available()))
	{
		PA12_Serial->read();
	}
}

void PA12::setPacketType(int type){
	mPacketType = type;
	if(mPacketType == IRP_PACKET_TYPE2){
		mPktIdIndex = 4;
		mPktLengthIndex = 5;
		mPktInstIndex = 7;
		mPktErrorIndex = 8;
		//mRxLengthOffset = 7;
	}else{           // irp 1.0
		mPktIdIndex = 3; //2;
		mPktLengthIndex = 4; //3;
		mPktInstIndex = 5; //4;
		mPktErrorIndex = 5; //4;
		//mRxLengthOffset = 4;
	}
}
int PA12::getPacketType(void){
	return mPacketType;
}
int PA12::checkPacketType(void){

	return 0;//default is 1.0 protocol IRP_PACKET_TYPE1
}

int PA12::getTxRxStatus(void)
{
	return mIRPtxrxStatus;
}


 
int  PA12::getResult(void){
	//	return mCommStatus;
	return this->getTxRxStatus();
}

int PA12::getError( int errbit ){

	return 0;
}

int PA12::txPacket(int bID, int bInstruction, int bParameterLength){

    int bCount,bCheckSum,bPacketLength;
	

    int offsetParamIndex;
   
	mTxBuffer[0] = 0xff;
	mTxBuffer[1] = 0xff;
	mTxBuffer[2] = 0xff; //
	mTxBuffer[3] = bID;  //[2]
	mTxBuffer[4] = bParameterLength+2; //[3] //2(int) <- instruction(1int) + checksum(1int) 
	mTxBuffer[5] = bInstruction; //[4]

	offsetParamIndex = 6; //5
	bPacketLength = bParameterLength+3+4; //+2+4;

  
    //copy parameters from mParamBuffer to mTxBuffer
    for(bCount = 0; bCount < bParameterLength; bCount++)
    {
    	mTxBuffer[bCount+offsetParamIndex] = mParamBuffer[bCount];
    }

	// chech sum
    bCheckSum = 0;
    for(bCount = 3; bCount < bPacketLength-1; bCount++){ //except 0xff,checksum //bCount = 2;
		bCheckSum += mTxBuffer[bCount];
	}
    mTxBuffer[bCount] = ~bCheckSum; //Writing Checksum with Bit Inversion
  
    //TxDStringC("bPacketLength = ");TxDHex8C(bPacketLength);TxDStringC("\r\n");
    this->irpTxEnable(); // this define is declared in irp.h
	//delay(2);
	//uDelay(1);
    for(bCount = 0; bCount < bPacketLength; bCount++)
    {
        writeRaw(mTxBuffer[bCount]);
    }
	flushPacket();
	//delay(1);
    this->irpTxDisable();// this define is declared in irp.h

    return(bPacketLength); // return packet length
}
int PA12::rxPacket(int bRxLength){

	unsigned long ulCounter, ulTimeLimit;
	int bCount, bLength, bChecksum;
	//int bCount, bLength, bChecksum; 

	int bTimeout;

	//#if false
	bTimeout = 0;
	if(bRxLength == 255 || bRxLength == 0xffff) 
		ulTimeLimit = RX_TIMEOUT_COUNT1;
	else
		ulTimeLimit = RX_TIMEOUT_COUNT2;
	for(bCount = 0; bCount < bRxLength; bCount++)
	{
		ulCounter = 0;
		while(!( PA12_Serial->available()))
		{
			nDelay(NANO_TIME_DELAY); 
			if(ulCounter++ > ulTimeLimit)
			{
				bTimeout = 1;
				//TxDStringC("Timeout\r\n");
				break;
			}
			uDelay(0); //if exist IRP 1.0 -> ok IRP 2.0 -> ok, if not exist 1.0 not ok, 2.0 ok
		}
		if(bTimeout) break;
		mRxBuffer[bCount] = this->readRaw(); // PA12_Serial->read(); // get packet data from USART device
	//	Serial.print(mRxBuffer[bCount]);Serial.print(" ");
		//TxDStringC("mRxBuffer = ");TxDHex8C(mRxBuffer[bCount]);TxDStringC("\r\n");
	}
	//Serial.println(" ");
	bLength = bCount;
	bChecksum = 0;
	if( mTxBuffer[mPktIdIndex] != IRP_BROADCAST_ID )
	{
	
		if(bTimeout && bRxLength != 255)
		{
		
		//	TxDStringC("Rx Timeout");
        //	TxDByteC(bLength);		
			mIRPtxrxStatus |= (1<<COMM_RXTIMEOUT);
			clearBuffer();			
			return 0;
		}
		
		if(bLength > 3) //checking available length.
		{
			
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER); //|| mRxBuffer[2] != 0xff
			else if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
			else if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
			else{
				for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
					bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
				}
				if(bChecksum != 0xff) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				//return 0;
			}
			
			
			
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ){ //|| mRxBuffer[2] != 0xff			
				//TxDStringC("Wrong Header");//[Wrong Header]
			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
				clearBuffer();
				return 0;
			}
			

			if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] )  //id check
			{
			
			//if(update_crc_ir(0, mRxBuffer, bRxLength-2) == bChecksum)
				//TxDStringC("[Error:TxID != RxID]");
			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
				clearBuffer();
				return 0;
			}

			if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) // status packet length check
			{			
				//TxDStringC("RxLength Error");			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
				clearBuffer();
				return 0;
			}
	
			for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
				bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
			}

			bChecksum &= 0xff;

			if(bChecksum != 0xff)
			{			
				//TxDStringC("[RxChksum Error]");			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				clearBuffer();
				return 0;
			}
		//end of checksum
		}//(bLength > 3)
	}//end of Rx status packet check
//	#endif
	return bLength;
}
void PA12::printBuffer(int *bpPrintBuffer, int bLength)
{
#ifdef	PRINT_OUT_TRACE_ERROR_PRINT_TO_USART2
	int bCount;
	if(bLength == 0)
	{
		if(mTxBuffer[3] == IRP_BROADCAST_ID)
		{
			TxDStringC("\r\n No Data[at Broadcast ID 0xFE]");
		}
		else
		{
			TxDStringC("\r\n No Data(Check ID, Operating Mode, Baud rate)");//TxDString("\r\n No Data(Check ID, Operating Mode, Baud rate)");
		}
	}
	for(bCount = 0; bCount < bLength; bCount++)
	{
		TxDHex8C(bpPrintBuffer[bCount]);
		TxDByteC(' ');
	}
	TxDStringC(" LEN:");//("(LEN:")
	TxDHex8C(bLength);
	TxDStringC("\r\n");
#endif
}

int PA12::txRxPacket(int bID, int bInst, int bTxParaLen){
//#if false
	mIRPtxrxStatus = 0;

	int bTxLen, bRxLenEx, bTryCount;

	mBusUsed = 1;
	mRxLength = bRxLenEx = bTxLen = 0;

//	for(bTryCount = 0; bTryCount < gbIRPNumberTxRxAttempts; bTryCount++)
	for(bTryCount = 0; bTryCount < 1; bTryCount++)
	{
		while((this->available())){
			PA12_Serial->read();
		}

		/**************************************   Transfer packet  ***************************************************/
		bTxLen = this->txPacket(bID, bInst, bTxParaLen);
		
		if (bTxLen == (bTxParaLen+4+3))	mIRPtxrxStatus = (1<<COMM_TXSUCCESS); //+4+2
				
		if(bInst == CMD_PING){		
			if(bID == IRP_BROADCAST_ID)	mRxLength = bRxLenEx = 0xff;
			else mRxLength = bRxLenEx = 7; //6; // basic response packet length			
		}
		else if(bInst == CMD_READ){
			mRxLength = bRxLenEx = 7+mParamBuffer[1]; //6+
		}
		else if( bID == IRP_BROADCAST_ID ){
			if(bInst == CMD_SYNC_READ || bInst == CMD_BULK_READ) mRxLength = bRxLenEx = 0xffff; //only 2.0 case
			else mRxLength = bRxLenEx = 0; // no response packet
		}
		else{
			if (gbIRPStatusReturnLevel>1){
				if(mPacketType == IRP_PACKET_TYPE1) mRxLength = bRxLenEx = 7; //6 //+mParamBuffer[1];
				else mRxLength = bRxLenEx = 11;
			}
			else{
				mRxLength = bRxLenEx = 0;
			}
		}


		if(bRxLenEx){
			if(SmartDelayFlag == 1)
				delay(150);
			/**************************************   Receive packet  ***************************************************/
			mRxLength = this->rxPacket(bRxLenEx);

		}//bRxLenEx is exist
	} //for() gbIRPNumberTxRxAttempts

	mBusUsed = 0;
	
	if((mRxLength != bRxLenEx) && (mTxBuffer[mPktIdIndex] != IRP_BROADCAST_ID))
	{


	//	return 0;
	}else if((mRxLength == 0) && (mTxBuffer[mPktInstIndex] == CMD_PING)){ 
		return 0;
	}
	

	mIRPtxrxStatus = (1<<COMM_RXSUCCESS);


	return 1;
}

int PA12::fast_rxPacket(int bRxLength){

	unsigned long ulCounter, ulTimeLimit;
	int bCount, bLength, bChecksum;
	//int bCount, bLength, bChecksum; 

	int bTimeout;

	//#if false
	bTimeout = 0;
	if(bRxLength == 255 || bRxLength == 0xffff) 
		ulTimeLimit = RX_TIMEOUT_COUNT1;
	else
		ulTimeLimit = RX_TIMEOUT_COUNT2;
	for(bCount = 0; bCount < bRxLength; bCount++)
	{
		ulCounter = 0;
		while(!( PA12_Serial->available()))
		{
			nDelay(NANO_TIME_DELAY); 
			if(ulCounter++ > ulTimeLimit)
			{
				bTimeout = 1;
				
				break;
			}
			uDelay(0); 
		}
		if(bTimeout) break;
		mRxBuffer[bCount] = this->readRaw(); 
	}
	
	
	bLength = bCount;
	bChecksum = 0;
	if( mTxBuffer[mPktIdIndex] != IRP_BROADCAST_ID )
	{
		if(bTimeout && bRxLength != 255)
		{
			mIRPtxrxStatus |= (1<<COMM_RXTIMEOUT);
			clearBuffer();			
			return 0;
		}
		
		if(bLength > 3) 
		{
			
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER); //|| mRxBuffer[2] != 0xff
			else if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] ) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
			else if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
			else{
				for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
					bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
				}
				if(bChecksum != 0xff) mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				//return 0;
			}					
			
			if(mRxBuffer[0] != 0xff || mRxBuffer[1] != 0xff || mRxBuffer[2] != 0xff ){ //|| mRxBuffer[2] != 0xff			
				//TxDStringC("Wrong Header");//[Wrong Header]
			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXHEADER);
				clearBuffer();
				return 0;
			}
			
			if(mRxBuffer[mPktIdIndex] != mTxBuffer[mPktIdIndex] )  //id check
			{						
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXID);
				clearBuffer();
				return 0;
			}

			if(mRxBuffer[mPktLengthIndex] != bLength-mPktInstIndex) // status packet length check
			{			
				//TxDStringC("RxLength Error");			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXLENGTH);
				clearBuffer();
				return 0;
			}
	
			for(bCount = 3; bCount < bLength; bCount++){ //bCount = 2
				bChecksum += mRxBuffer[bCount]; //Calculate checksum of received data for compare
			}

			bChecksum &= 0xff;

			if(bChecksum != 0xff)
			{			
				//TxDStringC("[RxChksum Error]");			
				mIRPtxrxStatus |= (1<<COMM_RXCORRUPT);//RXCHECKSUM);
				clearBuffer();
				return 0;
			}
		//end of checksum
		}//(bLength > 3)
	}//end of
	return bLength;
}
int PA12::fast_txRxPacket(int bID, int bInst, int bTxParaLen){
	mIRPtxrxStatus = 0;

	int bTxLen, bRxLenEx, bTryCount;

	mBusUsed = 1;
	mRxLength = bRxLenEx = bTxLen = 0;


	while((this->available())){
		PA12_Serial->read();
	}

	/**************************************   Transfer packet  ***************************************************/
	bTxLen = this->txPacket(bID, bInst, bTxParaLen);
	
	if (bTxLen == (bTxParaLen+4+3))	mIRPtxrxStatus = (1<<COMM_TXSUCCESS); //+4+2
			
	
	mRxLength = bRxLenEx = 7+mParamBuffer[1]; //6+
	
	if(bRxLenEx){

		/**************************************   Receive packet  ***************************************************/
		mRxLength = this->fast_rxPacket(bRxLenEx);

	}//bRxLenEx is exist


	//TxDStringC("\r\n TEST POINT 2");//TxDString("\r\n Err ID:0x");
	mBusUsed = 0;	

	mIRPtxrxStatus = (1<<COMM_RXSUCCESS);
	
	return 1;
}


int PA12::Dummy(int tmp){
	return tmp;
}
void PA12::uDelay(int uTime){
	int cnt, max;
		static int tmp = 0;

		for( max=0; max < uTime; max++)
		{
			for( cnt=0; cnt < 10 ; cnt++ )
			{
				tmp +=Dummy(cnt);
			}
		}
		//tmpdly = tmp;
}
void PA12::nDelay(int nTime){
	int cnt, max;
		cnt=0;
		static int tmp = 0;

		for( max=0; max < nTime; max++)
		{
			//for( cnt=0; cnt < 10 ; cnt++ )
			//{
				tmp +=Dummy(cnt);
			//}
		}
		//tmpdly = tmp;
}


int  PA12::ping(int  bID ){

	if(this->txRxPacket(bID, CMD_PING, 0)){
		if(mPacketType == IRP_PACKET_TYPE1) return (mRxBuffer[3]); //1.0
		else return IRP_MAKEint(mRxBuffer[9],mRxBuffer[10]); 
	}else{
		return 0xff;  //no irp in bus.
	}

}

int  PA12::writeByte(int bID, int bAddress, int bData){
	int param_length = 0;

	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = bData;
	param_length = 2;
	
	return this->txRxPacket(bID, CMD_WRITE, param_length);
}

int PA12::readByte(int bID, int bAddress){
	this->clearBuffer();
	
	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = 1;
	if( this->txRxPacket(bID, CMD_READ, 2 )){
		mCommStatus = 1;
		return(mRxBuffer[6]);// [5] //refer to 1.0 packet structure
	}
	else{
		mCommStatus = 0;
		return 0xff;
	}
}



int PA12::writeint(int bID, int bAddress, short wData){
    int param_length = 0;
    this->clearBuffer();

	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = IRP_LOBYTE(wData);//(int)(wData&0xff);
	mParamBuffer[2] = IRP_HIBYTE(wData);//(int)((wData>>8)&0xff);
	param_length = 3;
	
	return this->txRxPacket(bID, CMD_WRITE, param_length);

}



int PA12::readint(int bID, int bAddress){
	this->clearBuffer();
	
	mParamBuffer[0] = bAddress;
	mParamBuffer[1] = 2;
	if(this->txRxPacket(bID, CMD_READ, 2)){
		return IRP_MAKEint(mRxBuffer[6],mRxBuffer[7]);//( (((int)mRxBuffer[6])<<8)+ mRxBuffer[5] );
	}
	else{
		return 0xffff;
	}	
}


int PA12::setPosition(int bID, int Position, int Speed){

    int param_length = 0;
	
	mParamBuffer[0] = (unsigned char)0x86; //30;
	mParamBuffer[1] = (unsigned char)IRP_LOBYTE(Position);
	mParamBuffer[2] = (unsigned char)IRP_HIBYTE(Position);
	mParamBuffer[3] = (unsigned char)IRP_LOBYTE(Speed);
	mParamBuffer[4] = (unsigned char)IRP_HIBYTE(Speed);
	param_length = 5;

	
	return (this->txRxPacket(bID, CMD_WRITE, param_length));

}
#if 0
int PA12::bulkRead(int *param, int param_length){
	//mResult = 0;
	unsigned long bulkReadlength=0;

	int n, i, k=0;
	int num = param_length / 5; 

	for(n=0; n < param_length; n++){
		mParamBuffer[n] = param[n];
	}


	this->txRxPacket(IRP_BROADCAST_ID, CMD_BULK_READ, param_length);


	for(n = 0; n < num; n++){
	   // int id = param[n*5+0];

		bulkReadlength = this->rxPacket(param_length+11);
	
		if(mRxBuffer[7] == 0x55){ //packet instruction index
			mBulkData[n].iID = mRxBuffer[4]; //packet ID index
			mBulkData[n].iAddr = IRP_MAKEint(mParamBuffer[5*n+1],mParamBuffer[5*n+2]); //get address
			mBulkData[n].iLength = IRP_MAKEint(mParamBuffer[5*n+3],mParamBuffer[5*n+4]);//IRP_MAKEint(gbpRxBufferEx[PKT_LENGTH_L],gbpRxBufferEx[PKT_LENGTH_H]);
			mBulkData[n].iError = mRxBuffer[7+1]; //Error code
			for(i=0; i < mBulkData[n].iLength ; i++){
				mBulkData[n].iData[i] = mRxBuffer[7+2+i]; //DATA1
			}
		}
		for(k=0;k < IRP_RX_BUF_SIZE ; k++){
			mRxBuffer[k] = 0; //buffer clear
		}
		this->clearBuffer();
	}
	return bulkReadlength;

}
#endif


void PA12::setTxPacketId(int id){
	mbIDForPacketMaking = id;

}
void PA12::setTxPacketInstruction(int instruction){
	mbInstructionForPacketMaking = instruction;

}
void PA12::setTxPacketParameter( int index, int value ){
	mParamBuffer[index] = value;

}
void PA12::setTxPacketLength( int length ){
	mbLengthForPacketMaking = length;

}
int PA12::txrxPacket(void){
	mCommStatus = this->txRxPacket(mbIDForPacketMaking, mbInstructionForPacketMaking, mbLengthForPacketMaking);
	return mCommStatus;
}

int PA12::getRxPacketParameter( int index ){
	//return irp_get_rxpacket_parameter( index );
	return mRxBuffer[6 + index]; //5
}
int PA12::getRxPacketLength(void){
	//return irp_get_rxpacket_length();
	return mRxBuffer[4]; //length index is 3 in status packet //3
}
/****************************************************************/
/* Model Number													*/
/* Type : Read Only												*/
/****************************************************************/

// Data memory Map
int PA12::getModelNumber(int bID){
	return this->readint(bID, 0);
}
/****************************************************************/
/* Model Virsion												*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::Version(int bID){
	return this->readByte(bID, 2);
}
/****************************************************************/
/* ServoID														*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::ServoID(int bID, int new_ID){
	this->writeByte(bID, 3, new_ID);
}
int PA12::ServoID(int bID){
	return this->readByte(bID, 3);
}
/****************************************************************/
/* BaudRate														*/
/*   2 : 400000													*/
/*   4 : 250000													*/
/*   8 : 200000													*/
/*  16 : 115200													*/
/*  32 :  57600													*/
/*  64 :  19200													*/
/* 128 :   9600													*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::BaudRate(int bID, int baud_num){
	this->writeByte(bID, 4, baud_num);
}
int PA12::BaudRate(int bID){
	return this->readByte(bID, 4);
}
/****************************************************************/
/*  Return DelayTime											*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::returnDelayTime(int bID, int time){
	this->writeByte(bID, 5, time);
}
int PA12::returnDelayTime(int bID){
	return this->readByte(bID, 5);
}
/****************************************************************/
/* Strok Limit                                                  */
/* Dir : Stroke.Short, Stroke.Long								*/
/* Short Strok Adrress 0x06~0x07								*/
/* Long Strok Adrress 0x06~0x07									*/
/****************************************************************/
void PA12::StrokeLimit(int bID,int dir, int position){
	this->writeint(bID, 6+dir*2, position);
}
int PA12::StrokeLimit(int bID,int dir){
	return this->readint(bID, 6+dir*2);
}
// Short Stroke Limit
void PA12::ShortStrokeLimit(int bID, int position){
	this->writeint(bID, 6, position);
}
int PA12::ShortStrokeLimit(int bID){
	return this->readint(bID, 6);
}
// Long Stroke Limit
void PA12::LongStrokeLimit(int bID, int position){
	this->writeint(bID, 8, position);
}
int PA12::LongStrokeLimit(int bID){
	return this->readint(bID, 8);
}
/****************************************************************/
/* the Highest Limit Temperature								*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::maxTemperature(int bID, int temp){
	this->writeByte(bID, 0x0b, temp);
}
int PA12::maxTemperature(int bID){	
	return this->readByte(bID, 0x0b);
}

/****************************************************************/
/* Limit Voltage												*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::limitVolt(int bID,int level, int value){	
	this->writeByte(bID, 0x0c+level, value);
}
int PA12::limitVolt(int bID,int level){	
	return this->readByte(bID, 0x0c+level);
}
// the Lowest Limit Voltage
void PA12::minVolt(int bID, int value){
	this->writeByte(bID, 0x0c, value);
}
int PA12::minVolt(int bID){	
	return this->readByte(bID, 0x0c);
}
// the Highest Limit Voltage
void PA12::maxVolt(int bID, int value){	
	this->writeByte(bID, 0x0d, value);
}
int PA12::maxVolt(int bID){
	return this->readByte(bID, 0x0d);
}
/****************************************************************/
/* Max Force													*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::maxForce(int bID, int value){	
	this->writeint(bID, 14, value);	
}
int PA12::maxForce(int bID){	

	return this->readint(bID, 14);		
}
/****************************************************************/
/* Feedback Return Mode											*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::returnMode(int bID, int mode){	
	this->writeByte(bID, 16, mode);		
}
int PA12::returnMode(int bID){
	return this->readByte(bID, 16);
}
/****************************************************************/
/* Alarm LED													*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::alarmLed(int bID, int value){	
	this->writeByte(bID, 17, value);		
}
int PA12::alarmLed(int bID){
	return this->readByte(bID, 17);
}
/****************************************************************/
/* Alarm Shutdown												*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::alarmShutdown(int bID,int option){
	this->writeByte(bID, 18, option);
}
int PA12::alarmShutdown(int bID){
	return this->readByte(bID, 18);
}
/****************************************************************/
/* Resolution Factor											*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::resolutionFactor(int bID,int value){
	this->writeByte(bID, 22, value);
}
int PA12::resolutionFactor(int bID){
	return this->readByte(bID, 22);
}
/****************************************************************/
/* Calibration Stroke											*/
/* Type : Short,Loing - Read Only								*/
/*  	  Center - Read/write									*/
/****************************************************************/
int PA12::CalStroke(int bID, int dir){
	return this->readint(bID, 24+dir*2);
}
void PA12::CalStroke(int bID,int dir, int value){
	if(dir==2)
		this->writeint(bID, 28, value);	
}
/****************************************************************/
/* Third Party Program Interface								*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::ThirdPartyProgramInterface(int bID,int value){
	this->writeint(bID, 30, value);
}
int PA12::ThirdPartyProgramInterface(int bID){
	return this->readint(bID, 30);
}
/****************************************************************/
/* Third Party Program Firmware Version							*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::ThirdPartyProgramVersion(int bID,int value){
	this->writeByte(bID, 32, value);
}
int PA12::ThirdPartyProgramVersion(int bID){
	return this->readByte(bID, 32);
}
/****************************************************************/
/*  PID Gain													*/
/* Type : Read/write											*/
/****************************************************************/
/*void PA12::pidGain(int bID,int proportional, int integral,int derivative)
{
	this->writeByte(bID, 37, derivative);
	this->writeByte(bID, 38, integral);
	this->writeByte(bID, 39, proportional);
}*/
void PA12::pidGain(int bID,int num, int value){
	this->writeByte(bID, 37+num , value);
}
int PA12::pidGain(int bID,int num){
	return this->readByte(bID, 37+num);
}
/****************************************************************/
/* Stroke Pulse Width											*/
/* Dir : Pulse.Short, Pulse.Long, Pulse.Middle					*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::StrokePulseWidth(int bID,int num, int value){
	this->writeint(bID, 40+num*2, value);
}
int PA12::StrokePulseWidth(int bID,int num){
	return this->readint(bID, 40+num*2);
}

/****************************************************************/
/* Center Difference											*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::CenterDiffernce(int bID,int value){
	this->writeint(bID, 50, value);
}
int PA12::CenterDiffernce(int bID){
	return this->readint(bID, 50);
}
/****************************************************************/
/* Punch Initial Value											*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::PunchInitial(int bID,int value){
	this->writeint(bID, 52, value);
}
int PA12::PunchInitial(int bID){
	return this->readint(bID, 52);
}
// Parameter Map
/****************************************************************/
/* Force Enable													*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::forceEnable(int bID,int value){
	this->writeByte(bID, 0x80, value);
}
int PA12::forceEnable(int bID){
	return this->readByte(bID, 0x80);
}
/****************************************************************/
/* LED On/Off													*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::ledOn(int bID,int value){
	this->writeByte(bID, 0x81, value);//2 4 8
}
int PA12::ledOn(int bID){
	return this->readByte(bID, 0x81);
}
/****************************************************************/
/* Stroke Compliance Margin										*/
/* Dir : stroke.Short, stroke.Long		 						*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::complianceMargin(int bID,int dir, int value){	
	this->writeByte(bID, 0x82+dir, value);	
}
int PA12::complianceMargin(int bID,int dir){	
	return this->readByte(bID, 0x82+dir);
}
/****************************************************************/
/* Goal Position												*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::goalPosition(int bID,int value){	
	this->writeint(bID, 0x86, value);	
}
int PA12::goalPosition(int bID){	
	return this->readint(bID, 0x86);
}
/****************************************************************/
/* Moving Speed													*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::movingSpeed(int bID,int value){	
	this->writeint(bID, 0x88, value);	
}
int PA12::movingSpeed(int bID){	
	return this->readint(bID, 0x88);
}
/****************************************************************/
/* Force Limit													*/
/* Type : Read/write											*/
/****************************************************************/
void PA12::forceLimit(int bID,int value){	
	this->writeint(bID, 0x8a, value);	
}
int PA12::forceLimit(int bID){	
	return this->readint(bID, 0x8a);
}
/****************************************************************/
/* Present Position 											*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::presentPosition(int bID){	
	return this->readint(bID, 0x8c);
		
}
int PA12::quick_presentPosition(int bID){	
	//return this->readint(bID, 0x8c);
		
	this->clearBuffer();
	
	mParamBuffer[0] = 0x8c;
	mParamBuffer[1] = 2;
	if(this->fast_txRxPacket(bID, CMD_READ, 2)){
		return IRP_MAKEint(mRxBuffer[6],mRxBuffer[7]);//( (((int)mRxBuffer[6])<<8)+ mRxBuffer[5] );
	}
	else{
		return 0xffff;
	}	
}
/****************************************************************/
/* Present Speed	 											*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::presentSpeed(int bID){	
	return this->readint(bID, 0x8e);
}
/****************************************************************/
/* Present Load	 												*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::presentLoad(int bID){	
	return this->readint(bID, 0x90);
}
/****************************************************************/
/* Present Voltage 												*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::presentVolt(int bID){	
	return this->readByte(bID, 0x92);
}
/****************************************************************/
/* Present Temperature											*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::presentTemperature(int bID){	
	return this->readByte(bID, 0x93);
}
/****************************************************************/
/* Receive Data													*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::receiveData(int bID){	
	return this->readByte(bID, 0x94);
}
/****************************************************************/
/* Moving														*/
/* Type : Read Only												*/
/****************************************************************/
int PA12::Moving(int bID){	
	return this->readByte(bID, 0x96);
}
/****************************************************************/
/* Lock															*/
/* Type : Read/Write											*/
/****************************************************************/
void PA12::Lock(int bID,int value){	
	this->writeByte(bID, 0x97, value);	
}
int PA12::Lock(int bID){	
	return this->readByte(bID, 0x97);
}
/****************************************************************/
/* Punch														*/
/* Type : Read/Write											*/
/****************************************************************/

void PA12::Punch(int bID,int value){	
	this->writeByte(bID, 0x98, value);	
}
int PA12::Punch(int bID){	
	return this->readByte(bID, 0x98);
}


/*
 * @brief initialize parameter and get ID, instruction for making packet
 * */
void PA12::initPacket(int bID, int bInst){
	mbLengthForPacketMaking = 0;
	mbIDForPacketMaking = bID;
	mbInstructionForPacketMaking = bInst;
	mCommStatus = 0;
}
/*
 * @brief just push parameters, individual ID or moving data, individual data length
 * */
void PA12::pushByte(int value){
	
	if(mbLengthForPacketMaking > 255)//prevent violation of memory access
		return;
	mParamBuffer[mbLengthForPacketMaking++] = value;
}
void PA12::pushParam(byte value){
	if(mbLengthForPacketMaking > 255)//prevent violation of memory access
			return;
	mParamBuffer[mbLengthForPacketMaking++] = value;
}
void PA12::pushParam(int value){

	if(mbLengthForPacketMaking > 255)//prevent violation of memory access
			return;
	mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)IRP_LOBYTE(value);
	mParamBuffer[mbLengthForPacketMaking++] = (unsigned char)IRP_HIBYTE(value);
}
/*
 * @brief transfers packets to PA12 bus
 * */
int PA12::flushPacket(void){

	//TxDString("\r\n");
	//TxD_Dec_U8(gbLengthForPacketMaking);
	PA12_Serial->flush();             // Waiting to transmit packet
	return 0;
}
/*
 * @brief return current the total packet length
 * */


/*
 * backlog.c
 *
 *  Created on: Jul 19, 2017
 *      Author: paull
 */
#include <backlog.h>
#include <stdint.h>
#include <spi_flash.h>
#include <string.h>
#include <stdlib.h>


/*
 * DESCRIPTION OF THE SYSTEM
 * VTS device is a vehicle tracking device which capture location and time stamped data along with emergency button
 * This captured data are send through GSM to IP network Server
 *
 * DESIGN RULES FOR STORAGE SUPPORTING MULTI SOCKET/SERVER BACKLOG
 * 1. All data captured are directly stored as a backlog to the record database in SPI Flash section of sector.
 * 2. All data captured are tag as TxTag with 2 reasons, (a) where to send (b)which all IP packet should be made
 * 3. Backlog data are read on availability of Server Socket connection, each server/socket transmit process handle read independently
 * 4. Backlog are store as a record (data + TxTag) in a circular log fashion with Head and Tail Pointer of Sector.
 * 5. Backlog data are override when circular log hit back the Tail Pointer of the Sector.
 * 6. Bad sector are detected on every write by write-read-back-check 3/5 times, and are recorded and tracked not to be reused.
 * 6. Wear leveling of SPI Flash is improve by moving one sector-up of Head and Tail Pointer on every first write when/after backlog is empty.
 * 7. To avoid successive flash write, sector buffer are maintained on RAM, This Ram buffer is auto adjusted/shuffled
 * 	  by deleting intermediate Zero TxTag record entry and rearranging the record backlog
 * 8. Another multiple sector Read-Write-buffer on RAM are maintained which can accommodate each socket transmit read pointer of record independently.
 * 9. Each socket/server transmit process reads the record in last-in-first-out fashion only.
 * 10.During transmission of backlog if new data is generated for that socket/server, recent record has to be send first.
 * 	  But TCP ack of pending transmission on that socket/server can block this transmission.
 * 11.VTS should not go into sleep mode if there is a backlog and connectivity
 *
 *
 */

struct spi_flash *spiflashHandler;
int retn;

static uint8_t FlashAccessInterlock=0;

BacklogHousekeepingTypes LDRecordManager;
//BacklogDataTypes LDdataRecord;

//***************************************************************************************
//########## 1 x 4096 BUFFER for RECORD ACQUISITION AND WRITE TO SECTOR ###############
//Write Buffer or ram copy of the first sector to be written
//This stack is used to accumulate all data acquired,
BacklogDataTypes ramDataWriteBuffer[DATARECORDINSECTOR]; //WRITE BUFFER ON ENTRY of record for Data
TxTagTypes ramTxTagWriteBuffer[TXTAGRECORDINSECTOR]; //WRITE BUFFER ON ENTRY of record for TxTag

uint8_t tempSectorBuff[FSECTORSIZE];
//***************************************************************************************
//########  6 x 4096 BUFFER FOR SOCKET RW CACHE ##################
//This buffer array is used by each per socket Sector Ptr have a cache for sector write operation.
//writing of this sector write cache is done when None of the SocketRWpointer point to it.
TxTagSectorBuffTypes ramTxTagRWbuff[NO_OF_SOCKET];
//bytes = 4096*6 array of sector chunk-wise record copy
//*****************************************************************************************
//*****************************************************************************************

uint8_t searchUnUsedRWPtrBuff(void);//return index of buffer in value of 1-6
uint8_t searchUsedRWPtrBuff(uint16_t sectorNo);
uint8_t checkUsed_RWPtrBuff(uint8_t buffIndx);//check anyone used this buffer, return no of buffer user
uint8_t checkAllTxFlagCleared(uint8_t *flagArray);

//uint8_t writeSPIFlashSector(uint16_t sectorNum, BacklogHousekeepingTypes* manager, BacklogDataTypes* buff);



void IncrementSectorPtr( uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorLast);
void DecrementSectorPtr( uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorLast)

uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff);

uint8_t EnterBadSector(uint16_t sectorNum, uint16_t* badsectorCnt,uint16_t* badsectorBuff, uint16_t maxBad);
uint8_t CheckBadsectorList(uint16_t sectorNum, uint16_t* badsectorCnt,uint16_t* badsectorBuff);

uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,TxTagTypes* rambuff,TxTagTypes* recordfound,uint8_t* recordindx);

void txsuccessCallback_Socket(uint8_t socketNo, uint8_t updatedTxStatus);


uint32_t getBacklogCnt(uint8_t socketNo)
{
	uint32_t remainRecord;
	remainRecord=LDRecordManager.RecordRemaining[socketNo];
	return remainRecord;
}

void txsuccessCallback_Socket(uint8_t socketNo, uint8_t updatedTxStatus)
{
//	while(FlashAccessInterlock==1);
//
//	FlashAccessInterlock=1;
//
//	//update the TxTag on RAM copy, if all packet are cleared,
//	if(updatedTxStatus == 0)//if all cleared i.e all packet if socket is transmitted successfully, clear counter
//	{
//		if(LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg==0)
//		{
//			//Normal backlog transmission
//			//clear or update flag
//			LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer->TxTag.FlagField.Socket[socketNo].Pkt_TxStatus=updatedTxStatus;
//
//			LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex--;
//			LDRecordManager.RecordRemaining[socketNo]--;
//			if(checkAllTxFlagCleared(LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer->TxTag.AllFlag)==0)
//				LDRecordManager.RecordWriteCnt--;//decrement only if all Pkt_TxStatus are clear for all socket
//
//			//clear the buffer index if current index move out of this buffer
//			if(LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex==0)
//			{
//				if(checkUsed_RWPtrBuff(LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex)==0)
//				{	//all clear, record finish on this sector copy, no other socket ptr use the buffer
//					//#####write data to flash for this case **********************#########
//
//					if(ERROR == writeSPIFlashSector(LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr, &LDRecordManager, LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer))
//					{	//write FAIL,
//						_Error_Handler(__FILE__, __LINE__);
//					}
//					//retn=spiflashHandler->erase(spiflashHandler,LDRecordManager.Socket_RWptr[socketNo].SectorPtr,4096);
//					//retn=spiflashHandler->write(spiflashHandler,LDRecordManager.Socket_RWptr[socketNo].SectorPtr,sizeof(SectorDataBuffTypes),LDRecordManager.Socket_RWptr[socketNo].buffer);
//				}
//				LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=0; //clear buffer by reseting buffer index
//				LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer =ramDataWriteBuffer;//point to buffer_index=0, idle pointing buffer
//
//				if(LDRecordManager.RecordRemaining[socketNo]!=0)
//				{	//point sector backward
//					LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr +=1;
//					if(LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr==LDRecordManager.DataSectorLast)//boundary
//						LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr=LDRecordManager.DataSectorStart;
//				}
//				else
//				{	//point back to current sector HeadPtr
//					LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr=LDRecordManager.DataSectorHeadPtr;
//				}
//			}
//		}
//		else
//		{   //intermediate data while reading/sending backlog is transmitted and is completed
//			LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg=0;//reset the flag
//			//clear txflag directly from Writebuffer
//			ramDataWriteBuffer[LDRecordManager.Socket_RWptr[socketNo].InterRecordSendRecIndx].TxTag.FlagField.Socket[socketNo].Pkt_TxStatus = updatedTxStatus;
//			LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt--;
//		}
//	}
//
//	FlashAccessInterlock=0;
}

uint8_t checkAllTxFlagCleared(uint8_t *flagArray)
{
	uint8_t noOfNotclear=0;
	for(int8_t g=0;g<NO_OF_SOCKET;g++)
	{
		if(flagArray[g]!=0)
			noOfNotclear++;
	}
	return noOfNotclear;
}
uint8_t enterRecordwithTxTag( TxTagTypes* recordtxtag,BacklogDataTypes* recorddata )
{
	uint16_t maxbadsector=DATAFLASH_MAXBADSECTOR;// On failure of Sector write it move to next automatically till max sector bad cnt

	while(FlashAccessInterlock==1);

	FlashAccessInterlock=1;

	//rearrange uncompleted TxTag in the ramstackbuff if some TxTag in stackbbuffer has been all cleared by some record
	//Not require right now, can be implemented later, since other multiple socket TxTag pointing to the buffer needs to be handled.

	//Enter TXTAG record on rambuffer,
	//Enter the data by copying data record on buffer
	memcpy(&ramDataWriteBuffer[LDRecordManager.DataEntrystackIndex], &recorddata, sizeof(BacklogDataTypes));//say 91 bytes of data
	memcpy(&ramTxTagWriteBuffer[LDRecordManager.TxTagEntrystackIndex], &recordtxtag, sizeof(TxTagTypes));//say 6 bytes of data

	//increment index
	LDRecordManager.DataEntrystackIndex++;//say max is 45
	LDRecordManager.TxTagEntrystackIndex++;//say max is 682
	LDRecordManager.RecordWriteCnt++;//increment total count
	if(LDRecordManager.RecordWriteCnt>=LDRecordManager.MaxNoRecord)
	{
		LDRecordManager.RecordWriteCnt=LDRecordManager.MaxNoRecord;//saturate flag overflow
		LDRecordManager.RecordFull=1;
	}
	LDRecordManager.RecordNumber++;//record no maxrecord,

	for(int socketnum=0;socketnum < NO_OF_SOCKET; socketnum++)
	{
		if(recordtxtag->AllFlag[socketnum]!= 0)
		{
			LDRecordManager.RecordRemaining[socketnum]++; //for specific socket //LDRecordManager.DataEntrystackIndex++;
			if(LDRecordManager.Socket_RWptr[socketnum].InterRecordSendFlg==1)
				LDRecordManager.Socket_RWptr[socketnum].InterRecordGenerateCnt++;
		}
	}

	//if data record buffer is full push to Flash
	if(LDRecordManager.DataEntrystackIndex == LDRecordManager.NoOfDataRecordinSector)
	{
		//reset datarecord index
		LDRecordManager.DataEntrystackIndex=0;
		//write datarecord sector buffer to Flash memory in circular fashion
		maxbadsector=DATAFLASH_MAXBADSECTOR;
		while(maxbadsector !=0)//try multiple write on sector in sequence circular fashion until bad sector limit is crossed
		{
			//proceed if it is not a bad sector
			if(NOTFOUND == CheckBadsectorList(LDRecordManager.DataSectorHeadPtr,&LDRecordManager.DataBadsectorCnt,&LDRecordManager.DataBadsectorbuff))
			{
				//HeadPtr sector not found on badsector list, write the data
				if(ERROR == writeSPIFlashSector(LDRecordManager.DataSectorHeadPtr, sizeof(DataSectorBuffTypes), ramDataWriteBuffer))
				{	//write FAIL, enter into record bad sector list
					if(ERROR==EnterBadSector(LDRecordManager.DataSectorHeadPtr,&LDRecordManager.DataBadsectorCnt,&LDRecordManager.DataBadsectorbuff,DATAFLASH_MAXBADSECTOR))
					{    //bad sector trace list is full, can not handle store anymore
						FlashAccessInterlock=0;
						_Error_Handler(__FILE__, __LINE__);
					}
					//recalculate Max Record which is based on Data record sector bad
					LDRecordManager.MaxNoRecord = ((((LDRecordManager.DataSectorLast-LDRecordManager.DataSectorStart)+1) - LDRecordManager.DataBadsectorCnt) * (LDRecordManager.NoOfDataRecordinSector));
				}
				else
				{  	//####******* Write successfully, clear buffer by resetting DataEntrystackIndex

					LDRecordManager.DataEntrystackIndex=0;

					//increment HeadPtr
					IncrementSectorPtr(&LDRecordManager.DataSectorHeadPtr,LDRecordManager.DataSectorStart,LDRecordManager.DataSectorLast);//increment Headptr, noted to pass reference of Ptr

					if(LDRecordManager.DataSectorHeadPtr == LDRecordManager.DataSectorTailPtr)//if full
					{
						//record is FULL, start overwriting increment TailPtr
						IncrementSectorPtr(&LDRecordManager.DataSectorTailPtr,LDRecordManager.DataSectorStart,LDRecordManager.DataSectorLast);
					}
					break;
				}

			}
			IncrementSectorPtr(&LDRecordManager.DataSectorHeadPtr,LDRecordManager.DataSectorStart,LDRecordManager.DataSectorLast);
			maxbadsector--;//decrement for next try on other sector
		}
		if(maxbadsector==0)//too many bad sector exit program
		{    //bad sector trace list is full
			FlashAccessInterlock=0;
			_Error_Handler(__FILE__, __LINE__);//EXCEPTION CATCH
		}
	}

	if(LDRecordManager.RecordNumber == (LDRecordManager.MaxNoRecord+1))
		LDRecordManager.RecordNumber=0;//retain the record number 1 - maxNoRecord till new data is genenerated

	//if max record reach TXTAG buffer is full/half filled, push to Flash
	if(LDRecordManager.RecordNumber == LDRecordManager.MaxNoRecord)//MaxNoRecord is updated dynamically on event of bad sector in Data Section
	{


		//write at this case is max record reached and index to record to zero and TxTagSectorHeadpointer point to start
		maxbadsector=TXTAGFLASH_MAXBADSECTOR;
		while(maxbadsector !=0)
		{
			//proceed if it is not a bad sector
			if(NOTFOUND == CheckBadsectorList(LDRecordManager.TxTagSectorHeadPtr,&LDRecordManager.TxTagBadsectorCnt,&LDRecordManager.TxTagBadsectorbuff))
			{
				if(ERROR == writeSPIFlashSector(LDRecordManager.TxTagSectorHeadPtr, sizeof(TxTagTypes)*LDRecordManager.TxTagEntrystackIndex, ramTxTagWriteBuffer))
				{	//write FAIL, enter into record bad sector list
					if(ERROR==EnterBadSector(LDRecordManager.TxTagSectorHeadPtr,&LDRecordManager.TxTagBadsectorCnt,&LDRecordManager.TxTagBadsectorbuff,TXTAGFLASH_MAXBADSECTOR))
					{    //bad sector trace list is full, cannot handle anymore storage mechanism
						FlashAccessInterlock=0;
						_Error_Handler(__FILE__, __LINE__);
					}
				}
				else
				{  	//####******* Write successfully, clear buffer and enter record
					//Enter the data by copying record on buffer
					LDRecordManager.TxTagEntrystackIndex=0;

					//if it hits max record do TxTagSectorHeadPtr point to TxTagSectorTailPtr, start override
					LDRecordManager.TxTagSectorHeadPtr = LDRecordManager.TxTagSectorTailPtr;
					if(LDRecordManager.TxTagSectorHeadPtr == LDRecordManager.TxTagSectorTailPtr)
					{
						//record is FULL, start overwriting
						IncrementSectorPtr(&LDRecordManager.TxTagSectorTailPtr,LDRecordManager.TxTagSectorStart,LDRecordManager.TxTagSectorLast);
					}
					break;
				}

			}
			IncrementSectorPtr(&LDRecordManager.TxTagSectorHeadPtr,LDRecordManager.TxTagSectorStart,LDRecordManager.TxTagSectorLast);
			maxbadsector--;//decrement for next try on other sector
		}
		if(maxbadsector==0)//too many bad sector exit program
		{    //bad sector trace list is full, cannot handle anymore storage mechanism
			_Error_Handler(__FILE__, __LINE__);//EXCEPTION CATCH
		}

	}///else normal TxTagEntry
	else if(LDRecordManager.TxTagEntrystackIndex == LDRecordManager.NoOfTxTagRecordinSector)
	{
		//############### Check some socket using the buffer and are wait for transmit complete


		//###########################

		//write sector on HeadPtr
		//write data to flash for this case **********************#########
//		if(LDRecordManager.TxTagSectorHeadPtr == LDRecordManager.TxTagSectorTailPtr)
//		{
//			if(LDRecordManager.RecordWriteCnt == LDRecordManager.NoOfTxTagRecordinSector)
//			{	//record empty and then even of first write sector
//				//increment both the sector to avoid multiple write to same sector on fluctuating connectivity (wear leveling 1)
//				IncrementSectorPtr(&LDRecordManager,&LDRecordManager.TxTagSectorHeadPtr);
//				IncrementSectorPtr(&LDRecordManager,&LDRecordManager.TxTagSectorTailPtr);
//			}
//		}
		maxbadsector=TXTAGFLASH_MAXBADSECTOR;
		while(maxbadsector !=0)
		{
			//proceed if it is not a bad sector
			if(NOTFOUND == CheckBadsectorList(LDRecordManager.TxTagSectorHeadPtr,&LDRecordManager.TxTagBadsectorCnt,&LDRecordManager.TxTagBadsectorbuff))
			{
				if(ERROR == writeSPIFlashSector(LDRecordManager.TxTagSectorHeadPtr, sizeof(TxTagSectorBuffTypes), ramTxTagWriteBuffer))
				{
					//write FAIL, enter into record bad sector list
					if(ERROR==EnterBadSector(LDRecordManager.TxTagSectorHeadPtr,&LDRecordManager.TxTagBadsectorCnt,&LDRecordManager.TxTagBadsectorbuff,TXTAGFLASH_MAXBADSECTOR))
					{    //bad sector trace list is full, cannot handle anymore storage mechanism
						_Error_Handler(__FILE__, __LINE__);
					}
				}
				else
				{  	//####******* Write successfully, clear buffer and enter record
					//Enter the data by copying record on buffer
					LDRecordManager.TxTagEntrystackIndex=0;

					IncrementSectorPtr(&LDRecordManager.TxTagSectorHeadPtr,LDRecordManager.TxTagSectorStart,LDRecordManager.TxTagSectorLast);//increment Headptr note to pass reference of Ptr

					if(LDRecordManager.TxTagSectorHeadPtr == LDRecordManager.TxTagSectorTailPtr)
					{
						//record is FULL, start overwriting
						IncrementSectorPtr(&LDRecordManager.TxTagSectorTailPtr,LDRecordManager.TxTagSectorStart,LDRecordManager.TxTagSectorLast);
					}
					break;//breaking while loop
				}

			}
			IncrementSectorPtr(&LDRecordManager.TxTagSectorHeadPtr,LDRecordManager.TxTagSectorStart,LDRecordManager.TxTagSectorLast);
			maxbadsector--;//decrement for next try on other sector
		}
		if(maxbadsector==0)//too many bad sector exit program
		{    //bad sector trace list is full
			FlashAccessInterlock=0;
			_Error_Handler(__FILE__, __LINE__);//EXCEPTION CATCH
		}


	}

	FlashAccessInterlock=0;
	return SUCCESS;
}

uint8_t getRecForSocket(uint8_t socketNo,BacklogDataTypes* recorddata )
{
	BacklogDataTypes tempRecord;
	uint16_t txtagrecordindx;

	while(FlashAccessInterlock==1);

	FlashAccessInterlock=1;

	if(LDRecordManager.RecordRemaining[socketNo]==0)//how many record for that socket???
	{	FlashAccessInterlock=0;
		return EMPTY; //No address or empty record
	}
	else//record present for one or more socket.
	{
		//check first read for that socket
		if(LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer==NULL)
		{
			//Point to RAM buffer of Data
			if(LDRecordManager.DataEntrystackIndex!=0)

			LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex=LDRecordManager.DataEntrystackIndex-1;
			LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr=LDRecordManager.DataSectorHeadPtr;
			LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer=ramTxTagWriteBuffer;
			LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex=0;
			LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt=0;
			LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg=0;
			LDRecordManager.Socket_RWptr[socketNo].InterRecordSendRecIndx=0;
			LDRecordManager.Socket_RWptr[socketNo].OldTxTagSectorPtr=0;
			LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=0;
			LDRecordManager.Socket_RWptr[socketNo].TxTagRecordIndex=LDRecordManager.TxTagEntrystackIndex-1;
			LDRecordManager.Socket_RWptr[socketNo].TxTagSectorPtr=LDRecordManager.TxTagSectorHeadPtr;


			//Point to RAM buffer of Data
			if(LDRecordManager.DataEntrystackIndex!=0)
			{


				txtagrecordindx=LDRecordManager.TxTagEntrystackIndex;
				while(txtagrecordindx!=0)//search backward
				{
					if(LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer[txtagrecordindx++].AllFlag!=0)
					{
						if(LDRecordManager.RecordFull!=0)
						{

						}
					}
				}

			}
			else
			{   //just written to Flash, search data from Flash
				//search for

			}
		}

		if(LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr == LDRecordManager.DataSectorHeadPtr )
		{	//only ram

		}
		if(LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt==0)
		{	//No intermediate generated data entry... continue reading from previous backward
			if(LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex !=0)
			{	//
				readSPIFlash(LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex,LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr,)

			}

		}
		else






		if(LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr != LDRecordManager.DataSectorHeadPtr )
		//if(LDRecordManager.Socket_RWptr[socketNo].BufferIndex!=0)//same as above
		{   // there is a ram copy of data record buffer for that socket pointer

			if(LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt==0)
			{  //No new data generated in-between reading record from ram copy of sector, continue reading the buffer
				memcpy(&LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer[LDRecordManager.Socket_RWptr[socketNo].TxTagRecordIndex], &recorddata, sizeof(BacklogDataTypes));
				FlashAccessInterlock=0;
				return SUCCESS;
			}
			else//***** if between read record before completing transmission, new data/record is generated one or more *****
			{
				//check on RAM buffer from latest
				//uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,BacklogDataTypes* rambuff,BacklogDataTypes* recordfound,uint8_t* recordindx)
				if(SUCCESS==searchTagRecordOnBuffer(socketNo, &LDRecordManager, ramDataWriteBuffer, recorddata, &LDRecordManager.Socket_RWptr[socketNo].InterRecordSendRecIndx))
				{
					    //search function already copy recorddata and InterRecordSendRecIndx
						LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg=1; //to flag for tx complete callback
						// *** No not update SocketPtr and buffer in this case
						FlashAccessInterlock=0;
						return SUCCESS;//ramDataWriteBuffer[i];
				}
			}
		}


		//####socket is not pointing specific ramRWbuffer ****SEARCH****#//
		if(LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex==0)
		{
			//check on RAM buffer first and then to FLASH sector
			if(SUCCESS==searchTagRecordOnBuffer(socketNo, &LDRecordManager, ramDataWriteBuffer, recorddata, &LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex))
			{
					//updated recorddata to return and bufferindex by the baove function
					LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=0;//buffer index =0 means data record on flash unwritten ram copy
					//LDRecordManager.Socket_RWptr[socketNo].RecordIndex=i;
					LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr=LDRecordManager.DataSectorHeadPtr;
					LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer=ramDataWriteBuffer;
					FlashAccessInterlock=0;
					return SUCCESS;

			}


			//check/search on FLASH if not found on ramDataWriteBuffer & ramStackTxRWBuff
			if(LDRecordManager.RecordRemaining[socketNo]!=0)//check record on SPI FLash for that socket
			//if(LDRecordManager.RecordWriteCnt!=0)//check record on SPI FLash
			{

				int i=LDRecordManager.DataSectorHeadPtr;//start search for sector (Headptr-1) onward down
				while(i!=LDRecordManager.DataSectorTailPtr)//Search from Head to Tail on Flash
				{
					i=i-1;//move to last recent write sector
					if(i==LDRecordManager.DataSectorStart)//boundary
						i=LDRecordManager.DataSectorLast;

					//check if current point sector is a bad sector entry
					if(LDRecordManager.BadsectorCnt!=0)
					{
						for(int j=0;j<LDRecordManager.BadsectorCnt;j++)
						{
							if(i == LDRecordManager.Badsectorbuff[j])
							{
								i=i-1;//skip the sector and move one step toward sector Tailptr
								if(i==LDRecordManager.DataSectorStart)//check boundary
									i=LDRecordManager.DataSectorLast;
							}
						}
					}

					//read sector
					uint8_t k=DATARECORDINSECTOR;//0-44 (for a case of 45 record per sector)
					while(k!=0)
					{	k=k-1;//move from latest record within a sector i.e from last
						//************* Read Flash **********************#########
						retn=spiflashHandler->read(spiflashHandler, (i*LDRecordManager.SectorSizeBytes)+(k*LDRecordManager.DataRecordSizeBytes), sizeof(BacklogDataTypes),&tempRecord);
						if(tempRecord.TxTag.FlagField.Socket[socketNo].Pkt_TxStatus!=0)//found data to transmit for Socket 1
						{
							memcpy(&tempRecord, &recorddata, sizeof(BacklogDataTypes));

							//update SocketPtr
							LDRecordManager.Socket_RWptr[socketNo].DataRecordIndex=k;
							LDRecordManager.Socket_RWptr[socketNo].DataSectorPtr=i;
							if(LDRecordManager.TxTagBuffCnt==0)
							{
								//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
								LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=1;
								LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer=ramDataWriteBuffer;
							}
							else
							{
								//check first that current sector has been map to Ram buffer cache
								uint8_t bufindx=searchUsedRWPtrBuff(i);
								if(bufindx!=0)//found socket copy on RAM buffer cache
								{
									//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
									LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=bufindx;//1-6 mapp to buff[0] to buff[5]
									LDRecordManager.Socket_RWptr[socketNo].TxTagbuffer=ramTxTagRWbuff[bufindx-1].TxTagBuff;

								}
								else//if not mapped search unused buffer and map to that buffer
								{
									bufindx=searchUnUsedRWPtrBuff();
									if(bufindx!=0)//found unused buffer
									{
										//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
										LDRecordManager.Socket_RWptr[socketNo].TxTagBufferIndex=bufindx;//1-6 mapp on buff[0] to buff[5]
										//Copy sector record to buffer pointed by Socket1 RW Ptr
										//read flash **********************#########
										retn=spiflashHandler->read(spiflashHandler, (i*LDRecordManager.SectorSizeBytes), sizeof(TxTagSectorBuffTypes),ramTxTagRWbuff[bufindx-1].TxTagBuff);
									}
									else
										_Error_Handler(__FILE__, __LINE__);
								}
							}
							FlashAccessInterlock=0;
							return SUCCESS; //return tempRecord;
						}
					}
				}


			}

		}
		return NOTFOUND;
	}

}

void initLDBacklogManager()
{
	//first write default value
	LDRecordManager.mtag = 0X5A5A5A5A;
	LDRecordManager.DataBadsectorCnt=0;
	LDRecordManager.TxTagBadsectorCnt=0;
	for(int i=0;i<FLASH_MAXBADDATASECTORBUFF;i++)
	{
		LDRecordManager.DataBadsectorbuff[i]=0;
	}
	for(int i=0;i<FLASH_MAXBADTXTAGSECTORBUFF;i++)
	{
		LDRecordManager.TxTagBadsectorbuff[i]=0;
	}

	LDRecordManager.DataEntrystackIndex=0;
	LDRecordManager.SectorSizeBytes=FSECTORSIZE;
	LDRecordManager.DataRecordSizeBytes=DATARECORDSIZE;
	LDRecordManager.TxTagRecordSizeBytes=TXTAGRECORDSIZE;
	LDRecordManager.TotalSectorInFlash=FTOTALSECTOR;
	//LDRecordManager.TotalSector=FTOTALSECTOR-DATARECORDSTART_SECTOR;
	LDRecordManager.DataSectorStart=DATARECORDSTART_SECTOR;
	LDRecordManager.DataSectorLast=DATARECORDEND_SECTOR;
	LDRecordManager.DataSectorUsed=DATARECORDSTART_SECTOR;//consider offset as used

	LDRecordManager.TxTagSectorStart=TXTAGRECORDSTART_SECTOR;
	LDRecordManager.TxTagSectorLast=TXTAGRECORDEND_SECTOR;
	LDRecordManager.TxTagSectorUsed=TXTAGRECORDSTART_SECTOR;//consider offset as used
	//LDRecordManager.SectorUsed=0;

	LDRecordManager.NoOfDataRecordinSector=(LDRecordManager.SectorSizeBytes/LDRecordManager.DataRecordSizeBytes);
	LDRecordManager.NoOfTxTagRecordinSector=(LDRecordManager.SectorSizeBytes/LDRecordManager.TxTagRecordSizeBytes);
	LDRecordManager.MaxNoRecord = ((((LDRecordManager.DataSectorLast-LDRecordManager.DataSectorStart)+1) - LDRecordManager.DataBadsectorCnt) * (LDRecordManager.NoOfDataRecordinSector));
	LDRecordManager.DataSectorHeadPtr=DATARECORDSTART_SECTOR;
	LDRecordManager.DataSectorTailPtr=DATARECORDSTART_SECTOR;
	LDRecordManager.TxTagSectorHeadPtr=TXTAGRECORDSTART_SECTOR;
	LDRecordManager.TxTagSectorTailPtr=TXTAGRECORDSTART_SECTOR;
	LDRecordManager.RecordWriteCnt=0;
	for(int i=0;i<NO_OF_SOCKET;i++)
	{
		LDRecordManager.RecordRemaining[i]=0;
	}
	for(int i=0;i<NO_OF_SOCKET;i++)
	{
		LDRecordManager.Socket_RWptr[i].TxTagBufferIndex=0; //point to WRecBuff i.e index 0
		LDRecordManager.Socket_RWptr[i].TxTagRecordIndex=0;
		LDRecordManager.Socket_RWptr[i].DataSectorPtr=LDRecordManager.DataSectorHeadPtr;
		LDRecordManager.Socket_RWptr[i].DataRecordIndex=0;
		LDRecordManager.Socket_RWptr[i].TxTagSectorPtr=LDRecordManager.TxTagSectorHeadPtr;
		LDRecordManager.Socket_RWptr[i].TxTagRecordIndex=0;

		LDRecordManager.Socket_RWptr[i].TxTagbuffer=ramDataWriteBuffer;
		LDRecordManager.Socket_RWptr[i].InterRecordGenerateCnt=0;
		LDRecordManager.Socket_RWptr[i].InterRecordSendFlg=0;
		LDRecordManager.Socket_RWptr[i].InterRecordSendRecIndx=0;
		LDRecordManager.Socket_RWptr[i].TxTagbuffer=NULL;

	}



	LDRecordManager.checksum=0;

	uint8_t cp[sizeof(BacklogHousekeepingTypes)];
	memcpy(cp, &LDRecordManager, sizeof(BacklogHousekeepingTypes));
	for(int i=8;i<sizeof(BacklogHousekeepingTypes);i++)
	{
		LDRecordManager.checksum=*(cp+i);
	}

	//ALERT AND NOTE: spi flash on erase result in 0xFF, so its good to write all Zero for once during first format.


//	LDdataRecord.TxTag.AllFlag=0;
//	LDdataRecord.TxTag.FlagField.Socket1.pktType.Pvt=0;
}


//#################### Utility function SECTION
uint8_t searchUnUsedRWPtrBuff(void)
{
	int g,l;
	for(g=0;g<NO_OF_SOCKET;g++)
	{
		for(l=1;l<(NO_OF_SOCKET+1);l++)//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
		{
			if(LDRecordManager.Socket_RWptr[g].TxTagBufferIndex!=l)
				return l;//return first founded unused buffer, i.e not point by anyone
		}
	}
	return 0;

}
uint8_t searchUsedRWPtrBuff(uint16_t sectorNo)//check RAM buffer who has a copy of this sector no.
{
	int g=0;
	for(g=0;g<NO_OF_SOCKET;g++)
	{
		if(LDRecordManager.Socket_RWptr[g].DataSectorPtr==sectorNo)
			return LDRecordManager.Socket_RWptr[g].TxTagBufferIndex;
	}
	return ERROR;

}

uint8_t checkUsed_RWPtrBuff(uint8_t buffIndx)//check anyone used this buffer, return no of buffer user
{
	uint8_t p=0;
	uint8_t NoUser=0;
	if(buffIndx!=0)//buffIndx point 0 mean pointing transmit buffer No need to check
	{
		for(p=0;p<NO_OF_SOCKET;p++)
		{
			if(LDRecordManager.Socket_RWptr[p].TxTagBufferIndex == buffIndx)
				NoUser++;

		}
	}
	return NoUser;
}


uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,TxTagTypes* rambuff,TxTagTypes* recordfound,uint8_t* recordindx)
{
	int16_t i=manager->DataEntrystackIndex;
	while( i<0 )//look for TxTag in ramstack first, if found , write RWbuffer data to flash, reset buffer
	{
		if(rambuff[i].FlagField.Socket[socketNumber].Pkt_TxStatus != 0)//found
		{
			memcpy(&rambuff[i], &recordfound, sizeof(BacklogDataTypes));
			*recordindx=i;
			return SUCCESS;
		}
		i=i-1;//index back to record on current record write copy on RAM.
	}
	return NOTFOUND;
}

uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff)
{

	uint8_t retrycnt=FLASHWRITE_RETRY; //Handle multiple retry of Sector Write
	uint16_t retn=0;
	uint32_t addrs= sectorNum*FSECTORSIZE;

	while(retrycnt!=0)//try multiple write if not successful
	{
		retn=0;
		retn=spiflashHandler->erase(spiflashHandler,addrs,4096);
		//to stimulate badsector change buff to corrupt the data if sector at specific sector no.

		retn|=spiflashHandler->write(spiflashHandler,addrs,lenByte,buff);
		//to stimulate badsector change back buff

		retn|=spiflashHandler->read(spiflashHandler, addrs,lenByte,tempSectorBuff);
		if(retn==0 && memcmp(buff,tempSectorBuff,lenByte)==0)
		{
			return SUCCESS;
		}
		retrycnt--;//decrement for next try if not success
	}
	return ERROR;//write fail on a sector
}
uint8_t EnterBadSector(uint16_t sectorNum, uint16_t* badsectorCnt,uint16_t* badsectorBuff, uint16_t maxBad)
{
	//record bad sector

	*(badsectorBuff+*badsectorCnt)=sectorNum;
	*badsectorCnt=*badsectorCnt+1;
	if(*badsectorCnt > maxBad)
		return ERROR;
	else
		return SUCCESS;
}

uint8_t CheckBadsectorList(uint16_t sectorNum, uint16_t* badsectorCnt,uint16_t* badsectorBuff)
{
	//check if badsector if so skip

	for(int q=0;q<*badsectorCnt;q++)
	{
		if(sectorNum == *(badsectorBuff+q))
		{
			return FOUND;
		}
	}
	return NOTFOUND;
}
void IncrementSectorPtr( uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorLast)
{
	*sectorPtr=*sectorPtr+1; //go ahead to try write on Next sector
	if(*sectorPtr > sectorLast)
	{
		*sectorPtr= sectorStart;
	}
}
void DecrementSectorPtr(uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorLast)
{
	*sectorPtr=*sectorPtr-1; //go ahead to try write on Next sector
	if(*sectorPtr < sectorStart)
	{
		*sectorPtr= sectorLast;
	}
}

uint16_t getSectorNumberDataRecord(uint32_t recordNo)
{
	uint16_t sectorNo=LDRecordManager.DataSectorStart;
	if(LDRecordManager.DataBadsectorCnt==0)
	{
		if()
			if(recordNo <= LDRecordManager.NoOfDataRecordinSector )
			{
				return sectorNo;
			}
			else
			{
				sectorNo=recordNo /(LDRecordManager.DataSectorHeadPtr-LDRecordManager.DataSectorTailPtr);
			}
	}

		return LDRecordManager.Da

}


//##########################################



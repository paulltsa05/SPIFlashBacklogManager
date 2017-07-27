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

BacklogHousekeepingTypes LDRecordManager;
//BacklogDataTypes LDdataRecord;

//***************************************************************************************
//########## 1 x 4096 BUFFER for RECORD ACQUISITION AND WRITE TO SECTOR ###############
//Write Buffer or ram copy of the first sector to be written
//This stack is used to accumulate all data acquired,
BacklogDataTypes ramStack_LD_WRecBuff[RECORDINSECTOR];

//***************************************************************************************
//########  6 x 4096 BUFFER FOR SOCKET RW CACHE ##################
//This buffer array is used by each per socket Sector Ptr have a cache for sector write operation.
//writing of this sector write cache is done when None of the SocketRWpointer point to it.
SectorBuffTypes ramStackTxRWBuff[NO_OF_SOCKET];
//bytes = 4096*6 array of sector chunk-wise record copy
//*****************************************************************************************
//*****************************************************************************************

uint8_t searchUnUsedRWPtrBuff(void);//return index of buffer in value of 1-6
uint8_t searchUsedRWPtrBuff(uint16_t sectorNo);
uint8_t checkUsed_RWPtrBuff(uint8_t buffIndx);//check anyone used this buffer, return no of buffer user
uint8_t checkAllTxFlagCleared(uint8_t *flagArray);

uint8_t writeSPIFlashSector(uint16_t* sectorPtr, BacklogHousekeepingTypes* manager, SectorBuffTypes* buff);

uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,BacklogDataTypes* rambuff,BacklogDataTypes* recordfound,uint8_t* recordindx);

void txsuccessCallback_Socket(uint8_t socketNo, uint8_t updatedTxStatus);


uint32_t getBacklogCnt(uint8_t socketNo)
{
	uint32_t remainRecord;
	remainRecord=LDRecordManager.RecordCntSocket[socketNo];
	return remainRecord;
}

void txsuccessCallback_Socket(uint8_t socketNo, uint8_t updatedTxStatus)
{


	//update the TxTag on RAM copy, if all packet are cleared,
	if(updatedTxStatus == 0)//if all cleared i.e all packet if socket is transmitted successfully, clear counter
	{
		if(LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg==0)
		{
			//Normal backlog transmission
			//clear or update flag
			LDRecordManager.Socket_RWptr[socketNo].buffer->TxTag.FlagField.Socket[socketNo].Pkt_TxStatus=updatedTxStatus;

			LDRecordManager.Socket_RWptr[socketNo].RecordIndex--;
			LDRecordManager.RecordCntSocket[socketNo]--;
			if(checkAllTxFlagCleared(LDRecordManager.Socket_RWptr[socketNo].buffer->TxTag.AllFlag)==0)
				LDRecordManager.RecordWriteCnt--;//decrement only if all Pkt_TxStatus are clear for all socket

			//clear the buffer index if current index move out of this buffer
			if(LDRecordManager.Socket_RWptr[socketNo].RecordIndex==0)
			{
				if(checkUsed_RWPtrBuff(LDRecordManager.Socket_RWptr[socketNo].BufferIndex)==0)
				{	//all clear, record finish on this sector copy, no other socket ptr use the buffer
					//#####write data to flash for this case **********************#########
					if(ERROR == writeSPIFlashSector(&LDRecordManager.Socket_RWptr[socketNo].SectorPtr, &LDRecordManager, &LDRecordManager.Socket_RWptr[socketNo].buffer))
					{
						_Error_Handler(__FILE__, __LINE__);
					}
					//retn=spiflashHandler->erase(spiflashHandler,LDRecordManager.Socket_RWptr[socketNo].SectorPtr,4096);
					//retn=spiflashHandler->write(spiflashHandler,LDRecordManager.Socket_RWptr[socketNo].SectorPtr,sizeof(SectorBuffTypes),LDRecordManager.Socket_RWptr[socketNo].buffer);
				}
				LDRecordManager.Socket_RWptr[socketNo].BufferIndex=0; //clear buffer by reseting buffer index
				LDRecordManager.Socket_RWptr[socketNo].buffer =ramStack_LD_WRecBuff;//point to buffer_index=0, idle pointing buffer

				if(LDRecordManager.RecordCntSocket[socketNo]!=0)
				{	//point sector backward
					LDRecordManager.Socket_RWptr[socketNo].SectorPtr +=1;
					if(LDRecordManager.Socket_RWptr[socketNo].SectorPtr==LDRecordManager.SectorLast)//boundary
						LDRecordManager.Socket_RWptr[socketNo].SectorPtr=LDRecordManager.SectorStart;
				}
				else
				{	//point back to current sector HeadPtr
					LDRecordManager.Socket_RWptr[socketNo].SectorPtr=LDRecordManager.SectorHeadPtr;
				}
			}
		}
		else
		{   //intermediate data while reading/sending backlog is transmitted and is completed
			LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg=0;//reset the flag
			//clear txflag directly from Writebuffer
			ramStack_LD_WRecBuff[LDRecordManager.Socket_RWptr[socketNo].InterRecordSendRecIndx].TxTag.FlagField.Socket[socketNo].Pkt_TxStatus = updatedTxStatus;
			LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt--;
		}
	}

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
uint8_t enterRecForSocket(uint8_t socketNo,BacklogDataTypes* recorddata )
{
	//rearrange uncompleted TxTag in the ramstackbuff if some TxTag in stackbbuffer has been all cleared by some record

	//increment the record pointer
	LDRecordManager.ramWstackIndex++;

	if(LDRecordManager.ramWstackIndex < LDRecordManager.NoOfRecordinSector)
	{ 	//ramstack has room for new record
		//Enter the data by copying record on buffer
		memcpy(&ramStack_LD_WRecBuff[LDRecordManager.ramWstackIndex], &recorddata, sizeof(BacklogDataTypes));
		LDRecordManager.RecordWriteCnt++;
	}
	else//ramstack is FULL, we have to write/override sector on SPI Flash
	{
		if(LDRecordManager.SectorHeadPtr == LDRecordManager.SectorTailPtr)
		{	//First entry on record empty, or all sector are Full
			LDRecordManager.MaxNoRecord = ((((LDRecordManager.SectorLast-LDRecordManager.SectorStart)+1) - LDRecordManager.BadsectorCnt) * LDRecordManager.NoOfRecordinSector);
			if(LDRecordManager.RecordWriteCnt>=LDRecordManager.MaxNoRecord)
			{   //All sector full
				LDRecordManager.RecordWriteCnt=LDRecordManager.MaxNoRecord;//clamp to Max.
				//Override the sector
				LDRecordManager.SectorTailPtr++;
				if(LDRecordManager.SectorTailPtr > LDRecordManager.SectorLast)//boundary
					LDRecordManager.SectorTailPtr=LDRecordManager.SectorStart;
				//write sector on HeadPtr
				//write data to flash for this case **********************#########
				if(ERROR == writeSPIFlashSector(&LDRecordManager.SectorHeadPtr, &LDRecordManager, ramStack_LD_WRecBuff))
				{
					_Error_Handler(__FILE__, __LINE__);
				}
			}

		}

	}
	return SUCCESS;
}

uint8_t getRecForSocket(uint8_t socketNo,BacklogDataTypes* recorddata )
{
	BacklogDataTypes tempRecord;


	if(LDRecordManager.RecordCntSocket[socketNo]==0)//how many record for that socket???
		return EMPTY; //No address or empty record
	else//record present for one or more socket.
	{
		if(LDRecordManager.Socket_RWptr[socketNo].SectorPtr != LDRecordManager.SectorHeadPtr )
		//if(LDRecordManager.Socket_RWptr[socketNo].BufferIndex!=0)//same as above
		{   // there is a ram copy of buffer for that socket pointer already

			if(LDRecordManager.Socket_RWptr[socketNo].InterRecordGenerateCnt==0)
			{  //No new data generated in-between reading record from ram copy of sector, continue reading the buffer
				memcpy(&LDRecordManager.Socket_RWptr[socketNo].buffer[LDRecordManager.Socket_RWptr[socketNo].RecordIndex], &recorddata, sizeof(BacklogDataTypes));
				return SUCCESS;
			}
			else//***** if between read record before completing transmission, new data/record is generated one or more *****
			{
				//check on RAM buffer from latest
				//uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,BacklogDataTypes* rambuff,BacklogDataTypes* recordfound,uint8_t* recordindx)
				if(SUCCESS==searchTagRecordOnBuffer(socketNo, &LDRecordManager, ramStack_LD_WRecBuff, recorddata, &LDRecordManager.Socket_RWptr[socketNo].InterRecordSendRecIndx))
				{
					    //search function already copy recorddata and InterRecordSendRecIndx
						LDRecordManager.Socket_RWptr[socketNo].InterRecordSendFlg=1; //to flag for tx complete callback
						// *** No not update SocketPtr and buffer in this case
						return SUCCESS;//ramStack_LD_WRecBuff[i];
				}
			}
		}


		//####socket is not pointing specific ramRWbuffer ****SEARCH****#//
		if(LDRecordManager.Socket_RWptr[socketNo].BufferIndex==0)
		{
			//check on RAM buffer first and then to FLASH sector
			if(SUCCESS==searchTagRecordOnBuffer(socketNo, &LDRecordManager, ramStack_LD_WRecBuff, recorddata, &LDRecordManager.Socket_RWptr[socketNo].RecordIndex))
			{
					//updated recorddata to return and bufferindex by the baove function
					LDRecordManager.Socket_RWptr[socketNo].BufferIndex=0;//buffer index =0 means data record on flash unwritten ram copy
					//LDRecordManager.Socket_RWptr[socketNo].RecordIndex=i;
					LDRecordManager.Socket_RWptr[socketNo].SectorPtr=LDRecordManager.SectorHeadPtr;
					LDRecordManager.Socket_RWptr[socketNo].buffer=ramStack_LD_WRecBuff;

					return SUCCESS;

			}


			//check/search on FLASH if not found on ramStack_LD_WRecBuff & ramStackTxRWBuff
			if(LDRecordManager.RecordCntSocket[socketNo]!=0)//check record on SPI FLash for that socket
			//if(LDRecordManager.RecordWriteCnt!=0)//check record on SPI FLash
			{

				int i=LDRecordManager.SectorHeadPtr;//start search for sector (Headptr-1) onward down
				while(i!=LDRecordManager.SectorTailPtr)//Search from Head to Tail on Flash
				{
					i=i-1;//move to last recent write sector
					if(i==LDRecordManager.SectorStart)//boundary
						i=LDRecordManager.SectorLast;

					//check if current point sector is a bad sector entry
					if(LDRecordManager.BadsectorCnt!=0)
					{
						for(int j=0;j<LDRecordManager.BadsectorCnt;j++)
						{
							if(i == LDRecordManager.Badsectorbuff[j])
							{
								i=i-1;//skip the sector and move one step toward sector Tailptr
								if(i==LDRecordManager.SectorStart)//check boundary
									i=LDRecordManager.SectorLast;
							}
						}
					}

					//read sector
					uint8_t k=RECORDINSECTOR;//0-44 (for a case of 45 record per sector)
					while(k!=0)
					{	k=k-1;//move from latest record within a sector i.e from last
						//************* Read Flash **********************#########
						retn=spiflashHandler->read(spiflashHandler, (i*LDRecordManager.SectorSizeBytes)+(k*LDRecordManager.RecordSizeBytes), sizeof(BacklogDataTypes),&tempRecord);
						if(tempRecord.TxTag.FlagField.Socket[socketNo].Pkt_TxStatus!=0)//found data to transmit for Socket 1
						{
							memcpy(&tempRecord, &recorddata, sizeof(BacklogDataTypes));

							//update SocketPtr
							LDRecordManager.Socket_RWptr[socketNo].RecordIndex=k;
							LDRecordManager.Socket_RWptr[socketNo].SectorPtr=i;
							if(LDRecordManager.TxRWBuffCnt==0)
							{
								//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
								LDRecordManager.Socket_RWptr[socketNo].BufferIndex=1;
								LDRecordManager.Socket_RWptr[socketNo].buffer=ramStack_LD_WRecBuff;
							}
							else
							{
								//check first that current sector has been map to Ram buffer cache
								uint8_t bufindx=searchUsedRWPtrBuff(i);
								if(bufindx!=0)//found socket copy on RAM buffer cache
								{
									//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
									LDRecordManager.Socket_RWptr[socketNo].BufferIndex=bufindx;//1-6 mapp to buff[0] to buff[5]
									LDRecordManager.Socket_RWptr[socketNo].buffer=ramStackTxRWBuff[bufindx-1].ramStackBuff;

								}
								else//if not mapped search unused buffer and map to that buffer
								{
									bufindx=searchUnUsedRWPtrBuff();
									if(bufindx!=0)//found unused buffer
									{
										//NOTE: indexing of Buffer Index start from 1, since 0 is use as default pointer to ramstack write buffer
										LDRecordManager.Socket_RWptr[socketNo].BufferIndex=bufindx;//1-6 mapp on buff[0] to buff[5]
										//Copy sector record to buffer pointed by Socket1 RW Ptr
										//read flash **********************#########
										retn=spiflashHandler->read(spiflashHandler, (i*LDRecordManager.SectorSizeBytes), sizeof(SectorBuffTypes),ramStackTxRWBuff[bufindx-1].ramStackBuff);
									}
									else
										_Error_Handler(__FILE__, __LINE__);
								}
							}
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
	LDRecordManager.BadsectorCnt=0;
	LDRecordManager.ramWstackIndex=0;
	LDRecordManager.SectorSizeBytes=FSECTORSIZE;
	LDRecordManager.RecordSizeBytes=DATARECORDSIZE;
	LDRecordManager.TotalSector=FTOTALSECTOR;
	//LDRecordManager.TotalSector=FTOTALSECTOR-DATARECORDSTART_SECTOR;
	LDRecordManager.SectorStart=DATARECORDSTART_SECTOR;
	LDRecordManager.SectorLast=DATARECORDEND_SECTOR;
	LDRecordManager.SectorUsed=DATARECORDSTART_SECTOR;//consider offset as used
	//LDRecordManager.SectorUsed=0;

	LDRecordManager.NoOfRecordinSector=(LDRecordManager.SectorSizeBytes/LDRecordManager.RecordSizeBytes);
	LDRecordManager.MaxNoRecord = ((((LDRecordManager.SectorLast-LDRecordManager.SectorStart)+1) - LDRecordManager.BadsectorCnt) * (LDRecordManager.NoOfRecordinSector));
	LDRecordManager.SectorHeadPtr=DATARECORDSTART_SECTOR;
	LDRecordManager.SectorTailPtr=DATARECORDSTART_SECTOR;
	LDRecordManager.RecordWriteCnt=0;
	for(int i=0;i<NO_OF_SOCKET;i++)
	{
		LDRecordManager.RecordCntSocket[i]=0;
	}
	for(int i=0;i<NO_OF_SOCKET;i++)
	{
		LDRecordManager.Socket_RWptr[i].BufferIndex=0; //point to WRecBuff i.e index 0
		LDRecordManager.Socket_RWptr[i].RecordIndex=0;
		LDRecordManager.Socket_RWptr[i].SectorPtr=LDRecordManager.SectorTailPtr;
		LDRecordManager.Socket_RWptr[i].buffer=ramStack_LD_WRecBuff;
		LDRecordManager.Socket_RWptr[i].InterRecordGenerateCnt=0;
		LDRecordManager.Socket_RWptr[i].InterRecordSendFlg=0;
		LDRecordManager.Socket_RWptr[i].InterRecordSendRecIndx=0;

	}


	for(int i=0;i<256;i++)
	{
		LDRecordManager.Badsectorbuff[i]=0;
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
			if(LDRecordManager.Socket_RWptr[g].BufferIndex!=l)
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
		if(LDRecordManager.Socket_RWptr[g].SectorPtr==sectorNo)
			return LDRecordManager.Socket_RWptr[g].BufferIndex;
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
			if(LDRecordManager.Socket_RWptr[p].BufferIndex == buffIndx)
				NoUser++;

		}
	}
	return NoUser;
}


uint8_t searchTagRecordOnBuffer(uint8_t socketNumber, BacklogHousekeepingTypes* manager,BacklogDataTypes* rambuff,BacklogDataTypes* recordfound,uint8_t* recordindx)
{
	int16_t i=manager->ramWstackIndex;
	while( i<0 )//look for TxTag in ramstack first, if found , write RWbuffer data to flash, reset buffer
	{
		if(rambuff[i].TxTag.FlagField.Socket[socketNumber].Pkt_TxStatus != 0)//found
		{
			memcpy(&rambuff[i], &recordfound, sizeof(BacklogDataTypes));
			*recordindx=i;
			return SUCCESS;
		}
		i=i-1;//index back to record on current record write copy on RAM.
	}
	return NOTFOUND;
}

uint8_t writeSPIFlashSector(uint16_t* sectorPtr, BacklogHousekeepingTypes* manager, SectorBuffTypes* buff)
{
	uint8_t retrycnt=FLASHWRITE_RETRY; //Handle multiple retry of Sector Write
	uint16_t maxbad=FLASH_MAXBADSECTOR;// On failure of Sector write it move to next automatically till max sector bad cnt
	uint16_t retn=0;
	while(maxbad !=0)
	{
		retrycnt=FLASHWRITE_RETRY;
		while(retrycnt!=0)//try multiple write if not successful
		{
			retn=0;
			retn=spiflashHandler->erase(spiflashHandler,*sectorPtr,4096);
			retn|=spiflashHandler->write(spiflashHandler,*sectorPtr,sizeof(SectorBuffTypes),buff);
			if(retn==0)
			{
				return SUCCESS;
			}
			retrycnt--;//decrement for next try
		}
		//record bad sector
		manager->Badsectorbuff[manager->BadsectorCnt]=manager->SectorHeadPtr;
		manager->BadsectorCnt++;

		*sectorPtr=*sectorPtr+1; //go ahead to try write on Next sector
		if(*sectorPtr == manager->SectorLast)
		{
			*sectorPtr=manager->SectorStart;
		}

		maxbad--;//decrement for next try on other sector
	}

	return ERROR;
}

//##########################################



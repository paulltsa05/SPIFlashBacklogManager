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
 * DESIGN RULES FOR STORAGE
###  RULES
* Flash memory are divided into section of sector as shown below
* Different Record are manage by Recordmanager where the management data is stored at beginning SPI Flash section of record, loaded on startup and write back on sleep/shutdown
* Record Manager contains a Sector Table which store Availability, ReadIndex, write count tag for each sector of recordsector entry
* Availablity keep track of data availability in a sector where 0 signify its usable, and higest number representing the latest data of sector holder, negative availabilty value represent the sector is bad
* Readindex is used to track last read when  mutiple new data generate while half send of record already performed
* Writecnt track the number of erase-write cycle perform on that sector, 0 to 65535 of range and are staturated on 65535
* Badsector are tracked using negative availability, and are detected on erase-write & never reuse for write
 *
 *
 */

struct spi_flash *spiflashHandler;
uint8_t retn;

static uint8_t FlashAccessInterlock=0;


BacklogManagerTypes DBManager;
BacklogDataTypes rambuffEMGRecord[NOOF_EMGRECORDINSECTOR];
BacklogDataTypes rambuffALTRecord[NOOF_ALTRECORDINSECTOR];
BacklogDataTypes rambuffLDRecord[NOOF_LDRECORDINSECTOR];

uint8_t tempSectorBuff[FLASHSECTORSIZEBYTES];


//FUNCTIONS
static uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff);
static void IncrementSectorPtr( uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorSize);
static void DecrementSectorPtr(uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorSize);


//porting function to any SPI windbond driver
uint8_t SPIFlashInit(void);
uint8_t SPIFlashRead(uint32_t address, uint32_t byteLen, void *databuff);
uint8_t SPIFlashErase(uint32_t address, uint32_t byteLen);
uint8_t SPIFlashWrite(uint32_t address, uint32_t byteLen, void *databuff);


//_Error_Handler(__FILE__, __LINE__);

uint8_t ReadRecord(enum RecordTypes storeType, void *buff)
{
	FlashAccessInterlock=1;

	uint32_t readaddress=0;
	uint8_t tempsectorcnt;
	SectorTableType *SectorTable;
	BacklogDataTypes *EntrySectorBuffer;

	//Assigned SectorBuffer and SectorTable depend on Storage types Entry of record
	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
						EntrySectorBuffer =rambuffEMGRecord;
						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
						EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
						EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :  FlashAccessInterlock=0;
						return ERROR;
						break;
	}

	if(DBManager.EntryRecordbuffIndex[storeType]!=0)
	{
		memcpy( buff,&EntrySectorBuffer[DBManager.EntryRecordbuffIndex[storeType]-1],sizeof(BacklogDataTypes));
		DBManager.EntryRecordbuffIndex[storeType]--;
	}
	else
	{
		if(DBManager.StoredRecordCnt ==0)
		{
			FlashAccessInterlock=0;
			return EMPTY;
		}
		else
		{
			//read directly from Flash sector on where ReadSectorTablePtr Points
			tempsectorcnt=0;
			while(tempsectorcnt < DBManager.SectorSize[storeType])//traverse SectorTable[] by ReadSectorTablePtr backward
			{
				tempsectorcnt++;

				if(SectorTable[DBManager.ReadSectorTablePtr[storeType]].AvailabilityNumber < 0)//if it is bad sector skip
				{
					//move back
					DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
					tempsectorcnt++;
				}

				if(SectorTable[DBManager.ReadSectorTablePtr[storeType]].ReadRecordIndex!=0) //data not empty
						break;

				//move back decrement
				DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
			}
			if((tempsectorcnt == DBManager.SectorSize[storeType])&&(SectorTable[DBManager.ReadSectorTablePtr[storeType]].ReadRecordIndex==0))
			{	//all record send condition from flash
				FlashAccessInterlock=0;
				return EMPTY;
			}

			readaddress=( DBManager.ReadSectorTablePtr[storeType] *DBManager.FlashSectorSize); //get based address
			readaddress= readaddress + (SectorTable[DBManager.ReadSectorTablePtr[storeType]].ReadRecordIndex * RECORDSIZEBYTES );
			retn=SPIFlashRead(readaddress, sizeof(BacklogDataTypes),buff);
			if(retn != 0)
			{
				_Error_Handler(__FILE__, __LINE__);
			}
			//if read of record is finish for a particular sector, then make availabilty entry in sector Table to 0.


		}
		DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
		SectorTable[DBManager.EntrySectorTablePtr[storeType]].ReadRecordIndex--;
	}
	DBManager.StoredRecordCnt[storeType]--;

	if(DBManager.StoredRecordCnt[storeType] < DBManager.LatestAvailabilityNumber[storeType])
		DBManager.LatestAvailabilityNumber[storeType]--;

	FlashAccessInterlock=0;
	return SUCCESS;
}

uint8_t EnterRecord(enum RecordTypes storeType, void *buff)
{
	FlashAccessInterlock=1;

	uint16_t Sectindx=0,tempsectorcnt=0;
	SectorTableType *SectorTable;
	BacklogDataTypes *EntrySectorBuffer;

#ifdef DEBUG_BACKLOG
//	PRINTF("\n\r\n\rEnter RecordType : %d \n\r",storeType);
//	PRINTF("EntryRecordbuffIndex : %d \n\r",DBManager.EntryRecordbuffIndex[storeType]);

#endif


	//Assigned SectorBuffer and SectorTable depend on Storage types Entry of record
	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
						EntrySectorBuffer =rambuffEMGRecord;
						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
						EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
						EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :
						FlashAccessInterlock=0;
						return ERROR;
						break;

	}

	//check all boundary of buffer full, memory overflow, bad sector jump over and other housekeeping and management.
	if(DBManager.EntryRecordbuffIndex[storeType] >= NOOF_RECORDINSECTOR)
	{
		//WRITE to FLASH REQUIRED
		//write to flash,first check boundary and management of sector table


		if(DBManager.LatestAvailabilityNumber[storeType] == DBManager.SectorSize[storeType] - DBManager.badsectorCnt[storeType])
		{   //At event of one write cycle finish for the this record section of sector
			//decrement availability number, to make most old sector available to write recordbuffer
#ifdef DEBUG_BACKLOG
			PRINTF("Storage OverFlow $$$$$$$$############ Decrement all sector availabiltyNumber\n\r");
#endif
			DBManager.LatestAvailabilityNumber[storeType]--;

			for(Sectindx=0;Sectindx<DBManager.SectorSize[storeType];Sectindx++)
			{
				if(SectorTable[Sectindx].AvailabilityNumber <= 0)//if it is bad sector skip, or Zero-i.e available
					Sectindx++;
				SectorTable[Sectindx].AvailabilityNumber--;
				if((SectorTable[Sectindx].AvailabilityNumber==0) && (SectorTable[Sectindx].ReadRecordIndex !=0))
					SectorTable[Sectindx].ReadRecordIndex=0;
			}
		}


		tempsectorcnt=0;
		while(tempsectorcnt < DBManager.SectorSize[storeType])//traverse SectorTable[] by ALTSectorTablePtr
		{
			tempsectorcnt++;

			if(SectorTable[DBManager.EntrySectorTablePtr[storeType]].AvailabilityNumber < 0)//if it is bad sector skip
			{
				IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
				tempsectorcnt++;
			}
			if(SectorTable[DBManager.EntrySectorTablePtr[storeType]].AvailabilityNumber==0)//found sector to write
			{
				break;
			}
//			if(DBManager.LatestMaxSectorWriteCnt[storeType] > SectorTable[DBManager.EntrySectorTablePtr[storeType]].SectorWriteCnt)
//				break;


			IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
		}
		DBManager.LatestAvailabilityNumber[storeType]++;
		SectorTable[DBManager.EntrySectorTablePtr[storeType]].AvailabilityNumber=DBManager.LatestAvailabilityNumber[storeType];
		SectorTable[DBManager.EntrySectorTablePtr[storeType]].SectorWriteCnt++;

#ifdef DEBUG_BACKLOG
		PRINTF("Write Sector @ %d\n\r",DBManager.EntrySectorTablePtr[storeType]);
#endif

		//write the sector at ALTSectorTablePtr
		if(ERROR == writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], sizeof(BacklogDataTypes)*NOOF_EMGRECORDINSECTOR, EntrySectorBuffer))
		{
			SectorTable[DBManager.EntrySectorTablePtr[storeType]].AvailabilityNumber = -1; //record as bad sector
			DBManager.badsectorCnt[storeType]++;

			FlashAccessInterlock=0;
#ifdef DEBUG_BACKLOG
			PRINTF("Fail to write Sector.. Record as bad sector - %d\n\r",DBManager.EntrySectorTablePtr[storeType]);
#endif
			return ERROR;

		}
		DBManager.EntryRecordbuffIndex[storeType]=0;
		SectorTable[DBManager.EntrySectorTablePtr[storeType]].ReadRecordIndex=NOOF_RECORDINSECTOR;
		DBManager.LatestMaxSectorWriteCnt[storeType]=SectorTable[DBManager.EntrySectorTablePtr[storeType]].SectorWriteCnt;
		DBManager.ReadSectorTablePtr[storeType]=DBManager.EntrySectorTablePtr[storeType];
		//DBManager.EntrySectorTablePtr[storeType]++;
		IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);

#ifdef DEBUG_BACKLOG
		PRINTF("LatestAvailabilityNumber - %d\n\r",DBManager.LatestAvailabilityNumber[storeType]);
		PRINTF("LatestMaxSectorWriteCnt - %d\n\r",DBManager.LatestMaxSectorWriteCnt[storeType]);
		PRINTF("ReadSectorTablePtr - %d\n\r",DBManager.ReadSectorTablePtr[storeType]);
		PRINTF("StoredRecordCnt:%d\n\r",DBManager.StoredRecordCnt[storeType]);
		PRINTF("\r\n");
#endif

	}
	//boundary check done without error, then enter record on rambuffer
	memcpy(&EntrySectorBuffer[DBManager.EntryRecordbuffIndex[storeType]], buff, sizeof(BacklogDataTypes));
	DBManager.StoredRecordCnt[storeType]++;
	DBManager.EntryRecordbuffIndex[storeType]++;

	FlashAccessInterlock=0;

#ifdef DEBUG_BACKLOG
//	PRINTF("Record enter Successfully..StoredRecordCnt:%d\n\r",DBManager.StoredRecordCnt[storeType]);

#endif
	return SUCCESS;
}


uint8_t initRecordManager(void)
{
	FlashAccessInterlock=1;

	uint32_t chcksum=0,sizeOfmanager;
	uint8_t* bytebybyte;

	SPIFlashInit();

	//first write default value
	//check first time initialization of Record Manager
	bytebybyte=(uint8_t*)&DBManager;
	retn=SPIFlashRead(RECORDMANAGE_SECTORSTART*FLASHSECTORSIZEBYTES, sizeof(BacklogManagerTypes),bytebybyte);

#ifdef DEBUG_BACKLOG
//	PRINTF("\n\rConfiguration Read \n\r");
//	PRINTF("sector start:%d\n\r",RECORDMANAGE_SECTORSTART);
//	PRINTF("Length:%d\n\r",sizeof(BacklogManagerTypes));
//
//	for(int i=0;i<sizeof(BacklogManagerTypes);i++)
//	{
//		PRINTF("%x,",bytebybyte[i]);
//		if(i%32==0)
//			PRINTF("\n\r");
//		if(i==4096)
//			PRINTF("\n\r\n\rChunk2\n\r");
//	}
#endif


	chcksum=0;
	bytebybyte=(uint8_t*)&DBManager;
	for(int i=8; i < sizeof(BacklogManagerTypes);i++ )
	{
		chcksum = chcksum + bytebybyte[i];
	}
	if(DBManager.checksum!= chcksum)
	{
#ifdef DEBUG_BACKLOG
		PRINTF("\n\r\n\r Record Manager ChecksumError \n\r");
		PRINTF("Received checksum  : %x \n\r", DBManager.checksum );
		PRINTF("calculated checksum : %x \n\r", chcksum );
#endif
		if(DBManager.mtag == 0X5A5A5A5A)//check for first format
		{
			FlashAccessInterlock=0;
			return ERROR;
		}
	}
	if(DBManager.mtag != 0X5A5A5A5A)//if no tag match write default value
	{	//needs format to default state
		DBManager.mtag=0X5A5A5A5A; //store tag
		DBManager.FlashSectorSize=FLASHSECTORSIZEBYTES;
		DBManager.FlashNoOfSector=FLASHTOTALNOOFSECTOR;
		DBManager.FlashPageSize=FLASHPAGESIZEBYTES;
		DBManager.FlashNoOfPageinSector=FLASHNOOFPAGEINSECTOR;

		DBManager.SectorStart[RECORDMANAGER]=RECORDMANAGE_SECTORSTART;
		DBManager.SectorSize[RECORDMANAGER]=RECORDMANAGE_SECTORSIZE;
		DBManager.SectorStart[EMGRECORD]=EMGRECORD_SECTORSTART;
		DBManager.SectorSize[EMGRECORD]=EMGRECORD_SECTORSIZE;
		DBManager.SectorStart[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.SectorSize[ALTRECORD]=ALTRECORD_SECTORSIZE;
		DBManager.SectorStart[LDRECORD]=LDRECORD_SECTORSTART;
		DBManager.SectorSize[LDRECORD]=LDRECORD_SECTORSIZE;

		for(int i=1;i<NOOFRECORDTYPES;i++)
		{
			DBManager.LatestAvailabilityNumber[i]=0;
			DBManager.LatestMaxSectorWriteCnt[i]=0;
			DBManager.StoredRecordCnt[i]=0;

			DBManager.EntryRecordbuffIndex[i]=0;

			DBManager.badsectorCnt[i]=0;
		}



		DBManager.EntrySectorTablePtr[EMGRECORD]=EMGRECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[LDRECORD]=LDRECORD_SECTORSTART;

		DBManager.ReadSectorTablePtr[EMGRECORD]=EMGRECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[LDRECORD]=LDRECORD_SECTORSTART;

		DBManager.EntrySectorTablePtr[RECORDMANAGER]=RECORDMANAGE_SECTORSTART;//Not used
		DBManager.ReadSectorTablePtr[RECORDMANAGER]=RECORDMANAGE_SECTORSTART;//Not used
		//for(int i=0;i<(EMGRECORD_SECTORSIZE*sizeof(SectorTableType));i++)
		//	*(((uint8_t*)DBManager.EMGSectorTable) + i)=0;
		for(int i=0;i<(EMGRECORD_SECTORSIZE);i++)
		{	DBManager.EMGSectorTable[i].AvailabilityNumber=0;
			DBManager.EMGSectorTable[i].ReadRecordIndex=0;
			DBManager.EMGSectorTable[i].SectorWriteCnt=0;
		}

		for(int i=0;i<(ALTRECORD_SECTORSIZE);i++)
		{	DBManager.ALTSectorTable[i].AvailabilityNumber=0;
			DBManager.ALTSectorTable[i].ReadRecordIndex=0;
			DBManager.ALTSectorTable[i].SectorWriteCnt=0;
		}

		for(int i=0;i<(LDRECORD_SECTORSIZE);i++)
		{	DBManager.LDSectorTable[i].AvailabilityNumber=0;
			DBManager.LDSectorTable[i].ReadRecordIndex=0;
			DBManager.LDSectorTable[i].SectorWriteCnt=0;
		}

//		for(int i=0;i<(ALTRECORD_SECTORSIZE*sizeof(SectorTableType));i++)
//			*(((uint8_t*)DBManager.ALTSectorTable) + i)=0;
//		for(uint16_t i=0;i<(LDRECORD_SECTORSIZE*sizeof(SectorTableType));i++)
//			*(((uint8_t*)DBManager.LDSectorTable) + i)=0;


		//write to Record management section of SPI flash
		sizeOfmanager=sizeof(BacklogManagerTypes);
		bytebybyte=(uint8_t*)&DBManager;
		chcksum=0;
		for(int i=8; i < sizeof(BacklogManagerTypes);i++ )//calculate checksum
		{
			chcksum = chcksum + bytebybyte[i];
		}
		DBManager.checksum=chcksum;//store the new checksum
#ifdef DEBUG_BACKLOG
		PRINTF("\n\r\n\r********INITIALIZING AND FORMATING BACKLOG STORAGE*********** \n\r");
		PRINTF("\n\rCalculated checksum : %x \n\r", chcksum );
#endif
		retn=0;
		if(sizeOfmanager <= FLASHSECTORSIZEBYTES)
		{
			retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER], sizeOfmanager,bytebybyte);
		}
		else if(sizeOfmanager <= FLASHSECTORSIZEBYTES*2)//it support manager data size more than 1 sector or >4096 bytes
		{
			retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER],FLASHSECTORSIZEBYTES,bytebybyte);
			retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),sizeOfmanager-FLASHSECTORSIZEBYTES,bytebybyte+FLASHSECTORSIZEBYTES);
		}
		else ////it support manager data size more than 2 sector
		{
			retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER],FLASHSECTORSIZEBYTES,bytebybyte);
			retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),FLASHSECTORSIZEBYTES, bytebybyte+FLASHSECTORSIZEBYTES);
			retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+2),sizeOfmanager - FLASHSECTORSIZEBYTES, bytebybyte+(FLASHSECTORSIZEBYTES*2));
		}

		if(retn != SUCCESS)
		{	//unexceptional error
#ifdef DEBUG_BACKLOG
			PRINTF("\n\r\n\r********FLASH MEMORY ISSUE(or Corrupted ON RECORD MANAGER*********** \n\r");
#endif
			_Error_Handler(__FILE__, __LINE__);

		}

	}

	FlashAccessInterlock=0;
	return SUCCESS;

}


//#################### Utility function SECTION


uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff)
{

	uint8_t retrycnt=FLASHWRITE_RETRY; //Handle multiple retry of Sector Write
	uint16_t retn=0;
	uint32_t addrs= sectorNum*FLASHSECTORSIZEBYTES;
	uint8_t byteacessBuff[4096];

#ifdef DEBUG_BACKLOG
//	uint8_t* byteacess=(uint8_t *)buff;
//	PRINTF("\n\rSPI write function call \n\r");
//	PRINTF("sector:%d\n\r",sectorNum);
//	PRINTF("Length:%d\n\r",lenByte);
//	for(int i=0;i<lenByte;i++)
//	{
//		byteacessBuff[i]=byteacess[i];
//		PRINTF("%x,",byteacess[i]);
//		if(i%32==0)
//			PRINTF("\n\r");
//	}
#endif

	while(retrycnt!=0)//try multiple write if not successful
	{
		retn=0;
		retn=SPIFlashErase(addrs,4096);
		//to stimulate badsector change buff to corrupt the data if sector at specific sector no.
		//HAL_Delay(5);
		retn|=SPIFlashWrite(addrs,lenByte,byteacessBuff);
		//to stimulate badsector change back buff
		//HAL_Delay(5);
		retn|=SPIFlashRead(addrs,lenByte,tempSectorBuff);
		if(retn==0 && memcmp(byteacessBuff,tempSectorBuff,lenByte)==0)
		{
#ifdef DEBUG_BACKLOG
			PRINTF("Success Write\n\r");
#endif

			return SUCCESS;
		}
		retrycnt--;//decrement for next try if not success
	}

#ifdef DEBUG_BACKLOG
	PRINTF("Succes ERROR\n\r");
#endif
	return ERROR;//write fail on a sector
}

void IncrementSectorPtr( uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorSize)
{
	*sectorPtr=*sectorPtr+1; //go ahead to try write on Next sector
	if(*sectorPtr == (sectorStart+sectorSize))
	{
		*sectorPtr= sectorStart;
	}
}
void DecrementSectorPtr(uint16_t* sectorPtr, uint16_t sectorStart, uint16_t sectorSize)
{
	*sectorPtr=*sectorPtr-1; //go ahead to try write on Next sector
	if(*sectorPtr < (sectorStart))
	{
		*sectorPtr= (sectorStart+sectorSize)-1;
	}
}


//####################################################################################
//							DRIVER PORTING LAYER
//####################################################################################
uint8_t SPIFlashInit(void)
{
	//assume SPI driver is already initialize
	//it used spi_flash.c /.h and winbond driver to access SPI FLash over SPI

	spiflashHandler->spi =spi_setup_slave();

	spiflashHandler =spi_flash_probe();
	return SUCCESS;
}

uint8_t SPIFlashRead(uint32_t address, uint32_t byteLen, void *databuff)
{
	if(0==spiflashHandler->read(spiflashHandler, address,byteLen,databuff))
		return SUCCESS;
	else
		return ERROR;
}
uint8_t SPIFlashErase(uint32_t address, uint32_t byteLen)
{
	if(0==spiflashHandler->erase(spiflashHandler, address,byteLen))
		return SUCCESS;
	else
		return ERROR;
}
uint8_t SPIFlashWrite(uint32_t address, uint32_t byteLen, void *databuff)
{
	if(0==spiflashHandler->write(spiflashHandler, address,byteLen,databuff))
		return SUCCESS;
	else
		return ERROR;
}
//##################################################################################


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

static uint8_t FlashAccessInterlock=0,memoryoverrun=0;


BacklogManagerTypes DBManager;//manager data types

EMG_BacklogDataTypes rambuffEMGRecord[NOOF_EMGRECORDINSECTOR];
OTA_BacklogDataTypes rambuffOTARecord[NOOF_OTARECORDINSECTOR];
ALT_BacklogDataTypes rambuffALTRecord[NOOF_ALTRECORDINSECTOR];
LD_BacklogDataTypes rambuffLDRecord[NOOF_LDRECORDINSECTOR];
CAN_BacklogDataTypes rambuffCANRecord[NOOF_LDRECORDINSECTOR];

uint8_t tempSectorBuff[FLASHSECTORSIZEBYTES];


//FUNCTIONS
static uint8_t SaveRecordManager(void);
static uint8_t handleRecordSectorWrite(enum RecordTypes storeType);
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
	uint8_t* buffptr;
	SectorTableType *SectorTable;
	//BacklogDataTypes *EntrySectorBuffer;

	buffptr=(uint8_t*)buff;

	//Assigned SectorTable depend on Storage types Entry of record
	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
						//EntrySectorBuffer =rambuffEMGRecord;
						break;
		case OTARecord: SectorTable=DBManager.OTASectorTable;
						//EntrySectorBuffer =rambuffOTARecord;
						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
						//EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		case CANRecord:	SectorTable=DBManager.CANSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :  FlashAccessInterlock=0;
						return ERROR;
						break;
	}

	if(DBManager.EntryRecordbuffIndex[storeType]>0 && DBManager.EntryRecordbuffIndex[storeType] <DBManager.NoOfRecordInSector[storeType])
	{
		switch(storeType)
		{
			case EMGRecord: memcpy( buff,&rambuffEMGRecord[DBManager.EntryRecordbuffIndex[storeType]-1],DBManager.RecordSizeInBytes[storeType]);
							break;
			case OTARecord: memcpy( buff,&rambuffOTARecord[DBManager.EntryRecordbuffIndex[storeType]-1],DBManager.RecordSizeInBytes[storeType]);
							break;
			case ALTRecord: memcpy( buff,&rambuffALTRecord[DBManager.EntryRecordbuffIndex[storeType]-1],DBManager.RecordSizeInBytes[storeType]);
							break;
			case LDRecord:	memcpy( buff,&rambuffLDRecord[DBManager.EntryRecordbuffIndex[storeType]-1],DBManager.RecordSizeInBytes[storeType]);
							break;
			case CANRecord:	memcpy( buff,&rambuffCANRecord[DBManager.EntryRecordbuffIndex[storeType]-1],DBManager.RecordSizeInBytes[storeType]);
							break;
			default		 :  return ERROR;
							break;
		}


#ifdef DEBUG_BACKLOG
		PRINTF("\n\rREADING record from RAMwriteBuffer recordindex=%d\n\r",DBManager.EntryRecordbuffIndex[storeType]-1);
#endif
		DBManager.EntryRecordbuffIndex[storeType]--;
	}
	else
	{
		if(DBManager.StoredRecordCnt ==0)
		{
#ifdef DEBUG_BACKLOG
			PRINTF("RECORD EMPTY\n\r");
#endif
			if(DBManager.LatestAvailabilityNumber[storeType]>0)
				DBManager.LatestAvailabilityNumber[storeType]=0;
			FlashAccessInterlock=0;
			return EMPTY;
		}
		else
		{
			//read directly from Flash sector on where ReadSectorTablePtr Points

			if(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex==0)
			{ //search from flash else continue with readindex
				SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber=0;//reset Availability for that sector table, move to next
				//decrement the sector pointer first
				DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
				tempsectorcnt=0;
				while(tempsectorcnt < DBManager.SectorSize[storeType])//traverse SectorTable[] by ReadSectorTablePtr backward
				{
					tempsectorcnt++;

					if(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber <= 0)//if it is bad sector skip
					{
						//move back
						DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
						tempsectorcnt++;
					}

					if(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex!=0) //data not empty
							break;

					//move back decrement
					DecrementSectorPtr( &DBManager.ReadSectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
				}

				if((tempsectorcnt == DBManager.SectorSize[storeType])&&(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex==0))
				{	//all record send condition from flash
#ifdef DEBUG_BACKLOG
					PRINTF("RECORD EMPTY HAS WRITEN ALL FROM FLASH\n\r");
#endif
					if(DBManager.LatestAvailabilityNumber[storeType]>0)//avoid to reset badsector tag
						DBManager.LatestAvailabilityNumber[storeType]=0;
					FlashAccessInterlock=0;
					return EMPTY;
				}

				if(DBManager.LatestAvailabilityNumber[storeType]>0)
					DBManager.LatestAvailabilityNumber[storeType]--;

				if(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber >0)
					SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber--;


			}

			//just decrement and move on to read
			if(SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex >0)
				SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex--;

			readaddress=( DBManager.ReadSectorTablePtr[storeType] *DBManager.FlashSectorSize); //get based address
			readaddress= readaddress + ((SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex) * (DBManager.RecordSizeInBytes[storeType]) );
			retn=SPIFlashRead(readaddress, (uint32_t)DBManager.RecordSizeInBytes[storeType],buffptr);

//			PRINTF("\n\r Read Flash %d : %d , %u, %d",DBManager.ReadSectorTablePtr[storeType],SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex,readaddress, ((buffptr[1]<<8)+buffptr[0]));
			if(retn != 0)
			{
#ifdef DEBUG_BACKLOG
			PRINTF("########**********Flash READ ERROR @address %u",readaddress);
#endif
				_Error_Handler(__FILE__, __LINE__);
			}
			//if read of record is finish for a particular sector, then make availabilty entry in sector Table to 0.
#ifdef DEBUG_BACKLOG
			PRINTF("\n\rREADING record from Flash directly @Sector=%d, index=%d\n\r",DBManager.ReadSectorTablePtr[storeType],SectorTable[DBManager.ReadSectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex);
#endif

		}


	}
	DBManager.StoredRecordCnt[storeType]--;

	if(DBManager.StoredRecordCnt[storeType] < ((DBManager.SectorSize[storeType] -DBManager.badsectorCnt[storeType]) *DBManager.NoOfRecordInSector[storeType]))
		memoryoverrun=0;
	if(DBManager.StoredRecordCnt[storeType] == 0 && DBManager.LatestAvailabilityNumber[storeType]>0)
		DBManager.LatestAvailabilityNumber[storeType]=0;

	FlashAccessInterlock=0;
#ifdef DEBUG_BACKLOG
	PRINTF("StoredRecordCnt:%d\n\r",DBManager.StoredRecordCnt[storeType]);
	PRINTF("LatestAvailabilityNumber:%d\n\r",DBManager.LatestAvailabilityNumber[storeType]);
#endif
	return SUCCESS;
}

uint8_t EnterRecord(enum RecordTypes storeType, void *buff)
{
	FlashAccessInterlock=1;


	SectorTableType *SectorTable;
	//BacklogDataTypes *EntrySectorBuffer;

#ifdef DEBUG_BACKLOG
//	PRINTF("\n\r\n\rEnter RecordType : %d \n\r",storeType);
//	PRINTF("EntryRecordbuffIndex : %d \n\r",DBManager.EntryRecordbuffIndex[storeType]);

#endif


	//Assigned SectorBuffer and SectorTable depend on Storage types Entry of record
	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
						//EntrySectorBuffer =rambuffEMGRecord;
						break;
		case OTARecord: SectorTable=DBManager.OTASectorTable;
						//EntrySectorBuffer =rambuffOTARecord;
						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
						//EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		case CANRecord:	SectorTable=DBManager.CANSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :
						FlashAccessInterlock=0;
						return ERROR;
						break;

	}

	//check all boundary of buffer full, memory overflow, bad sector jump over and other housekeeping and management.
	if(DBManager.EntryRecordbuffIndex[storeType] >= DBManager.NoOfRecordInSector[storeType])
	{
		//WRITE to FLASH REQUIRED
		if(handleRecordSectorWrite(storeType)!=SUCCESS)
			return ERROR;

		SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].ReadRecordIndex=DBManager.NoOfRecordInSector[storeType];
		DBManager.ReadSectorTablePtr[storeType]=DBManager.EntrySectorTablePtr[storeType];//points read pointer

		DBManager.EntryRecordbuffIndex[storeType]=0;
		IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);




	#ifdef DEBUG_BACKLOG
		PRINTF("Write Successfully on Sector \n\r");
		PRINTF("LatestAvailabilityNumber - %d\n\r",DBManager.LatestAvailabilityNumber[storeType]);
		PRINTF("LatestMaxSectorWriteCnt - %d\n\r",DBManager.LatestMaxSectorWriteCnt[storeType]);
		PRINTF("ReadSectorTablePtr - %d\n\r",DBManager.ReadSectorTablePtr[storeType]);

		PRINTF("\r\n");

		debug_DisplaySectorTable(storeType);

	#endif

	}
	//boundary check done without error, then enter record on rambuffer
	switch(storeType)
	{
		case EMGRecord: memcpy(&rambuffEMGRecord[DBManager.EntryRecordbuffIndex[storeType]], buff, DBManager.RecordSizeInBytes[storeType]);
						break;
		case OTARecord: memcpy(&rambuffOTARecord[DBManager.EntryRecordbuffIndex[storeType]], buff, DBManager.RecordSizeInBytes[storeType]);
						break;
		case ALTRecord: memcpy(&rambuffALTRecord[DBManager.EntryRecordbuffIndex[storeType]], buff, DBManager.RecordSizeInBytes[storeType]);
						break;
		case LDRecord:	memcpy(&rambuffLDRecord[DBManager.EntryRecordbuffIndex[storeType]], buff, DBManager.RecordSizeInBytes[storeType]);
						break;
		case CANRecord:	memcpy(&rambuffCANRecord[DBManager.EntryRecordbuffIndex[storeType]], buff, DBManager.RecordSizeInBytes[storeType]);
						break;
		default		 :
						FlashAccessInterlock=0;
						return ERROR;
						break;

	}

	DBManager.StoredRecordCnt[storeType]++;
	DBManager.EntryRecordbuffIndex[storeType]++;

	FlashAccessInterlock=0;

#ifdef DEBUG_BACKLOG
//	PRINTF("Record enter Successfully..StoredRecordCnt:%d\n\r",DBManager.StoredRecordCnt[storeType]);
	PRINTF("StoredRecordCnt:%d\n\r",DBManager.StoredRecordCnt[storeType]);
	PRINTF("EntryRecordbuffIndex:%d\n\r",DBManager.EntryRecordbuffIndex[storeType]);
#endif
	return SUCCESS;
}

uint8_t handleRecordSectorWrite(enum RecordTypes storeType)
{
	//write to flash,first check boundary and management of sector table
	uint16_t Sectindx=0,tempsectorcnt=0;
	SectorTableType *SectorTable;
	//BacklogDataTypes *EntrySectorBuffer;

	//Assigned SectorTable depend on Storage types Entry of record
	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
						//EntrySectorBuffer =rambuffEMGRecord;
						break;
		case OTARecord: SectorTable=DBManager.OTASectorTable;
						//EntrySectorBuffer =rambuffOTARecord;
						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
						//EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		case CANRecord:	SectorTable=DBManager.CANSectorTable;
						//EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :
						FlashAccessInterlock=0;
						return ERROR;
						break;

	}


	if(DBManager.LatestAvailabilityNumber[storeType] == DBManager.SectorSize[storeType] - DBManager.badsectorCnt[storeType])
	{   //At event of one write cycle finish for the this record section of sector
		//decrement availability number, to make most old sector available to write recordbuffer
#ifdef DEBUG_BACKLOG
		PRINTF("Storage OverFlow $$$$$$$$############ Decrement all sector availabiltyNumber\n\r");
#endif
		if(DBManager.LatestAvailabilityNumber[storeType]>0)
			DBManager.LatestAvailabilityNumber[storeType]--; //decrement latest availability
		memoryoverrun=1;

		for(Sectindx=0;Sectindx<DBManager.SectorSize[storeType];Sectindx++)
		{
			if(SectorTable[Sectindx].AvailabilityNumber <= 0)//if it is bad sector skip, or Zero-i.e available
				Sectindx++;
			if(SectorTable[Sectindx].AvailabilityNumber>0)
				SectorTable[Sectindx].AvailabilityNumber--;
			if((SectorTable[Sectindx].AvailabilityNumber==0) && (SectorTable[Sectindx].ReadRecordIndex !=0))
				SectorTable[Sectindx].ReadRecordIndex=0;
		}
	}


	tempsectorcnt=0;
	while(tempsectorcnt < DBManager.SectorSize[storeType])//traverse SectorTable[] by ALTSectorTablePtr
	{
		tempsectorcnt++;

		if(SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber < 0)//if it is bad sector skip
		{
			IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
			tempsectorcnt++;
		}
		if(SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber==0)//found sector to write
		{
			break;
		}
//			if(DBManager.LatestMaxSectorWriteCnt[storeType] > SectorTable[DBManager.EntrySectorTablePtr[storeType]].SectorWriteCnt)
//				break;


		IncrementSectorPtr( &DBManager.EntrySectorTablePtr[storeType], DBManager.SectorStart[storeType], DBManager.SectorSize[storeType]);
	}


#ifdef DEBUG_BACKLOG
	PRINTF("Attempt to Write to Sector @ %d\n\r",DBManager.EntrySectorTablePtr[storeType]);
#endif

	//write the sector at SectorTablePtr
	switch(storeType)
	{
		case EMGRecord: retn= writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], (DBManager.RecordSizeInBytes[storeType]*DBManager.NoOfRecordInSector[storeType]), rambuffEMGRecord);
						break;
		case OTARecord: retn= writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], (DBManager.RecordSizeInBytes[storeType]*DBManager.NoOfRecordInSector[storeType]), rambuffOTARecord);
						break;
		case ALTRecord: retn= writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], (DBManager.RecordSizeInBytes[storeType]*DBManager.NoOfRecordInSector[storeType]), rambuffALTRecord);
						break;
		case LDRecord:	retn= writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], (DBManager.RecordSizeInBytes[storeType]*DBManager.NoOfRecordInSector[storeType]), rambuffLDRecord);
						break;
		case CANRecord:	retn= writeSPIFlashSector(DBManager.EntrySectorTablePtr[storeType], (DBManager.RecordSizeInBytes[storeType]*DBManager.NoOfRecordInSector[storeType]), rambuffCANRecord);
						break;
		default		 :
						return ERROR;
						break;

	}
	if(retn == ERROR)
	{
		SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber = -1; //record as bad sector
		DBManager.badsectorCnt[storeType]++;

		FlashAccessInterlock=0;
#ifdef DEBUG_BACKLOG
		PRINTF("Fail to write Sector.. Record as bad sector - %d\n\r",DBManager.EntrySectorTablePtr[storeType]);
#endif
		return ERROR;

	}

	//write succesfull
	DBManager.LatestAvailabilityNumber[storeType]++;
	SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].AvailabilityNumber=DBManager.LatestAvailabilityNumber[storeType];
	SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].SectorWriteCnt++;


	DBManager.LatestMaxSectorWriteCnt[storeType]=SectorTable[DBManager.EntrySectorTablePtr[storeType]-DBManager.SectorStart[storeType]].SectorWriteCnt;



	//on overflow/memory overrun decrement the StoredRecordCnt
	if(memoryoverrun==1)
		DBManager.StoredRecordCnt[storeType]=DBManager.StoredRecordCnt[storeType]-DBManager.NoOfRecordInSector[storeType];



#ifdef DEBUG_BACKLOG
	PRINTF("Write Successfully on Sector \n\r");
	PRINTF("LatestAvailabilityNumber - %d\n\r",DBManager.LatestAvailabilityNumber[storeType]);
	PRINTF("LatestMaxSectorWriteCnt - %d\n\r",DBManager.LatestMaxSectorWriteCnt[storeType]);
	PRINTF("ReadSectorTablePtr - %d\n\r",DBManager.ReadSectorTablePtr[storeType]);

	PRINTF("\r\n");

	debug_DisplaySectorTable(storeType);

#endif
	return SUCCESS;
}


uint8_t initRecordManager(void)
{
	FlashAccessInterlock=1;

	uint32_t chcksum=0;
	uint8_t* bytebybyte;

	SPIFlashInit();

	//first write default value
	//check first time initialization of Record Manager
	bytebybyte=(uint8_t*)&DBManager;
	retn=SPIFlashRead(RECORDMANAGE_SECTORSTART*FLASHSECTORSIZEBYTES, FLASHSECTORSIZEBYTES,bytebybyte);
	retn=SPIFlashRead((RECORDMANAGE_SECTORSTART+1)*FLASHSECTORSIZEBYTES, sizeof(BacklogManagerTypes)-FLASHSECTORSIZEBYTES,bytebybyte+FLASHSECTORSIZEBYTES);
//	retn=SPIFlashRead(RECORDMANAGE_SECTORSTART*FLASHSECTORSIZEBYTES, FLASHSECTORSIZEBYTES,bytebybyte);
//	retn=SPIFlashRead((RECORDMANAGE_SECTORSTART+1)*FLASHSECTORSIZEBYTES, sizeof(BacklogManagerTypes)-FLASHSECTORSIZEBYTES,bytebybyte+FLASHSECTORSIZEBYTES);

#ifdef DEBUG_BACKLOG
	PRINTF("\n\rBacklogManagerTypes address:%x ",&DBManager);
	PRINTF("\n\rBacklogManagerTypes mtag address:%x,%x",&DBManager.mtag,DBManager.mtag);
	PRINTF("\n\rBacklogManagerTypes checksum address:%x,%d",&DBManager.checksum,DBManager.checksum);

	PRINTF("\n\rConfiguration Read \n\r");
	PRINTF("sector start:%d\n\r",RECORDMANAGE_SECTORSTART);
	PRINTF("Length:%d\n\r",sizeof(BacklogManagerTypes));
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
		DBManager.SectorStart[OTARECORD]=OTARECORD_SECTORSTART;
		DBManager.SectorSize[OTARECORD]=OTARECORD_SECTORSIZE;
		DBManager.SectorStart[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.SectorSize[ALTRECORD]=ALTRECORD_SECTORSIZE;
		DBManager.SectorStart[LDRECORD]=LDRECORD_SECTORSTART;
		DBManager.SectorSize[LDRECORD]=LDRECORD_SECTORSIZE;
		DBManager.SectorStart[CANRECORD]=CANRECORD_SECTORSTART;
		DBManager.SectorSize[CANRECORD]=CANRECORD_SECTORSIZE;
		for(int i=1;i<NOOFRECORDTYPES;i++)
		{
			DBManager.LatestAvailabilityNumber[i]=0;
			DBManager.LatestMaxSectorWriteCnt[i]=0;
			DBManager.StoredRecordCnt[i]=0;

			DBManager.EntryRecordbuffIndex[i]=0;

			DBManager.badsectorCnt[i]=0;
		}

		DBManager.EntrySectorTablePtr[EMGRECORD]=EMGRECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[OTARECORD]=OTARECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[LDRECORD]=LDRECORD_SECTORSTART;
		DBManager.EntrySectorTablePtr[CANRECORD]=CANRECORD_SECTORSTART;

		DBManager.ReadSectorTablePtr[EMGRECORD]=EMGRECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[OTARECORD]=OTARECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[ALTRECORD]=ALTRECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[LDRECORD]=LDRECORD_SECTORSTART;
		DBManager.ReadSectorTablePtr[CANRECORD]=CANRECORD_SECTORSTART;

		DBManager.EntrySectorTablePtr[RECORDMANAGER]=RECORDMANAGE_SECTORSTART;//Not used
		DBManager.ReadSectorTablePtr[RECORDMANAGER]=RECORDMANAGE_SECTORSTART;//Not used

		DBManager.RecordSizeInBytes[EMGRECORD]=EMGRECORDSIZEBYTES;
		DBManager.NoOfRecordInSector[EMGRECORD]=NOOF_EMGRECORDINSECTOR;

		DBManager.RecordSizeInBytes[OTARECORD]= OTARECORDSIZEBYTES;
		DBManager.NoOfRecordInSector[OTARECORD]= NOOF_OTARECORDINSECTOR;

		DBManager.RecordSizeInBytes[ALTRECORD]= ALTRECORDSIZEBYTES;
		DBManager.NoOfRecordInSector[ALTRECORD]= NOOF_ALTRECORDINSECTOR;

		DBManager.RecordSizeInBytes[LDRECORD]= LDRECORDSIZEBYTES;
		DBManager.NoOfRecordInSector[LDRECORD]= NOOF_LDRECORDINSECTOR;

		DBManager.RecordSizeInBytes[CANRECORD]= CANRECORDSIZEBYTES;
		DBManager.NoOfRecordInSector[CANRECORD]= NOOF_CANRECORDINSECTOR;

		DBManager.RecordSizeInBytes[RECORDMANAGER]=sizeof(BacklogManagerTypes);
		DBManager.NoOfRecordInSector[RECORDMANAGER]=1;

		//for(int i=0;i<(EMGRECORD_SECTORSIZE*sizeof(SectorTableType));i++)
		//	*(((uint8_t*)DBManager.EMGSectorTable) + i)=0;
		for(int i=0;i<(EMGRECORD_SECTORSIZE);i++)
		{	DBManager.EMGSectorTable[i].AvailabilityNumber=0;
			DBManager.EMGSectorTable[i].ReadRecordIndex=0;
			DBManager.EMGSectorTable[i].SectorWriteCnt=0;
		}
		for(int i=0;i<(OTARECORD_SECTORSIZE);i++)
		{	DBManager.OTASectorTable[i].AvailabilityNumber=0;
			DBManager.OTASectorTable[i].ReadRecordIndex=0;
			DBManager.OTASectorTable[i].SectorWriteCnt=0;
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
		for(int i=0;i<(CANRECORD_SECTORSIZE);i++)
		{	DBManager.CANSectorTable[i].AvailabilityNumber=0;
			DBManager.CANSectorTable[i].ReadRecordIndex=0;
			DBManager.CANSectorTable[i].SectorWriteCnt=0;
		}

		//write to Record management section of SPI flash
		SaveRecordManager();


	}
	else
	{
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
			PRINTF("Received checksum  : %u \n\r", DBManager.checksum );
			PRINTF("calculated checksum : %u \n\r", chcksum );
			#endif
			FlashAccessInterlock=0;
			return ERROR;

		}
		else
		{
			#ifdef DEBUG_BACKLOG
					PRINTF("\n\r*****Normal Startup reading previous record entry \n\r");
			#endif
			for(int i=1;i<NOOFRECORDTYPES;i++)
			{

				if(DBManager.EntryRecordbuffIndex[i]!=0)
				{

					#ifdef DEBUG_BACKLOG
							PRINTF("\n\r*****Detect previous Half store for Record type %d of recordNo %d\n\r",i,DBManager.EntryRecordbuffIndex[i]);
					#endif

					DBManager.LatestAvailabilityNumber[i]--;


					//retreive the Half sector write to ram buffer
					switch(i)
					{
						case EMGRecord:
							retn=SPIFlashRead(DBManager.EntrySectorTablePtr[i]*FLASHSECTORSIZEBYTES, ((DBManager.RecordSizeInBytes[i])*DBManager.EntryRecordbuffIndex[i]),rambuffEMGRecord);
							DBManager.EMGSectorTable[DBManager.EntrySectorTablePtr[i]-DBManager.SectorStart[i]].AvailabilityNumber=0;
							break;
						case OTARecord:
							retn=SPIFlashRead(DBManager.EntrySectorTablePtr[i]*FLASHSECTORSIZEBYTES, ((DBManager.RecordSizeInBytes[i])*DBManager.EntryRecordbuffIndex[i]),rambuffOTARecord);
							DBManager.OTASectorTable[DBManager.EntrySectorTablePtr[i]-DBManager.SectorStart[i]].AvailabilityNumber=0;
							break;
						case ALTRecord:
							retn=SPIFlashRead(DBManager.EntrySectorTablePtr[i]*FLASHSECTORSIZEBYTES, ((DBManager.RecordSizeInBytes[i])*DBManager.EntryRecordbuffIndex[i]),rambuffALTRecord);
							DBManager.ALTSectorTable[DBManager.EntrySectorTablePtr[i]-DBManager.SectorStart[i]].AvailabilityNumber=0;
							break;
						case LDRecord:
							retn=SPIFlashRead(DBManager.EntrySectorTablePtr[i]*FLASHSECTORSIZEBYTES, ((DBManager.RecordSizeInBytes[i])*DBManager.EntryRecordbuffIndex[i]),rambuffLDRecord);
							DBManager.LDSectorTable[DBManager.EntrySectorTablePtr[i]-DBManager.SectorStart[i]].AvailabilityNumber=0;
							break;
						case CANRecord:
							retn=SPIFlashRead(DBManager.EntrySectorTablePtr[i]*FLASHSECTORSIZEBYTES, ((DBManager.RecordSizeInBytes[i])*DBManager.EntryRecordbuffIndex[i]),rambuffCANRecord);
							DBManager.CANSectorTable[DBManager.EntrySectorTablePtr[i]-DBManager.SectorStart[i]].AvailabilityNumber=0;
							break;
						default		 :  FlashAccessInterlock=0;
										return ERROR;
										break;
					}
				}

			}

		}



	}

	FlashAccessInterlock=0;
	return SUCCESS;

}

uint8_t SaveRecordManager(void)
{
	uint32_t chcksum1=0,sizeOfmanager1;
	uint8_t* bytebybyte1;



	sizeOfmanager1=sizeof(BacklogManagerTypes);
	bytebybyte1=(uint8_t*)&DBManager;
	chcksum1=0;
	for(int i=8; i < sizeof(BacklogManagerTypes);i++ )//calculate checksum
	{
		chcksum1 = chcksum1 + bytebybyte1[i];
	}
	DBManager.checksum=chcksum1;//store the new checksum
#ifdef DEBUG_BACKLOG
	PRINTF("\n\r\n\r********SAVING RECORD MANAGER*********** \n\r");
	PRINTF("\n\rSize of Manager Record : %d \n\r",sizeof(BacklogManagerTypes));
	PRINTF("\n\rCalculated checksum : %u \n\r", chcksum1 );
	PRINTF("\n\rCalculated sector to write : %u \n\r",DBManager.SectorStart[RECORDMANAGER]);
#endif
	retn=0;
	bytebybyte1=(uint8_t*)&DBManager;
	if(sizeOfmanager1 <= FLASHSECTORSIZEBYTES)
	{
		retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER], sizeOfmanager1,bytebybyte1);
	}
	else if(sizeOfmanager1 <= FLASHSECTORSIZEBYTES*2)//it support manager data size more than 1 sector or >4096 bytes
	{
		retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER],FLASHSECTORSIZEBYTES,bytebybyte1);
		//retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),sizeOfmanager-FLASHSECTORSIZEBYTES,bytebybyte+FLASHSECTORSIZEBYTES);
		retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),FLASHSECTORSIZEBYTES,bytebybyte1+FLASHSECTORSIZEBYTES);
	}
	else ////it support manager data size more than 2 sector
	{
		retn=writeSPIFlashSector(DBManager.SectorStart[RECORDMANAGER],FLASHSECTORSIZEBYTES,bytebybyte1);
		retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+1),FLASHSECTORSIZEBYTES, bytebybyte1+FLASHSECTORSIZEBYTES);
		retn|=writeSPIFlashSector((DBManager.SectorStart[RECORDMANAGER]+2),sizeOfmanager1 - FLASHSECTORSIZEBYTES, bytebybyte1+(FLASHSECTORSIZEBYTES*2));
	}

		//check first time initialization of Record Manager
		bytebybyte1=(uint8_t*)&DBManager;
		retn=SPIFlashRead(DBManager.SectorStart[RECORDMANAGER]*FLASHSECTORSIZEBYTES, FLASHSECTORSIZEBYTES,bytebybyte1);
		retn|=SPIFlashRead((DBManager.SectorStart[RECORDMANAGER]+1)*FLASHSECTORSIZEBYTES, sizeof(BacklogManagerTypes)-FLASHSECTORSIZEBYTES,bytebybyte1+FLASHSECTORSIZEBYTES);
	//	retn=SPIFlashRead(RECORDMANAGE_SECTORSTART*FLASHSECTORSIZEBYTES, FLASHSECTORSIZEBYTES,bytebybyte);
	//	retn=SPIFlashRead((RECORDMANAGE_SECTORSTART+1)*FLASHSECTORSIZEBYTES, sizeof(BacklogManagerTypes)-FLASHSECTORSIZEBYTES,bytebybyte+FLASHSECTORSIZEBYTES);

	#ifdef DEBUG_BACKLOG
		PRINTF("\n\rBacklogManagerTypes address:%x",&DBManager);
		PRINTF("\n\rBacklogManagerTypes mtag address:%x,%x",&DBManager.mtag,DBManager.mtag);
		PRINTF("\n\rBacklogManagerTypes checksum address:%x,%d",&DBManager.checksum,DBManager.checksum);

		PRINTF("\n\rConfiguration Read \n\r");
		PRINTF("sector start:%d\n\r",RECORDMANAGE_SECTORSTART);
		PRINTF("Length:%d\n\r",sizeof(BacklogManagerTypes));

	#endif

	if(retn != SUCCESS)
	{	//unexceptional error
#ifdef DEBUG_BACKLOG
		PRINTF("\n\r\n\r********FLASH MEMORY ISSUE(or Corrupted ON RECORD MANAGER*********** \n\r");
#endif
		_Error_Handler(__FILE__, __LINE__);

	}
	return SUCCESS;
}
uint8_t SaveAllRecord(void)
{

	for(int i=1;i<NOOFRECORDTYPES;i++)
	{

		if(DBManager.EntryRecordbuffIndex[i]!=0)
		{

			if(handleRecordSectorWrite(i)!=SUCCESS)//write record
				return ERROR;

		}

	}
	//save current record management data also
	if(SaveRecordManager()!=SUCCESS)//write record manager
		return ERROR;

	return SUCCESS;
}

uint8_t ClearTagRecord(void)
{
	DBManager.mtag=0;
	DBManager.checksum=0;
	return SaveRecordManager();
}


//#################### Utility function SECTION


uint8_t writeSPIFlashSector(uint16_t sectorNum, uint16_t lenByte, void* buff)
{

	uint8_t retrycnt=FLASHWRITE_RETRY; //Handle multiple retry of Sector Write
	uint16_t retn=0;
	uint32_t addrs= sectorNum*FLASHSECTORSIZEBYTES;
	//uint8_t byteacessBuff[4096];

#ifdef SIMULATE_BADSECTOR
	if(sectorNum==BADSECTOR1 || sectorNum==BADSECTOR2)
		return ERROR;
#endif


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
		retn|=SPIFlashWrite(addrs,lenByte,(uint8_t *)buff);
		//to stimulate badsector change back buff
		//HAL_Delay(5);
		retn|=SPIFlashRead(addrs,lenByte,tempSectorBuff);
//		PRINTF("\n\r");
//		for(int i=0;i<4;i++)
//			PRINTF("%x,",tempSectorBuff[i]);
//		PRINTF("\n\r");

		if(retn==0 && memcmp((uint8_t *)buff,tempSectorBuff,lenByte)==0)
		{
#ifdef DEBUG_BACKLOG
			PRINTF("Success Write @%d, len=%d\n\r",sectorNum,lenByte);
#endif

			return SUCCESS;
		}
		retrycnt--;//decrement for next try if not success
	}

#ifdef DEBUG_BACKLOG
	PRINTF("Sector Write ERROR\n\r");
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

#ifdef DEBUG_BACKLOG
	PRINTF("\n\r\n\r*********** SPI Flash probe done SUCCESS ******\n\r");
	PRINTF("Make \t: %s\n\r",spiflashHandler->name);
	PRINTF("Size \t: %u\n\r",spiflashHandler->size);
	PRINTF("SectorSize\t: %d\n\r",spiflashHandler->sector_size);
	PRINTF("TotalSector\t: %d\n\r",((spiflashHandler->size)/(spiflashHandler->sector_size)));

#endif
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

//utility function
void debug_DisplaySectorTable(enum RecordTypes storeType)
{
	SectorTableType *SectorTable;
//	BacklogDataTypes *EntrySectorBuffer;

	switch(storeType)
	{
		case EMGRecord: SectorTable=DBManager.EMGSectorTable;
//						EntrySectorBuffer =rambuffEMGRecord;

						break;
		case OTARecord: SectorTable=DBManager.OTASectorTable;
//						EntrySectorBuffer =rambuffOTARecord;

						break;
		case ALTRecord: SectorTable=DBManager.ALTSectorTable;
//						EntrySectorBuffer =rambuffALTRecord;
						break;
		case LDRecord:	SectorTable=DBManager.LDSectorTable;
//						EntrySectorBuffer =rambuffLDRecord;
						break;
		case CANRecord:	SectorTable=DBManager.CANSectorTable;
//						EntrySectorBuffer =rambuffLDRecord;
						break;
		default		 :  return;
						break;
	}
#ifdef DEBUG_BACKLOG
	PRINTF("\n\rTable for Storage type : %d\n\r",storeType);
	PRINTF("************************************************************\n\r");
	PRINTF("Index \tSectorNo \tAvailability \tReadIndex \tWriteCnt\n\r");
	for(int i=0;i<DBManager.SectorSize[storeType];i++)
	{
		PRINTF("   %d\t",i);//index
		PRINTF("%d\t\t",i+DBManager.SectorStart[storeType]);//sectorNo
		PRINTF("%d\t\t",SectorTable[i].AvailabilityNumber);//availability
		PRINTF("%d\t\t",SectorTable[i].ReadRecordIndex);//read
		PRINTF("%d\t\t",SectorTable[i].SectorWriteCnt);//sector writecnt
		PRINTF("\n\r");
	}
	PRINTF("\n\r");
	PRINTF("LatestAvailabilityNumber \t: %d \n\r",DBManager.LatestAvailabilityNumber[storeType]);
	PRINTF("LatestMaxSectorWriteCnt  \t: %d \n\r",DBManager.LatestMaxSectorWriteCnt[storeType]);
	PRINTF("StoredRecordCnt			   : %d records\n\r",DBManager.StoredRecordCnt[storeType]);
	PRINTF("No of Record onRAMBuffer \t: %d records\n\r",DBManager.EntryRecordbuffIndex[storeType]);
	PRINTF("Total Record			   : %d records\n\r",getTotalRecordSpace(storeType));
	PRINTF("RecordSize in bytes		   : %d bytes\n\r",DBManager.RecordSizeInBytes[storeType]);
	PRINTF("************************************************************\n\r\n\r");
#endif
}

uint16_t getTotalRecordSpace(enum RecordTypes storeType)
{
	return DBManager.NoOfRecordInSector[storeType]*((DBManager.SectorSize[storeType]-DBManager.badsectorCnt[storeType])+1);
}

uint16_t getStoredNoOfRecord(enum RecordTypes storeType)
{
	return DBManager.StoredRecordCnt[storeType];
}

uint16_t getRemainingRecordSpace(enum RecordTypes storeType)
{
	return getTotalRecordSpace(storeType)-getStoredNoOfRecord(storeType);
}
uint16_t getRecordSize(enum RecordTypes storeType)
{
	return DBManager.RecordSizeInBytes[storeType];
}
uint16_t getNoOfRecordInSector(enum RecordTypes storeType)
{
	return DBManager.NoOfRecordInSector[storeType];
}



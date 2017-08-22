/*
 * backlog.h
 *
 *  Created on: Jul 19, 2017
 *      Author: paull
 */

#ifndef BACKLOG_H_
#define BACKLOG_H_
#include <spi_flash.h>
#include <stdbool.h>
#include <backlog_if.h>

//#define NO_OF_SOCKET		6 //bytes = 4096*6 array of sector chunk-wise record copy

#define DEBUG_BACKLOG

#define FLASHWRITE_RETRY			5


#define FLASHTOTALNOOFSECTOR				1024 //sectors
#define FLASHSECTORSIZEBYTES				4096 //bytes
#define FLASHPAGESIZEBYTES					256
#define FLASHNOOFPAGEINSECTOR				FLASHSECTORSIZEBYTES//


#define LDRECORDSIZEBYTES					sizeof(LD_BacklogDataTypes)
#define EMGRECORDSIZEBYTES					sizeof(EMG_BacklogDataTypes)
#define OTARECORDSIZEBYTES					sizeof(OTA_BacklogDataTypes)
#define ALTRECORDSIZEBYTES					sizeof(ALT_BacklogDataTypes)
#define CANRECORDSIZEBYTES					sizeof(CAN_BacklogDataTypes)

#define NOOF_LDRECORDINSECTOR				(FLASHSECTORSIZEBYTES/LDRECORDSIZEBYTES)
#define NOOF_EMGRECORDINSECTOR				(FLASHSECTORSIZEBYTES/EMGRECORDSIZEBYTES)
#define NOOF_OTARECORDINSECTOR				(FLASHSECTORSIZEBYTES/OTARECORDSIZEBYTES)
#define NOOF_ALTRECORDINSECTOR				(FLASHSECTORSIZEBYTES/ALTRECORDSIZEBYTES)
#define NOOF_CANRECORDINSECTOR				(FLASHSECTORSIZEBYTES/CANRECORDSIZEBYTES)

#define RECORDMANAGE_SECTORSTART			124 //sector
#define RECORDMANAGE_SECTORSIZE				4  //in sector count
#define EMGRECORD_SECTORSTART				RECORDMANAGE_SECTORSTART+RECORDMANAGE_SECTORSIZE //128=124+4 sector
#define EMGRECORD_SECTORSIZE				2
#define OTARECORD_SECTORSTART				EMGRECORD_SECTORSTART+EMGRECORD_SECTORSIZE //130=128+2 sector
#define OTARECORD_SECTORSIZE				2
#define ALTRECORD_SECTORSTART				OTARECORD_SECTORSTART+OTARECORD_SECTORSIZE //132=130+2 sector
#define ALTRECORD_SECTORSIZE				10
#define LDRECORD_SECTORSTART				ALTRECORD_SECTORSTART+ALTRECORD_SECTORSIZE //142=132+10 sector
#define LDRECORD_SECTORSIZE					880
#define CANRECORD_SECTORSTART				(LDRECORD_SECTORSTART+LDRECORD_SECTORSIZE) //1021=142+880 sector
#define CANRECORD_SECTORSIZE				FLASHTOTALNOOFSECTOR-CANRECORD_SECTORSTART // 1023-1021 = 2 sectors


//just for SIMULATION OF BAD SECTOR
//#define SIMULATE_BADSECTOR
//#ifdef SIMULATE_BADSECTOR
//	#define BADSECTOR1 	ALTRECORD_SECTORSTART+3
//	#define BADSECTOR2 	ALTRECORD_SECTORSTART+5
//#endif



typedef struct
{   //structure for one sector
	int16_t AvailabilityNumber; //1- NoOfSectorinSection, 0 is data Not Available, negative value represent bad sector
	uint8_t ReadRecordIndex;
	uint16_t SectorWriteCnt; //track write count of each index sector
}__attribute__ ((packed))SectorTableType;

typedef struct
{
	uint32_t mtag;//0xA5A5A5A5 to flag SPI Flash is formatted for Backlog storage, set once and write during format operation
	uint32_t checksum; //storage checksum of this meta/house keeping data structure

	//SPI Flash information
	uint16_t FlashSectorSize;//in bytes
	uint16_t FlashNoOfSector;// in no of sector
	uint16_t FlashPageSize; //in bytes 256bytes
	uint16_t FlashNoOfPageinSector; //4096/256=16 pages

	//SPI Flash section information
	uint16_t SectorStart[NOOFRECORDTYPES];
	uint16_t SectorSize[NOOFRECORDTYPES];


	//Database information for each Record Types section partition
	uint16_t RecordSizeInBytes[NOOFRECORDTYPES];
	uint16_t NoOfRecordInSector[NOOFRECORDTYPES];


	uint16_t EntryRecordbuffIndex[NOOFRECORDTYPES];
	uint32_t StoredRecordCnt[NOOFRECORDTYPES];
	uint16_t EntrySectorTablePtr[NOOFRECORDTYPES];
	uint16_t ReadSectorTablePtr[NOOFRECORDTYPES];
	uint16_t LatestAvailabilityNumber[NOOFRECORDTYPES]; //range 1- NoOfSectorinSection, 0 is data Not Available
	uint32_t LatestMaxSectorWriteCnt[NOOFRECORDTYPES]; //track write count of each index sector
	uint16_t badsectorCnt[NOOFRECORDTYPES];

	//Table of Sector for each section of Record types
	SectorTableType EMGSectorTable[EMGRECORD_SECTORSIZE];
	SectorTableType OTASectorTable[OTARECORD_SECTORSIZE];
	SectorTableType ALTSectorTable[ALTRECORD_SECTORSIZE];
	SectorTableType LDSectorTable[LDRECORD_SECTORSIZE];
	SectorTableType CANSectorTable[CANRECORD_SECTORSIZE];


}__attribute__ ((packed))BacklogManagerTypes;



#endif /* BACKLOG_H_ */

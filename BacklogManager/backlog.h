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


#define RECORDSIZEBYTES						91		//bytes or sizeof(BacklogDataTypes)
#define LDRECORDSIZEBYTES					RECORDSIZEBYTES
#define EMGRECORDSIZEBYTES					LDRECORDSIZEBYTES
#define ALTRECORDSIZEBYTES					64//LDRECORDSIZEBYTES
#define NOOF_RECORDINSECTOR					(FLASHSECTORSIZEBYTES/LDRECORDSIZEBYTES)
#define NOOF_LDRECORDINSECTOR				NOOF_RECORDINSECTOR
#define NOOF_EMGRECORDINSECTOR				NOOF_RECORDINSECTOR
#define NOOF_ALTRECORDINSECTOR				(FLASHSECTORSIZEBYTES/ALTRECORDSIZEBYTES)//NOOF_RECORDINSECTOR


#define RECORDMANAGE_SECTORSTART			124 //sector
#define RECORDMANAGE_SECTORSIZE				4  //in sector count
#define EMGRECORD_SECTORSTART				RECORDMANAGE_SECTORSTART+RECORDMANAGE_SECTORSIZE //128=124+4 sector
#define EMGRECORD_SECTORSIZE				2
#define ALTRECORD_SECTORSTART				EMGRECORD_SECTORSTART+EMGRECORD_SECTORSIZE //130=128+2 sector
#define ALTRECORD_SECTORSIZE				10
#define LDRECORD_SECTORSTART				ALTRECORD_SECTORSTART+ALTRECORD_SECTORSIZE //140=130+10 sector
#define LDRECORD_SECTORSIZE					FLASHTOTALNOOFSECTOR-LDRECORD_SECTORSTART // 1024-140 = 844 sectors



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
}SectorTableType;

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
	SectorTableType ALTSectorTable[ALTRECORD_SECTORSIZE];
	SectorTableType LDSectorTable[LDRECORD_SECTORSIZE];

}BacklogManagerTypes;



#endif /* BACKLOG_H_ */

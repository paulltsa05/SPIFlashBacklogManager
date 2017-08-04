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

#define NO_OF_SOCKET		6 //bytes = 4096*6 array of sector chunk-wise record copy

#define FSECTORSIZE					4096
#define FTOTALSECTOR				1024
#define FLASHWRITE_RETRY			5

#define DATAFLASH_MAXBADSECTOR		200 // quarter(200/889) bad sector will stop device operation for backlog storage
#define TXTAGFLASH_MAXBADSECTOR		7   // 7 bad sector on TxTag section of Flash will stop device operation for backlog storage

#define FLASH_MAXBADDATASECTORBUFF	256
#define FLASH_MAXBADTXTAGSECTORBUFF	16

#define RECORDMANAGE_SECTOR1		62
#define RECORDMANAGE_SECTOR2		63 		//MANAGE: 2 sectors store management data twice

#define TXTAGRECORDSTART_SECTOR		64
#define TXTAGRECORDEND_SECTOR		134		//TXTAG:  71 sectors, extra 7 sector included to map bad sector limit

#define DATARECORDSTART_SECTOR		135
#define DATARECORDEND_SECTOR		1023	//DATA :  889 sectors, can accomodate 45 (91byteDatarecord) in a sector = 40005 data record

#define DATARECORDSIZE				91 //one record size in bytes
#define DATARECORDINSECTOR			45 //  4096/91=45

#define TXTAGRECORDSIZE				NO_OF_SOCKET //one record size in bytes
#define TXTAGRECORDINSECTOR			682 //4096/(MAX NO_OF_SOCKET=6 SUPPORTED)

#define TOTALRECORD					40000

#define NULL0			0
//return types
#define SUCCESS			0
#define FOUND			0
#define EMPTY			1
#define NOTFOUND		2

#define ERROR			255



#define SOCKET1				0
#define SOCKET2				1
#define SOCKET3				2
#define SOCKET4				3
#define SOCKET5				4
#define SOCKET6				5

typedef union //5
{
		uint8_t Pkt_TxStatus;
		struct //5
		{
			bool Pvt:1;
			bool Epb:1;
			bool Hmp:1;
			bool Ota:1;
			bool Alt:1;
			bool reserved1:1;
			bool reserved2:1;
			bool reserved3:1;
		}pktType;
}PackageType;

//struct PackageType//5bit
//{
//	bool Pvt;
//	bool Epb;
//	bool Hmp;
//	bool Ota;
//	bool Alt;
//}__attribute__((packed));


typedef struct //6  ,, 8x6 =48 (6 bytes)
{
	 PackageType Socket[NO_OF_SOCKET];

}SocketType;// __attribute__((packed));

typedef union
{
	uint8_t AllFlag[NO_OF_SOCKET];	//
	SocketType FlagField;			//
}TxTagTypes;


//Storage Backlog record types
typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint8_t GPSFix;				//	1
	uint32_t Latitude;			//	4
	uint32_t Longitude;			//	4
	uint32_t GPSDate;			//	4
	uint32_t GPSTime;			//	4
	uint16_t Speed;				//	2
	uint16_t Heading;			//	2
	uint32_t Altitude;			//	4
	uint8_t NoOfSatellites;		//No. of Satellites;//	1
	uint16_t HorDilPrecision;	//Horizontal dilution of precision;//	2
	uint16_t PosDilPrecision;	//Positional dilution of precision;//	2
	uint16_t MainVoltage_raw;	//Main Input Voltage;//	2
	uint16_t IntBatteryVoltage;	//Internal Battery Voltage;//	2
	uint8_t gsmSignalStrength;	//GSM Signal Strength;//	1
	uint16_t MCC;				//	2
	uint16_t MNC;				//	2
	uint16_t LAC;				//	2
	uint16_t CellID;			//	2
	uint32_t NMR[4];			//	24
	uint16_t DigitalStatus1;	//IGN/PWR/EMG/TA/BD; //2
	uint8_t DigitalInput;		//Digital I/p 4	1
	uint8_t DigitalOutput; 		//Digital o/p 2
	uint16_t PrevDistanceEMR;	//Distance calculated from prev. GPS	2
	uint16_t AnalogIN1;			//Analog I/p 2	4
	uint16_t AnalogIN2;
	uint8_t BatteryPercent;		//	1
	uint8_t MemoryPercentage;	//	1
	uint16_t Checksum;			//	2


}BacklogDataTypes;


typedef struct // To store multiple copy of sector in RAM to handle multiple socket read write operation
{
	TxTagTypes TxTagBuff[TXTAGRECORDINSECTOR];

}TxTagSectorBuffTypes;

typedef struct // To store multiple copy of sector in RAM to handle multiple socket read write operation
{
	BacklogDataTypes DataBuff[DATARECORDINSECTOR];

}DataSectorBuffTypes;

typedef struct
{
	uint16_t DataSectorPtr; //denote sector no of SPI Flash
	uint8_t DataRecordIndex; // current record index in the buffer

	uint16_t TxTagSectorPtr;
	uint16_t OldTxTagSectorPtr;//record previous TxTag to quickly jump back incase of multiple intermediate record generated
	uint8_t TxTagRecordIndex;
	uint8_t InterRecordGenerateCnt;//track no of data generated between record read, before whole read of backlog
	uint8_t InterRecordSendRecIndx;//track Send recordindex pointer use by tx complete to know which flag to clear
	uint8_t InterRecordSendFlg; //used by tx complete to reset the flag
	uint8_t TxTagBufferIndex; //ramstack index follows: 0-WRecBuff,1-TxRWBuff1,2-TxRWBuff2, 3-TxRWBuff3, 4-TxRWBuff4,5-TxRWBuff5,6-TxRWBuff6
	TxTagTypes* TxTagbuffer; //dynamically point to ram record buffer depend on

}VirtualAddressType;

typedef struct
{
	uint32_t mtag;//0xA5A5A5A5 to flag SPI Flash is formatted for Backlog storage, set once and write during format operation
	uint32_t checksum; //storage checksum of this meta/houskeeping data structure
	uint32_t SectorSizeBytes;
	uint32_t TotalSectorInFlash;
	uint16_t DataRecordSizeBytes;
	uint16_t TxTagRecordSizeBytes;

	uint16_t DataEntrystackIndex; //always point or index on ramstack_WRecBuff[] not required virtual address
	uint16_t TxTagEntrystackIndex;

	uint16_t DataSectorHeadPtr;
	uint16_t DataSectorTailPtr;
	uint16_t TxTagSectorHeadPtr;
	uint16_t TxTagSectorTailPtr;

	uint16_t DataSectorStart ;
	uint16_t DataSectorLast ;
	uint16_t TxTagSectorStart ;
	uint16_t TxTagSectorLast ;

	uint16_t DataSectorUsed;   // track sector use in Data Section increment n srite only
	uint16_t TxTagSectorUsed;  // track sector use in TxTag Section

	uint16_t NoOfDataRecordinSector;
	uint16_t NoOfTxTagRecordinSector;
	uint32_t MaxNoRecord;
	uint32_t RecordNumber;
	uint16_t RecordFull;
	uint32_t RecordWriteCnt;//track number of record ENTERED in a record section independent of socket
	//uint8_t	recordEntryCycleCnt[NO_OF_SOCKET]; //count no of generated data for the socket in-between record read
	uint32_t RecordRemaining[NO_OF_SOCKET]; //No of current record pending to be transmitted on each socket, per record theie can be multiple package made
	VirtualAddressType Socket_RWptr[NO_OF_SOCKET]; //pointer for socket1 read
	uint8_t TxTagBuffCnt;//it keeps track of how many buffer is used by multiple Socket_RWPtr
	uint16_t DataBadsectorCnt; // track No of Badsector within a fixed Section of sectors
	uint16_t DataBadsectorbuff[FLASH_MAXBADDATASECTORBUFF]; //sector Number
	uint16_t TxTagBadsectorCnt; // track No of Badsector within a fixed Section of sectors
	uint16_t TxTagBadsectorbuff[FLASH_MAXBADTXTAGSECTORBUFF]; //sector Number
}BacklogHousekeepingTypes;



//************FUNCTION PROTOTYPE********************//

// NOTE: below function to be called while porting this backlog storage management code
// FUNCTION TO BE CALL ON TX SUCCESS ON PARTICULAR SOCKET
extern void txsuccessCallback_Socket1(void);
extern void txsuccessCallback_Socket2(void);
extern void txsuccessCallback_Socket3(void);
extern void txsuccessCallback_Socket4(void);
extern void txsuccessCallback_Socket5(void);
extern void txsuccessCallback_Socket6(void);



#endif /* BACKLOG_H_ */

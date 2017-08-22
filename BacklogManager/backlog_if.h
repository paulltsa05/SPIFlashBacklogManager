/*
 * backlog_if.h
 *
 *  Created on: Aug 9, 2017
 *      Author: paull
 */


#ifndef BACKLOG_IF_H_
#define BACKLOG_IF_H_


//return types
#define SUCCESS			0
#define FOUND			0
#define EMPTY			1
#define NOTFOUND		2

#define ERROR			255


//storeTypes
#define NOOFRECORDTYPES		6
//storeTypes
#define RECORDMANAGER		0
#define EMGRECORD			1
#define OTARECORD			2
#define ALTRECORD			3
#define LDRECORD			4
#define CANRECORD			5

enum RecordTypes{RecordMANAGER,EMGRecord,OTARecord,ALTRecord,LDRecord,CANRecord};

//Storage Backlog record types for each RecordType except for RecordMANAGER
typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint32_t recordNum;         // 1 to MaxRecordNo
	uint8_t alerttype;
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
	uint16_t Checksum;			//	2  84
	uint16_t reserve1;
	uint16_t reserve2;
	uint16_t reserve3;
	uint16_t reserve4;
	uint16_t reserve5;
	uint16_t reserve6;
	uint8_t reserve7;


}__attribute__ ((packed))LD_BacklogDataTypes;

typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint32_t recordNum;         // 1 to MaxRecordNo
	uint8_t alerttype;
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
	uint16_t Checksum;			//	2  84
	uint16_t reserve1;
	uint16_t reserve2;
	uint16_t reserve3;
	uint16_t reserve4;
	uint16_t reserve5;
	uint16_t reserve6;
	uint8_t reserve7;


}__attribute__ ((packed))ALT_BacklogDataTypes;

typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint32_t recordNum;         // 1 to MaxRecordNo
	uint8_t alerttype;
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
	uint16_t Checksum;			//	2  84
	uint16_t reserve1;
	uint16_t reserve2;
	uint16_t reserve3;
	uint16_t reserve4;
	uint16_t reserve5;
	uint16_t reserve6;
	uint8_t reserve7;


}__attribute__ ((packed))EMG_BacklogDataTypes;

typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint32_t recordNum;         // 1 to MaxRecordNo
	uint8_t alerttype;
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
	uint16_t Checksum;			//	2  84


}__attribute__ ((packed))OTA_BacklogDataTypes;

typedef struct {
	//TxTagTypes TxTag; 			//  6 bytes
	uint32_t recordNum;         // 1 to MaxRecordNo
	uint8_t alerttype;
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
	uint16_t Checksum;			//	2  84
	uint16_t reserve1;
	uint16_t reserve2;
	uint16_t reserve3;
	uint16_t reserve4;
	uint16_t reserve5;
	uint16_t reserve6;
	uint8_t reserve7;


}__attribute__ ((packed))CAN_BacklogDataTypes;
//external interface function

extern uint8_t initRecordManager(void);

extern uint8_t EnterRecord(enum RecordTypes storeType, void *buff);

extern uint8_t ReadRecord(enum RecordTypes storeType, void *buff);

extern uint8_t ClearTagRecord(void);

extern uint8_t SaveAllRecord(void); //to be call before low power

//utility function
extern uint16_t getTotalRecordSpace(enum RecordTypes storeType);
extern uint16_t getStoredNoOfRecord(enum RecordTypes storeType);
extern uint16_t getRemainingRecordSpace(enum RecordTypes storeType);



extern void debug_DisplaySectorTable(enum RecordTypes storeType);

#endif /* BACKLOG_IF_H_ */

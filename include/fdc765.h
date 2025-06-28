#ifndef FDC765_H__
#define FDC765_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef U765_EXPORTS
#if defined(_WIN32) || defined(__CYGWIN__)
#define U765_EXPORT extern __declspec(dllexport)
#elif defined(__GNUC__) || defined(__clang__)
#define U765_EXPORT __attribute__((visibility("default")))
#else
#define U765_EXPORT
#endif
#else
#define U765_EXPORT
#endif

#define U765_FUNCTION(n) __stdcall n

typedef struct {
    uint8_t  DiskInfoBlock[34]; // BYTE 34 dup(?)
    uint8_t  pad1[14];          // BYTE 14 dup(?)
    uint8_t  NumTracks;         // BYTE ?
    uint8_t  NumSides;          // BYTE ?
    uint16_t TrackSize;         // WORD ?
    uint8_t  pad2[204];         // BYTE 204 dup(?)
}
u765_DiskInfoBlock;

typedef struct {
    uint8_t TrackData[13];       // BYTE 13 dup(?)          ; start of TrackInfoBlock = "Track-Info\r\n"
    uint8_t pad1[3];             // BYTE 3  dup(?)          ; unused
    uint8_t TrackNum;            // BYTE ?
    uint8_t SideNum;             // BYTE ?
    uint8_t pad2[2];             // WORD ?                  ; unused
    uint8_t SectorSize;          // BYTE ?                  ; size of each sector in this track
    uint8_t NumSectors;          // BYTE ?                  ; number of sectors in this track
    uint8_t GapLength;           // BYTE ?                  ; gap length
    uint8_t FillerByte;          // BYTE ?
    uint8_t SectorInfoList[232]; // BYTE 232   dup(?)       ; start of the SectorInfoList area (8 bytes/sector)
    uint8_t SectorData[32768];   // BYTE 32768 dup(?)       ; start of SectorData for this track
}
u765_TrackInfoBlock;

typedef struct {
    void*   DiskArrayPtr;      // DWORD ?         ; pointer to allocated memory   
    size_t  DiskArrayLen;      // DWORD ?         ; sizeof allocated memory
    bool    DiskInserted;      // BYTE  ?         ; TRUE when disk is inserted in this drive
    bool    ContentsChanged;   // BYTE  ?         ; TRUE when this disk has been written to
    bool    WriteProtect;      // BYTE  ?         ; TRUE if disk is write protected
    bool    EDSK;              // BYTE  ?         ; TRUE if this unit has an EDSK file; FALSE for DSK
    bool    DriveStateChanged; // BYTE  ?         ; TRUE if this drive's state has changed
    uint8_t CTK;               // BYTE  ?         ; current physical track the head is over
    uint8_t CHEAD;             // BYTE  ?         ; current head in operation for this command
    uint8_t CSR;               // BYTE  ?         ; current sector the head is over
    bool    SeekDone;          // BYTE  ?         ; TRUE if this drive has just completed a SEEK command

    u765_DiskInfoBlock  DiskBlock;  // TDSKInfoBlock   <>
    u765_TrackInfoBlock TrackBlock; // TTRKInfoBlock   <>
}
u765_DiskUnit;

typedef enum {
    u765_FDCReadData,
    u765_FDCReadDeletedData,
    u765_FDCReadTrack
}
u765_ReadMode;

typedef enum {
    u765_FDCWriteData,
    u765_FDCWriteDeletedData
}
u765_WriteMode;

typedef struct {
    void* (*Realloc)(void*, size_t);

    uint8_t* FDC_RCVDLoc;       // DWORD ?
    uint8_t* FDC_SENDLoc;       // DWORD ?
    uint32_t CurrentSectorSize; // DWORD ?
    uint8_t* CurrentSectorData; // DWORD ?
    uint8_t* CurrentSectorInfo; // DWORD ?
    unsigned SectorToCPUReturn; // DWORD ?
    unsigned CPUToSectorReturn; // DWORD ?

    unsigned FDCVector;       // DWORD ?     ; vector to FDC handler
    unsigned FDCReturn;       // DWORD ?     ; return vector from subroutine
    unsigned FDCBufferReturn; // DWORD ?

    uint32_t CurrentFDDArrayPtr;  // DWORD ?     ; pointer to disk data for selected drive

    u765_DiskUnit* UnitPtr;     // DWORD ?     ; pointer to current TFDDUnit structure
    u765_DiskUnit* SeekUnitPtr; // DWORD ?     ; drive unit which was issued a Seek/Recalibrate command

    uint32_t BytesSaved; // DWORD ?

    void (*ActiveCallback)(void);                     // DWORD ?     ; application callback when disk system becomes active
    void (*CommandCallback)(uint8_t const*, uint8_t); // DWORD   ?   ; application callback when FDC command/parameters have been received

    uint32_t PhysicalSectorSize;  // DWORD   ?   ; 128 Shl N
    uint32_t AvailableSectorData; // DWORD   ?   ; available bytes of sector data
    uint32_t MultipleSectorPick;  // DWORD   ?

    uint16_t FDCCmdPC;    // WORD ?
    uint16_t FDC_RCVDCnt; // WORD ?
    uint16_t FDC_SENDCnt; // WORD ?

    uint8_t SelectedUnit; // BYTE ?  ; the drive unit currently in operation

    uint8_t LED;           // BYTE ?
    uint8_t MainStatusReg; // BYTE ?  ; this is the byte read from port 2FFD
    uint8_t Byte_3FFD;     // BYTE ?  ; byte being sent/received through the data register
    uint8_t ST0;           // BYTE ?  ; status register 0
    uint8_t ST1;           // BYTE ?  ; status register 1
    uint8_t ST2;           // BYTE ?  ; status register 2
    uint8_t ST3;           // BYTE ?  ; status register 3
    uint8_t SeekResult;    // BYTE ?  ; result returned from last seek command
    uint8_t TSEError;      // BYTE ?
    uint8_t ST2DAMBit;     // BYTE ?

    uint8_t IndexHoleCount; // BYTE ?
    uint8_t LastFDCCmd;     // BYTE ?
    uint8_t FDCRandomSeed;  // BYTE ?
    uint8_t DAM_Mask;       // BYTE ?
    uint8_t SectorsRead;    // BYTE ?
    uint8_t OverRunCounter; // BYTE ?
    uint8_t MotorOffTimer;  // BYTE ?
    uint8_t NewMotorState;  // BYTE ?
    uint8_t MotorState;     // BYTE ?

    uint8_t DTL_BytesSent; // BYTE ?
    uint8_t ValidTrack;    // BYTE ?
    uint8_t OverRunTest;   // BYTE ?
    uint8_t OverRunError;  // BYTE ?

    bool MultiSectorRead; // BYTE ?  ; Boolean

    uint8_t SectorsTransferred; // BYTE ?
    uint8_t NumParams;          // BYTE ?
    uint8_t NumResults;         // BYTE ?
    uint8_t OriginalR;          // BYTE ?
    uint8_t RetCSR0;            // BYTE ?

    uint8_t CurrentSectorNumber; // BYTE ?
    uint8_t DskRndMethod;        // BYTE ?

    // structures for 2 available drive units
    u765_DiskUnit FDDUnit0; // TFDDUnit    <>
    u765_DiskUnit FDDUnit1; // TFDDUnit    <>

    // current read mode in operation
    //ReadMode            BYTE ? //RESETENUM
    u765_ReadMode ReadMode; // ENUM                FDCReadData, FDCReadDeletedData, FDCReadTrack

    // current write mode in operation
    //WriteMode           BYTE ? //RESETENUM
    u765_WriteMode WriteMode; // ENUM                FDCWriteData, FDCWriteDeletedData

    uint8_t FDCCommandByte;       // BYTE    ?               ; command received by FDC
    uint8_t FDCParameters[32];    // BYTE    32      dup(?)  ; parameters for each command
    uint8_t FDCResults[32];       // BYTE    32      dup(?)  ; command result bytes for each command
    uint8_t FDCRandomData[16384]; // BYTE    16384   dup(?)  ; buffer for random bytes
}
u765_Controller;

typedef struct {
    uint8_t MSR;         // BYTE    ?
    uint8_t ST0;         // BYTE    ?
    uint8_t ST1;         // BYTE    ?
    uint8_t ST2;         // BYTE    ?
    uint8_t ST3;         // BYTE    ?
    uint8_t Unit0_CTRK;  // BYTE    ?
    uint8_t Unit0_CHEAD; // BYTE    ?
    uint8_t Unit0_CSR;   // BYTE    ?
    uint8_t Unit1_CTRK;  // BYTE    ?
    uint8_t Unit1_CHEAD; // BYTE    ?
    uint8_t Unit1_CSR;   // BYTE    ?
}
u765_State;

U765_EXPORT void U765_FUNCTION(u765_Initialise)(u765_Controller* FdcHandle, void* (*lpRealloc)(void*, size_t));
U765_EXPORT void U765_FUNCTION(u765_Shutdown)(u765_Controller* FdcHandle);
U765_EXPORT void U765_FUNCTION(u765_ResetDevice)(u765_Controller* FdcHandle);
U765_EXPORT void U765_FUNCTION(u765_InsertDisk)(u765_Controller* FdcHandle, const void* lpDisk, size_t Size, bool WriteProtect, uint8_t Unit);
U765_EXPORT bool U765_FUNCTION(u765_WriteProtected)(u765_Controller* FdcHandle, uint8_t Unit);
U765_EXPORT bool U765_FUNCTION(u765_ContentsChanged)(u765_Controller* FdcHandle, uint8_t Unit);
U765_EXPORT void U765_FUNCTION(u765_EjectDisk)(u765_Controller* FdcHandle, uint8_t Unit);
U765_EXPORT bool U765_FUNCTION(u765_GetMotorState)(u765_Controller* FdcHandle);
U765_EXPORT void U765_FUNCTION(u765_SetMotorState)(u765_Controller* FdcHandle, uint8_t Value);
U765_EXPORT uint8_t U765_FUNCTION(u765_StatusPortRead)(u765_Controller* FdcHandle);
U765_EXPORT uint8_t U765_FUNCTION(u765_DataPortRead)(u765_Controller* FdcHandle);
U765_EXPORT void U765_FUNCTION(u765_DataPortWrite)(u765_Controller* FdcHandle, uint8_t DataByte);
U765_EXPORT void U765_FUNCTION(u765_SetActiveCallback)(u765_Controller* FdcHandle, void (*lpActiveCallback)(void));
U765_EXPORT void U765_FUNCTION(u765_SetCommandCallback)(u765_Controller* FdcHandle, void (*lpCommandCallback)(uint8_t const*, uint8_t));
U765_EXPORT bool U765_FUNCTION(u765_DiskInserted)(u765_Controller* FdcHandle, uint8_t Unit);
U765_EXPORT void U765_FUNCTION(u765_SetRandomMethod)(u765_Controller* FdcHandle, uint8_t RndMethod);
U765_EXPORT void U765_FUNCTION(u765_GetFDCState)(u765_Controller* FdcHandle, u765_State* lpFDCState);

#endif // FDC765_H__

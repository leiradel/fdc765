#include <fdc765.h>

#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    typedef union { \
        struct { uint8_t l, h; }; \
        uint16_t x; \
        uint32_t e; \
        void* ptr; \
        uint8_t* u8; \
        u765_Controller* ctrl; \
        u765_DiskUnit* disk; \
        u765_State* stat; \
    } \
    Reg;
#else
    typedef union { \
        struct { uint8_t pad[2]; uint8_t h, l; }; \
        struct { uint16_t pad; uint16_t x; }; \
        uint32_t e; \
        void* ptr; \
        uint8_t* u8; \
        u765_Controller* ctrl; \
        u765_DiskUnit* disk; \
        u765_State* stat; \
    } \
    Reg;
#endif

#define STACK_SIZE 8

typedef struct {
    Reg eax, ebx, ecx, edx;
    Reg esi, edi, esp;
    Reg stack[STACK_SIZE];
    bool zero, carry;
}
Context;

#define ARG(ctx, index) ((ctx)->stack[(ctx)->esp.e + (index)])
#define PUSH(ctx, val) do { (ctx)->stack[(ctx)->esp.e++] = val; } while (0)
#define POP(ctx) ((ctx)->stack[--(ctx)->esp.e])
#define CALL(ctx, label) do { run((ctx), (label)); } while (0)
#define AND(ctx, left, right) do { (left) &= (right); (ctx)->zero = (left) == 0; (ctx)->carry = 0; } while (0)
#define OR(ctx, left, right) do { (left) |= (right); (ctx)->zero = (left) == 0; (ctx)->carry = 0; } while (0)
#define XOR(ctx, left, right) do { (left) ^= (right); (ctx)->zero = (left) == 0; (ctx)->carry = 0; } while (0)
#define SHL(ctx, left, right) do { (left) <<= (right); } while (0)
#define SHR(ctx, left, right) do { (left) >>= (right); } while (0)
#define INC(ctx, val) do { (val)++; (ctx)->zero = (val) == 0; } while (0)
#define DEC(ctx, val) do { (val)--; (ctx)->zero = (val) == 0; } while (0)
#define ADD(ctx, left, right) do { (left) += (right); (ctx)->zero = (left) == 0; (ctx)->carry = (left) < right; } while (0)
#define SUB(ctx, left, right) do { (ctx)->carry = (left) < right; (left) -= (right); (ctx)->zero = (left) == 0; } while (0)
#define CMP(ctx, left, right) do { (ctx)->zero = ((left) == (right)); (ctx)->carry = ((left) < (right)); } while (0)
#define TEST(ctx, left, right) do { (ctx)->zero = ((left) & (right)) == 0; (ctx)->carry = false; } while (0)
#define JE(ctx, label) do { if ((ctx)->zero) goto label; } while (0)
#define JNE(ctx, label) do { if (!(ctx)->zero) goto label; } while (0)
#define JNZ JNE
#define JC(ctx, label) do { if ((ctx)->carry) goto label; } while (0)
#define JNC(ctx, label) do { if (!(ctx)->carry) goto label; } while (0)
#define JA(ctx, label) do { if (!(ctx)->zero && !(ctx)->carry) goto label; } while (0)
#define JPREG(ctx, val) do { label = (val); goto again; } while (0)
#define SETNE(ctx) (!(ctx)->zero)
#define READW(ptr) ((ptr)[0] | (uint16_t)(ptr)[1] << 8)
#define WRITEW(ptr, val) do { (ptr)[0] = (uint8_t)(val); (ptr)[1] = (uint8_t)((val) >> 8); } while (0)
#define READDW(ptr) ((ptr)[0] | (uint32_t)(ptr)[1] << 8 | (uint32_t)(ptr)[2] << 16 | (uint32_t)(ptr)[3] << 24)
#define WRITEDW(ptr, val) do { (ptr)[0] = (uint8_t)(val); (ptr)[1] = (uint8_t)((val) >> 8); (ptr)[2] = (uint8_t)((val) >> 16); (ptr)[3] = (uint8_t)((val) >> 24); } while (0)

static void rep_movsb(Context* ctx) {
    memcpy(ctx->edi.u8, ctx->esi.u8, ctx->ecx.e);
    ctx->edi.u8 += ctx->ecx.e;
    ctx->esi.u8 += ctx->ecx.e;
    ctx->ecx.e = 0;
}

static void rep_movsd(Context* ctx) {
    ctx->ecx.e *= 4;
    rep_movsb(ctx);
}

/*-----------------------------------------------------------------------------
HIGH LEVEL INTERFACE
-----------------------------------------------------------------------------*/

static void LowLevelInitialise(Context*, u765_Controller*);
static void GetUnitPtr(Context*, uint8_t);
static void EDsk2Dsk(Context*, uint8_t);
static void run(Context*, unsigned);

void U765_FUNCTION(u765_SetRandomMethod)(u765_Controller* FdcHandle, uint8_t RndMethod) {
    Context ctx;
    ctx.esp.e = 0;
    ctx.ecx.ctrl = FdcHandle;
    ctx.eax.l = RndMethod;

    if ((ctx.eax.l == 255) || (ctx.eax.l <= 2)) {
        ctx.ecx.ctrl->DskRndMethod = ctx.eax.l;
    }
}

void U765_FUNCTION(u765_SetActiveCallback)(u765_Controller* FdcHandle, void (*lpActiveCallback)(void)) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.eax.ctrl = FdcHandle;
    ctx.eax.ctrl->ActiveCallback = lpActiveCallback;
}

void U765_FUNCTION(u765_SetCommandCallback)(u765_Controller* FdcHandle, void (*lpCommandCallback)(uint8_t const*, uint8_t)) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.eax.ctrl = FdcHandle;
    ctx.eax.ctrl->CommandCallback = lpCommandCallback;
}

void U765_FUNCTION(u765_SetMotorState)(u765_Controller* FdcHandle, uint8_t Value) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.ecx.ctrl = FdcHandle;
    ctx.eax.l = Value;

    AND(&ctx, ctx.eax.l, 8);
    SHR(&ctx, ctx.eax.l, 3);

    ctx.ecx.ctrl->NewMotorState = ctx.eax.l;

    // when the motor is turned off we still need a simple timer method
    // where we will accept further FDC read commands.
    // some disks require this behaviour, such as Scrabble Deluxe

    if ((ctx.ecx.ctrl->MotorState == 1) && (ctx.ecx.ctrl->NewMotorState == 0)) {
        ctx.ecx.ctrl->MotorOffTimer = 3; // 255
    }

    ctx.eax.l = ctx.ecx.ctrl->NewMotorState;
    ctx.ecx.ctrl->MotorState = ctx.eax.l;
}

bool U765_FUNCTION(u765_GetMotorState)(u765_Controller* FdcHandle) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.eax.ctrl = FdcHandle;
    return ctx.eax.ctrl->MotorState;
}

uint8_t U765_FUNCTION(u765_StatusPortRead)(u765_Controller* FdcHandle) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.edi.ctrl = FdcHandle;

    if (ctx.edi.ctrl->OverRunTest == true) {
        if (ctx.edi.ctrl->OverRunCounter == 0) {
            ctx.edi.ctrl->OverRunTest = false;
            ctx.edi.ctrl->OverRunError = true;

            // pushad
            // mov     eax, [edi].FDCReturn
            // mov     [edi].FDCVector, eax
            // call    eax
            // popad

            AND(&ctx, ctx.edi.ctrl->ST0, 0x3f);
            OR(&ctx, ctx.edi.ctrl->ST0, 0x40); // AT
            OR(&ctx, ctx.edi.ctrl->ST1, 0x10); // set OverRun bit (Lost Data bit)
            AND(&ctx, ctx.edi.ctrl->MainStatusReg, 0xdf); // clear Execution mode

            // do after clearing execution mode in MSR, fixes Italia 1990
            Context ad = ctx;
            ctx.eax.e = ctx.edi.ctrl->FDCReturn;
            ctx.edi.ctrl->FDCVector = ctx.eax.e;
            run(&ctx, ctx.eax.e);
            ctx = ad;
        }
        else {
            DEC(&ctx, ctx.edi.ctrl->OverRunCounter);
        }
    }

    ctx.eax.l = ctx.edi.ctrl->MainStatusReg;
    return ctx.eax.l;
}

uint8_t U765_FUNCTION(u765_DataPortRead)(u765_Controller* FdcHandle) {
    Context ctx;
    ctx.esp.e = 0;
    ctx.edi.ctrl = FdcHandle;
    ctx.eax.l = ctx.edi.ctrl->MainStatusReg;
    AND(&ctx, ctx.eax.l, 192);

    if (ctx.eax.l == 192) {
        Context ad = ctx;
        run(&ctx, ctx.edi.ctrl->FDCVector);
        ctx = ad;
        ctx.eax.l = ctx.edi.ctrl->Byte_3FFD;
    }

    return ctx.eax.l;
}

void U765_FUNCTION(u765_DataPortWrite)(u765_Controller* FdcHandle, uint8_t DataByte) {
    Context ctx;
    ctx.esp.e = 0;
    ctx.edi.ctrl = FdcHandle;

    ctx.eax.l = DataByte;
    ctx.edi.ctrl->Byte_3FFD = ctx.eax.l;

    ctx.eax.l = ctx.edi.ctrl->MainStatusReg;
    AND(&ctx, ctx.eax.l, 192);

    if (ctx.eax.l == 128) {
        Context ad = ctx;
        run(&ctx, ctx.edi.ctrl->FDCVector);
        ctx = ad;
    }
}

void U765_FUNCTION(u765_Initialise)(u765_Controller * FdcHandle, void* (*lpRealloc)(void*, size_t)) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.eax.ctrl = FdcHandle;
    memset(ctx.eax.ctrl, 0, sizeof(u765_Controller));
    
    if (lpRealloc != NULL) {
        ctx.eax.ctrl->Realloc = lpRealloc;
    }
    else {
        ctx.eax.ctrl->Realloc = realloc;
    }

    LowLevelInitialise(&ctx, ctx.eax.ctrl);
}

void U765_FUNCTION(u765_Shutdown)(u765_Controller* FdcHandle) {
    u765_EjectDisk(FdcHandle, 0);
    u765_EjectDisk(FdcHandle, 1);
}

void U765_FUNCTION(u765_ResetDevice)(u765_Controller* FdcHandle) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.eax.ctrl = FdcHandle;
    LowLevelInitialise(&ctx, ctx.eax.ctrl);
}

void U765_FUNCTION(u765_InsertDisk)(u765_Controller* FdcHandle, const void* lpDisk, size_t Size, bool WriteProtect, uint8_t Unit) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.edi.ctrl = FdcHandle;

    u765_EjectDisk(ctx.edi.ctrl, Unit); // close any open disk on this unit

    GetUnitPtr(&ctx, Unit);
    ctx.ebx = ctx.eax;

    ctx.ebx.disk->EDSK = false;
    ctx.ebx.disk->DiskInserted = false;
    ctx.ebx.disk->ContentsChanged = false;

    ctx.ebx.disk->DiskArrayPtr = lpDisk;
    ctx.ebx.disk->DiskArrayLen = Size;
    ctx.ebx.disk->WriteProtect = WriteProtect;

    ctx.ebx.disk->DiskInserted = true;
    ctx.ebx.disk->DriveStateChanged = true;

    ctx.eax.u8 = ctx.ebx.disk->DiskArrayPtr;

    if (*ctx.eax.u8 == 'E') {
        EDsk2Dsk(&ctx, Unit);
    }

    Context ad = ctx;
    ctx.esi.ptr = ctx.ebx.disk->DiskArrayPtr;
    ctx.edi.ptr = ctx.ebx.disk->DiskBlock.DiskInfoBlock;
    ctx.ecx.e = 256 / 4;
    rep_movsd(&ctx);

    ctx.ebx.disk->ContentsChanged = false;
    LowLevelInitialise(&ctx, FdcHandle);
}

U765_EXPORT bool U765_FUNCTION(u765_WriteProtected)(u765_Controller* FdcHandle, uint8_t Unit) {
    return Unit == 0 ? FdcHandle->FDDUnit0.WriteProtect : FdcHandle->FDDUnit1.WriteProtect;
}

bool U765_FUNCTION(u765_ContentsChanged)(u765_Controller* FdcHandle, uint8_t Unit) {
    return Unit == 0 ? FdcHandle->FDDUnit0.ContentsChanged : FdcHandle->FDDUnit1.ContentsChanged;
}

void U765_FUNCTION(u765_EjectDisk)(u765_Controller* FdcHandle, uint8_t Unit) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.edi.ctrl = FdcHandle;

    GetUnitPtr(&ctx, Unit);
    ctx.ebx = ctx.eax;

    ctx.ebx.disk->DiskArrayPtr = NULL;
    ctx.ebx.disk->DiskInserted = false;

    LowLevelInitialise(&ctx, FdcHandle);
}

bool U765_FUNCTION(u765_DiskInserted)(u765_Controller* FdcHandle, uint8_t Unit) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.edi.ctrl = FdcHandle;

    GetUnitPtr(&ctx, Unit);
    ctx.ebx = ctx.eax;

    // lea     eax, [ebx].TFDDUnit.Filename
    // invoke  SetLastError, eax

    ctx.eax.e = ctx.ebx.disk->DiskInserted;
    return ctx.eax.e != 0;    // true if disk inserted, else false
}

void U765_FUNCTION(u765_GetFDCState)(u765_Controller* FdcHandle, u765_State* lpFDCState) {
    Context ctx;
    ctx.esp.e = 0;

    ctx.ecx.ctrl = FdcHandle;
    ctx.edx.stat = lpFDCState;

    ctx.eax.l = ctx.ecx.ctrl->MainStatusReg;
    ctx.edx.stat->MSR = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->ST0;
    ctx.edx.stat->ST0 = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->ST1;
    ctx.edx.stat->ST1 = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->ST2;
    ctx.edx.stat->ST2 = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->ST3;
    ctx.edx.stat->ST3 = ctx.eax.l;

    ctx.eax.l = ctx.ecx.ctrl->FDDUnit0.CTK;
    ctx.edx.stat->Unit0_CTRK = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->FDDUnit0.CHEAD;
    ctx.edx.stat->Unit0_CHEAD = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->FDDUnit0.CSR;
    ctx.edx.stat->Unit0_CSR = ctx.eax.l;

    ctx.eax.l = ctx.ecx.ctrl->FDDUnit1.CTK;
    ctx.edx.stat->Unit1_CTRK = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->FDDUnit1.CHEAD;
    ctx.edx.stat->Unit1_CHEAD = ctx.eax.l;
    ctx.eax.l = ctx.ecx.ctrl->FDDUnit1.CSR;
    ctx.edx.stat->Unit1_CSR = ctx.eax.l;
}

/*-----------------------------------------------------------------------------
LOW LEVEL FUNCTIONS
-----------------------------------------------------------------------------*/

enum {
    case_AdvanceSectorPtrs,
    case_CDTS_1,
    case_CDTS_2,
    case_command,
    case_CPUDataToSector,
    case_CPUDataToSector_1,
    case_CPUDataToSector_Done,
    case_eax,
    case_edi,
    case_FDC_FormatTrack,
    case_FDC_Invalid,
    case_FDC_Invalid1,
    case_FDC_NewCommand,
    case_FDC_RdScDone,
    case_FDC_RdScL1,
    case_FDC_ReadData,
    case_FDC_ReadData1,
    case_FDC_ReadData2,
    case_FDC_ReadDataEntry,
    case_FDC_ReadDeletedData,
    case_FDC_ReadSectorID,
    case_FDC_ReadSectorID1,
    case_FDC_ReadSectorID2,
    case_FDC_ReadTrack,
    case_FDC_Recalibrate,
    case_FDC_Recalibrate1,
    case_FDC_Recalibrate2,
    case_FDC_ReceiveData,
    case_FDC_ReceiveDataEnd,
    case_FDC_ReceiveDataLoop,
    case_FDC_RecExit,
    case_FDC_RSSkip1,
    case_FDC_ScanEqual,
    case_FDC_ScanHighOrEqual,
    case_FDC_ScanLowOrEqual,
    case_FDC_SDS1,
    case_FDC_SDS2,
    case_FDC_SDSResults,
    case_FDC_Seek,
    case_FDC_Seek1,
    case_FDC_SendData,
    case_FDC_SendData1,
    case_FDC_SendData2,
    case_FDC_SenseCont,
    case_FDC_SenseDriveStatus,
    case_FDC_SenseDriveStatus1,
    case_FDC_SenseDriveStatus2,
    case_FDC_SenseInterruptStatus,
    case_FDC_SenseInterruptStatus1,
    case_FDC_Specify,
    case_FDC_Specify1,
    case_FDC_Version,
    case_FDC_Version1,
    case_FDC_WriteData,
    case_FDC_WriteData1,
    case_FDC_WriteData2,
    case_FDC_WriteDataEntry,
    case_FDC_WriteDeletedData,
    case_FDCBuff_ReturnSectorResults,
    case_FDCR_NotBadC,
    case_FDCR_SameC,
    case_GetSectorSize,
    case_GSS_Exit,
    case_InitFDC,
    case_InitReadSector,
    case_IRSDone,
    case_IRSJmp1,
    case_IRSLoop,
    case_LFRS_1,
    case_LFRS_3,
    case_LFRS_4,
    case_LFWS_1,
    case_LocateFirstWriteSector,
    case_LocateReadSector,
    case_LocateTrack,
    case_LockTrkDone,
    case_LocSingleSide,
    case_LocTrk1,
    case_NoDiskChange,
    case_NotReadTrk1,
    case_Rd_IgnoreDAM,
    case_Rd_NotMS1,
    case_Rd_Skip0,
    case_Rd_TransferData,
    case_Read_CheckDAM,
    case_Read_Com1,
    case_Read_CompareSectorID,
    case_Read_ID_1,
    case_ReadCurrTrack,
    case_ReadID_Results,
    case_ReadID_SendResults,
    case_ReadSectorData,
    case_ReadSectorData_1,
    case_ReceiveCommandBytes,
    case_ReturnSectorRWResults,
    case_RSJmp1,
    case_RSNoSec0,
    case_SDTC_NormalRandom,
    case_SDTC_RandomData,
    case_SDTC_TransferData,
    case_SectorDataToCPU,
    case_SectorDataToCPU_Done,
    case_SeekNotDone,
    case_SkipNextSector,
    case_SkipReadSector,
    case_SkipSectorEDSK,
    case_SkipWriteSector,
    case_SkNxtSec1,
    case_STrk_Valid,
    case_TrapStandardErrors,
    case_TSE_1,
    case_TSE_2,
    case_TSE_4,
    case_TSE_NotReady,
    case_TSE_Quit,
    case_VTrk_Exit,
    case_VTrk_Loop,
    case_WriteCurrTrack,
    case_WriteSectorData,
    case_WriteSectorData_1,
    case_WSD_WProt,
};

static void LowLevelInitialise(Context* ctx, u765_Controller* FdcHandle) {
    ctx->eax.ctrl = FdcHandle;
    ctx->eax.ctrl->MotorState = 0;
    ctx->eax.ctrl->FDCVector = case_FDC_NewCommand;
    ctx->eax.ctrl->MainStatusReg = 0x80; // 10000000b
    ctx->eax.ctrl->FDDUnit0.SeekDone = false;
    ctx->eax.ctrl->FDDUnit1.SeekDone = false;
    ctx->eax.ctrl->FDDUnit0.CTK = 0;
    ctx->eax.ctrl->FDDUnit1.CTK = 0;
    ctx->eax.ctrl->FDDUnit0.CHEAD = 0;
    ctx->eax.ctrl->FDDUnit1.CHEAD = 0;
    OR(ctx, ctx->eax.ctrl->ST3, 0x20); // 00100000b
    ctx->eax.ctrl->FDCRandomSeed = 0;
}

static void FDCCommandCallback(Context* ctx, uint8_t NumCmdBytes) {
    if (ctx->edi.ctrl->CommandCallback != NULL) {
        Context ad = *ctx;
        ctx->eax.e = NumCmdBytes;
        PUSH(ctx, ctx->eax);
        ctx->eax.u8 = &ctx->edi.ctrl->FDCCommandByte;
        PUSH(ctx, ctx->eax);
        ctx->edi.ctrl->CommandCallback(ARG(ctx, -1).u8, ARG(ctx, -2).l);
        *ctx = ad;
    }
}

static void GetUnitPtr(Context* ctx, uint8_t Unit) {
    AND(ctx, Unit, 1);

    if (Unit == 0) {
        ctx->eax.disk = &ctx->edi.ctrl->FDDUnit0;
    }
    else {
        ctx->eax.disk = &ctx->edi.ctrl->FDDUnit1;
    }
}

static void EDsk2Dsk(Context* ctx, uint8_t Unit) {
    uint8_t NumTracks, NumSectors, NumSides;
    uint32_t F, G, SectorLen, DOffset, DskOffset, EDskOffset, MaxTrackLen;
    void* DskArray;

    void* (*Realloc)(void*, size_t) = ctx->edi.ctrl->Realloc;

    GetUnitPtr(ctx, Unit);
    ctx->ebx = ctx->eax;

    ctx->ebx.disk->EDSK = true;

    ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
    ctx->eax.l = ctx->esi.u8[0x30];
    NumTracks = ctx->eax.l;
    ctx->eax.l = ctx->esi.u8[0x31];
    NumSides = ctx->eax.l;

    // the track size block starts at offset $34
    DOffset = 0x34;
    MaxTrackLen = 0;

    // and now we walk through the tracks and find the largest.
    ctx->eax.l = NumTracks;
    ctx->ecx.l = NumSides;
    ctx->eax.x = (uint16_t)ctx->eax.l * (uint16_t)ctx->ecx.l;
    AND(ctx, ctx->eax.e, 0xffff);

    XOR(ctx, ctx->ecx.e, ctx->ecx.e);
    ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
    ctx->esi.u8 += DOffset;

    for (int loop_counter = ctx->eax.e; loop_counter >= 0; loop_counter--) {
        ctx->eax.e = ctx->esi.u8[ctx->ecx.e];
        INC(ctx, ctx->ecx.e);

        if (ctx->eax.e > MaxTrackLen) {
            MaxTrackLen = ctx->eax.e;
        }
    }

    // having done that, each track is 256*Tracklen bytes.
    MaxTrackLen <<= 8;

    // now inflate the array to the required size.
    ctx->eax.l = NumTracks;
    ctx->ecx.l = NumSides;
    ctx->eax.x = (uint16_t)ctx->eax.l * (uint16_t)ctx->ecx.l;
    AND(ctx, ctx->eax.e, 0xffff);
    ctx->ecx.e = MaxTrackLen;
    ctx->eax.e *= ctx->ecx.e;
    ADD(ctx, ctx->eax.e, 256);
    ADD(ctx, ctx->eax.e, 100000);         // add safe space beyond disk space

    ctx->eax.ptr = Realloc(NULL, ctx->eax.e);

    DskArray = ctx->eax.ptr;
    if (ctx->eax.ptr == NULL) {
        return;         // *** memory allocation error
    }

    ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
    ctx->esi.u8 += 0x22;
    ctx->edi.u8 = DskArray;
    ctx->edi.u8 += 0x22;
    ctx->ecx.e = 14;       // $22 to $2f
    rep_movsb(ctx);

    // fill in the info we already know.
    ctx->edi.u8 = DskArray;
    ctx->eax.l = NumTracks;
    ctx->edi.u8[0x30] = ctx->eax.l;
    ctx->eax.l = NumSides;
    ctx->edi.u8[0x31] = ctx->eax.l;
    ctx->eax.e = MaxTrackLen;
    WRITEW(&ctx->edi.u8[0x32], ctx->eax.x);

    // and start gathering tracks from offset $100 (tracks start there, immediately after the header).
    EDskOffset = 0x100;

    ctx->eax.l = NumTracks;
    ctx->ecx.l = NumSides;

    ctx->eax.x = (uint16_t)ctx->eax.l * (uint16_t)ctx->ecx.l;
    AND(ctx, ctx->eax.e, 0xffff);

    F = 0;

    for (int loop_counter = ctx->eax.e; loop_counter >= 0; loop_counter--) {
        // the offset into the dsk that we will write the next track.
        ctx->eax.e = F;
        ctx->ecx.e = MaxTrackLen;
        ctx->eax.e *= ctx->ecx.e;
        ADD(ctx, ctx->eax.e, 0x100);
        DskOffset = ctx->eax.e;

        // Number of sectors in this track.
        ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
        ctx->esi.u8 += EDskOffset;
        ctx->esi.u8 += 0x15;
        ctx->eax.l = *ctx->esi.u8;
        NumSectors = ctx->eax.l;

        // Is the track size > 0?
        ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
        ctx->esi.u8 += F;
        ctx->esi.u8 += 0x34;
        ctx->eax.l = *ctx->esi.u8;

        if (ctx->eax.l != 0) {
            // While there's sectors available in this track
            if (NumSectors > 0) {
                // Copy the TrackInfoBlock and Sector Info List across
                ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
                ctx->esi.u8 += EDskOffset;
                ctx->edi.u8 = DskArray;
                ctx->edi.u8 += DskOffset;

                ctx->ecx.e = NumSectors;
                SHL(ctx, ctx->ecx.e, 3);
                ADD(ctx, ctx->ecx.e, 0x18);
                rep_movsb(ctx);

                // Now copy the sector data itself. Edsk has varying lengths, dsk does not.

                // Sectors begin 256 bytes after the start of the track
                ADD(ctx, DskOffset, 0x100);

                ctx->eax.e = EDskOffset;
                ADD(ctx, ctx->eax.e, 0x100);
                DOffset = ctx->eax.e;

                G = 0;
                ctx->eax.e = NumSectors;

                for (int loop_counter = ctx->eax.e; loop_counter >= 0; loop_counter--) {
                    // The edsk's sector length is found in the Sector Info List
                    ctx->eax.e = G;
                    SHL(ctx, ctx->eax.e, 3);
                    ADD(ctx, ctx->eax.e, 30);
                    ADD(ctx, ctx->eax.e, EDskOffset);
                    ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
                    ctx->esi.u8 += ctx->eax.e;
                    ctx->eax.e = READW(ctx->esi.u8);
                    SectorLen = ctx->eax.e;

                    // copy the data across
                    if (SectorLen > 0) {
                        ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
                        ctx->esi.u8 += DOffset;
                        ctx->edi.u8 = DskArray;
                        ctx->edi.u8 += DskOffset;
                        
                        ctx->ecx.e = SectorLen;
                        ADD(ctx, DOffset, ctx->ecx.e);
                        SHR(ctx, ctx->ecx.e, 2);
                        rep_movsd(ctx);
                        ctx->ecx.e = SectorLen;
                        AND(ctx, ctx->ecx.e, 3);
                        rep_movsb(ctx);
                    }

                    // Move to the next sector
                    ctx->eax.e = SectorLen;
                    ADD(ctx, DskOffset, ctx->eax.e);

                    INC(ctx, G);
                }
            }

            // move to the start of the next track in the edsk.
            ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
            ctx->esi.u8 += F;
            ctx->esi.u8 += 0x34;
            ctx->eax.e = *ctx->esi.u8;
            SHL(ctx, ctx->eax.e, 8);
            ADD(ctx, EDskOffset, ctx->eax.e);
        }

        INC(ctx, F);
    }

    Realloc(ctx->ebx.disk->DiskArrayPtr, 0);
    ctx->eax.ptr = DskArray;
    ctx->ebx.disk->DiskArrayPtr = ctx->eax.ptr;

    ctx->ebx.disk->WriteProtect = true;
}

static void SetFastDisk(Context* ctx) {
    if (ctx->edi.ctrl->ActiveCallback != NULL) {
        ctx->edi.ctrl->ActiveCallback();
    }
}

static void InitFDC(Context* ctx) {
    ctx->edi.ctrl->LED = 0;
    ctx->edi.ctrl->FDCVector = case_FDC_NewCommand;
    ctx->edi.ctrl->OverRunTest = false;
    ctx->edi.ctrl->OverRunError = false;
    ctx->edi.ctrl->MainStatusReg = 128;          // FDC is ready for a new command

    // call command callback with no cmd executing
    ctx->edi.ctrl->FDCCommandByte = 0;         // no command executing
    FDCCommandCallback(ctx, 1);
}

static void run(Context* ctx, unsigned label) {
again:
    switch (label) {
        // --------------------------------------------------------------------------------
        case case_FDC_NewCommand: label_FDC_NewCommand:
            ctx->edi.ctrl->NumParams = 0;
            ctx->edi.ctrl->NumResults = 0;
            ctx->edi.ctrl->SectorsTransferred = 0;

            ctx->eax.l = ctx->edi.ctrl->Byte_3FFD;
            ctx->edi.ctrl->FDCCommandByte = ctx->eax.l;

            ctx->eax.l = ctx->edi.ctrl->FDCCommandByte;
            AND(ctx, ctx->eax.e, 31);                 // mask command bits

            ctx->edi.ctrl->LastFDCCmd = ctx->eax.l;    // for debugging purposes only

            // a Sense Interrupt Status command must be sent after a Seek or Recalibrate interrupt,
            // otherwise the FDC will consider the next command to be an Invalid Command.

            // Breaks New Zealand Story
            // ctx->ecx.e = ctx->edi.ctrl->FDDUnit0;
            // .if     (ctx->eax.l != 8) && (ctx->ecx.e->TFDDUnit.SeekDone == true)
            // ctx->ecx.e->TFDDUnit.SeekDone = false;
            // goto label_FDC_Invalid;
            // .endif

            switch (ctx->eax.l & 31) {
                case  0: goto label_FDC_Invalid;
                case  1: goto label_FDC_Invalid;
                case  2: goto label_FDC_ReadTrack;               // 2   LED on
                case  3: goto label_FDC_Specify;                 // 3
                case  4: goto label_FDC_SenseDriveStatus;        // 4
                case  5: goto label_FDC_WriteData;               // 5   LED on
                case  6: goto label_FDC_ReadData;                // 6   LED on
                case  7: goto label_FDC_Recalibrate;             // 7
                case  8: goto label_FDC_SenseInterruptStatus;    // 8
                case  9: goto label_FDC_WriteDeletedData;        // 9   LED on
                case 10: goto label_FDC_ReadSectorID;            // 10  LED on
                case 11: goto label_FDC_Invalid;
                case 12: goto label_FDC_ReadDeletedData;         // 12  LED on
                case 13: goto label_FDC_FormatTrack;             // 13  LED on
                case 14: goto label_FDC_Invalid;
                case 15: goto label_FDC_Seek;                    // 15
                case 16: goto label_FDC_Version;                 // 16
                case 17: goto label_FDC_ScanEqual;               // 17
                case 18: goto label_FDC_Invalid;
                case 19: goto label_FDC_Invalid;
                case 20: goto label_FDC_Invalid;
                case 21: goto label_FDC_Invalid;
                case 22: goto label_FDC_Invalid;
                case 23: goto label_FDC_Invalid;
                case 24: goto label_FDC_Invalid;
                case 25: goto label_FDC_ScanLowOrEqual;          // 25
                case 26: goto label_FDC_Invalid;
                case 27: goto label_FDC_Invalid;
                case 28: goto label_FDC_Invalid;
                case 29: goto label_FDC_ScanHighOrEqual;         // 29
                case 30: goto label_FDC_Invalid;
                case 31: goto label_FDC_Invalid;
            }

        case case_InitFDC: label_InitFDC:
            ctx->edi.ctrl->LED = 0;
            ctx->edi.ctrl->FDCVector = case_FDC_NewCommand;
            ctx->edi.ctrl->OverRunTest = false;
            ctx->edi.ctrl->OverRunError = false;
            ctx->edi.ctrl->MainStatusReg = 128; // FDC is ready for a new command

            // call command callback with no cmd executing
            ctx->edi.ctrl->FDCCommandByte = 0; // no command executing
            FDCCommandCallback(ctx, 1);
            return;

        // this subroutine traps standard errors in FDC commands
        // only used for commands which take full 9 byte command bytes
        // CC0,CC1,C,H,R,N,EOT,GPL,DTL bytes.

        case case_TrapStandardErrors: label_TrapStandardErrors:
            ctx->edi.ctrl->TSEError = false;

            GetUnitPtr(ctx, ctx->edi.ctrl->FDCParameters[0]); // CC1
            ctx->edi.ctrl->UnitPtr = ctx->eax.disk;  // ptr to currently selected unit
            ctx->ebx.disk = ctx->eax.disk;

            ctx->edi.ctrl->ST0 = 0;
            ctx->edi.ctrl->ST1 = 0;

            CMP(ctx, ctx->ebx.disk->DiskInserted, false);
            JE(ctx, label_TSE_NotReady);              // no disk in drive
            CMP(ctx, ctx->edi.ctrl->MotorState, 1);
            JE(ctx, label_TSE_1);                     // jump if motor is running

            CMP(ctx, ctx->edi.ctrl->MotorOffTimer, 0);     // if the motor off timer is zero
            JE(ctx, label_TSE_NotReady);              // then the drive is not ready

            DEC(ctx, ctx->edi.ctrl->MotorOffTimer);        // else decrement the motor timer
            goto label_TSE_1;                     // and accept the command

        case case_TSE_NotReady: label_TSE_NotReady:

            ctx->edi.ctrl->TSEError = true;
            OR(ctx, ctx->edi.ctrl->ST0, 0x8);       // FDD is in the not-ready state
            AND(ctx, ctx->edi.ctrl->ST3, 0xdf);
            goto label_TSE_Quit;

        case case_TSE_1: label_TSE_1:
            AND(ctx, ctx->edi.ctrl->ST3, 0xfb);
            AND(ctx, ctx->edi.ctrl->ST0, 0xfb);
            ctx->eax.l = ctx->edi.ctrl->FDCParameters[0]; // CC1
            SHR(ctx, ctx->eax.l, 2);                         // HD >> bit 0
            AND(ctx, ctx->eax.l, 1);                         // mask off the HD value
            ctx->ebx.disk->CHEAD = ctx->eax.l;               // set current head for this unit
            CMP(ctx, ctx->eax.l, 0);
            JE(ctx, label_TSE_2);                        // branch forward if H = 0

            OR(ctx, ctx->edi.ctrl->ST3, 0x4);
            OR(ctx, ctx->edi.ctrl->ST0, 0x4);
            CMP(ctx, ctx->ebx.disk->DiskBlock.NumSides, 2);   // HD can be 1 if the disk is double sided
            JE(ctx, label_TSE_2);

            AND(ctx, ctx->edi.ctrl->ST3, 0xfb);
            AND(ctx, ctx->edi.ctrl->ST0, 0xfb);
            ctx->ebx.disk->CHEAD = 0;                // else current head is reset to zero
            ctx->edi.ctrl->TSEError = true;          // and signal the error
            OR(ctx, ctx->edi.ctrl->ST0, 0x8);
            // fallthrough

        case case_TSE_2: label_TSE_2:
            // any further tests required will go here
            // fallthrough
        
        case case_TSE_Quit: label_TSE_Quit:
            CMP(ctx, ctx->edi.ctrl->TSEError, true);
            JNE(ctx, label_TSE_4);
            OR(ctx, ctx->edi.ctrl->ST0, 0x40);   // abnormal termination of command
            // fallthrough

        case case_TSE_4: label_TSE_4:
            return;

        // ######################################################################

        case case_FDC_ReadData: label_FDC_ReadData:
            ctx->edi.ctrl->ReadMode = u765_FDCReadData;
            ctx->edi.ctrl->DAM_Mask = 0;        // set Data Address mask for normal data sectors

            // entry point for READ_DATA, READ_DELETED_DATA and READ_TRACK commands
            // receives the parameters for the FDC commands and then jumps to the main sector reading code
            // fallthrough

        case case_FDC_ReadDataEntry: label_FDC_ReadDataEntry:
            SetFastDisk(ctx);

            ctx->edi.ctrl->FDCReturn = case_FDC_ReadData1;
            ctx->ecx.x = 8;                                   // expect 8 bytes
            goto label_ReceiveCommandBytes;

        case case_FDC_ReadData1: label_FDC_ReadData1:
            FDCCommandCallback(ctx, 9);

            ctx->eax.l = ctx->edi.ctrl->FDCParameters[3];     // R param
            ctx->edi.ctrl->OriginalR = ctx->eax.l;                     // preserve R value

            ctx->edi.ctrl->FDCBufferReturn = case_FDC_ReadData2;
            goto label_ReadSectorData;                         // jump to the main sector reading code

        case case_FDC_ReadData2: label_FDC_ReadData2:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_ReadDeletedData: label_FDC_ReadDeletedData:
            ctx->edi.ctrl->ReadMode = u765_FDCReadDeletedData;
            ctx->edi.ctrl->DAM_Mask = 64;       // set Data Address mask for deleted data sectors
            goto label_FDC_ReadDataEntry;

        // ######################################################################

        case case_FDC_WriteData: label_FDC_WriteData:
            ctx->edi.ctrl->WriteMode = u765_FDCWriteData;
            ctx->edi.ctrl->DAM_Mask = 0;
            // fallthrough

        case case_FDC_WriteDataEntry: label_FDC_WriteDataEntry:
            SetFastDisk(ctx);

            ctx->edi.ctrl->FDCReturn = case_FDC_WriteData1;
            ctx->ecx.x = 8;                   // expect 8 bytes
            goto label_ReceiveCommandBytes;

        case case_FDC_WriteData1: label_FDC_WriteData1:
            FDCCommandCallback(ctx, 9);

            ctx->eax.l = ctx->edi.ctrl->FDCParameters[3]; // R param
            ctx->edi.ctrl->OriginalR = ctx->eax.l;                // preserve R value

            ctx->edi.ctrl->FDCBufferReturn = case_FDC_WriteData2;
            goto label_WriteSectorData;

        case case_FDC_WriteData2: label_FDC_WriteData2:
            CALL(ctx, case_InitFDC);
            return;


        // ######################################################################

        case case_FDC_WriteDeletedData: label_FDC_WriteDeletedData:
            ctx->edi.ctrl->WriteMode = u765_FDCWriteDeletedData;
            ctx->edi.ctrl->DAM_Mask = 64;
            goto label_FDC_WriteDataEntry;

        // ######################################################################

        case case_FDC_ReadTrack: label_FDC_ReadTrack:
            ctx->edi.ctrl->ReadMode = u765_FDCReadTrack;
            goto label_FDC_ReadDataEntry;

        // ######################################################################

        case case_FDC_ReadSectorID: label_FDC_ReadSectorID:
            SetFastDisk(ctx);

            ctx->edi.ctrl->FDCReturn = case_FDC_ReadSectorID1;
            ctx->ecx.x = 1;                   // expect 1 byte
            goto label_ReceiveCommandBytes;

        case case_FDC_ReadSectorID1: label_FDC_ReadSectorID1:
            // FDCCommandCallback(ctx, 2);

            OR(ctx, ctx->edi.ctrl->MainStatusReg, 16);       // FDC is busy
            ctx->edi.ctrl->ST2 = 0;
            CALL(ctx, case_TrapStandardErrors);
            CMP(ctx, ctx->edi.ctrl->TSEError, true);
            JNE(ctx, label_FDC_RSSkip1);

            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.SectorInfoList[0];
            goto label_ReadID_Results;

        case case_FDC_RSSkip1: label_FDC_RSSkip1:
            CALL(ctx, case_ReadCurrTrack);

            CMP(ctx, ctx->edi.ctrl->ValidTrack, true);
            JE(ctx, label_Read_ID_1);

            // this is not a valid track number
            ctx->edi.ctrl->FDCResults[0] = 0;    // ST0
            ctx->edi.ctrl->FDCResults[1] = 1;    // ST1 = MA (1)
            ctx->eax.l = ctx->ebx.disk->CTK;                      // C
            ctx->edi.ctrl->FDCResults[3] = ctx->eax.l;
            goto label_ReadID_SendResults;

        case case_Read_ID_1: label_Read_ID_1:
            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.SectorInfoList[0];
            ctx->ecx.u8 = &ctx->ebx.disk->TrackBlock.SectorData[0];

            // after a seek/recalibrate we return the sector ID for the first disk sector
            // on the new track until RetSCR0 counts down to 0
            CMP(ctx, ctx->edi.ctrl->RetCSR0, 0);
            JE(ctx, label_RSNoSec0);

            DEC(ctx, ctx->edi.ctrl->RetCSR0);
            ctx->ebx.disk->CSR = 0;
            // fallthrough

        case case_RSNoSec0: label_RSNoSec0:
            ctx->edx.l = ctx->ebx.disk->CSR;
            INC(ctx, ctx->edx.l);
            CMP(ctx, ctx->edx.l, ctx->ebx.disk->TrackBlock.NumSectors);
            JC(ctx, label_RSJmp1);
            XOR(ctx, ctx->edx.l, ctx->edx.l);
            // fallthrough

        case case_RSJmp1: label_RSJmp1:
            ctx->ebx.disk->CSR = ctx->edx.l;
            OR(ctx, ctx->edx.l, ctx->edx.l);
            JE(ctx, label_FDC_RdScDone);
            // fallthrough

        case case_FDC_RdScL1: label_FDC_RdScL1:
            CALL(ctx, case_SkipNextSector);
            DEC(ctx, ctx->edx.l);
            JNE(ctx, label_FDC_RdScL1);
            // fallthrough

        case case_FDC_RdScDone: label_FDC_RdScDone:
            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);  // normal termination
            // fallthrough

        case case_ReadID_Results: label_ReadID_Results:
            AND(ctx, ctx->edi.ctrl->ST0, 0xfc);
            ctx->eax.l = ctx->edi.ctrl->FDCParameters[0];
            AND(ctx, ctx->eax.l, 3);
            OR(ctx, ctx->edi.ctrl->ST0, ctx->eax.l);

            ctx->edx.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->eax.l = ctx->edi.ctrl->ST0;               // ST0
            ctx->edx.u8[0] = ctx->eax.l;
            ctx->eax.x = READW(ctx->esi.u8 + 4);  // ST1, ST2

            // Epyx 21 fix,
            // we don't read the sector data in a read sector ID command,
            // so we can't have a CRC error in the sector data reported.
            AND(ctx, ctx->eax.l, 255 - 32);           // ST1
            AND(ctx, ctx->eax.h, 255 - 32);           // ST2

            WRITEW(ctx->edx.u8 + 1, ctx->eax.x);
            ctx->eax.e = READDW(ctx->esi.u8);  // C,H,R,N
            WRITEDW(ctx->edx.u8 + 3, ctx->eax.e);
            // fallthrough

        case case_ReadID_SendResults: label_ReadID_SendResults:
            // clone CHRN from Result bytes into the Command bytes in order to return Result phase bytes early in a Command Callback
            ctx->eax.e = READDW(&ctx->edi.ctrl->FDCResults[3]);
            WRITEDW(&ctx->edi.ctrl->FDCCommandByte + 2, ctx->eax.e);

            FDCCommandCallback(ctx, 6);   // 2 command bytes + CHRN early Results phase bytes);

            // return Results phase bytes to CPU
            ctx->edi.ctrl->FDCReturn = case_FDC_ReadSectorID2;
            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->ecx.x = 7;             // 7 bytes of result data
            ctx->edi.ctrl->NumResults = 7;
            goto label_FDC_SendData;    // transfer data to CPU

        case case_FDC_ReadSectorID2: label_FDC_ReadSectorID2:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_FormatTrack: label_FDC_FormatTrack:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_ScanEqual: label_FDC_ScanEqual:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_ScanLowOrEqual: label_FDC_ScanLowOrEqual:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_ScanHighOrEqual: label_FDC_ScanHighOrEqual:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        // Command = 7
        case case_FDC_Recalibrate: label_FDC_Recalibrate:
            AND(ctx, ctx->edi.ctrl->ST0, 0xdf);
            ctx->edi.ctrl->FDCReturn = case_FDC_Recalibrate1;
            ctx->ecx.x = 1;                   // expect 1 byte
            goto label_ReceiveCommandBytes;

        case case_FDC_Recalibrate1: label_FDC_Recalibrate1:
            FDCCommandCallback(ctx, 2);

            GetUnitPtr(ctx, ctx->edi.ctrl->FDCParameters[0]);
            ctx->edi.ctrl->SeekUnitPtr = ctx->eax.disk;
            ctx->ebx.disk = ctx->eax.disk;
            // fallthrough

        case case_FDC_Recalibrate2: label_FDC_Recalibrate2:
            ctx->ebx.disk->CTK = 0;
            OR(ctx, ctx->edi.ctrl->ST3, 0x10);
            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);
            OR(ctx, ctx->edi.ctrl->ST0, 0x20);
            ctx->edi.ctrl->SeekResult = 0x20;         // Normal Termination of Recalibrate Command
            // fallthrough

        case case_FDC_RecExit: label_FDC_RecExit:
            ctx->ebx.disk->SeekDone = true;

            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        // Command = 8
        case case_FDC_SenseInterruptStatus: label_FDC_SenseInterruptStatus:
            FDCCommandCallback(ctx, 1);

            OR(ctx, ctx->edi.ctrl->MainStatusReg, 16);       // FDC is busy
            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];

            ctx->ebx.disk = ctx->edi.ctrl->SeekUnitPtr;       // drive unit which was issued a Seek/Recalibrate command
            if (ctx->ebx.disk == NULL) {
                ctx->ebx.disk = &ctx->edi.ctrl->FDDUnit0;            // safeguard
            }

            CMP(ctx, ctx->ebx.disk->SeekDone, true);      // interrupt caused by seek completion?
            JNE(ctx, label_SeekNotDone);

            ctx->edi.ctrl->RetCSR0 = 9;
            ctx->ebx.disk->SeekDone = false;
            ctx->eax.l = ctx->edi.ctrl->SeekResult;          // result from seek command
            ctx->ecx.l = ctx->ebx.disk->CHEAD;
            SHL(ctx, ctx->ecx.l, 2);
            OR(ctx, ctx->eax.l, ctx->ecx.l);
            ctx->ecx.x = 2;                   // 2 result bytes after a seek
            ctx->edi.ctrl->NumResults = 2;
            goto label_FDC_SenseCont;

        case case_SeekNotDone: label_SeekNotDone:
            ctx->ecx.x = 1;                   // else only 1 result byte (ST0)
            ctx->edi.ctrl->NumResults = 1;
            CMP(ctx, ctx->ebx.disk->DriveStateChanged, true);
            JNE(ctx, label_NoDiskChange);

            ctx->ebx.disk->DriveStateChanged = false;
            ctx->eax.l = 0xc0;                 // Ready Line Changed state, either polarity
            goto label_FDC_SenseCont;

        case case_NoDiskChange: label_NoDiskChange:
            ctx->eax.l = 0x80;
            // fallthrough

        case case_FDC_SenseCont: label_FDC_SenseCont:
            ctx->edx.disk = &ctx->edi.ctrl->FDDUnit1;
            if (ctx->ebx.disk == ctx->edx.disk) {      // offset FDDUnit1
                OR(ctx, ctx->eax.l, 1);        // set unit 1 bit in result
            }

            ctx->edi.ctrl->ST0 = ctx->eax.l;
            ctx->esi.u8[0] = ctx->eax.l;
            ctx->eax.l = ctx->ebx.disk->CTK;
            ctx->esi.u8[1] = ctx->eax.l;

            ctx->edi.ctrl->FDCReturn = case_FDC_SenseInterruptStatus1;
            goto label_FDC_SendData;

        case case_FDC_SenseInterruptStatus1: label_FDC_SenseInterruptStatus1:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_Specify: label_FDC_Specify:
            ctx->edi.ctrl->FDCReturn = case_FDC_Specify1;
            ctx->ecx.x = 2;                   // expect 2 bytes
            goto label_ReceiveCommandBytes;

        case case_FDC_Specify1: label_FDC_Specify1:
            FDCCommandCallback(ctx, 3);

            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);

            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_SenseDriveStatus: label_FDC_SenseDriveStatus:
            ctx->edi.ctrl->FDCReturn = case_FDC_SenseDriveStatus1;
            ctx->ecx.x = 1;                   // expect 1 byte
            goto label_ReceiveCommandBytes;

        case case_FDC_SenseDriveStatus1: label_FDC_SenseDriveStatus1:
            FDCCommandCallback(ctx, 2);

            OR(ctx, ctx->edi.ctrl->MainStatusReg, 16);       // FDC is busy
            ctx->edi.ctrl->ST3 = 0x40;                // assume write-protected here

            GetUnitPtr(ctx, ctx->edi.ctrl->FDCParameters[0]);
            ctx->ebx = ctx->eax;

            ctx->eax.l = ctx->edi.ctrl->FDCParameters[0];
            AND(ctx, ctx->eax.l, 3);
            OR(ctx, ctx->edi.ctrl->ST3, ctx->eax.l);                 // Unit
            // fallthrough

        case case_FDC_SDS1: label_FDC_SDS1:
            ctx->ecx.l = ctx->ebx.disk->CHEAD;
            SHL(ctx, ctx->ecx.l, 2);
            OR(ctx, ctx->edi.ctrl->ST3, ctx->ecx.l);

            CMP(ctx, ctx->ebx.disk->CTK, 0);
            JNE(ctx, label_FDC_SDS2);
            OR(ctx, ctx->edi.ctrl->ST3, 0x10);    // Track 0 signal
            // fallthrough

        case case_FDC_SDS2: label_FDC_SDS2:
            CMP(ctx, ctx->ebx.disk->DiskInserted, true);
            JNE(ctx, label_FDC_SDSResults);

            CMP(ctx, ctx->edi.ctrl->MotorState, 1);     // Motor on?
            JNE(ctx, label_FDC_SDSResults);
            OR(ctx, ctx->edi.ctrl->ST3, 0x20);    // Drive is Ready

            CMP(ctx, ctx->ebx.disk->WriteProtect, true);
            JE(ctx, label_FDC_SDSResults);
            AND(ctx, ctx->edi.ctrl->ST3, 0xbf);          // clear write-protected bit
            // fallthrough

        case case_FDC_SDSResults: label_FDC_SDSResults:
            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->eax.l = ctx->edi.ctrl->ST3;
            ctx->esi.u8[0] = ctx->eax.l;

            ctx->edi.ctrl->FDCReturn = case_FDC_SenseDriveStatus2;
            ctx->ecx.x = 1;
            ctx->edi.ctrl->NumResults = 1;
            goto label_FDC_SendData;

        case case_FDC_SenseDriveStatus2: label_FDC_SenseDriveStatus2:
            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);

            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_Seek: label_FDC_Seek:
            ctx->edi.ctrl->FDCReturn = case_FDC_Seek1;
            ctx->ecx.x = 2;                   // expect 2 bytes
            goto label_ReceiveCommandBytes;

        case case_FDC_Seek1: label_FDC_Seek1:
            FDCCommandCallback(ctx, 3);

            ctx->edi.ctrl->SeekResult = 0x20;         // normal termination of seek

            GetUnitPtr(ctx, ctx->edi.ctrl->FDCParameters[0]);
            ctx->edi.ctrl->SeekUnitPtr = ctx->eax.disk;
            ctx->ebx = ctx->eax;

            ctx->eax.l = ctx->edi.ctrl->FDCParameters[1];  // C parameter
            CMP(ctx, ctx->eax.l, ctx->ebx.disk->DiskBlock.NumTracks);
            JC(ctx, label_STrk_Valid);            // jump if seeking a valid cylinder

            ctx->edi.ctrl->SeekResult = 0x60;         // abnormal termination of seek

            // if seeking beyond the final cylinder then stop at final cylinder
            ctx->eax.l = ctx->ebx.disk->DiskBlock.NumTracks;
            DEC(ctx, ctx->eax.l);
            // fallthrough

        case case_STrk_Valid: label_STrk_Valid:
            ctx->ebx.disk->CTK = ctx->eax.l;           // update current track head is over
            ctx->ebx.disk->CSR = 0;
            AND(ctx, ctx->edi.ctrl->ST0, 0x1b);    // Normal termination, clear HD bit
            OR(ctx, ctx->edi.ctrl->ST0, 0x20);    // seek complete

            ctx->eax.l = ctx->ebx.disk->CHEAD;
            SHL(ctx, ctx->eax.l, 2);
            OR(ctx, ctx->edi.ctrl->ST0, ctx->eax.l);           // set HD bit

            ctx->ebx.disk->SeekDone = true;

            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_Version: label_FDC_Version:
            FDCCommandCallback(ctx, 1);

            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->eax.l = 0x80;    // $80 = uPD765A identifier
            ctx->edi.ctrl->ST0 = ctx->eax.l;
            ctx->esi.u8[0] = ctx->eax.l;

            OR(ctx, ctx->edi.ctrl->MainStatusReg, 16);       // FDC is busy
            ctx->edi.ctrl->FDCReturn = case_FDC_Version1;
            ctx->ecx.x = 1;
            ctx->edi.ctrl->NumResults = 1;
            goto label_FDC_SendData;

        case case_FDC_Version1: label_FDC_Version1:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        case case_FDC_Invalid: label_FDC_Invalid:
            FDCCommandCallback(ctx, 1);

            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->eax.l = ctx->edi.ctrl->ST0;
            AND(ctx, ctx->eax.l, 0x3f);
            OR(ctx, ctx->eax.l, 0x80);
            ctx->edi.ctrl->ST0 = ctx->eax.l;
            ctx->esi.u8[0] = ctx->eax.l;

            OR(ctx, ctx->edi.ctrl->MainStatusReg, 16);       // FDC is busy
            ctx->edi.ctrl->FDCReturn = case_FDC_Invalid1;
            ctx->ecx.x = 1;
            ctx->edi.ctrl->NumResults = 1;
            goto label_FDC_SendData;

        case case_FDC_Invalid1: label_FDC_Invalid1:
            CALL(ctx, case_InitFDC);
            return;

        // ######################################################################

        // Low Level Floppy Disc Controller Functions

        // ######################################################################

        // CX = number of command bytes to receive
        case case_ReceiveCommandBytes: label_ReceiveCommandBytes:
            ctx->edx.u8 = &ctx->edi.ctrl->FDCParameters[0];             // address for command bytes
            ctx->edi.ctrl->NumParams = ctx->ecx.l;                     // number of FDC command paramater bytes to receive
            // fallthrough

        // CPU->FDC
        // FDC receives CX bytes from Z80, and stores them at [EDX]
        // byte received is in Byte_3FFD var

        case case_FDC_ReceiveData: label_FDC_ReceiveData:
            ctx->edi.ctrl->FDC_RCVDCnt = ctx->ecx.x;
            ctx->edi.ctrl->FDC_RCVDLoc = ctx->edx.u8;
            ctx->edi.ctrl->FDCVector = case_FDC_ReceiveDataLoop;
            AND(ctx, ctx->edi.ctrl->MainStatusReg, 0x3f);
            OR(ctx, ctx->edi.ctrl->MainStatusReg, 0x80);
            return;

        case case_FDC_ReceiveDataLoop: label_FDC_ReceiveDataLoop:
            ctx->edx.u8 = ctx->edi.ctrl->FDC_RCVDLoc;
            ctx->eax.l = ctx->edi.ctrl->Byte_3FFD;
            *ctx->edx.u8 = ctx->eax.l;
            INC(ctx, ctx->edi.ctrl->FDC_RCVDLoc);
            DEC(ctx, ctx->edi.ctrl->FDC_RCVDCnt);
            JE(ctx, label_FDC_ReceiveDataEnd);
            return;

        case case_FDC_ReceiveDataEnd: label_FDC_ReceiveDataEnd:
            ctx->eax.e = ctx->edi.ctrl->FDCReturn;
            ctx->edi.ctrl->FDCVector = ctx->eax.e;
            JPREG(ctx, ctx->eax.e);

        // ######################################################################

        // FDC->CPU
        // FDC sends CX bytes from [ESI] to Z80
        case case_FDC_SendData: label_FDC_SendData:
            ctx->edi.ctrl->FDC_SENDCnt = ctx->ecx.x;
            ctx->edi.ctrl->FDC_SENDLoc = ctx->esi.u8;
            ctx->edi.ctrl->FDCVector = case_FDC_SendData1;
            OR(ctx, ctx->edi.ctrl->MainStatusReg, 0xc0);
            return;

        case case_FDC_SendData1: label_FDC_SendData1:
            ctx->esi.u8 = ctx->edi.ctrl->FDC_SENDLoc;
            ctx->eax.l = *ctx->esi.u8;
            ctx->edi.ctrl->Byte_3FFD = ctx->eax.l;
            INC(ctx, ctx->edi.ctrl->FDC_SENDLoc);
            DEC(ctx, ctx->edi.ctrl->FDC_SENDCnt);
            JNE(ctx, label_FDC_SendData2);

            ctx->eax.e = ctx->edi.ctrl->FDCReturn;
            ctx->edi.ctrl->FDCVector = ctx->eax.e;
            JPREG(ctx, ctx->eax.e);

        case case_FDC_SendData2: label_FDC_SendData2:
            ctx->edi.ctrl->OverRunTest = true;
            ctx->edi.ctrl->OverRunCounter = 64;
            return;

        // ######################################################################

        // End of Cylinder (bit 7 in ST1) is set if:
        // 1. sector data is read completely. (i.e. no other errors occur like no data)
        // 2. sector being read is same specified by EOT
        // 3. terminal count is not received

        case case_ReadSectorData: label_ReadSectorData:
            XOR(ctx, ctx->eax.l, ctx->eax.l);
            ctx->edi.ctrl->ST0 = ctx->eax.l;
            ctx->edi.ctrl->ST1 = ctx->eax.l;
            ctx->edi.ctrl->ST2 = ctx->eax.l;
            ctx->edi.ctrl->SectorsRead = ctx->eax.l;  // for Read Track only

            ctx->eax.l = ctx->edi.ctrl->FDCParameters[3];          // R
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[5]); // EOT
            ctx->edi.ctrl->MultiSectorRead = SETNE(ctx);   // if R != EOT then this is a multi-sector transfer

            CALL(ctx, case_TrapStandardErrors);
            CMP(ctx, ctx->edi.ctrl->TSEError, true);
            JNE(ctx, label_ReadSectorData_1);

            ctx->eax.e = READDW(&ctx->edi.ctrl->FDCParameters[1]);   // copy C,H,R,N from command
            WRITEDW(&ctx->edi.ctrl->FDCResults[3], ctx->eax.e);      // into results buffer
            goto label_ReturnSectorRWResults;         // and exit returning the error

        case case_ReadSectorData_1: label_ReadSectorData_1:
            OR(ctx, ctx->edi.ctrl->MainStatusReg, 32 + 16);      // enter Execution mode + busy

            ctx->edi.ctrl->IndexHoleCount = 0;
            ctx->edi.ctrl->DTL_BytesSent = false;

            CALL(ctx, case_ReadCurrTrack);

            CMP(ctx, ctx->edi.ctrl->ReadMode, u765_FDCReadTrack);
            JNE(ctx, label_InitReadSector);

            ctx->ebx.disk->CSR = -1;  // Read Track cmd always starts from first physical sector
            // fallthrough

        // locate physical sector the head is currently over
        case case_InitReadSector: label_InitReadSector:
            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.SectorInfoList[0];
            ctx->ecx.u8 = &ctx->ebx.disk->TrackBlock.SectorData[0];

            ctx->edx.l = ctx->ebx.disk->CSR;
            INC(ctx, ctx->edx.l);
            CMP(ctx, ctx->edx.l, ctx->ebx.disk->TrackBlock.NumSectors);
            JC(ctx, label_IRSJmp1);
            XOR(ctx, ctx->edx.l, ctx->edx.l);
            // fallthrough

        case case_IRSJmp1: label_IRSJmp1:
            ctx->ebx.disk->CSR = ctx->edx.l;
            OR(ctx, ctx->edx.l, ctx->edx.l);
            JE(ctx, label_IRSDone);
            // fallthrough

        case case_IRSLoop: label_IRSLoop:
            CALL(ctx, case_SkipNextSector);
            DEC(ctx, ctx->edx.l);
            JNE(ctx, label_IRSLoop);
            // fallthrough

        case case_IRSDone: label_IRSDone:
            ctx->edi.ctrl->CurrentSectorData = ctx->ecx.u8;    // ptr to this sector's data
            ctx->edi.ctrl->CurrentSectorInfo = ctx->esi.u8;    // ptr to this sector's info
            CALL(ctx, case_GetSectorSize);
            ctx->edi.ctrl->CurrentSectorSize = ctx->eax.e;    // this sector's size (in bytes)
            ctx->edi.ctrl->ST2DAMBit = 0;              // assume no DAM error at this stage
            // fallthrough

        case case_LocateReadSector: label_LocateReadSector:
            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorInfo;

            ctx->eax.e = READDW(ctx->esi.u8);           // copy CHRN from the DSK's sectorinfo
            WRITEDW(&ctx->edi.ctrl->FDCResults[3], ctx->eax.e);   // to Results buffer

            ctx->eax.x = READW(&ctx->esi.u8[4]);  // ctx->eax.l=ST1, ctx->eax.h=ST2
            AND(ctx, ctx->eax.x, 0x2125);             // DE, ND or MA in ST1, DD or MD in ST2
            WRITEW(&ctx->edi.ctrl->ST1, ctx->eax.x);    // DAM in ST2 ignored at this point

            CMP(ctx, ctx->edi.ctrl->ValidTrack, true);      // basically - is this track formatted?
            JE(ctx, label_Read_CompareSectorID);

            // this is not a valid track number
            ctx->edi.ctrl->ST0 = 0x40;           // AT
            ctx->edi.ctrl->ST1 = 1;             // MA
            ctx->eax.l = ctx->ebx.disk->CTK;
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[1]);    // C
            JE(ctx, label_FDCR_SameC);
            ctx->eax.h = 0x10;            // No Cylinder
            CMP(ctx, ctx->eax.l, 0xff);
            JNE(ctx, label_FDCR_NotBadC);
            ctx->eax.h = 2;              // Bad Cylinder
            // fallthrough

        case case_FDCR_NotBadC: label_FDCR_NotBadC:
            OR(ctx, ctx->edi.ctrl->ST2, ctx->eax.h);
            // fallthrough

        case case_FDCR_SameC: label_FDCR_SameC:
            ctx->eax.e = READDW(&ctx->edi.ctrl->FDCParameters[1]); // copy CHRN from the FDC command
            ctx->eax.l = ctx->ebx.disk->CTK;                         // replacing C with the current physical track number
            WRITEDW(&ctx->edi.ctrl->FDCResults[3], ctx->eax.e);    // to Results buffer
            goto label_ReturnSectorRWResults;         // and exit returning the error

        // is this the sector that we're searching for?
        case case_Read_CompareSectorID: label_Read_CompareSectorID:
            CMP(ctx, ctx->edi.ctrl->ReadMode, u765_FDCReadTrack);
            JE(ctx, label_Rd_TransferData);                     // transfer anyway if Read Track command

            ctx->eax.e = READDW(ctx->esi.u8);                 // ctx->eax.e = CHRN from sectorinfo
            CMP(ctx, ctx->eax.e, READDW(&ctx->edi.ctrl->FDCParameters[1])); // is this the sector we are looking for?
            JNE(ctx, label_SkipReadSector);

            // if this is a multisector read command then we ignore disk errors per sector
            CMP(ctx, ctx->edi.ctrl->MultiSectorRead, true);
            JNE(ctx, label_Rd_NotMS1);    // branch if not a multisector read cmd

            WRITEW(&ctx->edi.ctrl->ST1, 0x4000);
            goto label_Read_CheckDAM;

        // if we already have an error here then we ignore the DAM bit in the results phase
        // the DAM bit already in ST2 (from DSK file) is ignored in this test

        case case_Rd_NotMS1: label_Rd_NotMS1:
            XOR(ctx, ctx->eax.l, ctx->eax.l);
            ctx->edi.ctrl->ST2DAMBit = ctx->eax.l;
            OR(ctx, ctx->eax.l, ctx->edi.ctrl->ST1);
            OR(ctx, ctx->eax.l, ctx->edi.ctrl->ST2);
            AND(ctx, ctx->eax.l, 0x3f);
            JNE(ctx, label_Rd_IgnoreDAM);  // also need to avoid setting AT in ST0
            // fallthrough

        // now we have to verify the Data Address Marks (found in bit 6 of ST2),
        // DAM_Mask = 0 for ReadData and 64 for ReadDeletedData

        case case_Read_CheckDAM: label_Read_CheckDAM:
            ctx->eax.l = ctx->esi.u8[5];            // ST2 from sectorinfo
            AND(ctx, ctx->eax.l, 64);
            XOR(ctx, ctx->eax.l, ctx->edi.ctrl->DAM_Mask);
            JE(ctx, label_Rd_TransferData);      // DAM bits verify so we transfer the data

            // if DAM doesn't match then we either,
            // skip this sector if SK is set or,
            // return the Control Mark error (bit 6 in ST2 again) and transfer the data

            TEST(ctx, ctx->edi.ctrl->FDCCommandByte, 32);
            JNE(ctx, label_SkipReadSector);          // SK=1 so just skip this sector

            ctx->edi.ctrl->ST2DAMBit = 64;             // set ST2 Control Mark bit
            OR(ctx, ctx->edi.ctrl->ST0, 0x40);                  // abnormal termination of command
            // fallthrough

        case case_Rd_IgnoreDAM: label_Rd_IgnoreDAM:
            ctx->edi.ctrl->SectorToCPUReturn = case_Rd_Skip0;
            goto label_SectorDataToCPU;

        case case_Rd_Skip0: label_Rd_Skip0:
            OR(ctx, ctx->edi.ctrl->ST1, 0x80);                  // end of cylinder
            goto label_ReturnSectorRWResults;   // and exit returning the error

        // we have found the requested sector so transfer the sector data to the CPU
        case case_Rd_TransferData: label_Rd_TransferData:
            INC(ctx, ctx->edi.ctrl->SectorsRead);    // sector counter for Read Track command
            ctx->edi.ctrl->SectorToCPUReturn = case_LFRS_1;
            goto label_SectorDataToCPU;

        // check disk error fields for this sector
        case case_LFRS_1: label_LFRS_1:
            TEST(ctx, ctx->edi.ctrl->MainStatusReg, 0x20);  // execution mode ended? (overrun/lost data condition in status port read)
            JE(ctx, label_ReturnSectorRWResults);          // exit returning the error

            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorInfo;
            TEST(ctx, READW(&ctx->edi.ctrl->ST1), 0x2125);   // DE, ND or MA in ST1, DD or MD in ST2?
            JE(ctx, label_LFRS_3);

            ctx->edi.ctrl->ST0 = 0x40;                // AT
            OR(ctx, ctx->edi.ctrl->ST1, 0x80);                // end of cylinder

            CMP(ctx, ctx->edi.ctrl->ReadMode, u765_FDCReadTrack);
            JE(ctx, label_LFRS_3);
            goto label_ReturnSectorRWResults;

        // when R = EOT then we have read all requested sectors so return result bytes
        case case_LFRS_3: label_LFRS_3:
            ctx->eax.l = ctx->esi.u8[2];
            CMP(ctx, ctx->edi.ctrl->ReadMode, u765_FDCReadTrack);
            JNE(ctx, label_NotReadTrk1);

            // Read Track complete?
            ctx->eax.l = ctx->edi.ctrl->SectorsRead;
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[5]); // EOT
            JE(ctx, label_Read_Com1);  // terminate command

            // any more sectors to read on this track?
            CMP(ctx, ctx->eax.l, ctx->ebx.disk->TrackBlock.NumSectors);
            JC(ctx, label_LFRS_4);  // continue reading if sectors available

            // else sectors expired before reaching EOT sector count
            // in Read Track command

            // so we terminate the command and enter the Results stage
            // (perhaps set different error flags?)
            goto label_Read_Com1;

        case case_NotReadTrk1: label_NotReadTrk1:
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[5]); // EOT
            JNE(ctx, label_LFRS_4);
            // fallthrough

        case case_Read_Com1: label_Read_Com1:
            ctx->edi.ctrl->ST0 = 0x40;
            ctx->edi.ctrl->ST1 = 0x80;

            ctx->eax.e = READDW(ctx->esi.u8);             // copy CHRN
            ctx->eax.l = ctx->ebx.disk->CTK;
            WRITEDW(&ctx->edi.ctrl->FDCResults[3], ctx->eax.e);     // to Results buffer

            goto label_ReturnSectorRWResults;

        // else increment R parameter and search for the next sector to continue reading,
        // we search from the next physical sector (CSR is incremented by @InitReadSector)
        // each sector in a multi-sector transfer is treated as an entirely separate
        // sector read so we reset the IndexHoleCount back to zero again

        case case_LFRS_4: label_LFRS_4:
            INC(ctx, ctx->edi.ctrl->FDCParameters[3]);  // R parameter
            ctx->edi.ctrl->IndexHoleCount = 0;           // 1st revolution for this sector read
            goto label_InitReadSector;

        case case_SkipReadSector: label_SkipReadSector:
            CALL(ctx, case_AdvanceSectorPtrs);

            INC(ctx, ctx->ebx.disk->CSR);                // move to the next physical sector
            ctx->eax.l = ctx->ebx.disk->CSR;
            CMP(ctx, ctx->eax.l, ctx->ebx.disk->TrackBlock.NumSectors);
            JC(ctx, label_LocateReadSector);

            ctx->ebx.disk->CSR = -1;             // InitReadSector increments this to zero
            INC(ctx, ctx->edi.ctrl->IndexHoleCount);
            CMP(ctx, ctx->edi.ctrl->IndexHoleCount, 2);   // search for two disk revolutions
            JC(ctx, label_InitReadSector);

            ctx->ebx.disk->CSR = 0;

            // if we get this far then the sector could not be found
            ctx->edi.ctrl->CurrentSectorInfo -= 8;
            OR(ctx, ctx->edi.ctrl->ST1, 4);              // No Data
            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);
            OR(ctx, ctx->edi.ctrl->ST0, 0x40);      // AT
            ctx->eax.l = 1;               // maybe should be the first sector ID?
            ctx->edi.ctrl->FDCResults[5] = ctx->eax.l;   // set final R result
            goto label_ReturnSectorRWResults;

        // ######################################################################

        case case_SectorDataToCPU: label_SectorDataToCPU:
            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorInfo;

            ctx->ecx.l = ctx->edi.ctrl->FDCParameters[4]; // N value
            if (ctx->ecx.l > 8) {
                ctx->ecx.l = 8;                       // max N = 8 (32K sector)
            }

            if (ctx->ecx.l == 0) {
                // if N = 0 (use DTL bytes)
                XOR(ctx, ctx->ecx.e, ctx->ecx.e);
                ctx->ecx.l = ctx->edi.ctrl->FDCParameters[7]; // DTL from command
                if (ctx->ecx.l > 128) {
                    ctx->ecx.l = 128;                     // DTL max bytes = 128
                }
                ctx->edx.e = ctx->ecx.e;                            // ecx & edx = DTL bytes available
                ctx->edi.ctrl->DTL_BytesSent = true;
            }
            else {
                ctx->eax.e = 128;
                SHL(ctx, ctx->eax.e, ctx->ecx.l);
                ctx->ecx.e = ctx->eax.e;                            // ecx = physical sectorsize based on N value
                ctx->edx.e = ctx->ecx.e;                            // edx = bytes of available sector data
    
                if (ctx->ebx.disk->EDSK == true) {
                    ctx->edx.e = READW(&ctx->esi.u8[6]);       // edx = available sector data from EDsk sectorinfo
                }
            }

            if (ctx->ecx.e > 32768) {
                ctx->ecx.e = 32768;
            }

            ctx->edi.ctrl->PhysicalSectorSize = ctx->ecx.e;      // = (128 Shl N) or DTL bytes
            ctx->edi.ctrl->AvailableSectorData = ctx->edx.e;

            ctx->eax.l = ctx->esi.u8[1];                         // H
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[2]); // matching H params?
            JNE(ctx, label_SDTC_RandomData);

            ctx->eax.x = READW(&ctx->esi.u8[4]);                // fetch ST1 & ST2 from sectorinfo
            AND(ctx, ctx->eax.x, 0x2020);                           // mask data error bits for ST1 & ST2
            CMP(ctx, ctx->eax.x, 0x2020);                           // if both data error bits set
            JE(ctx, label_SDTC_RandomData);                    // then return a randomised sector

            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorData;
            goto label_SDTC_TransferData;

        case case_SDTC_RandomData: label_SDTC_RandomData:
            // ctx->ecx.e = physical sector size
            // ctx->edx.e = available sector data

            if (ctx->edx.e > ctx->ecx.e) {                   // multiple sector data available?
                //push    ctx->ecx.e
                //push    ctx->edx.e
                //invoke  IntDiv, ctx->edx.e, ctx->ecx.e    // get number of multiple sectors available
                //pop     ctx->edx.e
                //pop     ctx->ecx.e
                ctx->eax.e = ctx->edx.e / ctx->ecx.e;

                CMP(ctx, ctx->eax.e, 2);
                JC(ctx, label_SDTC_NormalRandom);  // normal random method if less than 2 sectors are available

                INC(ctx, ctx->edi.ctrl->MultipleSectorPick);
                if (ctx->edi.ctrl->MultipleSectorPick >= ctx->eax.e) {
                    ctx->edi.ctrl->MultipleSectorPick = 0;
                }

                //push    ctx->ecx.e
                //invoke  IntMul, ctx->edi.ctrl->MultipleSectorPick, ctx->ecx.e
                //pop     ctx->ecx.e
                ctx->eax.e = ctx->edi.ctrl->MultipleSectorPick * ctx->ecx.e;

                ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorData;
                ctx->esi.u8 += ctx->eax.e;
                goto label_SDTC_TransferData;
            }
            else {
                    // available sector data <= physical sector size
        case case_SDTC_NormalRandom: label_SDTC_NormalRandom:
                PUSH(ctx, ctx->edi);
                PUSH(ctx, ctx->ecx);
                ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorData;
                ctx->edi.u8 = &ctx->edi.ctrl->FDCRandomData[0];
                ctx->ecx.e = ctx->edx.e;                    // copy available sector data

                rep_movsb(ctx);
                ctx->ecx = POP(ctx);

                // if required, top up sector data with random bytes
                SUB(ctx, ctx->ecx.e, ctx->edx.e);    // ctx->ecx.e = top-up bytes required

                ctx->eax.l = ctx->edi.ctrl->FDCRandomSeed;
                ctx->eax.h = 3;
                if (ctx->edi.ctrl->DskRndMethod == 255) {
                    XOR(ctx, ctx->eax.x, ctx->eax.x);
                }

                while  (ctx->ecx.e > 0) {
                    *ctx->edi.u8 = ctx->eax.l;
                    ADD(ctx, ctx->eax.l, ctx->eax.h);
                    DEC(ctx, ctx->ecx.e);
                }
                ctx->edi.ctrl->FDCRandomSeed = ctx->eax.l;
                ctx->edi = POP(ctx);

                ctx->esi.u8 = &ctx->edi.ctrl->FDCRandomData[0];
                ctx->ecx.e = ctx->edi.ctrl->PhysicalSectorSize;

                // for N >= 6 sectors, transfer data now
                CMP(ctx, ctx->ecx.e, 8192);
                JNC(ctx, label_SDTC_TransferData);

                CMP(ctx, ctx->edi.ctrl->DskRndMethod, 255);
                JE(ctx, label_SDTC_TransferData);

                // else now we need different methods of setting the random byte in this data stream
                ctx->eax.l = ctx->edi.ctrl->FDCRandomSeed;
                ADD(ctx, ctx->eax.l, 0x03);
                ctx->edi.ctrl->FDCRandomSeed = ctx->eax.l;

                ctx->eax.h = ctx->edi.ctrl->DskRndMethod;
                if (ctx->eax.h == 0) {
                    // auto-sense random method
                    ctx->eax.h = 2;   // assume to randomise the first byte of sector data

                    // test for Dixon's Premiere Collection (disk 1)
                    if (READDW(ctx->esi.u8) == 0x1ce2ae94) {
                        CMP(ctx, READDW(&ctx->esi.u8[4]), 0x80a40824);
                        JE(ctx, label_SkipAutoSense);
                    }

                    // test for Dixon's Premiere Collection (disk 2)
                    if (READDW(ctx->esi.u8) == 0xaac6f5b5) {
                        CMP(ctx, READDW(&ctx->esi.u8[4]), 0x2a041840);
                        JE(ctx, label_SkipAutoSense);
                    }

                    // test for Hopping Mad
                    if (READDW(ctx->esi.u8) == 0x92831270) {
                        CMP(ctx, READDW(&ctx->esi.u8[4]), 0x9134d31);
                        JE(ctx, label_SkipAutoSense);
                    }

                    ctx->eax.h = 1;   // else randomise the final byte of sector data
                }

        label_SkipAutoSense:
                if (ctx->eax.h == 1) {
                    // randomise the final byte of sector data
                    AND(ctx, ctx->ecx.e, 0xffff);
                    ctx->esi.u8[ctx->ecx.e - 1] = ctx->eax.l;
                }
                else {
                    // randomise the first byte of sector data (Dixon's Premiere Collection, etc)
                    *ctx->esi.u8 = ctx->eax.l;
                }
            }
            // fallthrough

        case case_SDTC_TransferData: label_SDTC_TransferData:
            if (ctx->edi.ctrl->SectorsTransferred > 0) {
                FDCCommandCallback(ctx, 9);   // send a new callback for each successive sector read);
            }

            // transfer CX bytes from [ESI] to Z80
            ctx->edi.ctrl->UnitPtr = ctx->ebx.disk;          // preserve FDD Unit ptr
            ctx->edi.ctrl->FDCReturn = case_SectorDataToCPU_Done;
            goto label_FDC_SendData;               // transfer data to CPU

        case case_SectorDataToCPU_Done: label_SectorDataToCPU_Done:
            INC(ctx, ctx->edi.ctrl->SectorsTransferred);    // count number of sectors sent each command

            ctx->ebx.disk = ctx->edi.ctrl->UnitPtr;          // restore FDD Unit ptr
            ctx->eax.e = ctx->edi.ctrl->SectorToCPUReturn;
            JPREG(ctx, ctx->eax.e);

        // ######################################################################

        case case_AdvanceSectorPtrs: label_AdvanceSectorPtrs:
            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorInfo;
            ctx->eax.e = READW(&ctx->esi.u8[6]);           // size of sectordata in edsk
            CMP(ctx, ctx->ebx.disk->EDSK, true);
            JE(ctx, label_SkipSectorEDSK);
            ctx->eax.e = ctx->edi.ctrl->CurrentSectorSize;    // size of sectordata in dsk
            // fallthrough

        case case_SkipSectorEDSK: label_SkipSectorEDSK:
            ctx->edi.ctrl->CurrentSectorData += ctx->eax.e;
            ctx->edi.ctrl->CurrentSectorInfo += 8;
            return;

        // ######################################################################

        // disk writing is incomplete and will only work with standard
        // +3DOS single-sector writes and only to normal DSK files.

        case case_WriteSectorData: label_WriteSectorData:
            XOR(ctx, ctx->eax.l, ctx->eax.l);
            ctx->edi.ctrl->ST0 = ctx->eax.l;
            ctx->edi.ctrl->ST1 = ctx->eax.l;
            ctx->edi.ctrl->ST2 = ctx->eax.l;

            ctx->edi.ctrl->ST2DAMBit = ctx->eax.l;

            CALL(ctx, case_TrapStandardErrors);
            CMP(ctx, ctx->ebx.disk->WriteProtect, true);
            JNE(ctx, label_WSD_WProt);        // jump if not write-protected

            ctx->edi.ctrl->TSEError = true;     // force the error
            AND(ctx, ctx->edi.ctrl->ST0, 0x3f);
            OR(ctx, ctx->edi.ctrl->ST0, 0x40);           // AT
            OR(ctx, ctx->edi.ctrl->ST1, 2);             // write-protected
            // fallthrough

        case case_WSD_WProt: label_WSD_WProt:
            CMP(ctx, ctx->edi.ctrl->TSEError, true);
            JNE(ctx, label_WriteSectorData_1);

            ctx->eax.e = READDW(&ctx->edi.ctrl->FDCParameters[1]);   // copy C,H,R,N from command
            WRITEDW(&ctx->edi.ctrl->FDCResults[3], ctx->eax.e);      // into results buffer
            goto label_ReturnSectorRWResults;         // and exit returning the error

        case case_WriteSectorData_1: label_WriteSectorData_1:
            OR(ctx, ctx->edi.ctrl->MainStatusReg, 32 + 16);      // enter Execution mode + busy

            CALL(ctx, case_ReadCurrTrack);

            CALL(ctx, case_GetSectorSize);
            ctx->edi.ctrl->CurrentSectorSize = ctx->eax.e;                   // this sector's size (in bytes)

            PUSH(ctx, ctx->esi);
            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.SectorData[0];
            ctx->edi.ctrl->CurrentSectorData = ctx->esi.u8;     // ptr to this sector's data
            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.SectorInfoList[0];
            ctx->edi.ctrl->CurrentSectorInfo = ctx->esi.u8; // ptr to this sector's info
            ctx->esi = POP(ctx);
            ctx->edi.ctrl->CurrentSectorNumber = 0;
            // fallthrough

        case case_LocateFirstWriteSector: label_LocateFirstWriteSector:
            ctx->esi.u8 = ctx->edi.ctrl->CurrentSectorInfo;
            ctx->eax.l = ctx->esi.u8[2];          // sector ID (R)
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[3]); // is this the sector we are looking for?
            JC(ctx, label_SkipWriteSector);     // skip if not
            CMP(ctx, ctx->eax.l, ctx->edi.ctrl->FDCParameters[5]);
            JA(ctx, label_SkipWriteSector);     // skip if not

            ctx->edi.ctrl->CPUToSectorReturn = case_LFWS_1;
            goto label_CPUDataToSector;

        case case_LFWS_1: label_LFWS_1:
            ctx->eax.l = ctx->edi.ctrl->FDCParameters[4]; // N value
            CMP(ctx, ctx->eax.l, 0);
            JE(ctx, label_ReturnSectorRWResults);
            // fallthrough

        case case_SkipWriteSector: label_SkipWriteSector:
            ctx->edi.ctrl->CurrentSectorInfo += 8;
            ctx->eax.e = ctx->edi.ctrl->CurrentSectorSize;
            ctx->edi.ctrl->CurrentSectorData += ctx->eax.e;

            INC(ctx, ctx->edi.ctrl->CurrentSectorNumber);
            ctx->eax.l = ctx->edi.ctrl->CurrentSectorNumber;
            CMP(ctx, ctx->eax.l, ctx->ebx.disk->TrackBlock.NumSectors);
            JNE(ctx, label_LocateFirstWriteSector);

            goto label_ReturnSectorRWResults;

        case case_CPUDataToSector: label_CPUDataToSector:
            ctx->eax.l = ctx->edi.ctrl->FDCParameters[4]; // N value
            CMP(ctx, ctx->eax.l, 0);
            JNE(ctx, label_CDTS_1);

            XOR(ctx, ctx->ecx.x, ctx->ecx.x);
            ctx->ecx.l = ctx->edi.ctrl->FDCParameters[7]; // DTL from command
            goto label_CDTS_2;

        case case_CDTS_1: label_CDTS_1:
            ctx->ecx.e = ctx->edi.ctrl->CurrentSectorSize;
            // fallthrough

        case case_CDTS_2: label_CDTS_2:
            ctx->edx.u8 = ctx->edi.ctrl->CurrentSectorData;
            ctx->edi.ctrl->UnitPtr = ctx->ebx.disk;             // preserve FDD Unit ptr
            ctx->edi.ctrl->FDCReturn = case_CPUDataToSector_1;
            goto label_FDC_ReceiveData;    // receive new sector data from CPU

        case case_CPUDataToSector_1: label_CPUDataToSector_1:
            ctx->ebx.disk = ctx->edi.ctrl->UnitPtr;             // restore FDD Unit ptr
            CALL(ctx, case_WriteCurrTrack);
            // fallthrough

        case case_CPUDataToSector_Done: label_CPUDataToSector_Done:
            ctx->eax.e = ctx->edi.ctrl->CPUToSectorReturn;
            JPREG(ctx, ctx->eax.e);

        // ######################################################################

        case case_ReturnSectorRWResults: label_ReturnSectorRWResults:
            PUSH(ctx, ctx->eax);
            ctx->edx.u8 = &ctx->edi.ctrl->FDCResults[0];

            AND(ctx, ctx->edi.ctrl->ST0, 0xfb);
            ctx->eax.l = ctx->ebx.disk->CHEAD;
            SHL(ctx, ctx->eax.l, 2);
            OR(ctx, ctx->edi.ctrl->ST0, ctx->eax.l);

            AND(ctx, ctx->edi.ctrl->ST0, 0xfc);
            ctx->eax.l = ctx->edi.ctrl->FDCParameters[0];
            AND(ctx, ctx->eax.l, 3);
            OR(ctx, ctx->edi.ctrl->ST0, ctx->eax.l);            // insert unit number

            ctx->eax.l = ctx->edi.ctrl->ST0;      // ST0
            ctx->edx.u8[0] = ctx->eax.l;

            ctx->eax.l = ctx->edi.ctrl->ST1;
            ctx->edx.u8[1] = ctx->eax.l;

            ctx->eax.l = ctx->edi.ctrl->ST2;
            AND(ctx, ctx->eax.l, 0xbf);
            OR(ctx, ctx->eax.l, ctx->edi.ctrl->ST2DAMBit);
            ctx->edi.ctrl->ST2 = ctx->eax.l;
            ctx->eax.l = ctx->edi.ctrl->ST2;
            ctx->edx.u8[2] = ctx->eax.l;
            ctx->eax = POP(ctx);

            ctx->edi.ctrl->OverRunError = false;
            AND(ctx, ctx->edi.ctrl->MainStatusReg, 0xdf); // execution phase has ended and result phase has started
            ctx->edi.ctrl->FDCReturn = case_FDCBuff_ReturnSectorResults;
            ctx->esi.u8 = &ctx->edi.ctrl->FDCResults[0];
            ctx->ecx.x = 7;             // 7 bytes of result data
            ctx->edi.ctrl->NumResults = 7;
            goto label_FDC_SendData;    // transfer data to CPU

        case case_FDCBuff_ReturnSectorResults: label_FDCBuff_ReturnSectorResults:
            ctx->eax.e = ctx->edi.ctrl->FDCBufferReturn;
            JPREG(ctx, ctx->eax.e);

        // ######################################################################

        case case_SkipNextSector: label_SkipNextSector:
            XOR(ctx, ctx->eax.e, ctx->eax.e);
            ctx->eax.x = READW(&ctx->esi.u8[6]);     // size of EDSK sector data
            ctx->esi.u8 += 8;          // next SectorInfo entry in SectorInfoList

            CMP(ctx, ctx->ebx.disk->EDSK, true);
            JE(ctx, label_SkNxtSec1);

            CALL(ctx, case_GetSectorSize); // size of DSK sector data
            // fallthrough

        case case_SkNxtSec1: label_SkNxtSec1:
            ADD(ctx, ctx->ecx.e, ctx->eax.e);
            return;

        // ######################################################################

        case case_GetSectorSize: label_GetSectorSize:
            PUSH(ctx, ctx->ecx);
            ctx->ecx.l = ctx->ebx.disk->TrackBlock.SectorSize;
            ctx->eax.e = 128;
            SHL(ctx, ctx->eax.e, ctx->ecx.l);
            CMP(ctx, ctx->eax.e, 8192);
            JC(ctx, label_GSS_Exit);
            ctx->eax.e = 6144;
            // fallthrough

        case case_GSS_Exit: label_GSS_Exit:
            ctx->ecx = POP(ctx);
            return;

        // ######################################################################

        case case_ReadCurrTrack: label_ReadCurrTrack:
            CALL(ctx, case_LocateTrack);    // ctx->esi.u8 points to current track data

            PUSH(ctx, ctx->edi);
            ctx->edi.u8 = &ctx->ebx.disk->TrackBlock.TrackData[0];
            ctx->ecx.x = ctx->ebx.disk->DiskBlock.TrackSize;
            if (ctx->ecx.e > sizeof(u765_TrackInfoBlock)) {
                ctx->ecx.e = sizeof(u765_TrackInfoBlock);
            }

            rep_movsb(ctx);

            ctx->edi.u8 = &ctx->ebx.disk->TrackBlock.TrackData[0];
            ctx->esi.ptr = "Track-Info";
            ctx->edx.l = false;
            ctx->ecx.l = 10;
            // fallthrough

        case case_VTrk_Loop: label_VTrk_Loop:
            ctx->eax.l = *ctx->edi.u8;
            CMP(ctx, ctx->eax.l, *ctx->esi.u8);
            JNE(ctx, label_VTrk_Exit);
            INC(ctx, ctx->esi.u8);
            INC(ctx, ctx->edi.u8);
            DEC(ctx, ctx->ecx.l);
            JNZ(ctx, label_VTrk_Loop);
            ctx->edx.l = true;
            // fallthrough

        case case_VTrk_Exit: label_VTrk_Exit:
            ctx->edi = POP(ctx);
            ctx->edi.ctrl->ValidTrack = ctx->edx.l;
            return;

        // ######################################################################

        case case_WriteCurrTrack: label_WriteCurrTrack:
            ctx->ebx.disk->ContentsChanged = true;
            CALL(ctx, case_LocateTrack);    // ctx->esi.u8 points to current track data

            PUSH(ctx, ctx->edi);
            ctx->edi.u8 = ctx->esi.u8;         // edi=track data in FDDUnit0
            ctx->esi.u8 = &ctx->ebx.disk->TrackBlock.TrackData[0];
            ctx->ecx.x = ctx->ebx.disk->DiskBlock.TrackSize;
            AND(ctx, ctx->ecx.e, 0xffff);

            rep_movsb(ctx);

            ctx->edi = POP(ctx);
            return;

        // ######################################################################

        // sets ctx->esi.u8 to the start of the current track data in FDDUnit0 array
        case case_LocateTrack: label_LocateTrack:
            XOR(ctx, ctx->eax.e, ctx->eax.e);
            XOR(ctx, ctx->edx.e, ctx->edx.e);
            ctx->edx.x = ctx->ebx.disk->DiskBlock.TrackSize;
            AND(ctx, ctx->edx.e, 65535);
            ctx->ecx.l = ctx->ebx.disk->CTK;      // current physical track head is over
            INC(ctx, ctx->ecx.l);
            // fallthrough

        case case_LocTrk1: label_LocTrk1:
            DEC(ctx, ctx->ecx.l);
            JE(ctx, label_LockTrkDone);
            ADD(ctx, ctx->eax.e, ctx->edx.e);
            goto label_LocTrk1;

        case case_LockTrkDone: label_LockTrkDone:
            CMP(ctx, ctx->ebx.disk->DiskBlock.NumSides, 2);
            JNE(ctx, label_LocSingleSide);
            SHL(ctx, ctx->eax.e, 1);

            CMP(ctx, ctx->ebx.disk->CHEAD, 1);
            JNE(ctx, label_LocSingleSide);
            ADD(ctx, ctx->eax.e, ctx->edx.e);                  // add DiskBlock.TrackSize to skip to the interleaved track
            // fallthrough

        case case_LocSingleSide: label_LocSingleSide:
            ADD(ctx, ctx->eax.e, 0x100);    // and add the sizeof DiskInfoBlock
            ctx->esi.u8 = ctx->ebx.disk->DiskArrayPtr;
            ctx->esi.u8 += ctx->eax.e;
            return;
    }
}

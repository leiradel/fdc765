#include <fdc765.h>

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define WIN32_LEAN_AND_MEAN
#include <Windows.h>

static HMODULE dll = NULL;
static u765_Controller* fdc = NULL;
static u765_Controller* (__stdcall *u765_Initialise_Func)(void) = NULL;
static void (__stdcall *u765_Shutdown_Func)(u765_Controller* fdc) = NULL;
static void (__stdcall *u765_ResetDevice_Func)(u765_Controller* fdc) = NULL;
static void (__stdcall *u765_InsertDisk_Func)(u765_Controller* fdc, char const* filename, uint8_t unit) = NULL;
static void (__stdcall *u765_EjectDisk_Func)(u765_Controller* fdc, uint8_t unit) = NULL;
static bool (__stdcall *u765_GetMotorState_Func)(u765_Controller* fdc) = NULL;
static void (__stdcall *u765_SetMotorState_Func)(u765_Controller* fdc, uint8_t motor_state) = NULL;
static uint8_t (__stdcall *u765_StatusPortRead_Func)(u765_Controller* fdc) = NULL;
static uint8_t (__stdcall *u765_DataPortRead_Func)(u765_Controller* fdc) = NULL;
static void (__stdcall *u765_DataPortWrite_Func)(u765_Controller* fdc, uint8_t fdc_data_out) = NULL;
static void (__stdcall *u765_SetActiveCallback_Func)(u765_Controller* fdc, void (*callback)(void)) = NULL;
static void (__stdcall *u765_SetCommandCallback_Func)(u765_Controller* fdc, void (*callback)(uint8_t const*, uint8_t)) = NULL;
static bool (__stdcall *u765_DiskInserted_Func)(u765_Controller* fdc, uint8_t unit) = NULL;
static void (__stdcall *u765_SetRandomMethod_Func)(u765_Controller* fdc, uint8_t rndmethod) = NULL;
static void (__stdcall *u765_GetFDCState_Func)(u765_Controller* fdc, u765_State* state) = NULL;

static char const* dll_error(void) {
	static char msg[512];

	DWORD err = GetLastError();

	DWORD res = FormatMessage(
		FORMAT_MESSAGE_FROM_SYSTEM,
		NULL,
		err,
		MAKELANGID(LANG_ENGLISH, SUBLANG_DEFAULT),
		msg,
		sizeof(msg) - 1,
		NULL
	);

	if (res == 0) {
		snprintf(msg, sizeof(msg) - 1, "Error %lu", err);
		msg[sizeof(msg) - 1] = 0;
	}

	return msg;
}

#define GETSYM(n) \
	do { \
		void* ptr = (void*)GetProcAddress(dll, #n); \
		if (ptr == NULL) { \
			printf("GetProcAddress: %s\n", dll_error()); \
			FreeLibrary(dll); \
			return 0; \
		} \
		memcpy(&n ## _Func, &ptr, sizeof(n ## _Func)); \
	} \
	while (0)

static int load_dll(char const* path) {
	dll = LoadLibrary(path);

	if (dll == NULL) {
		printf("LoadLibrary: %s\n", dll_error());
		return 0;
	}

	GETSYM(u765_Initialise);
	GETSYM(u765_Shutdown);
	GETSYM(u765_ResetDevice);
	GETSYM(u765_InsertDisk);
	GETSYM(u765_EjectDisk);
	GETSYM(u765_GetMotorState);
	GETSYM(u765_SetMotorState);
	GETSYM(u765_StatusPortRead);
	GETSYM(u765_DataPortRead);
	GETSYM(u765_DataPortWrite);
	GETSYM(u765_SetActiveCallback);
	GETSYM(u765_SetCommandCallback);
	GETSYM(u765_DiskInserted);
	GETSYM(u765_SetRandomMethod);
	GETSYM(u765_GetFDCState);

	return 1;
}

enum {
	u765_Initialise_Command,
	u765_InsertDisk_Command,
	u765_EjectDisk_Command,
	u765_SetMotorState_Command,
	u765_StatusPortRead_Command,
	u765_DataPortRead_Command,
	u765_DataPortWrite_Command,
};

static uint8_t const commands[] = {
#include "replay.inl"
};

static int run(void) {
	static char const* const disks[] = {
		"D:\\test\\CP-M Plus v1.0 (19xx)(Locomotive Software)(Side 1).dsk",
		"D:\\test\\ghosts_speccy.dsk"
	};

	size_t i = 0;

	for (i = 0; i < sizeof(commands); i += 2) {
		uint8_t res = 0;

		if (i == 0x90) __debugbreak();

		switch (commands[i]) {
		case u765_Initialise_Command:
			fdc = u765_Initialise_Func();
			printf("%zu u765_Initialise() -> %p\n", i / 2 + 1, fdc);
			assert(fdc != NULL);
			break;

		case u765_InsertDisk_Command:
			u765_InsertDisk_Func(fdc, disks[commands[i + 1]], 0);
			printf("%zu u765_InsertDisk(\"%s\", 0)\n", i / 2 + 1, disks[commands[i + 1]]);
			break;

		case u765_EjectDisk_Command:
			u765_EjectDisk_Func(fdc, commands[i + 1]);
			printf("%zu u765_EjectDisk(%u)\n", i / 2 + 1, commands[i + 1]);
			break;

		case u765_SetMotorState_Command:
			u765_SetMotorState_Func(fdc, commands[i + 1]);
			printf("%zu u765_SetMotorState(0x%02x)\n", i / 2 + 1, commands[i + 1]);
			break;

		case u765_StatusPortRead_Command:
			res = u765_StatusPortRead_Func(fdc);
			printf("%zu u765_StatusPortRead() -> 0x%02x\n", i / 2 + 1, res);
			assert(res == commands[i + 1]);
			break;

		case u765_DataPortRead_Command:
			res = u765_DataPortRead_Func(fdc);
			printf("%zu u765_DataPortRead() -> 0x%02x\n", i / 2 + 1, res);
			assert(res == commands[i + 1]);
			break;

		case u765_DataPortWrite_Command:
			u765_DataPortWrite_Func(fdc, commands[i + 1]);
			printf("%zu u765_DataPortRead(0x%02x)\n", i / 2 + 1, commands[i + 1]);
			break;
		}
	}

	printf("done!\n");
	assert(i == sizeof(commands));
	return EXIT_SUCCESS;
}

int main(int argc, char const* argv[]) {
	if (argc != 2) {
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "old") == 0) {
		if (!load_dll("D:\\git\\Zero-Emulator\\run\\fdc765.dll")) {
			return EXIT_FAILURE;
		}
	}
	else {
		if (!load_dll("D:\\git\\Zero-Emulator\\run\\fdc765_new.dll")) {
			return EXIT_FAILURE;
		}
	}

	return run();
}

/*
 *
 * drvEK9000.h
 *
 * Driver for the EK9000 EtherCAT to Modbus coupler
 *
 *
 * General Info:
 *
 * --- INSTIO STRING FORMAT ---
 *  	@ek9k_name,slave_num,channel_num
 */ 
#pragma once

#include <drvModbusAsyn.h>
#include <epicsAtomic.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsMutex.h>
#include <epicsTypes.h>
#include <epicsSpin.h>
#include <deque>

/* Common EK9000 registers */
#define STATUS_REGISTER_BASE 	0x1000 
#define BUS_COUPLER_ID_REG 		0x1000 
#define BUS_COUPLER_ID_REG_END	0x1006
#define PDO_SIZE_AO_REG			0x1010
#define PDO_SIZE_AI_REG			0x1011
#define PDO_SIZE_BO_REG			0x1012
#define PDO_SIZE_BI_REG 		0x1013
#define WDT_CUR_TIME_REG		0x1020
#define NUM_FALLBACKS_REG		0x1021
#define NUM_TCP_CONN_REG		0x1022
#define HARDWARE_VER_REG		0x1030
#define SOFT_VER_MAIN_REG		0x1031
#define SOFT_VER_SUBMAIN_REG	0x1032
#define SOFT_VER_BETA_REG		0x1033
#define SERIAL_NUM_REG			0x1034
#define MFG_DAY_REG				0x1035
#define MFG_MONTH_REG			0x1036
#define MFG_YEAR_REG			0x1037
#define EBUS_STAT_REG			0x1040
#define WDT_TIME_REG			0x1120
#define WDT_RESET_REG			0x1121
#define WDT_TYPE_REG			0x1122
#define FALLBACK_MODE_REG		0x1123
#define WRITELOCK_REG			0x1124
#define EBUS_CTRL_REG			0x1140
#define FIRST_STATUS_REGISTER	BUS_COUPLER_ID_REG 
#define LAST_STATUS_REGISTER 	EBUS_CTRL_REG 

/* Slave mapping register */
#define EK9K_SLAVE_MAP_REG		0x6001

/* The control registers */
#define REG_0x1400				0x1400
#define REG_0x1401				0x1401
#define REG_0x1402 				0x1402
#define REG_0x1403 				0x1403
#define REG_0x1404 				0x1404
#define REG_0x1405 				0x1405
#define REG_DATA_START			0x1406
#define REG_DATA_END 			0x14FF 

#define INPUT_PDO_START 		0x1
#define OUTPUT_PDO_START 		0x800
#define INPUT_COIL_START 		0x0
#define OUTPUT_COIL_START	 	0x0

#define INPUT_REG_SIZE			0xFF
#define OUTPUT_REG_SIZE			0xFF
#define INPUT_BIT_SIZE			0xFF
#define OUTPUT_BIT_SIZE			0xFF 

#define EK9K_EXEC_BIT 			0x1
#define EK9K_ERROR_BIT			0x100
#define EK9K_BUSY_BIT			0x200
#define EK9K_DONE_BIT			0x400

#define EK9K_TERMINAL_NUM_MASK	0x4FFF
#define EK9K_WRITE_MASK			0x8000

typedef struct
{
	int length;
	uint16_t* pdata;
	int adserr;
} coe_resp_t;

#define COE_REQ_READ 0
#define COE_REQ_WRITE 1

/**
 *
 * Example read request:
 * 	coe_req_t req = {
 *		.index = 0x8010,
 *		.subindex = 0x20,
 *		.length = 4,
 *		.type = COE_REQ_READ,
 *		.pdata = mybuffer,
 *		.pfnCallback = mycallback,
 *		.pvt = NULL,
 * 	};
 * 	ek9000->RequestCoEIO(req);
 *
 */ 
typedef struct
{
	/* Length is in registers */
	int subindex, index, length, type, termid;

	/*
	 * This pointer will be reused in the response for any returned
	 * data
	 */ 
	uint16_t* pdata;

	/*
	 * Pointer to callback for completion
	 */ 
	void(*pfnCallback)(void*, coe_resp_t);

	/* pvt is private data you can attach to this for the callback */
	void* pvt;
} coe_req_t;

/**
 * Simple terminal structure to hold basic info about each terminal
 * these can be accessed using a terminal index
 */ 
typedef struct
{
	int id;
	int in_start, out_start, inb_start, outb_start;
	int in_size, out_size, inb_size, outb_size;

	enum {
		AI, AO, BI, BO, ENC, MOTOR
	} type;
} terminal_t;


/*
 *	drvEK9000
 *
 * This is a simple driver for the EK9000 bus coupler
 * This class itself is just tasked with reading all of the registers from the device
 * quickly. Device drivers then atomically read register values from the exposed 
 * PDOs. Mutex locking is minimal here.
 *
 */ 
class drvEK9000 : public drvModbusAsyn
{
protected:
	/* Internal function for CoE IO execution
	 * Returns the time taken for the request in ms */
	bool doCoEIO(coe_req_t req);

public:
	drvEK9000(const char* name, const char* port, const char* ipport, const char* ip);
	~drvEK9000();

	/**
	 * called to map terminals 
	 * coupler must be connected in order for this to work
	 */ 
	void MapTerminals();

	void PopulateSlaveList();
	void StartPollThread();

	/** 
	 * Forcefully performs CoE IO
	 * Not thread-safe
	 * rw = true means write
	 * rw = false means read
	 * bytesize is the size in bytes of the read/write payload
	 * pbuf should point to a VALID buffer to either receive or write data. It's size should == bytesize. Be careful of buffer overruns
	 */ 
	bool doCoEIO(bool rw, epicsUInt16 termid, epicsUInt16 index, epicsUInt16 subindex, epicsUInt16 bytesize, epicsUInt16* pbuf);
	
	/*
	 * Enqueues a CoE IO request with the driver
	 * this can be requested to be "immediate" or not
	 * The pfnCallback 
	 * Requests are served in the order in which they're enqueued; 
	 */ 
	void RequestCoEIO(coe_req_t req, bool immediate=false);
	void RequestCoEIO(coe_req_t* req, int nreq, bool immediate=false);

	/* Atomically read a register,
	 * type 0 = bo, 1 = bi, 2 = ao, 3 = ai */
	uint16_t ReadRegisterAtomic(int addr, int type);

	static void PollThreadFunc(void* lparam);

	/**
	 * DumpXXX functions are used to print debugging info to the ioc console
	 */ 
	void DumpTerminalMapping();
	void DumpStats();
	void DumpInfo();
	void DumpEverything();

public:
	const char* port, *ipport, *ip, *name;

	/* Queue of all the CoE IO requests, guarded by mutex */
	std::deque<coe_req_t> coeRequests;
	epicsSpinId coeMutex;

	/* Input/Output pdo sizes */
	int outputBytes, inputBytes, inputBits, outputBits;

	/* Input/output register space */
	uint16_t inputPDO[INPUT_REG_SIZE];
	uint16_t outputPDO[OUTPUT_REG_SIZE];

	/* Input/output bit register space */
	bool inputBitPDO[INPUT_BIT_SIZE];
	bool outputBitPDO[OUTPUT_BIT_SIZE];

	/* Status register space */
	epicsUInt16 statusRegisters[LAST_STATUS_REGISTER-FIRST_STATUS_REGISTER];

	/* Register swap spaces */
	/* These spaces will be sent over the wire, and should not be touched 
	 * by any device code. They are temporary holding buffers */
	uint16_t inputSwapSpace[INPUT_REG_SIZE];
	uint16_t outputSwapSpace[OUTPUT_REG_SIZE];
	uint16_t inputBitSwap[INPUT_BIT_SIZE];
	uint16_t outputBitSwap[OUTPUT_BIT_SIZE];

	/* Mutex for swapping of input/output spaces */
	/* Ideally a spinlock is used here since we're only going to be locking for a couple us */
	epicsSpinId swapMutex;
	 
	/* Polling thread handle */
	epicsThreadId pollThread;

	/* Poll period, we sleep for this many seconds */
	double pollPeriod;

	/* Previous time of last poll, in ms */
	double prevTime;

	/**
	 * Internal list of terminals
	 * you will be able to get info about each terminal mapping here.
	 * These are statically allocated because we've only got a max of 255 terminals
	 */ 
	terminal_t terminals[0xFF];
	int num_terminals;
};



extern "C"
{
#include "main.h"
}

#include "Command.h"
#include <cstring>

#ifdef USE_SERIAL
#include "Serial.h"

#define SERIAL_BUFFER_SIZE 50
static char bufferOutConsole[SERIAL_BUFFER_SIZE];
static char bufferInConsole[SERIAL_BUFFER_SIZE];

static SerialOutput SerialOutToConsole(&huart2, bufferOutConsole, SERIAL_BUFFER_SIZE); 	 // via USB
static SerialInput SerialInFromConsole(&huart2, bufferInConsole, SERIAL_BUFFER_SIZE);

static SerialOutput* pSerialOutToConsole;
static SerialInput* pSerialInFromConsole;

#endif


void initializeCommand()
{
#ifdef USE_SERIAL
	pSerialOutToConsole = &SerialOutToConsole;
	pSerialInFromConsole = &SerialInFromConsole;
	pSerialInFromConsole->initialize(pSerialOutToConsole);
	pSerialOutToConsole->puts("\r\nReady\r\n");
#endif

}

void sendSerial(const char* message)
{
	pSerialOutToConsole->puts(message);
}


char * my_itoa(int n, int maxVal = 100000)
{
	static char message[11];
	int leadingZeros = 0;
	bool negative = false;
	char d;
	int i = 0;
	char nextC = 0;
	message[0] = 0;
	if (n < 0) {
		negative = true;
		n = -n;
	}
	while (maxVal > 9 && (d = n / maxVal) == 0) {
		leadingZeros++;
		maxVal /= 10;
	}
	while (i < leadingZeros - 1) {
		message[i++] = ' ';
	}
	if (negative != 0) {
		message[i++] = '-';
	} else {
		message[i++] = ' ';
	}
	while (maxVal != 0) {
		d = '0' + n / maxVal;
		message[i++] = d;
		n = n % maxVal;
		maxVal = maxVal / 10;
	}
	message[i + 1] = 0;
	return &message[0];
}



#define STAT_BUFFER_SIZE 5000
uint16_t statBuffer[STAT_BUFFER_SIZE];
static int nextStat = 0;
#define PRE_TRIGGER_COUNT_INIT 10000
static int preTriggerCount = PRE_TRIGGER_COUNT_INIT;
#define TRIGGER_COUNT_INIT 10000
static int triggerCount = TRIGGER_COUNT_INIT;
bool dumpDone = false;

void doResetBuffer(void)
{
	for (int i = 0; i < STAT_BUFFER_SIZE;i++){
		statBuffer[i] = 0;
	}
}

void doDumpStats(void)
{
	int count_0 = 0;
	for (int i = 0; i < STAT_BUFFER_SIZE; i++) {
		if (statBuffer[i] == 0) {
			count_0++;
		} else {
			if (count_0 < 5) {
				while (count_0-- > 0) {
					sendSerial(my_itoa(0));
					sendSerial("\r\n");
				}
			} else {
				sendSerial(my_itoa(count_0));
				sendSerial(" zeros\r\n");
			}
			sendSerial(my_itoa(i));
			sendSerial("\t");
			sendSerial(my_itoa(statBuffer[i]));
			sendSerial("\r\n");
			count_0 = 0;
		}
	}
	if (count_0 > 0) {
		sendSerial(my_itoa(count_0));
		sendSerial(" zeros\n");
	}
}

void doDumpRecords(void)
{
	for (int i = 0; i < STAT_BUFFER_SIZE; i++) {
		sendSerial(my_itoa(statBuffer[i]));
		sendSerial("\n");
	}
}
void doRecord(uint16_t val)
{
	if (preTriggerCount == PRE_TRIGGER_COUNT_INIT){
		doResetBuffer();
	}
	if (preTriggerCount > 0) {
		preTriggerCount--;
		return;
	}
	if (nextStat >= STAT_BUFFER_SIZE) {
		if (!dumpDone) {
			doDumpRecords();
			dumpDone = true;
		}
		return;
	}
	statBuffer[nextStat++] = val;
}

void doStat(uint16_t val)
{
	if (preTriggerCount == PRE_TRIGGER_COUNT_INIT) {
		triggerCount = TRIGGER_COUNT_INIT;
		dumpDone = false;
		doResetBuffer();
	}
	if (preTriggerCount > 0) {
		preTriggerCount--;
		return;
	}
	if (triggerCount > 0) {
		triggerCount--;
		statBuffer[val]++;
		return;
	}
	if (!dumpDone) {
		doDumpStats();
		dumpDone = true;
	}
}

void peekProcessCommand()
{
	char strConsole[SERIAL_BUFFER_SIZE];
	if (pSerialInFromConsole->fgetsNonBlocking(strConsole, 48)) {
		int len = strlen(strConsole);
		if (len > 0) {
			preTriggerCount = PRE_TRIGGER_COUNT_INIT;
		}
	}
}

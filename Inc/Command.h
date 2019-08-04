#pragma once


void sendSerial(const char* message);
void peekProcessCommand(void);
char * my_itoa(int n, int maxVal);


#ifdef __cplusplus
extern "C"
{
#endif
	void initializeCommand();
	void doRecord(uint16_t val);
	void doStat(uint16_t val);
#ifdef __cplusplus
}
#endif
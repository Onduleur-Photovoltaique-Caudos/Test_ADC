extern "C"
{
#include "main.h"
}

//#include "gpio.h"
#include "Loop.h"
#include "Command.h"


void doLoop(void)
{
	int count = 0;
	char message[100];
	while (true) {
		//doNextWaveformSegment();
#ifdef USE_SERIAL	
		peekProcessCommand();
#endif
		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
		HAL_Delay(1);
		count++;
	}
}

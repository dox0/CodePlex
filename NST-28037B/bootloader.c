#include "NST-28037B.h"

void BootLoader(void)
{
	
	NVIC_SystemReset();
	while(1) { }
}


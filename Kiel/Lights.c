#include <stdint.h>
#include "Lights.h"
#include "tm4c1294ncpdt.h"
#include "SysTick.h"

void LED0(uint8_t on){													
	if(on)
		GPIO_PORTN_DATA_R |= 0x02;
	else
		GPIO_PORTN_DATA_R &= ~0x02;
}

void LED2(uint8_t on){													
	if(on)
		GPIO_PORTF_DATA_R |= 0x10;
	else
		GPIO_PORTF_DATA_R &= ~0x10;
}

void LED3(uint8_t on){													
	if(on)
		GPIO_PORTF_DATA_R |= 0x01;
	else
		GPIO_PORTF_DATA_R &= ~0x01;
}

void LEDs(uint8_t on){													
	if(on){
		GPIO_PORTN_DATA_R |= 0x03;	
		GPIO_PORTF_DATA_R |= 0x11;
	}
	else{
		GPIO_PORTN_DATA_R &= ~0x03;
		GPIO_PORTF_DATA_R &= ~0x11;
	}
}
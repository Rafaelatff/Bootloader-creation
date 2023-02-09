# Bootloader-creation
This repository was create to follow the content of the course 'STM32Fx Microcontroller Custom Bootloader Development'.  
* Board: NUCLEO-F401RE 
* MCU: STM32F401RET6U

## STM32CubeMX

I created a STM32 project using the Cube MX.

Configured the UART2 peripheral (already configured when using the board default setup).

![image](https://user-images.githubusercontent.com/58916022/217803018-25898c32-6819-47f8-9536-f5811d6d2714.png)

Then I also configured the UART1.

![image](https://user-images.githubusercontent.com/58916022/217803120-8d55359c-401d-4745-b0d2-a7f55e77a144.png)

And activated the CRC in 'Compution' tab.

![image](https://user-images.githubusercontent.com/58916022/217803394-4e4f56e4-d3b7-41d8-8dc6-0e55234a6979.png)

Default clock configuration is enought for our application. We keep the rest that way and created the project.

## Code

The following data was generated to test the UART2:

```c
	 #include <string.h> // to use strlen, add in begin of code
	  char somadata[] = "Hello from bootloader\r\n"; // add in begin of code
    // next add inside 'main' -> 'while(1)' loop:
	  HAL_UART_Transmit(&huart2, (uint8_t*)somadata, sizeof(somadata), HAL_MAX_DELAY);
	  uint32_t current_tick = HAL_GetTick();
	  while(HAL_GetTick() <= (current_tick + 500));
```

It doesn't show the hole text:

![image](https://user-images.githubusercontent.com/58916022/217812118-da8da3c7-c498-40d0-80ea-ba0b418a1dcd.png)

Solution can be found in this [other rep](https://github.com/Rafaelatff/_HAL-STM32F401-UART).

Just copy the line for the 'HAL_UART_Transmit' and changed to '&huart1'. Also used the TeraTerm but now connected to USB Serial Port (It uses PA9 and PA10 pins).

![image](https://user-images.githubusercontent.com/58916022/217814825-6cb9fea6-2c04-4aad-9ced-132136750869.png)

Add the following code to the 'main.c' file:

´´´c
#include <stdarg.h> // at begin
#define BL_DEBUG_MSG_EN
void printmsg(char *format,...); // prototype of the function
#define D_UART &huart1
#define C_UART &huart2

printmsg("current_tick = %d\r\n", current_tick); // instead of HAL functions

´´´

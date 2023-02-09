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
	  char somadata[] = "Hello from bootloader\r\n"; // add in begin of code
    // next add inside 'main' -> 'while(1)' loop:
	  HAL_UART_Transmit(&huart2, (uint8_t*)somadata, sizeof(somadata), HAL_MAX_DELAY);
	  uint32_t current_tick = HAL_GetTick();
	  while(HAL_GetTick() <= (current_tick + 500));
```


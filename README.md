# Bootloader-creation
This repository was create to follow the content of the course 'STM32Fx Microcontroller Custom Bootloader Development'.  
* Board: NUCLEO-F401RE 
* MCU: STM32F401RET6U

## STM32CubeMX

I created a STM32 project using the Cube MX. Configured the UART2 peripheral (already configured when using the board default setup). Then I also configured the UART1.

And activated the CRC in 'Compution' tab, as showed next:

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

Solution can be found in this [other rep](https://github.com/Rafaelatff/_HAL-STM32F401-UART). Sadly it didn't worked.

Also, at bottom of '' we add the NVIC configuration (still not working). See following code for NVIC:

```c
    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0); //changed to 15, 15
```
NOTE: I am no longer using BOOT0 and BOOT1, I can disconnect those pins now. After I did it, it worked perfectly!

Just copy the line for the 'HAL_UART_Transmit' and changed to '&huart1'. Also used the TeraTerm but now connected to USB Serial Port (It uses PA9 and PA10 pins).

![image](https://user-images.githubusercontent.com/58916022/217814825-6cb9fea6-2c04-4aad-9ced-132136750869.png)

Add the following code to the 'main.c' file:

```c
#include <stdarg.h> // at begin
#define BL_DEBUG_MSG_EN
void printmsg(char *format,...); // prototype of the function
#define D_UART &huart1
#define C_UART &huart2

printmsg("current_tick = %d\r\n", current_tick); // instead of HAL functions
```
Also, the content in 'main' -> 'while(1)' were changed to:

```c
	  if (HAL_GPIO_ReadPin(B1_GPIO_Port,B1_Pin) == GPIO_PIN_RESET){ // NUCLEO is low when pressed
	  	printmsg ("BL_DEBUG_MSG:Button is pressed .. going to BL mode\n\r");
	  	// we should continue in bootloader mode
	  	bootloader_uart_read_data();
	  } else {
	  	printmsg("BL_DEBUG_MSG: Button is not pressed .. executing user app\n\r");
	  	bootloader_jump_to_user_app();
	  }
```
 Button pressed when initializing?
* Y: bootloader_uart_read_data()
* N: bootloader_jump_to_user_app()

The function prototypes were added on 'main.h' file.

Then, we create with STM32CubeMX a new program called 'user_app_F401', we config the IT for the B1 button.  This program will also send data througt the UART to inform that program is in user_application.

After creating the code we need to configure the memory address to allocate this code. The file that has this function can be found in the file's tree.

![image](https://user-images.githubusercontent.com/58916022/218113272-e0027dc1-acdb-484b-9e94-29231acb9291.png)

* STM32F401RETX_FLASH.ld shows the code placement inside the flash memory
* STM32F401RETX_RAM.ld shows the code placement inside the RAM memory

According to RM0368 file (reference manual), the main memory (its a part of the flash memory) has several sectores. We will program the user application inside sector 2. The address can be seen in the following table/image.

![image](https://user-images.githubusercontent.com/58916022/218114749-5e060a20-1363-44a0-a274-9cf89fb318c3.png)

Open the file with notepad++ or even inside the IDE and go to 'Specify the memory areas' (in our case 'Memories definition') and change the start flash address.

![image](https://user-images.githubusercontent.com/58916022/218115487-ac68f9ed-f2ad-46b2-8869-9a762104ba59.png)

It's possible to erase the memory using the STM32 ST-LINK utility ([replaced by STM32CubeProgrammer](https://www.st.com/en/development-tools/stsw-link004.html)) and debug the code using the Memory monitor to check where the code is being alocated.

At this point, we have a Vector Table in bootloader memory and a different Vector Table inside the user application.

Vector table (0x0000 0000 address) is aliased to the bootloader VT. When entering the user application we need to infor (do the relocation) the VTOR (Vector table offset register). VTOR will be 0x0800 8000.

Then we implemented our 'bootloader_jump_to_user_app' function. All the macros were added to 'main.h' file.

```c
/*	Code to jump to user application
 *	Here we are assuming FLASH_SECTOR2_BASE_ADDRESS
 *	is where the user application is stored
 */
void bootloader_jump_to_user_app(void){

   //just a function pointer to hold the address of the reset handler of the user app.
    void (*app_reset_handler)(void);

    printmsg("BL_DEBUG_MSG:bootloader_jump_to_user_app\n");


    // 1. configure the MSP by reading the value from the base address of the sector 2
    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    	// we dereference the value *(x) to read its content
    printmsg("BL_DEBUG_MSG:MSP value : %#x\n",msp_value);

    //This function comes from CMSIS.
    __set_MSP(msp_value);

    //SCB->VTOR = FLASH_SECTOR1_BASE_ADDRESS;

    /* 2. Now fetch the reset handler address of the user application
     * from the location FLASH_SECTOR2_BASE_ADDRESS+4 (4 bytes = 32-bit)
     */
    uint32_t resethandler_address = *(volatile uint32_t *) (FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*) resethandler_address;

    printmsg("BL_DEBUG_MSG: app reset handler addr : %#x\n",app_reset_handler);

    //3. jump to reset handler of the user application
    app_reset_handler();
}
```

When the code goes to 'app_reset_handler();' and jumps from 'Bootloader-F401RE' to 'user_app_F401RE' program, it does all the initialization by going to 'system_stm32Fxx.c' source code. Inside 'SystemInit' function we have a place to configure the Vector Table location. Note that we have to #define USER_VECT_TAB_ADDRESS before use it.

![image](https://user-images.githubusercontent.com/58916022/218123694-e4c1b105-89a1-40f9-abf6-64b891e58490.png)

And, we need to add/change the macro for the VECT_TAB_OFFSET to match our new Vector Table address. 

In Udemy course, for the STM32 board of the instructor, initialization code showes two options for the VTOR address, being SRAM or Flash. (Investigate it better). New value is a sum of the FLASH_ADDR + offset.

Both codes were programmed in the NUCLEO board. When running, user code starts. When turn on the NUCLEO board holding the B1 at the same time, code stays inside bootloader.

In 'main.h' file, we added the macros and functions prototypes.
```c
/* USER CODE BEGIN EFP */
void bootloader_uart_read_data(void);
void bootloader_jump_to_user_app(void);

void bootloader_handle_getver_cmd(uint8_t *bl_rx_buffer);
void bootloader_handle_gethelp_cmd(uint8_t *pBuffer);
void bootloader_handle_getcid_cmd(uint8_t *pBuffer);
void bootloader_handle_getrdp_cmd(uint8_t *pBuffer);
void bootloader_handle_go_cmd(uint8_t *pBuffer);
void bootloader_handle_flash_erase_cmd(uint8_t *pBuffer);
void bootloader_handle_mem_write_cmd(uint8_t *pBuffer);
void bootloader_handle_en_rw_protect(uint8_t *pBuffer);
void bootloader_handle_mem_read (uint8_t *pBuffer);
void bootloader_handle_read_sector_protection_status(uint8_t *pBuffer);
void bootloader_handle_read_otp(uint8_t *pBuffer);
void bootloader_handle_dis_rw_protect(uint8_t *pBuffer);
/* USER CODE END EFP */

/* USER CODE BEGIN Private defines */
#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U
//#define  <command name >	<command_code>

//This command is used to read the bootloader version from the MCU
// Packet: Length to follow (1B) + Command code (1B) + CRC (4B) = 6B
// 1B answer
// Example: LtF = 5, CC = 0x51
#define BL_GET_VER				0x51

//This command is used to know what are the commands supported by the bootloader
#define BL_GET_HELP				0x52

//This command is used to read the MCU chip identification number
// 2B answer
#define BL_GET_CID				0x53

//This command is used to read the FLASH Read Protection level
#define BL_GET_RDP_STATUS		0x54

//This command is used to jump bootloader to specified address
// LtF (1B) + CC (1B) + Mem add (LE) (4B) + CRC (4B)
// 1B answer
#define BL_GO_TO_ADDR			0x55

//This command is used to mass erase or sector erase of the user flash
// LtF (1B) + CC (1B) + Sector Numb (1B) + Number of sectors (1B) + CRC (4B)
// 1B answer (status)
#define BL_FLASH_ERASE          0x56

//This command is used to write data in to different memories of the MCU
// LtF (1B) + CC (1B) + Base Mem add (LE) (4B) + Payload lenght (x) (1B)
// + payload (XB) + CRC (4B)
// 1B answer
#define BL_MEM_WRITE			0x57

//This command is used to enable or disable read/write protect on different sectors of the user flash .
// LtF (1B) + CC (1B) + sector details (1B, being sector numbers encoded in 8bits)
// + Protection Mode (1 - write, 2 R/W) (1B) + CRC (4B)
// 1B answer
#define BL_EN_RW_PROTECT		0x58

//This command is used to read data from different memories of the microcontroller.
#define BL_MEM_READ				0x59

//This command is used to read all the sector protection status.
// LtF (1B) + CC (1B) + CRC (4B)
// 2B answer
#define BL_READ_SECTOR_P_STATUS	0x5A

//This command is used to read the OTP contents.
#define BL_OTP_READ				0x5B

//This command is used disable all sector read/write protection
// LtF (1B) + CC (1B) + CRC (4B)
// 1B answer
#define BL_DIS_R_W_PROTECT				0x5C
```

In 'main.c' code we implemented the 'bootloader_uart_read_data' function and some global variables:
```c
#define BL_RX_LEN 200
uint8_t bl_rx_buffer[BL_RX_LEN];

void  bootloader_uart_read_data(void){
    uint8_t rcv_len=0;
	while(1){
		memset(bl_rx_buffer,0,200);
		//here we will read and decode the commands coming from host
		//first read only one byte from the host , which is the "length" field of the command packet
		HAL_UART_Receive(C_UART,bl_rx_buffer,1,HAL_MAX_DELAY);
		rcv_len= bl_rx_buffer[0];
		HAL_UART_Receive(C_UART,&bl_rx_buffer[1],rcv_len,HAL_MAX_DELAY);
		switch(bl_rx_buffer[1]){
            case BL_GET_VER:
                bootloader_handle_getver_cmd(bl_rx_buffer); //prototype in main.h
                break;
            case BL_GET_HELP:
                bootloader_handle_gethelp_cmd(bl_rx_buffer);
                break;
            case BL_GET_CID:
                bootloader_handle_getcid_cmd(bl_rx_buffer);
                break;
            case BL_GET_RDP_STATUS:
                bootloader_handle_getrdp_cmd(bl_rx_buffer);
                break;
            case BL_GO_TO_ADDR:
                bootloader_handle_go_cmd(bl_rx_buffer);
                break;
            case BL_FLASH_ERASE:
                bootloader_handle_flash_erase_cmd(bl_rx_buffer);
                break;
            case BL_MEM_WRITE:
                bootloader_handle_mem_write_cmd(bl_rx_buffer);
                break;
            case BL_EN_RW_PROTECT:
                bootloader_handle_en_rw_protect(bl_rx_buffer);
                break;
            case BL_MEM_READ:
                bootloader_handle_mem_read(bl_rx_buffer);
                break;
            case BL_READ_SECTOR_P_STATUS:
                bootloader_handle_read_sector_protection_status(bl_rx_buffer);
                break;
            case BL_OTP_READ:
                bootloader_handle_read_otp(bl_rx_buffer);
                break;
						case BL_DIS_R_W_PROTECT:
                bootloader_handle_dis_rw_protect(bl_rx_buffer);
                break;
             default:
                printmsg("BL_DEBUG_MSG:Invalid command code received from host \n");
                break;
		}
	}
}
```

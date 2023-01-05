/* ==================================================================== */
/* ============================= INCLUDES ============================= */
/* ==================================================================== */

#include "epaper.h"

/* ==================================================================== */
/* ========================= LOCAL VARIABLES ========================== */
/* ==================================================================== */

uint8_t blackScreenData[4736] = {[0 ... 4735] = 0xff};
uint8_t sendBuffer[8];
uint8_t WF_PARTIAL_LUT[LUT_SIZE] =
{
0x0,	0x40,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x80,	0x80,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x40,	0x40,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x80,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0A,	0x0,	0x0,	0x0,	0x0,	0x0,	0x2,	0x1,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x22,	0x22,	0x22,	0x22,	0x22,	0x22,	
0x0,	0x0,	0x0,	0x22,	0x17,	0x41,	0xB0,	0x32,	0x36
};
uint8_t WS_20_30_LUT[LUT_SIZE] =
{											
0x80,	0x66,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x40,	0x0,	
0x0,	0x0,	0x10,	0x66,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x20,	0x0,	0x0,	0x0,	0x80,	0x66,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x40,	0x0,	0x0,	0x0,	0x10,	0x66,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x20,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x14,	0x8,	0x0,	0x0,	0x0,	0x0,	0x1,	0xA,	0xA,	0x0,	
0xA,	0xA,	0x0,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x14,	0x8,	0x0,	0x1,	
0x0,	0x0,	0x1,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x1,	
0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	0x0,	
0x0,	0x0,	0x0,	0x0,	0x44,	0x44,	0x44,	0x44,	0x44,	0x44,	
0x0,	0x0,	0x0,	0x22,	0x17,	0x41,	0x0,	0x32,	0x36
};	

/* ==================================================================== */
/* ======================= EXTERNAL VARIABLES ========================= */
/* ==================================================================== */

extern osSemaphoreId BinarySem01Handle;
extern osSemaphoreId epapBusySemHandle;
extern SPI_HandleTypeDef hspi1;


/* ==================================================================== */
/* ======================= INTERUPT HANDLERS ========================== */
/* ==================================================================== */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi == &hspi1)
	{
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(BinarySem01Handle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BUSY_Pin)
	{
		static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(epapBusySemHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/* ==================================================================== */
/* =================== LOCAL FUNCTION DEFINITIONS ===================== */
/* ==================================================================== */

/*!
  @brief   Pull epaper chip select pin low, allowing communication
*/
static void csOn()
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
}

/*!
  @brief   Pull epaper chip select pin high, inhibiting communication
*/
static void csOff()
{
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/*!
  @brief   Pull epaper data/command# pin low, allowing commands to be sent to the epaper
*/
static void setCommandMode()
{
    HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_RESET);
}

/*!
  @brief   Pull epaper data/command# pin high, allowing data to be sent to the epaper
*/
static void setDataMode()
{
    HAL_GPIO_WritePin(DC_GPIO_Port, DC_Pin, GPIO_PIN_SET);
}

/*!
  @brief   Pull epaper reset pin low, disabling the epaper
*/
static void reset()
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

/*!
  @brief   	Pull epaper reset pin high, enabling the epaper
*/
static void enable()
{
	HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
	vTaskDelay(2 / portTICK_PERIOD_MS);
}

/*!
  @brief   Wait for busy pin external interupt to be disasserted 
*/
static void waitBusyRelease()
{
    if(HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin) == 1)
    {
        Debug("e-Paper busy\r\n");
        xSemaphoreTake(epapBusySemHandle, 0); // Guarantee epapBusySemHandle set to 0 
        if (!(xSemaphoreTake(epapBusySemHandle, 5000) == pdTRUE))
            {
                Debug("Interrupt failed to occur during SPI transmit\n");
            }
        Debug("e-Paper busy release\r\n");
    }
}

/*!
  @brief   Send a command to the epaper
  @param   command Register address of command 
*/
static void sendCommand(uint8_t command)
{
	// Set Epaper to command mode
    setCommandMode();
	// Open spi communication
    csOn();

	// Write command to spi 
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&command, 1);
	// Wait for spi interupt signaling data transfer complete - timeout 10ms
    if (!(xSemaphoreTake(BinarySem01Handle, 10) == pdTRUE))
	{
		Debug("Interrupt failed to occur during SPI transmit\n");
	}

	// Close spi communication
    csOff();
}

/*!
  @brief   Send a command followed by data to the epaper
  @param   command 	Register address of command 
  @param   data 	Data to be written to epaper 
  @param   numBytes Size of data transfer excluding command byte 
*/
static void sendMessage(uint8_t command, uint8_t* data, uint16_t numBytes)
{
	    // Set Epaper to command mode
		setCommandMode(); 
		// Open spi communication	
		csOn();

		// Write command to spi 
		HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&command, 1);
		// Wait for spi interupt signaling data transfer complete - timeout 10ms
		if (!(xSemaphoreTake(BinarySem01Handle, 10) == pdTRUE))
		{
			Debug("Interrupt failed to occur during SPI transmit\n");
		}

		// Set Epaper to data mode
		setDataMode();

		// Write command to spi
		HAL_SPI_Transmit_IT(&hspi1, data, numBytes);
		// Wait for spi interupt signaling data transfer complete - timout 50ms
		if (!(xSemaphoreTake(BinarySem01Handle, 50) == pdTRUE))
		{
			Debug("Interrupt failed to occur during SPI transmit\n");
		}

		// Close spi communication
		csOff();
}

/*!
  @brief   Set Epaper internal lookup table and voltage data
  @param   lut 	Look up table to write to epaper
*/
static void setLookupTable(uint8_t *lut)
{
	// Set look up table register - last 6 bytes belong to seperate registers
	sendMessage(CMD_WRITE_LUT_REGISTER, lut, LUT_SIZE - 6);
	// Wait for busy_release interupt
	waitBusyRelease(); 

	// Set Mystery Register 1
	sendBuffer[0] = lut[153];		
	sendMessage(CMD_WRITE_MYSTERY_REGISTER_2, sendBuffer, 1);

	// Set Gate voltage Control Register
	sendBuffer[0] = lut[154];		
	sendMessage(CMD_GATE_DRIVING_VOLTAGE_CONTROL, sendBuffer, 1);

	// Set Source Driving Voltage Register
	sendBuffer[0] = lut[155];
	sendBuffer[1] = lut[156];
	sendBuffer[2] = lut[157];
	sendMessage(CMD_SOURCE_DRIVING_VOLTAGE_CONTROL, sendBuffer, 3);

	// Set VCOM Register
	sendBuffer[0] = lut[158];
	sendMessage(CMD_WRITE_VCOM_REGISTER, sendBuffer, 1);
}

/*!
  @brief   Turn on Epaper display
*/
static void turnOnDisplay()
{
	// Set display update sequence
	sendBuffer[0] = 0xc7;
	sendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Activate display update sequence
	sendCommand(CMD_MASTER_ACTIVATION);

	// Wait for busy_release interupt
	waitBusyRelease();
}

/*!
  @brief   	Turn on Epaper display Partial
*/
static void turnOnDisplayPartial()
{
	// Set display update sequence
	sendBuffer[0] = 0x0F;
	sendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Activate display update sequence
	sendCommand(CMD_MASTER_ACTIVATION);

	// Wait for busy_release interupt
	waitBusyRelease();
}

/*!
  @brief	Set Window Size of epaper
  @param	Xstart	Starting X coordinate
  @param	Ystart	Starting Y coordinate
  @param	Xend 	Ending X coordinate
  @param	Yend	Ending Y coordinate
*/
static void setWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
	// SET_RAM_X_ADDRESS_START_END_POSITION
	sendBuffer[0] = (Xstart>>3) & 0xFF;
	sendBuffer[1] = (Xend>>3) & 0xFF;
	sendMessage(CMD_SET_RAM_X_WINDOW, sendBuffer, 2);
	
	// SET_RAM_Y_ADDRESS_START_END_POSITION
	sendBuffer[0] = Ystart & 0xFF;
	sendBuffer[1] = (Ystart >> 8) & 0xFF;
	sendBuffer[2] = Yend & 0xFF;
	sendBuffer[3] = (Yend >> 8) & 0xFF;
    sendMessage(CMD_SET_RAM_Y_WINDOW, sendBuffer, 4);
}

/*!
  @brief	Set Cursor position of epaper
  @param	Xstart	Starting X coordinate
  @param	Ystart	Starting Y coordinate
*/
static void setCursor(uint16_t Xstart, uint16_t Ystart)
{
	// SET_RAM_X_ADDRESS_COUNTER
	sendBuffer[0] = (Xstart & 0xFF);
	sendMessage(CMD_SET_RAM_X_COUNTER, sendBuffer, 1);

	// SET_RAM_Y_ADDRESS_COUNTER
	sendBuffer[0] = Ystart & 0xFF;
	sendBuffer[1] = ((Ystart >> 8) & 0xFF);
    sendMessage(CMD_SET_RAM_Y_COUNTER, sendBuffer, 2);
}

/*!
  @brief	Initialize epaper display
*/
static void setDefaultSettings()
{
	// Set initial pin configuration
	setCommandMode();
	csOn();
	enable();

	// Excecute delayed reset sequence
	vTaskDelay(100 / portTICK_PERIOD_MS);
	reset();
	enable();
	vTaskDelay(100 / portTICK_PERIOD_MS);
	waitBusyRelease();

	// Software reset
	sendCommand(CMD_SW_RESET); 
	waitBusyRelease();   

	// Configure driver output control
	sendBuffer[0] = 0x27;
	sendBuffer[1] = 0x01;
	sendBuffer[2] = 0x00;
	sendMessage(CMD_DRIVER_OUTPUT_CONTROL, sendBuffer, 3);   

	// Configure Data entry mode
	sendBuffer[0] = 0x03;
	sendMessage(CMD_DATA_ENTRY_MODE_SETTING, sendBuffer, 1); 

	// Set window border size
	setWindows(0, 0, EPD_WIDTH-1, EPD_HEIGHT-1);

	// Configure display update control register 1
	sendBuffer[0] = 0x00;
	sendBuffer[1] = 0x80;
	sendMessage(CMD_DISPLAY_UPDATE_CONTROL_1, sendBuffer, 2);

	// Set initial cursor position
	setCursor(0, 0);

	// Wait for busy_release interupt
	waitBusyRelease();
}

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DEFINITIONS ==================== */
/* ==================================================================== */

/*!
  @brief	Clear epaper display
*/
void epdClear()
{
	// Reset initial settings
	setDefaultSettings();
	
	// Set initial lookup table register 
	setLookupTable(WS_20_30_LUT);

	// Write white image to Black and White RAM
	sendMessage(CMD_WRITE_RAM_BLACK_WHITE, blackScreenData, (EPD_WIDTH * EPD_HEIGHT) / 8);
	
	// Activate display
	turnOnDisplay();
}

/*!
  @brief	Sends the image buffer in RAM to e-Paper and enables display
  @param	Image	Image to display
*/
void epdDisplay(uint8_t *Image)
{
	// Reset initial settings
	setDefaultSettings();

	// Set initial lookup table register 
	setLookupTable(WS_20_30_LUT);

	// Write image to Black and White RAM
	sendMessage(CMD_WRITE_RAM_BLACK_WHITE, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Write image to Red RAM
	sendMessage(CMD_WRITE_RAM_RED, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Activate display
	turnOnDisplay();
}

/*!
  @brief	Sends the partial image buffer in RAM to e-Paper and displays 
  @param	Image	Image to display
*/
void epdDisplayPartial(uint8_t *Image)
{
    // // Reset epaper communication
	reset();
	enable();

	// Update Look up table register
	setLookupTable(WF_PARTIAL_LUT);

	// Set mystery register 1
	sendBuffer[0] = 0x00;
	sendBuffer[1] = 0x00;
	sendBuffer[2] = 0x00;
	sendBuffer[3] = 0x00;
	sendBuffer[4] = 0x00;
	sendBuffer[5] = 0x40;
	sendBuffer[6] = 0x00;
	sendBuffer[7] = 0x00;
	sendBuffer[8] = 0x00;
	sendBuffer[9] = 0x00;
	sendMessage(CMD_WRITE_MYSTERY_REGISTER_1, sendBuffer, 10);
	
	// Set border waveform register
	sendBuffer[0] = 0x80;
	sendMessage(CMD_BORDER_WAVEFORM_CONTROL, sendBuffer, 1);

	// Set display update control register
	sendBuffer[0] = 0xC0;
	sendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Active display update sequence
	sendCommand(CMD_MASTER_ACTIVATION);

	// Wait for Busy* interupt
	waitBusyRelease();  
	
	// Set window and cursor position
	setWindows(0, 0, EPD_WIDTH-1, EPD_HEIGHT-1);
	setCursor(0, 0);

	// Write Black and White image to RAM
	sendMessage(CMD_WRITE_RAM_BLACK_WHITE, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Turn on epaper display
	turnOnDisplayPartial();
}

/*!
  @brief	Enable epaper sleep mode
*/
void epdSleep()
{
	// Wait for any operations to finish
	waitBusyRelease();

	// Enter deep sleep mode
	sendBuffer[0] = 0x01;
	sendMessage(CMD_DEEP_SLEEP_MODE, sendBuffer, 1);
	vTaskDelay(2000 / portTICK_PERIOD_MS); // Required

	// Set exit configuration
	setCommandMode();
    csOn();
	reset();

}
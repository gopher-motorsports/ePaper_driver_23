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
  @brief   Epaper communication reset
*/
static void epdReset()
{
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

/*!
  @brief   Wait for busy pin external interupt to be disasserted 
*/
static void edpReadBusy()
{
    if(HAL_GPIO_ReadPin(BUSY_GPIO_Port, BUSY_Pin) == 1)
    {
        Debug("e-Paper busy\r\n");
        xSemaphoreTake(epapBusySemHandle, 0); // Guarantee epapBusySemHandle set to 0 
        if (!(xSemaphoreTake(epapBusySemHandle, 10) == pdTRUE))
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
static void epdSendCommand(uint8_t command)
{
    setCommandMode();
    csOn();
    HAL_SPI_Transmit_IT(&hspi1, (uint8_t *)&command, 1);
    if (!(xSemaphoreTake(BinarySem01Handle, 10) == pdTRUE))
	{
		Debug("Interrupt failed to occur during SPI transmit\n");
	}
    csOff();
}

/*!
  @brief   Send a command followed by data to the epaper
  @param   command 	Register address of command 
  @param   data 	Data to be written to epaper 
  @param   numBytes Size of data transfer excluding command byte 
*/
static void epdSendMessage(uint8_t command, uint8_t* data, uint16_t numBytes)
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
  @brief   Set Epaper internal lookup table excluding voltage data
  @param   lut 	Look up table to write to epaper
*/
static void epdLUT(uint8_t *lut)
{       
	// Set look up table register - last 6 bytes belong to seperate registers
	epdSendMessage(CMD_WRITE_LUT_REGISTER, lut, LUT_SIZE - 6);
	// Wait for Busy* interupt
	edpReadBusy(); 
}

/*!
  @brief   Set Epaper internal lookup table and voltage data
  @param   lut 	Look up table to write to epaper
*/
static void epdHostLUT(uint8_t *lut)
{
	// Set Look up table Register
	epdLUT(lut);

	// Set Mystery Register 1
	sendBuffer[0] = lut[153];		
	epdSendMessage(CMD_WRITE_MYSTERY_REGISTER_2, sendBuffer, 1);

	// Set Gate voltage Control Register
	sendBuffer[0] = lut[153];		
	epdSendMessage(CMD_GATE_DRIVING_VOLTAGE_CONTROL, sendBuffer, 1);

	// Set Source Driving Voltage Register
	sendBuffer[0] = lut[155];
	sendBuffer[1] = lut[156];
	sendBuffer[2] = lut[157];
	epdSendMessage(CMD_SOURCE_DRIVING_VOLTAGE_CONTROL, sendBuffer, 3);

	// Set VCOM Register
	sendBuffer[0] = lut[158];
	epdSendMessage(CMD_WRITE_VCOM_REGISTER, sendBuffer, 1);
}

/*!
  @brief   Turn on Epaper display
*/
static void epdTurnOnDisplay()
{
	// Set display update sequence
	sendBuffer[0] = 0xc7;
	epdSendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Activate display update sequence
	epdSendCommand(CMD_MASTER_ACTIVATION);

	// Wait for Busy* interupt
	edpReadBusy();
}

/*!
  @brief   	Turn on Epaper display Partial
*/
static void epdTurnOnDisplay_Partial()
{
	// Set display update sequence
	sendBuffer[0] = 0x0F;
	epdSendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Activate display update sequence
	epdSendCommand(CMD_MASTER_ACTIVATION);

	// Wait for Busy* interupt
	edpReadBusy();
}

/*!
  @brief	Set Window Size of epaper
  @param	Xstart	Starting X coordinate
  @param	Ystart	Starting Y coordinate
  @param	Xend 	Ending X coordinate
  @param	Yend	Ending Y coordinate
*/
static void epdSetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
	// SET_RAM_X_ADDRESS_START_END_POSITION
	sendBuffer[0] = (Xstart>>3) & 0xFF;
	sendBuffer[1] = (Xend>>3) & 0xFF;
	epdSendMessage(CMD_SET_RAM_X_WINDOW, sendBuffer, 2);
	
	// SET_RAM_Y_ADDRESS_START_END_POSITION
	sendBuffer[0] = Ystart & 0xFF;
	sendBuffer[1] = (Ystart >> 8) & 0xFF;
	sendBuffer[2] = Yend & 0xFF;
	sendBuffer[3] = (Yend >> 8) & 0xFF;
    epdSendMessage(CMD_SET_RAM_Y_WINDOW, sendBuffer, 4);
}

/*!
  @brief	Set Cursor position of epaper
  @param	Xstart	Starting X coordinate
  @param	Ystart	Starting Y coordinate
*/
static void epdSetCursor(uint16_t Xstart, uint16_t Ystart)
{
	// SET_RAM_X_ADDRESS_COUNTER
	sendBuffer[0] = (Xstart & 0xFF);
	epdSendMessage(CMD_SET_RAM_X_COUNTER, sendBuffer, 1);

	// SET_RAM_Y_ADDRESS_COUNTER
	sendBuffer[0] = Ystart & 0xFF;
	sendBuffer[1] = ((Ystart >> 8) & 0xFF);
    epdSendMessage(CMD_SET_RAM_Y_COUNTER, sendBuffer, 2);
}

/* ==================================================================== */
/* =================== GLOBAL FUNCTION DEFINITIONS ==================== */
/* ==================================================================== */
/*!
  @brief	Open epaper SPI communication
*/
int epdCommunicationInit()
    {
        setCommandMode();
		csOn();
		HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_SET);
		return 0;
    }

/*!
  @brief	Close epaper SPI communication
*/
void epdCommunicationExit()
{
	setCommandMode();
    csOn();
    //close 5V
    HAL_GPIO_WritePin(RST_GPIO_Port, RST_Pin, GPIO_PIN_RESET);
}

/*!
  @brief	Initialize epaper display
*/
void epdInit()
{
	epdReset();
	vTaskDelay(100 / portTICK_PERIOD_MS);

	edpReadBusy();   
	epdSendCommand(CMD_SW_RESET);  //SWRESET
	edpReadBusy();   

	sendBuffer[0] = 0x27;
	sendBuffer[1] = 0x01;
	sendBuffer[2] = 0x00;
	epdSendMessage(CMD_DRIVER_OUTPUT_CONTROL, sendBuffer, 3); //Driver output control    

	sendBuffer[0] = 0x03;
	epdSendMessage(CMD_DATA_ENTRY_MODE_SETTING, sendBuffer, 1); //data entry mode 

	epdSetWindows(0, 0, EPD_WIDTH-1, EPD_HEIGHT-1);

	sendBuffer[0] = 0x00;
	sendBuffer[1] = 0x80;
	epdSendMessage(CMD_DISPLAY_UPDATE_CONTROL_1, sendBuffer, 2); //  Display update control	

	epdSetCursor(0, 0);
	edpReadBusy();

	epdHostLUT(WS_20_30_LUT);
}

/*!
  @brief	Clear epaper display
*/
void epdClear()
{
	// Write image to Black and White RAM
	epdSendMessage(CMD_WRITE_RAM_BLACK_WHITE, blackScreenData, (EPD_WIDTH * EPD_HEIGHT) / 8);
	
	// Activate display
	epdTurnOnDisplay();
}

/*!
  @brief	Sends the image buffer in RAM to e-Paper and displays
  @param	Image	Image to display
*/
void epdDisplay(uint8_t *Image)
{
	// Write image to Black and White RAM
	epdSendMessage(CMD_WRITE_RAM_BLACK_WHITE, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Activate display
	epdTurnOnDisplay();
}

/*!
  @brief	Sends the image buffer in RAM to e-Paper and displays
  @param	Image	Image to display
*/
void epdDisplay_Base(uint8_t *Image)
{
	// Write image to Black and White RAM
	epdSendMessage(CMD_WRITE_RAM_BLACK_WHITE, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Write image to Red RAM
	epdSendMessage(CMD_WRITE_RAM_RED, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Activate display
	epdTurnOnDisplay();
}

/*!
  @brief	Sends the partial image buffer in RAM to e-Paper and displays
  @param	Image	Image to display
*/
void epdDisplay_Partial(uint8_t *Image)
{
    // Reset epaper communication
	epdReset();

	// Update Look up table register
	epdLUT(WF_PARTIAL_LUT);

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
	epdSendMessage(CMD_WRITE_MYSTERY_REGISTER_1, sendBuffer, 10);
	
	// Set border waveform register
	sendBuffer[0] = 0x80;
	epdSendMessage(CMD_BORDER_WAVEFORM_CONTROL, sendBuffer, 1);

	// Set display update control register
	sendBuffer[0] = 0xC0;
	epdSendMessage(CMD_DISPLAY_UPDATE_CONTROL_2, sendBuffer, 1);

	// Active display update sequence
	epdSendCommand(CMD_MASTER_ACTIVATION);

	// Wait for Busy* interupt
	edpReadBusy();  
	
	// Set window and cursor position
	epdSetWindows(0, 0, EPD_WIDTH-1, EPD_HEIGHT-1);
	epdSetCursor(0, 0);

	// Write Black and White image to RAM
	epdSendMessage(CMD_WRITE_RAM_BLACK_WHITE, Image, (EPD_WIDTH * EPD_HEIGHT) / 8);

	// Turn on epaper display
	epdTurnOnDisplay_Partial();
}

/*!
  @brief	Enable epaper sleep mode
*/
void epdSleep()
{
	// Enable deep sleep mode
	sendBuffer[0] = 0x01;
	epdSendMessage(CMD_DEEP_SLEEP_MODE, sendBuffer, 1);
	vTaskDelay(100 / portTICK_PERIOD_MS);
}
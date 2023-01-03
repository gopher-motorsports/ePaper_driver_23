#include "main.h"
#include "cmsis_os.h"
#include "mainTask.h"
#include "EPD_Test.h"
#include <stdbool.h>
#include <stdio.h>

extern osSemaphoreId 		BinarySem01Handle;
extern SPI_HandleTypeDef 	hspi1;

bool init = true;

void runMain()
{
    if(init)
    {
        EPD_test();
        init = false;
    }
    else
    {

    }
}
#include "FreeRTOS.h"
#include "task.h"









extern "C"
{
    void infantry1_board_com_init(void *argument)
    {

        vTaskDelete(nullptr);
    }
}
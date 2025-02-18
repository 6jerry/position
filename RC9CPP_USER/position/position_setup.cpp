#include "position_setup.h"
TaskManager task_core;
demo test2, test3;

RC9Protocol esp32_serial(&huart2, false);

extern "C" void position_setup()
{
    esp32_serial.startUartReceiveIT();
    task_core.registerTask(0, &test2);
    task_core.registerTask(1, &test3);
    task_core.registerTask(5, &esp32_serial);

    test2.addport(&esp32_serial);

    esp32_serial.tx_frame_mat.data_length = 24;
    esp32_serial.tx_frame_mat.frame_id = 1;
    task_core.registerTask(8, &esp32_serial);
    osKernelStart();
}

void demo::process_data()
{
    currentTick = HAL_GetTick(); // ???? tick ?

    // ??????????????
    elapsedTime = (float)(currentTick - previousTick);

    if (elapsedTime >= 1) // ??????? 1 ??
    {
        previousTick = currentTick; // ????? tick ?
                                    //  ????????????
                                    //  ??,LED???
    }

    sendFloatData(1, &test_data, 1);
}
void demo::DataReceivedCallback(const uint8_t *byteData, const float *floatData, uint8_t id, uint16_t byteCount)
{
    test_data = floatData[0];
}
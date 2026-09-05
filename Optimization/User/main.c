#include "stm32f4xx.h"
#include "Application/System_Application.h"

int main(void)
{
    System_SoftWare_Init();
    System_HardWare_Init();
    App_Start();
}

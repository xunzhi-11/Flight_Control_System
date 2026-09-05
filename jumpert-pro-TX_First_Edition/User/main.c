#include "stm32f4xx.h"                  // Device header
//User Library
#include "Application/System_Application.h"


int main()
{
    System_SoftWare_Init() ; 
    System_HardWare_Init() ; 
    App_Start() ; 
}
#include "spi/ll_spi.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_spi.h"

void spi_bmm150_init(void)
{
    LL_APB2_GRP1_EnableClock (LL_APB2_GRP1_PERIPH_SPI1) ;
    LL_SPI_Disable(HW_SPI_BMM150_INSTANCE) ;
    
    LL_SPI_InitTypeDef bmm150_spi_initstructure = {0} ;
    
    bmm150_spi_initstructure.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32 ;
    bmm150_spi_initstructure.BitOrder = LL_SPI_MSB_FIRST ;
    bmm150_spi_initstructure.ClockPhase = LL_SPI_PHASE_1EDGE ;
    bmm150_spi_initstructure.ClockPolarity = LL_SPI_POLARITY_LOW ;
    bmm150_spi_initstructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE ;
    bmm150_spi_initstructure.CRCPoly = 0x00 ;
    bmm150_spi_initstructure.DataWidth = LL_SPI_DATAWIDTH_8BIT ;
    bmm150_spi_initstructure.Mode = LL_SPI_MODE_MASTER ;
    bmm150_spi_initstructure.NSS = LL_SPI_NSS_SOFT ;
    bmm150_spi_initstructure.TransferDirection = LL_SPI_FULL_DUPLEX ;
    
    LL_SPI_Init(HW_SPI_BMM150_INSTANCE , &bmm150_spi_initstructure) ;
    LL_SPI_Enable(HW_SPI_BMM150_INSTANCE) ;
}

void spi_icm42688_init(void)
{
    LL_APB1_GRP1_EnableClock (LL_APB1_GRP1_PERIPH_SPI2) ;
    LL_SPI_Disable(HW_SPI_ICM42688_INSTANCE) ;
    LL_SPI_InitTypeDef icm42688_spi_initstructure = {0} ;
    
    icm42688_spi_initstructure.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV4 ;
    icm42688_spi_initstructure.BitOrder = LL_SPI_MSB_FIRST ;
    icm42688_spi_initstructure.ClockPhase = LL_SPI_PHASE_1EDGE ;
    icm42688_spi_initstructure.ClockPolarity = LL_SPI_POLARITY_LOW ;
    icm42688_spi_initstructure.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE ;
    icm42688_spi_initstructure.CRCPoly = 7 ;
    icm42688_spi_initstructure.DataWidth = LL_SPI_DATAWIDTH_8BIT ;
    icm42688_spi_initstructure.Mode = LL_SPI_MODE_MASTER ;
    icm42688_spi_initstructure.NSS = LL_SPI_NSS_SOFT ;
    icm42688_spi_initstructure.TransferDirection = LL_SPI_FULL_DUPLEX ;
    
    LL_SPI_Init(HW_SPI_ICM42688_INSTANCE , &icm42688_spi_initstructure) ;
    LL_SPI_Enable(HW_SPI_ICM42688_INSTANCE) ;
}
#include "flash/ll_flash.h"
#include "stm32f4xx.h"

#define FLASH_UNLOCK_KEY1   0x45670123U
#define FLASH_UNLOCK_KEY2   0xCDEF89ABU
#define LL_FLASH_PSIZE_WORD   FLASH_CR_PSIZE_1
#define FLASH_WAIT_TIMEOUT  500000U

static bool flash_wait_not_busy(void)
{
    uint32_t timeout = FLASH_WAIT_TIMEOUT ;

    while ((FLASH->SR & FLASH_SR_BSY) != 0U)
    {
        if (--timeout == 0U)
        {
            return false ;
        }
    }

    return true ;
}

static void flash_clear_flags(void)
{
    FLASH->SR = FLASH_SR_EOP | FLASH_SR_SOP | FLASH_SR_WRPERR | FLASH_SR_PGAERR |
                FLASH_SR_PGPERR | FLASH_SR_PGSERR ;
}

bool LL_Flash_Unlock(void)
{
    if ((FLASH->CR & FLASH_CR_LOCK) == 0U)
    {
        return true ;
    }

    FLASH->KEYR = FLASH_UNLOCK_KEY1 ;
    FLASH->KEYR = FLASH_UNLOCK_KEY2 ;

    return ((FLASH->CR & FLASH_CR_LOCK) == 0U) ;
}

void LL_Flash_Lock(void)
{
    FLASH->CR |= FLASH_CR_LOCK ;
}

bool LL_Flash_EraseSector(uint32_t sector)
{
    if (!flash_wait_not_busy())
    {
        return false ;
    }

    flash_clear_flags() ;

    MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE | FLASH_CR_SNB, LL_FLASH_PSIZE_WORD | (sector << FLASH_CR_SNB_Pos)) ;
    FLASH->CR |= FLASH_CR_SER ;
    FLASH->CR |= FLASH_CR_STRT ;

    if (!flash_wait_not_busy())
    {
        FLASH->CR &= ~FLASH_CR_SER ;
        return false ;
    }

    FLASH->CR &= ~FLASH_CR_SER ;
    flash_clear_flags() ;

    return true ;
}

bool LL_Flash_ProgramWords(uint32_t address, const uint32_t *data, uint32_t word_count)
{
    for (uint32_t i = 0 ; i < word_count ; i++)
    {
        if (!flash_wait_not_busy())
        {
            return false ;
        }

        MODIFY_REG(FLASH->CR, FLASH_CR_PSIZE, LL_FLASH_PSIZE_WORD) ;
        FLASH->CR |= FLASH_CR_PG ;

        *(__IO uint32_t *)(address + (i * 4U)) = data[i] ;

        if (!flash_wait_not_busy())
        {
            FLASH->CR &= ~FLASH_CR_PG ;
            return false ;
        }

        FLASH->CR &= ~FLASH_CR_PG ;
        flash_clear_flags() ;
    }

    return true ;
}

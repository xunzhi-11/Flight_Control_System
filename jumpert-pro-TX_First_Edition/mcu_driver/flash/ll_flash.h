#ifndef __LL_FLASH_H
#define __LL_FLASH_H

#include "stdint.h"
#include "stdbool.h"

bool LL_Flash_Unlock(void) ;
void LL_Flash_Lock(void) ;
bool LL_Flash_EraseSector(uint32_t sector) ;
bool LL_Flash_ProgramWords(uint32_t address, const uint32_t *data, uint32_t word_count) ;

#endif

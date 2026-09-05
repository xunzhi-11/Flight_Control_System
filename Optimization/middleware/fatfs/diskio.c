#include "fatfs/ff.h"
#include "fatfs/diskio.h"
#include "sd_card/sd_card.h"
#include "sys_hardware_config.h"
#include "stm32f4xx_ll_usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

// 分配一块32字节对齐的全局缓冲，专门用来给FatFs处理非对齐的内存拷贝
__attribute__((aligned(32))) static uint8_t sd_scratch_buf[512];

/* 获取系统时间戳 (FatFs必需) */
DWORD get_fattime(void) {
    // 固定时间：2026年1月1日 00:00:00
    return ((DWORD)(2026 - 1980) << 25) | ((DWORD)1 << 21) | ((DWORD)1 << 16);
}

/* 初始化磁盘 */
DSTATUS disk_initialize (BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    printf("[DiskIO] disk_initialize\r\n");
    if (sd_card_init() == SD_OK) {
        while (!LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE)) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        LL_USART_TransmitData8(HW_UART_DEBUG_INSTANCE, (uint8_t)'+');
        return 0;
    }
    return STA_NOINIT;
}

/* 获取磁盘状态 */
DSTATUS disk_status (BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;
    return 0; // 假定初始化成功后就一直在线
}

/* 读扇区 */
DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count) {
    if (pdrv != 0) return RES_PARERR;
    
    // 循环读取每一块
    for (UINT i = 0; i < count; i++) {
        // 检查 FatFs 传来的地址是否 4 字节对齐
        if (((uint32_t)(buff + i * 512)) % 4 == 0) {
            if (SD_ReadBlock_DMA(sector + i, buff + i * 512) != SD_OK) {
                printf("[DiskIO] read fail LBA=%lu\r\n", (unsigned long)(sector + i));
                return RES_ERROR;
            }
        } else {
            if (SD_ReadBlock_DMA(sector + i, sd_scratch_buf) != SD_OK) {
                printf("[DiskIO] read fail LBA=%lu\r\n", (unsigned long)(sector + i));
                return RES_ERROR;
            }
            memcpy(buff + i * 512, sd_scratch_buf, 512);
        }
    }
    return RES_OK;
}

/* 写扇区 */
DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count) {
    if (pdrv != 0) return RES_PARERR;

    // 检查是否对齐
    if (((uint32_t)buff) % 4 == 0) {
        SD_Error err = SD_WriteMultiBlock_DMA(sector, buff, count);
        if (err == SD_OK) return RES_OK;
        printf("[DiskIO] write fail LBA=%lu cnt=%u err=%d\r\n",
               (unsigned long)sector, (unsigned)count, (int)err);
        return RES_ERROR;
    } else {
        // 未对齐，降级为循环单块写入
        for (UINT i = 0; i < count; i++) {
            memcpy(sd_scratch_buf, buff + i * 512, 512);
            if (SD_WriteBlock_DMA(sector + i, sd_scratch_buf) != SD_OK) return RES_ERROR;
        }
        return RES_OK;
    }
}

/* 磁盘控制指令 */
DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff) {
    if (pdrv != 0) return RES_PARERR;
    DRESULT res = RES_ERROR;

    switch (cmd) {
        case CTRL_SYNC: {
            uint32_t spin = 0;
            SD_Error rd;
            printf("[DiskIO] CTRL_SYNC enter\r\n");
            while ((rd = SD_CheckReady()) == SD_ERR_BUSY) {
                if ((++spin % 500U) == 0U) {
                    printf("[DiskIO] CTRL_SYNC busy, spin=%lu\r\n", spin);
                }
                vTaskDelay(pdMS_TO_TICKS(1));
                if (spin > 10000U) {
                    printf("[DiskIO] CTRL_SYNC timeout, last=%d\r\n", (int)rd);
                    return RES_ERROR;
                }
            }
            if (rd != SD_OK) {
                printf("[DiskIO] CTRL_SYNC ready err=%d\r\n", (int)rd);
                return RES_ERROR;
            }
            printf("[DiskIO] CTRL_SYNC ok, spin=%lu\r\n", spin);
            res = RES_OK;
            break;
        }
            
        case GET_SECTOR_COUNT:
            // 获取你底层记录的总扇区数
            *(DWORD*)buff = SD_GetCardInfo()->BlockCount; 
            res = RES_OK;
            break;
            
        case GET_SECTOR_SIZE:
            *(WORD*)buff = 512;
            res = RES_OK;
            break;
            
        case GET_BLOCK_SIZE:
            *(DWORD*)buff = 1; // 擦除块大小默认给 1 即可
            res = RES_OK;
            break;
            
        default:
            res = RES_PARERR;
    }
    return res;
}
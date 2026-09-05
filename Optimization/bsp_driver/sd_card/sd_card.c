#include "SD_CARD.h"
#include "sys_hardware_config.h"
#include "sys_software_config.h"
#include "dma/ll_dma_sdio.h"
#include "stm32f4xx_ll_usart.h"
#include <string.h>
#include "stdio.h"
static sd_card_delay_us_f sd_card_us_delay = NULL ;
static sd_card_delay_ms_f sd_card_ms_delay = NULL ;
static sd_get_tick_f s_sd_get_tick = NULL ;
static sd_yield_f    s_sd_yield    = NULL ;

void SD_Register_OS_Hooks(sd_get_tick_f get_tick_cb, sd_yield_f yield_cb)
{
    s_sd_get_tick = get_tick_cb;
    s_sd_yield    = yield_cb;
}

void SD_Card_Delay_us_Register(sd_card_delay_us_f  func)
{
    sd_card_us_delay = func ;
}

void SD_Card_Delay_ms_Register(sd_card_delay_ms_f  func)
{
    sd_card_ms_delay = func ;
}

static void sd_card_delay_us(uint32_t us)
{
    if(sd_card_us_delay != NULL)
    {
        sd_card_us_delay(us) ;
    }
    else
    {
        uint16_t i = 1000 ;
        while(i--) ;
    }
}

static void sd_card_delay_ms(uint32_t ms)
{
     if(sd_card_ms_delay != NULL)
    {
        sd_card_ms_delay(ms) ;
    }
    else
    {
        uint32_t i = 1000000 ;
        while(i--) ;
    }
}

static SD_CardInfo cardInfo;

static SD_Error SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t respType)
{
    uint32_t start_time = s_sd_get_tick ? s_sd_get_tick() : 0;
    uint32_t fallback_timeout = SDIO_CMD_TIMEOUT;

    SDIO->ICR = 0x5FF;

    volatile uint32_t dummy = SDIO->STA;
    (void)dummy;

    SDIO->ARG = arg;
    uint32_t cmdVal = (cmd & 0x3F) | respType | SDIO_CMD_CPSMEN;
    SDIO->CMD = cmdVal;

    if (respType == SD_RESP_NONE) {
        while (!(SDIO->STA & SDIO_STA_CMDSENT)) {
            if (s_sd_get_tick) {
                if ((s_sd_get_tick() - start_time) > 500U) {
                    return SD_ERR_TIMEOUT;
                }
            } else if (--fallback_timeout == 0) {
                return SD_ERR_TIMEOUT;
            }
            if (s_sd_yield) {
                s_sd_yield();
            }
        }
    } else {
        while (!(SDIO->STA & (SDIO_STA_CMDREND | SDIO_STA_CTIMEOUT | SDIO_STA_CCRCFAIL))) {
            if (s_sd_get_tick) {
                if ((s_sd_get_tick() - start_time) > 500U) {
                    return SD_ERR_TIMEOUT;
                }
            } else if (--fallback_timeout == 0) {
                return SD_ERR_TIMEOUT;
            }
            if (s_sd_yield) {
                s_sd_yield();
            }
        }
    }

    if (SDIO->STA & SDIO_STA_CTIMEOUT) {
        SDIO->ICR = SDIO_ICR_CTIMEOUTC;
        return SD_ERR_TIMEOUT;
    }

    if (SDIO->STA & SDIO_STA_CCRCFAIL) {
        SDIO->ICR = SDIO_ICR_CCRCFAILC;
        if (cmd != SD_CMD_ALL_SEND_CID &&
            cmd != SD_CMD_SEND_CSD &&
            cmd != SD_ACMD_SD_SEND_OP_COND) {
            return SD_ERR_CRC;
        }
    }

    return SD_OK;
}


void SDIO_SetHighSpeed(void)
{
    // 切换到24MHz时保持HWFC
    SDIO->CLKCR = (2U << SDIO_CLKCR_CLKDIV_Pos) |  // 48/(0+2)=24MHz
                  SDIO_CLKCR_WIDBUS_0 |             // 4-bit模式
                  SDIO_CLKCR_HWFC_EN |              // 硬件流控制
                  SDIO_CLKCR_CLKEN;
                  sd_card_delay_ms(2) ;
}

SD_CardInfo* SD_GetCardInfo(void)
{
    return &cardInfo;
}

static SD_Error SD_SendAppCommand(uint8_t acmd, uint32_t arg, uint8_t respType)
{
    SD_Error err;

    err = SD_SendCommand(SD_CMD_APP_CMD, cardInfo.RCA << 16, SD_RESP_SHORT);
    if (err != SD_OK && err != SD_ERR_CRC) {
        return err;
    }

    return SD_SendCommand(acmd, arg, respType);
}

uint32_t SD_GetResponse(uint8_t respReg)
{
    switch (respReg) {
        case 1:  return SDIO->RESP1;
        case 2:  return SDIO->RESP2;
        case 3:  return SDIO->RESP3;
        case 4:  return SDIO->RESP4;
        default: return 0;
    }
}

SD_Error sd_card_init(void)
{
    SD_Error err;
    uint32_t resp;
    uint32_t timeout;

    memset(&cardInfo, 0, sizeof(cardInfo));
    sd_card_delay_ms(10);

    printf("[SD Init] Sending CMD0 (Reset)...\r\n");
    for (int i = 0; i < 10; i++) {
        SD_SendCommand(SD_CMD_GO_IDLE_STATE, 0, SD_RESP_NONE);
        sd_card_delay_us(10);
    }

    printf("[SD Init] Sending CMD8 (Voltage Check)...\r\n");
    err = SD_SendCommand(SD_CMD_SEND_IF_COND, 0x1AA, SD_RESP_SHORT);
    if (err != SD_OK && err != SD_ERR_TIMEOUT) {
        printf("[SD Init] CMD8 Failed with err: %d\r\n", err);
        // 注意：v1.x 卡可能不支持 CMD8 返回超时，属于正常，继续往下走
    } else {
            printf("[SD Init] CMD8 OK or V1.x Card timeout.\r\n");
    }

    timeout = 1000;
    printf("[SD Init] Sending ACMD41 (App Op Cond)...\r\n");
    do {
        err = SD_SendCommand(SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55
        if (err != SD_OK && err != SD_ERR_CRC) {
            printf("[SD Init] CMD55 Failed! err: %d\r\n", err);
            return err;
        }

        uint32_t arg = 0x40FF8000;
        err = SD_SendCommand(SD_ACMD_SD_SEND_OP_COND, arg, SD_RESP_SHORT); // ACMD41
        resp = SDIO->RESP1;
        timeout--;
        sd_card_delay_ms(2);
    } while (((resp & 0x80000000) == 0) && (timeout > 0));

    if (timeout == 0) {
        printf("[SD Init] ACMD41 Timeout! Card didn't power up.\r\n");
        return SD_ERR_TIMEOUT;
    }
    printf("[SD Init] ACMD41 Success! Card is Ready.\r\n");

    /* 检查 CCS 位确定卡类型 */
    if (resp & 0x40000000) {
        cardInfo.CardType = SD_CARD_HIGH_CAPACITY;
    } else {
        cardInfo.CardType = SD_CARD_STD_CAPACITY_V2;
    }

    /*==================== CMD2 ====================*/
    printf("[SD Init] Sending CMD2 (ALL_SEND_CID)...\r\n");
    err = SD_SendCommand(SD_CMD_ALL_SEND_CID, 0, SD_RESP_LONG);
    if (err != SD_OK && err != SD_ERR_CRC) {
        printf("[SD Init] CMD2 Failed! err: %d\r\n", err);
        return err;
    }
sd_card_delay_ms(5);
    /*==================== CMD3 ====================*/
    printf("[SD Init] Sending CMD3 (SEND_REL_ADDR)...\r\n");
    err = SD_SendCommand(SD_CMD_SEND_REL_ADDR, 0, SD_RESP_SHORT);
    if (err != SD_OK) {
        printf("[SD Init] CMD3 Failed! err: %d\r\n", err);
        return err;
    }
    cardInfo.RCA = (SDIO->RESP1 >> 16) & 0xFFFF;
    printf("[SD Init] CMD3 Success! RCA: 0x%04X\r\n", (int)cardInfo.RCA);

   /*==================== CMD9 ====================*/

    printf("[SD Init] Sending CMD9 (SEND_CSD)...\r\n");

    sd_card_delay_ms(20);
    // 强制转换为无符号 32 位再左移，保证物理层参数绝对正确
    uint32_t rca_arg = ((uint32_t)cardInfo.RCA) << 16;

    // 重试机制
    for (int retry = 0; retry < 5; retry++) {
        err = SD_SendCommand(SD_CMD_SEND_CSD, rca_arg, SD_RESP_LONG);
        if (err == SD_OK || err == SD_ERR_CRC) {
            err = SD_OK;
            break;
        }
        printf("[SD Init] CMD9 Retry %d...\r\n", retry + 1);
        sd_card_delay_ms(10);
    }
    if (err != SD_OK) {
        printf("[SD Init] CMD9 Failed permanently! err: %d\r\n", err);
        return err;
    }
    printf("[SD Init] CMD9 Success!\r\n");
    /* 解析 CSD */
    if (cardInfo.CardType == SD_CARD_HIGH_CAPACITY) {
        uint32_t c_size = ((SDIO->RESP2 & 0x3F) << 16) | ((SDIO->RESP3 >> 16) & 0xFFFF);
        cardInfo.BlockCount = (c_size + 1) * 1024;
        cardInfo.Capacity = (uint64_t)cardInfo.BlockCount * SDIO_BLOCK_SIZE;
    } else {
        cardInfo.BlockCount = 0;
        cardInfo.Capacity = 0;
    }

    cardInfo.BlockSize = SDIO_BLOCK_SIZE;

    /*==================== CMD7 ====================*/
    printf("[SD Init] Sending CMD7 (Select Card)...\r\n");

    for (int retry = 0; retry < 5; retry++) {
        err = SD_SendCommand(SD_CMD_SEL_DESEL_CARD, rca_arg, SD_RESP_SHORT);
        if (err == SD_OK) {
            break;
        }
        printf("[SD Init] CMD7 Retry %d...\r\n", retry + 1);
        sd_card_delay_ms(10);
    }

    if (err != SD_OK) {
        printf("[SD Init] CMD7 Failed permanently! err: %d\r\n", err);
        return err;
    }
    printf("[SD Init] CMD7 Success! Card is in Transfer State.\r\n");

    /*==================== CMD16 ====================*/
    printf("[SD Init] Sending CMD16 (Set Blocklen)...\r\n");
    err = SD_SendCommand(SD_CMD_SET_BLOCKLEN, SDIO_BLOCK_SIZE, SD_RESP_SHORT);
    if (err != SD_OK) {
        printf("[SD Init] CMD16 Failed! err: %d\r\n", err);
        return err;
    }
    /*==================== ACMD6 ====================*/
    printf("[SD Init] Sending ACMD6 (Switch to 4-bit bus)...\r\n");
    err = SD_SendCommand(SD_CMD_APP_CMD, rca_arg, SD_RESP_SHORT);
    if (err == SD_OK || err == SD_ERR_CRC) {
        err = SD_SendCommand(SD_ACMD_SET_BUS_WIDTH, 2, SD_RESP_SHORT);
        if (err == SD_OK) {
            SDIO->CLKCR |= SDIO_CLKCR_WIDBUS_0;
            printf("[SD Init] 4-Bit Bus Enabled!\r\n");
        } else {
            printf("[SD Init] ACMD6 Failed! err: %d (Falling back to 1-bit)\r\n", err);
        }
    }
    printf("[SD Init] ALL DONE! Switching to transfer clock...\r\n");

    uint32_t tmpreg = SDIO->CLKCR;

    tmpreg &= ~SDIO_CLKCR_CLKDIV_Msk;
    tmpreg |= (SDIO_TRANSFER_CLK_DIV << SDIO_CLKCR_CLKDIV_Pos);
    tmpreg |= SDIO_CLKCR_HWFC_EN;
    SDIO->CLKCR = tmpreg;
    sd_card_delay_ms(2);

    printf("[SD Init] Clock ready, div=%u\r\n", (unsigned)SDIO_TRANSFER_CLK_DIV);
    while (!LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE)) {
        if (s_sd_yield) {
            s_sd_yield();
        }
    }
    LL_USART_TransmitData8(HW_UART_DEBUG_INSTANCE, (uint8_t)'>');
    return SD_OK;
}

/*============ DMA配置辅助函数 ============*/
static void SD_DMA_Config_Read(uint8_t *buf, uint32_t len)
{
    // 禁用DMA
    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
    while (LL_DMA_IsEnabledStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM));

    // 清除标志
    LL_DMA_ClearFlag_TC6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_TE6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_FE6(HW_DMA_SDIO_INSTANCE);

    // 配置方向：外设到内存
    LL_DMA_SetDataTransferDirection(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

    // 配置内存地址和长度
    LL_DMA_SetMemoryAddress(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, (uint32_t)buf);
    LL_DMA_SetDataLength(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, len / 4);  // 以字为单位

    LL_DMA_SetMemoryBurstxfer(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_MBURST_SINGLE);
    LL_DMA_SetPeriphBurstxfer(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_PBURST_SINGLE);

    // 使能DMA
    LL_DMA_EnableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
}

static void SD_DMA_Config_Write(const uint8_t *buf, uint32_t len)
{
    uint32_t dma_timeout = 0x1FFFF;
    // 禁用DMA
    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
    while (LL_DMA_IsEnabledStream(  HW_DMA_SDIO_INSTANCE,   HW_DMA_SDIO_STREAM) && dma_timeout--)
    {
        __NOP();
    }
    // 清除标志 (DMA2 Stream6)
    LL_DMA_ClearFlag_TC6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_HT6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_TE6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_DME6(HW_DMA_SDIO_INSTANCE);
    LL_DMA_ClearFlag_FE6(HW_DMA_SDIO_INSTANCE);

    // 配置方向：内存到外设
    LL_DMA_SetDataTransferDirection(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

    // 配置内存地址和长度
    LL_DMA_SetMemoryAddress(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, (uint32_t)buf);
    LL_DMA_SetDataLength(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, len / 4);

    LL_DMA_SetMemoryBurstxfer(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_MBURST_SINGLE);
    LL_DMA_SetPeriphBurstxfer(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM, LL_DMA_PBURST_SINGLE);

    // 使能DMA
    LL_DMA_EnableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
}

static SD_Error SD_WaitDmaTransferDone(uint32_t timeout_ms)
{
    uint32_t start_time = s_sd_get_tick ? s_sd_get_tick() : 0;
    uint32_t fallback_timeout = SDIO_DATA_TIMEOUT;

    while (1) {
        if (LL_DMA_IsActiveFlag_TC6(HW_DMA_SDIO_INSTANCE)) {
            LL_DMA_ClearFlag_TC6(HW_DMA_SDIO_INSTANCE);
            return SD_OK;
        }
        if (dma_sdio_GetState() == SDIO_DMA_COMPLETE) {
            return SD_OK;
        }
        if (dma_sdio_GetState() == SDIO_DMA_ERROR) {
            return SD_ERR_WRITE;
        }
        if (!LL_DMA_IsEnabledStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM)) {
            return SD_OK;
        }

        if (s_sd_get_tick) {
            if ((s_sd_get_tick() - start_time) > timeout_ms) {
                return SD_ERR_TIMEOUT;
            }
        } else if (--fallback_timeout == 0) {
            return SD_ERR_TIMEOUT;
        }
        if (s_sd_yield) {
            s_sd_yield();
        }
    }
}

static void SD_BusRecover(void)
{
    uint32_t i;

    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
    SD_SendCommand(SD_CMD_STOP_TRANSMISSION, 0, SD_RESP_SHORT);
    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();

    for (i = 0; i < 1000U; i++) {
        if (SD_CheckReady() == SD_OK) {
            break;
        }
        if (s_sd_yield) {
            s_sd_yield();
        }
    }
}

static SD_Error SD_ReadDmaFinish(uint32_t *out_sta)
{
    SD_Error err = SD_OK;
    uint32_t start_time = s_sd_get_tick ? s_sd_get_tick() : 0;
    uint32_t fallback_timeout = SDIO_DATA_TIMEOUT;
    uint32_t sta;

    while (!(SDIO->STA & (SDIO_STA_DATAEND | SDIO_STA_RXOVERR |
                          SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT)))
    {
        if (s_sd_get_tick) {
            if ((s_sd_get_tick() - start_time) > 5000U) {
                err = SD_ERR_TIMEOUT;
                break;
            }
        } else if (--fallback_timeout == 0) {
            err = SD_ERR_TIMEOUT;
            break;
        }
        if (s_sd_yield) {
            s_sd_yield();
        }
    }

    sta = SDIO->STA;
    if (out_sta) {
        *out_sta = sta;
    }

    if (sta & SDIO_STA_DCRCFAIL) {
        err = SD_ERR_CRC;
    } else if (sta & SDIO_STA_DTIMEOUT) {
        err = SD_ERR_TIMEOUT;
    } else if (sta & SDIO_STA_RXOVERR) {
        err = SD_ERR_READ;
    } else if (sta & SDIO_STA_DATAEND) {
        SD_Error dma_err = SD_WaitDmaTransferDone(5000U);
        if (dma_err != SD_OK) {
            err = dma_err;
        }
    } else if (err == SD_OK) {
        err = SD_ERR_TIMEOUT;
    }

    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);

    if (err != SD_OK) {
        SD_BusRecover();
        return err;
    }

    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();
    return SD_OK;
}

static SD_Error SD_WriteDmaFinish(uint32_t totalBytes, uint32_t *out_sta)
{
    SD_Error err = SD_OK;
    uint32_t start_time = s_sd_get_tick ? s_sd_get_tick() : 0;
    uint32_t fallback_timeout = SDIO_DATA_TIMEOUT;
    uint32_t sta;

    while (!(SDIO->STA & (SDIO_STA_DATAEND | SDIO_STA_TXUNDERR |
                          SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT)))
    {
        if (s_sd_get_tick) {
            if ((s_sd_get_tick() - start_time) > 5000U) {
                err = SD_ERR_TIMEOUT;
                break;
            }
        } else if (--fallback_timeout == 0) {
            err = SD_ERR_TIMEOUT;
            break;
        }
        if (s_sd_yield) {
            s_sd_yield();
        }
    }

    sta = SDIO->STA;
    if (out_sta) {
        *out_sta = sta;
    }

    if (sta & SDIO_STA_DCRCFAIL) {
        err = SD_ERR_CRC;
    } else if (sta & SDIO_STA_DTIMEOUT) {
        err = SD_ERR_TIMEOUT;
    } else if (sta & SDIO_STA_TXUNDERR) {
        err = SD_ERR_WRITE;
    } else if (sta & SDIO_STA_DATAEND) {
        SD_Error dma_err = SD_WaitDmaTransferDone(5000U);
        if (dma_err != SD_OK) {
            err = dma_err;
        }
    } else if (err == SD_OK) {
        err = SD_ERR_TIMEOUT;
    }

    LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);

    if (totalBytes > SDIO_BLOCK_SIZE) {
        SD_SendCommand(SD_CMD_STOP_TRANSMISSION, 0, SD_RESP_SHORT);
    }

    if (err != SD_OK) {
        SD_BusRecover();
        return err;
    }

    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();
    return SD_OK;
}


/*============ DMA单块读取 ============*/
SD_Error SD_ReadBlock_DMA(uint32_t blockAddr, uint8_t *buf)
{
    SD_Error err;

    uint32_t addr = (cardInfo.CardType == SD_CARD_HIGH_CAPACITY) ?
                    blockAddr : (blockAddr << 9);

    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();

    SD_DMA_Config_Read(buf, SDIO_BLOCK_SIZE);

    SDIO->DTIMER = SDIO_DATATIMER_VALUE;
    SDIO->DLEN = SDIO_BLOCK_SIZE;

    err = SD_SendCommand(SD_CMD_READ_SINGLE_BLOCK, addr, SD_RESP_SHORT);
    if (err != SD_OK) {
        printf("[SD] read cmd17 LBA=%lu err=%d\r\n", (unsigned long)blockAddr, (int)err);
        LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
        SD_BusRecover();
        return err;
    }

    SDIO->DCTRL = (9U << SDIO_DCTRL_DBLOCKSIZE_Pos) |
                  SDIO_DCTRL_DTDIR |
                  SDIO_DCTRL_DMAEN |
                  SDIO_DCTRL_DTEN;

    dma_sdio_SetState(SDIO_DMA_BUSY);
    return SD_ReadDmaFinish(NULL);
}

/*============ DMA单块写入 ============*/
SD_Error SD_WriteBlock_DMA(uint32_t blockAddr, const uint8_t *buf)
{
    SD_Error err;

    uint32_t addr = (cardInfo.CardType == SD_CARD_HIGH_CAPACITY) ?
                    blockAddr : (blockAddr << 9);

    // 清除所有标志
    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();

    // 配置DMA写入
    SD_DMA_Config_Write(buf, SDIO_BLOCK_SIZE);

    // 发送写命令
    err = SD_SendCommand(SD_CMD_WRITE_SINGLE_BLOCK, addr, SD_RESP_SHORT);
    if (err != SD_OK) {
        LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE,   HW_DMA_SDIO_STREAM);
        return err;
    }

    // 配置SDIO数据传输
    SDIO->DTIMER = SDIO_DATATIMER_VALUE;
    SDIO->DLEN = SDIO_BLOCK_SIZE;

    // 启动传输（方向：控制器到卡，DTDIR=0）
    SDIO->DCTRL = (9U << SDIO_DCTRL_DBLOCKSIZE_Pos) |
                  SDIO_DCTRL_DMAEN |
                  SDIO_DCTRL_DTEN;

    dma_sdio_SetState(SDIO_DMA_BUSY);
    return SD_WriteDmaFinish(SDIO_BLOCK_SIZE, NULL);
}


/*============ DMA多块写入 - 飞控日志关键函数 (同步阻塞版) ============*/
SD_Error SD_WriteMultiBlock_DMA(uint32_t blockAddr, const uint8_t *buf, uint32_t numBlocks)
{
    SD_Error err;
    uint32_t totalBytes = numBlocks * SDIO_BLOCK_SIZE;

    uint32_t addr = (cardInfo.CardType == SD_CARD_HIGH_CAPACITY) ?
                    blockAddr : (blockAddr << 9);

    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();

    SD_SendAppCommand(SD_ACMD_SET_WR_BLK_ERASE_COUNT, numBlocks, SD_RESP_SHORT);

    SD_DMA_Config_Write(buf, totalBytes);

    err = SD_SendCommand(SD_CMD_WRITE_MULT_BLOCK, addr, SD_RESP_SHORT);
    if (err != SD_OK) {
        LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE,   HW_DMA_SDIO_STREAM);
        return err;
    }

    SDIO->DTIMER = SDIO_DATATIMER_VALUE;
    SDIO->DLEN = totalBytes;
    SDIO->DCTRL = (9U << SDIO_DCTRL_DBLOCKSIZE_Pos) |
                  SDIO_DCTRL_DMAEN |
                  SDIO_DCTRL_DTEN;

    dma_sdio_SetState(SDIO_DMA_BUSY);
    return SD_WriteDmaFinish(totalBytes, NULL);
}

/**
 * @brief 非阻塞检查SD卡是否就绪
 * @return SD_OK: 就绪可写入
 *         SD_ERR_BUSY: 卡正忙（编程中）
 *         其他: 错误
 */
SD_Error SD_CheckReady(void)
{
    SD_Error err;
    uint32_t resp;

    err = SD_SendCommand(SD_CMD_SEND_STATUS, cardInfo.RCA << 16, SD_RESP_SHORT);
    if (err != SD_OK) {
        return err;
    }

    resp = SDIO->RESP1;

    // 检查错误位
    if (resp & 0xFDF90000) {
        return SD_ERR_WRITE;
    }

    // CURRENT_STATE [12:9]
    // 4 = Transfer (空闲，可接受新命令)
    // 6 = Receiving
    // 7 = Programming (编程中，忙)
    uint8_t state = (resp >> 9) & 0x0F;

    if (state == 4) {
        return SD_OK;       // 就绪
    } else {
        return SD_ERR_BUSY; // 忙
    }
}

/*============ 非阻塞DMA写入（异步） ============*/
static volatile uint32_t async_block_addr = 0;
static volatile uint32_t async_num_blocks = 0;

SD_Error SD_WriteMultiBlock_DMA_Start(uint32_t blockAddr, const uint8_t *buf, uint32_t numBlocks)
{
    SD_Error err;
    uint32_t totalBytes = numBlocks * SDIO_BLOCK_SIZE;

    // 检查是否有传输正在进行
    if (dma_sdio_GetState() == SDIO_DMA_BUSY) {
        return SD_ERR_BUSY;
    }

    uint32_t addr = (cardInfo.CardType == SD_CARD_HIGH_CAPACITY) ?
                    blockAddr : (blockAddr << 9);

    async_block_addr = blockAddr;
    async_num_blocks = numBlocks;

    SDIO->ICR = 0x7FF;
    dma_sdio_ClearState();

    // 预擦除
    SD_SendAppCommand(SD_ACMD_SET_WR_BLK_ERASE_COUNT, numBlocks, SD_RESP_SHORT);

    SD_DMA_Config_Write(buf, totalBytes);

    err = SD_SendCommand(SD_CMD_WRITE_MULT_BLOCK, addr, SD_RESP_SHORT);
    if (err != SD_OK) {
        LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);
        return err;
    }

    SDIO->DTIMER = SDIO_DATATIMER_VALUE;
    SDIO->DLEN = totalBytes;
    SDIO->DCTRL = (9U << SDIO_DCTRL_DBLOCKSIZE_Pos) |
                  SDIO_DCTRL_DMAEN |
                  SDIO_DCTRL_DTEN;

    // 标记为忙状态
    dma_sdio_SetState(SDIO_DMA_BUSY);

    return SD_OK;
}



SD_Error SD_WriteMultiBlock_DMA_Wait(void)
{
    uint32_t actual_sta = 0;
    uint32_t totalBytes = async_num_blocks * SDIO_BLOCK_SIZE;
    SD_Error result = SD_WriteDmaFinish(totalBytes, &actual_sta);

    if (result != SD_OK) {
        printf("[SD] Wait fail: err=%d sta=0x%08lX\r\n",
               (int)result, actual_sta);
    }

    return result;
}

// SD_Error SD_WriteMultiBlock_DMA_Wait(void)
// {
//     uint32_t start_time = s_sd_get_tick ? s_sd_get_tick() : 0;
//     uint32_t fallback_timeout = SDIO_DATA_TIMEOUT;

//     // 1. 等待SDIO传输完成
//     while (!(SDIO->STA & (SDIO_STA_DATAEND | SDIO_STA_TXUNDERR |
//                           SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT)))
//     {
//         if (s_sd_get_tick) {
//             if ((s_sd_get_tick() - start_time) > 5000) {
//                 break; // 超时直接跳出循环，不要用 goto，让下方统一处理
//             }
//         } else {
//             if (--fallback_timeout == 0) {
//                 break;
//             }
//         }

//         if (s_sd_yield) {
//             s_sd_yield();
//         }
//     }

// cleanup:
//     LL_DMA_DisableStream(HW_DMA_SDIO_INSTANCE, HW_DMA_SDIO_STREAM);

//     // 【关键修复 1】：在发送任何指令前，先缓存此时真实的硬件状态！
//     uint32_t actual_sta = SDIO->STA;

//     // 发送停止传输指令（内部会引发 ICR 清理）
//     SD_SendCommand(SD_CMD_STOP_TRANSMISSION, 0, SD_RESP_SHORT);

//     SD_Error result = SD_OK;

//     // 【关键修复 2】：使用缓存的 actual_sta 进行校验
//     if (actual_sta & SDIO_STA_DTIMEOUT) {
//         result = SD_ERR_TIMEOUT;
//     } else if (actual_sta & SDIO_STA_DCRCFAIL) {
//         result = SD_ERR_CRC;
//     } else if (actual_sta & SDIO_STA_TXUNDERR) {
//         result = SD_ERR_WRITE; // 发生欠载，说明 DMA 没喂上数据
//     } else if (!(actual_sta & SDIO_STA_DATAEND)) {
//         result = SD_ERR_WRITE;
//     }

//     SDIO->ICR = 0x7FF;
//     dma_sdio_ClearState();

//     return result;
// }

// 非阻塞检查（用于主循环轮询）
SD_Error SD_WriteMultiBlock_DMA_Poll(void)
{
    // 检查是否还在传输
    if (!(SDIO->STA & (SDIO_STA_DATAEND | SDIO_STA_TXUNDERR |
                       SDIO_STA_DCRCFAIL | SDIO_STA_DTIMEOUT))) {
        return SD_ERR_BUSY;  // 还在传输中
    }

    // 传输结束，清理
    return SD_WriteMultiBlock_DMA_Wait();
}


/*============ 飞控日志刷新函数（DMA版本） ============*/
void FlightLog_Flush_DMA(uint8_t *buffer, uint32_t startBlock)
{
    if (SD_CheckReady() == SD_ERR_BUSY)
    {
        return;
    }

    SD_WriteMultiBlock_DMA(startBlock, buffer, LOG_BUFFER_BLOCKS);
}
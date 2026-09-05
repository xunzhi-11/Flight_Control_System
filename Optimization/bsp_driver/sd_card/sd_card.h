#ifndef __SD_CARD_H
#define __SD_CARD_H

#include <stdint.h>

/*============ SD命令定义 ============*/
#define SD_CMD_GO_IDLE_STATE        0
#define SD_CMD_ALL_SEND_CID         2
#define SD_CMD_SEND_REL_ADDR        3
#define SD_CMD_SEL_DESEL_CARD       7
#define SD_CMD_SEND_IF_COND         8
#define SD_CMD_SEND_CSD             9
#define SD_CMD_STOP_TRANSMISSION    12
#define SD_CMD_SEND_STATUS          13
#define SD_CMD_SET_BLOCKLEN         16
#define SD_CMD_READ_SINGLE_BLOCK    17
#define SD_CMD_READ_MULT_BLOCK      18
#define SD_CMD_WRITE_SINGLE_BLOCK   24
#define SD_CMD_WRITE_MULT_BLOCK     25
#define SD_CMD_APP_CMD              55

#define SD_ACMD_SET_BUS_WIDTH       6
#define SD_ACMD_SD_SEND_OP_COND     41

/*============ 响应类型 ============*/
#define SD_RESP_NONE                0x00
#define SD_RESP_SHORT               0x40
#define SD_RESP_LONG                0xC0

/*============ 卡类型 ============*/
#define SD_CARD_STD_CAPACITY_V1     0
#define SD_CARD_STD_CAPACITY_V2     1
#define SD_CARD_HIGH_CAPACITY       2



/*============ 多区块处理 ============*/
#define SD_CMD_WRITE_MULT_BLOCK     25
#define SD_ACMD_SET_WR_BLK_ERASE_COUNT  23
#define LOG_BUFFER_BLOCKS  4  // 2KB缓冲

typedef void (*sd_card_delay_us_f)(uint32_t us) ;
typedef void (*sd_card_delay_ms_f)(uint32_t ms) ;
typedef uint32_t (*sd_get_tick_f)(void); // 返回毫秒级时间戳
typedef void     (*sd_yield_f)(void);    // 线程挂起/让出CPU

/*============ 错误码 ============*/
typedef enum {
    SD_OK = 0,
    SD_ERR_TIMEOUT,
    SD_ERR_CRC,
    SD_ERR_CMD,
    SD_ERR_NO_CARD,
    SD_ERR_UNSUPPORTED,
    SD_ERR_WRITE,
    SD_ERR_READ,
    SD_ERR_BUSY
} SD_Error;

/*============ SD卡信息结构 ============*/
typedef struct {
    uint32_t RCA;
    uint32_t CardType;
    uint32_t BlockSize;
    uint32_t BlockCount;
    uint64_t Capacity;
} SD_CardInfo;


/*============ 函数声明 ============*/
SD_Error sd_card_init(void);
SD_Error SD_ReadBlock_DMA(uint32_t blockAddr, uint8_t *buf);
SD_Error SD_WriteBlock_DMA(uint32_t blockAddr, const uint8_t *buf);
SD_CardInfo* SD_GetCardInfo(void);
SD_Error SD_CheckReady(void);
void FlightLog_Flush_DMA(uint8_t *buffer, uint32_t startBlock);
uint32_t SD_GetResponse(uint8_t respReg) ;
void SD_Card_Delay_us_Register(sd_card_delay_us_f func) ;
void SD_Card_Delay_ms_Register(sd_card_delay_ms_f  func) ;
SD_Error SD_WriteMultiBlock_DMA(uint32_t blockAddr, const uint8_t *buf, uint32_t numBlocks);
void SD_Register_OS_Hooks(sd_get_tick_f get_tick_cb, sd_yield_f yield_cb);

// 异步写入API
SD_Error SD_WriteMultiBlock_DMA_Start(uint32_t blockAddr, const uint8_t *buf, uint32_t numBlocks);
SD_Error SD_WriteMultiBlock_DMA_Wait(void);
SD_Error SD_WriteMultiBlock_DMA_Poll(void);



#endif
#include "app_task/app_task_log.h"
#include "sys_software_config.h"
#include "sys_hardware_config.h"
#include "sd_card/sd_card.h"
#include "fatfs/ff.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"
#include "stm32f4xx_ll_usart.h"
#include "pid_trace/pid_trace_config.h"
#if PIDTRACE_ENABLE
#include "pid_trace/pid_trace.h"
#endif

/* FatFs 对象放静态区，避免 log_task 栈溢出（LFN 开启时 FIL/FATFS 很大） */
static FATFS s_log_fs;
static FIL s_log_file;

static void log_uart_putc(char c)
{
#if PIDTRACE_ENABLE
    if (PidTrace_IsDumping()) {
        return;
    }
#endif
    while (!LL_USART_IsActiveFlag_TXE(HW_UART_DEBUG_INSTANCE)) {
        taskYIELD();
    }
    LL_USART_TransmitData8(HW_UART_DEBUG_INSTANCE, (uint8_t)c);
}

static void log_uart_puts(const char *s)
{
    if (s == NULL) {
        return;
    }
    while (*s) {
        log_uart_putc(*s++);
    }
}

// 定义二进制日志帧
#pragma pack(push, 1) // 必须确保字节对齐，防止内存空洞
typedef struct {
    uint32_t timestamp;    // 系统时间戳
    int16_t  accel[3];     // 加速度计
    int16_t  gyro[3];      // 陀螺仪
    float    quat[4];      // 四元数
    uint32_t log_index;    // 帧序列号
} FlightDataPacket_t;
#pragma pack(pop)

TaskHandle_t log_taskHandle = NULL;

// 使用 32 字节对齐，命中 DMA 最佳性能路径
__attribute__((aligned(32), section(".sram_data"))) uint8_t log_buf[512 * LOG_BUFFER_BLOCKS] = {0};

static DWORD log_start_sector = 0;
static DWORD current_write_sector = 0;
static DWORD max_sectors = 0;
static uint8_t raw_logging_ready = 0;

#define PRE_ALLOC_SIZE (10 * 1024 * 1024) // 10MB 测试文件

static __attribute__((aligned(32))) uint8_t sd_probe_buf[512];

static void sd_probe_volume(void)
{
    SD_Error err = SD_ReadBlock_DMA(0, sd_probe_buf);
    printf("[Log Init] probe LBA0 err=%d sig=%02X%02X b0=%02X\r\n",
           (int)err, sd_probe_buf[510], sd_probe_buf[511], sd_probe_buf[0]);

    if (err != SD_OK) {
        return;
    }

    if (sd_probe_buf[510] == 0x55 && sd_probe_buf[511] == 0xAA) {
        DWORD pbr_lba = (DWORD)sd_probe_buf[454]
                      | ((DWORD)sd_probe_buf[455] << 8)
                      | ((DWORD)sd_probe_buf[456] << 16)
                      | ((DWORD)sd_probe_buf[457] << 24);
        printf("[Log Init] probe MBR part1 LBA=%lu\r\n", (unsigned long)pbr_lba);

        err = SD_ReadBlock_DMA(pbr_lba, sd_probe_buf);
        printf("[Log Init] probe PBR err=%d oem=%.5s fs=%.8s\r\n",
               (int)err, sd_probe_buf + 3, sd_probe_buf + 82);
    }
}

/**
 * @brief 预分配连续空间并锁定物理扇区
 */
static void setup_raw_logging(void)
{
    FRESULT f_res;

    log_uart_puts("[Log] setup begin\r\n");

    f_res = f_mount(&s_log_fs, "0:", 1);
    log_uart_puts("[Log] post-mount\r\n");
    printf("[Log Init] f_mount res=%d\r\n", (int)f_res);
    if (f_res != FR_OK) {
        sd_probe_volume();
        printf("[Log Init] hint: res=13 means no FAT, format SD as FAT32\r\n");
        return;
    }

    log_uart_puts("[Log] f_open begin\r\n");
    f_res = f_open(&s_log_file, "0:BLACKBOX.BIN", FA_WRITE | FA_CREATE_ALWAYS);
    log_uart_puts("[Log] f_open done\r\n");
    printf("[Log Init] f_open res=%d\r\n", (int)f_res);
    if (f_res != FR_OK) {
        f_mount(NULL, "0:", 0);
        return;
    }

    f_res = f_expand(&s_log_file, PRE_ALLOC_SIZE, 1);
    printf("[Log Init] f_expand res=%d\r\n", (int)f_res);
    if (f_res == FR_OK) {
        DWORD first_cluster = s_log_file.obj.sclust;
        log_start_sector = s_log_fs.database + ((DWORD)s_log_fs.csize * (first_cluster - 2));
        max_sectors = PRE_ALLOC_SIZE / 512;
        current_write_sector = log_start_sector;
        raw_logging_ready = 1;
        printf("[Log Init] Ready. Sector=%lu cluster=%lu csize=%u\r\n",
               log_start_sector, (unsigned long)first_cluster, (unsigned)s_log_fs.csize);
    }

    f_res = f_close(&s_log_file);
    printf("[Log Init] f_close res=%d\r\n", (int)f_res);

    f_res = f_mount(NULL, "0:", 0);
    printf("[Log Init] f_unmount res=%d\r\n", (int)f_res);
    printf("[Log Init] setup done, ready=%u\r\n", (unsigned)raw_logging_ready);
}

static void log_task(void* pvParameters)
{
    uint32_t frame_count = 0;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    const TickType_t xFrequency = pdMS_TO_TICKS(5000);

    setup_raw_logging();

    if (!raw_logging_ready) {
        printf("[Log] init failed, task exit\r\n");
        vTaskDelete(NULL);
    }

    printf("[Log] enter write loop\r\n");

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FlightDataPacket_t* pkt_ptr = (FlightDataPacket_t*)log_buf;
        memset(log_buf, 0, sizeof(log_buf));

        pkt_ptr->timestamp = xTaskGetTickCount();
        pkt_ptr->accel[0] = 100;
        pkt_ptr->log_index = frame_count++;

        uint32_t wait_start = xTaskGetTickCount();
        SD_Error ready_err = SD_OK;
        while ((ready_err = SD_CheckReady()) != SD_OK) {
            vTaskDelay(pdMS_TO_TICKS(1));
            if (xTaskGetTickCount() - wait_start > 500) {
                printf("[Log] SD_CheckReady timeout, last=%d\r\n", (int)ready_err);
                break;
            }
        }

        if ((current_write_sector - log_start_sector) + LOG_BUFFER_BLOCKS >= max_sectors) {
            printf("[Log] Disk Full\r\n");
            vTaskSuspend(NULL);
        }

        printf("[Log] write LBA=%lu blocks=%d\r\n",
               (unsigned long)current_write_sector, LOG_BUFFER_BLOCKS);

        SD_Error start_err = SD_WriteMultiBlock_DMA_Start(
            current_write_sector, log_buf, LOG_BUFFER_BLOCKS);
        if (start_err != SD_OK) {
            printf("[Log] Start fail err=%d\r\n", (int)start_err);
            continue;
        }

        SD_Error wait_err = SD_WriteMultiBlock_DMA_Wait();
        if (wait_err == SD_OK) {
            current_write_sector += LOG_BUFFER_BLOCKS;
            printf("[Log] write ok, next LBA=%lu index=%lu\r\n",
                   (unsigned long)current_write_sector,
                   (unsigned long)pkt_ptr->log_index);
        } else {
            printf("[Log] Wait fail err=%d\r\n", (int)wait_err);
        }
    }
}

void task_log_init(void)
{
    xTaskCreate(log_task , "log_task" , 4096 , NULL , SW_TASK_PRORITY_LOG , &log_taskHandle) ;
}

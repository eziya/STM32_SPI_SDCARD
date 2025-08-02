#define TRUE  1
#define FALSE 0
#define bool BYTE

#include "stm32f4xx_hal.h"

#include "diskio.h"
#include "fatfs_sd.h"

// FIX 1: volatile 키워드 추가
// 인터럽트에 의해 값이 변경되는 변수는 컴파일러 최적화로 인한 오작동을 막기 위해 volatile로 선언해야 합니다.
volatile uint16_t Timer1, Timer2;           /* 1ms Timer Counter */

static volatile DSTATUS Stat = STA_NOINIT;  /* Disk Status */
static uint8_t CardType;                    /* Type 0:MMC, 1:SDC, 2:Block addressing */
static uint8_t PowerFlag = 0;               /* Power flag */

/***************************************
 * SPI functions
 **************************************/

/* slave select */
static void SELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

/* slave deselect */
static void DESELECT(void)
{
    HAL_GPIO_WritePin(SD_CS_PORT, SD_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
}

/* SPI transmit a byte */
static void SPI_TxByte(uint8_t data)
{
    while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
    HAL_SPI_Transmit(HSPI_SDCARD, &data, 1, SPI_TIMEOUT);
}

/* SPI transmit buffer */
static void SPI_TxBuffer(uint8_t *buffer, uint16_t len)
{
    while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
    HAL_SPI_Transmit(HSPI_SDCARD, buffer, len, SPI_TIMEOUT);
}

/* SPI receive a byte */
static uint8_t SPI_RxByte(void)
{
    uint8_t dummy, data;
    dummy = 0xFF;

    while(!__HAL_SPI_GET_FLAG(HSPI_SDCARD, SPI_FLAG_TXE));
    HAL_SPI_TransmitReceive(HSPI_SDCARD, &dummy, &data, 1, SPI_TIMEOUT);

    return data;
}

/* SPI receive a byte via pointer */
static void SPI_RxBytePtr(uint8_t *buff) 
{
    *buff = SPI_RxByte();
}

/***************************************
 * SD functions
 **************************************/

/* wait SD ready */
static uint8_t SD_ReadyWait(void)
{
    uint8_t res;

    /* timeout 500ms */
    Timer2 = 500;

    /* if SD goes ready, receives 0xFF */
    do {
        res = SPI_RxByte();
    } while ((res != 0xFF) && Timer2);

    return res;
}

/* power on */
static void SD_PowerOn(void) 
{
    uint8_t args[6];
    uint32_t cnt = 0x1FFF;

    /* transmit bytes to wake up */
    DESELECT();
    for(int i = 0; i < 10; i++)
    {
        SPI_TxByte(0xFF);
    }

    /* slave select */
    SELECT();

    /* make idle state */
    args[0] = CMD0;     /* CMD0:GO_IDLE_STATE */
    args[1] = 0;
    args[2] = 0;
    args[3] = 0;
    args[4] = 0;
    args[5] = 0x95;     /* CRC */

    SPI_TxBuffer(args, sizeof(args));

    /* wait response */
    while ((SPI_RxByte() != 0x01) && cnt)
    {
        cnt--;
    }

    DESELECT();
    SPI_TxByte(0XFF);

    PowerFlag = 1;
}

/* power off */
static void SD_PowerOff(void) 
{
    PowerFlag = 0;
}

/* check power flag */
static uint8_t SD_CheckPower(void) 
{
    return PowerFlag;
}

/* receive data block */
static bool SD_RxDataBlock(BYTE *buff, UINT len)
{
    uint8_t token;

    /* timeout 200ms */
    Timer1 = 200;

    /* loop until receive a response or timeout */
    do {
        token = SPI_RxByte();
    } while((token == 0xFF) && Timer1);

    /* invalid response */
    if(token != 0xFE) return FALSE;

    /* receive data */
    // FIX 2: do-while(len--) 루프 수정
    // 기존 코드는 len+1 만큼 실행되어 버퍼 오버플로우를 유발할 수 있습니다.
    // while(len--) 형태로 변경하여 정확히 len 만큼만 실행되도록 합니다.
    while(len--) {
        SPI_RxBytePtr(buff++);
    }

    /* discard CRC */
    SPI_RxByte();
    SPI_RxByte();

    return TRUE;
}

/* transmit data block */
#if _USE_WRITE == 1
static bool SD_TxDataBlock(const uint8_t *buff, BYTE token)
{
    uint8_t resp = 0xFF; // 초기화
    uint8_t i = 0;

    /* wait SD ready */
    if (SD_ReadyWait() != 0xFF) return FALSE;

    /* transmit token */
    SPI_TxByte(token);

    /* if it's not STOP token, transmit data */
    if (token != 0xFD)
    {
        SPI_TxBuffer((uint8_t*)buff, 512);

        /* discard CRC */
        SPI_RxByte();
        SPI_RxByte();

        /* receive response */
        while (i <= 64)
        {
            resp = SPI_RxByte();

            /* transmit 0x05 accepted */
            if ((resp & 0x1F) == 0x05) break;
            i++;
        }

        // FIX 3: 타임아웃 없는 무한 루프 수정
        // 카드가 계속 busy(0x00) 상태일 경우 시스템이 멈추는 것을 방지하기 위해 타임아웃을 추가합니다.
        Timer1 = 200; // 200ms 타임아웃
        while ((SPI_RxByte() == 0) && Timer1);
    }
    
    // resp가 초기화되지 않은 상태로 사용될 수 있어 수정
    if ((resp & 0x1F) == 0x05) return TRUE;

    return FALSE;
}
#endif /* _USE_WRITE */

/* transmit command */
static BYTE SD_SendCmd(BYTE cmd, uint32_t arg)
{
    uint8_t crc, res;

    /* wait SD ready */
    if (SD_ReadyWait() != 0xFF) return 0xFF;

    /* transmit command */
    SPI_TxByte(cmd);                    /* Command */
    SPI_TxByte((uint8_t)(arg >> 24));   /* Argument[31..24] */
    SPI_TxByte((uint8_t)(arg >> 16));   /* Argument[23..16] */
    SPI_TxByte((uint8_t)(arg >> 8));    /* Argument[15..8] */
    SPI_TxByte((uint8_t)arg);           /* Argument[7..0] */

    /* prepare CRC */
    if(cmd == CMD0) crc = 0x95; /* CRC for CMD0(0) */
    else if(cmd == CMD8) crc = 0x87;    /* CRC for CMD8(0x1AA) */
    else crc = 1;

    /* transmit CRC */
    SPI_TxByte(crc);

    /* Skip a stuff byte when STOP_TRANSMISSION */
    if (cmd == CMD12) SPI_RxByte();

    /* receive response */
    uint8_t n = 10;
    do {
        res = SPI_RxByte();
    } while ((res & 0x80) && --n);

    return res;
}

/***************************************
 * user_diskio.c functions
 **************************************/

/* initialize SD */
DSTATUS SD_disk_initialize(BYTE drv) 
{
    uint8_t n, type, ocr[4];

    /* single drive, drv should be 0 */
    if(drv) return STA_NOINIT;

    /* no disk */
    if(Stat & STA_NODISK) return Stat;

    /* power on */
    SD_PowerOn();

    /* slave select */
    SELECT();

    /* check disk type */
    type = 0;

    /* send GO_IDLE_STATE command */
    if (SD_SendCmd(CMD0, 0) == 1)
    {
        /* timeout 1 sec */
        Timer1 = 1000;

        /* SDC V2+ accept CMD8 command, http://elm-chan.org/docs/mmc/mmc_e.html */
        if (SD_SendCmd(CMD8, 0x1AA) == 1)
        {
            /* operation condition register */
            for (n = 0; n < 4; n++)
            {
                ocr[n] = SPI_RxByte();
            }

            /* voltage range 2.7-3.6V */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA)
            {
                /* ACMD41 with HCS bit */
                do {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 1UL << 30) == 0) break;
                } while (Timer1);

                /* READ_OCR */
                if (Timer1 && SD_SendCmd(CMD58, 0) == 0)
                {
                    /* Check CCS bit */
                    for (n = 0; n < 4; n++)
                    {
                        ocr[n] = SPI_RxByte();
                    }

                    /* SDv2 (HC or SC) */
                    type = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else
        {
            /* SDC V1 or MMC */
            type = (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) <= 1) ? CT_SD1 : CT_MMC;

            do
            {
                if (type == CT_SD1)
                {
                    if (SD_SendCmd(CMD55, 0) <= 1 && SD_SendCmd(CMD41, 0) == 0) break; /* ACMD41 */
                }
                else
                {
                    if (SD_SendCmd(CMD1, 0) == 0) break; /* CMD1 */
                }

            } while (Timer1);

            /* SET_BLOCKLEN */
            if (!Timer1 || SD_SendCmd(CMD16, 512) != 0) type = 0;
        }
    }

    CardType = type;

    /* Idle */
    DESELECT();
    SPI_RxByte();

    /* Clear STA_NOINIT */
    if (type)
    {
        Stat &= ~STA_NOINIT;
    }
    else
    {
        /* Initialization failed */
        SD_PowerOff();
    }

    return Stat;
}

/* return disk status */
DSTATUS SD_disk_status(BYTE drv) 
{
    if (drv) return STA_NOINIT;
    return Stat;
}

/* read sector */
DRESULT SD_disk_read(BYTE pdrv, BYTE* buff, DWORD sector, UINT count) 
{
    /* pdrv should be 0 */
    if (pdrv || !count) return RES_PARERR;

    /* no disk */
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    /* convert to byte address */
    if (!(CardType & CT_BLOCK)) sector *= 512;

    SELECT();

    if (count == 1)
    {
        /* READ_SINGLE_BLOCK */
        if ((SD_SendCmd(CMD17, sector) == 0) && SD_RxDataBlock(buff, 512)) count = 0;
    }
    else
    {
        /* READ_MULTIPLE_BLOCK */
        if (SD_SendCmd(CMD18, sector) == 0)
        {
            do {
                if (!SD_RxDataBlock(buff, 512)) break;
                buff += 512;
            } while (--count);

            /* STOP_TRANSMISSION */
            SD_SendCmd(CMD12, 0);
        }
    }

    /* Idle */
    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}

/* write sector */
#if _USE_WRITE == 1
DRESULT SD_disk_write(BYTE pdrv, const BYTE* buff, DWORD sector, UINT count) 
{
    /* pdrv should be 0 */
    if (pdrv || !count) return RES_PARERR;

    /* no disk */
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    /* write protection */
    if (Stat & STA_PROTECT) return RES_WRPRT;

    /* convert to byte address */
    if (!(CardType & CT_BLOCK)) sector *= 512;

    SELECT();

    if (count == 1)
    {
        /* WRITE_BLOCK */
        if ((SD_SendCmd(CMD24, sector) == 0) && SD_TxDataBlock(buff, 0xFE))
            count = 0;
    }
    else
    {
        /* WRITE_MULTIPLE_BLOCK */
        if (CardType & CT_SD1)
        {
            SD_SendCmd(CMD55, 0);
            SD_SendCmd(CMD23, count); /* ACMD23 */
        }

        if (SD_SendCmd(CMD25, sector) == 0)
        {
            do {
                if(!SD_TxDataBlock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);

            /* STOP_TRAN token */
            if(!SD_TxDataBlock(0, 0xFD))
            {
                count = 1;
            }
        }
    }

    /* Idle */
    DESELECT();
    SPI_RxByte();

    return count ? RES_ERROR : RES_OK;
}
#endif /* _USE_WRITE */

/* ioctl */
DRESULT SD_disk_ioctl(BYTE drv, BYTE ctrl, void *buff) 
{
    DRESULT res;
    uint8_t n, csd[16], *ptr = buff;
    
    /* pdrv should be 0 */
    if (drv) return RES_PARERR;
    res = RES_ERROR;

    if (ctrl == CTRL_POWER)
    {
        switch (*ptr)
        {
        case 0:
            SD_PowerOff();      /* Power Off */
            res = RES_OK;
            break;
        case 1:
            SD_PowerOn();       /* Power On */
            res = RES_OK;
            break;
        case 2:
            *(ptr + 1) = SD_CheckPower();
            res = RES_OK;       /* Power Check */
            break;
        default:
            res = RES_PARERR;
        }
    }
    else
    {
        /* no disk */
        if (Stat & STA_NOINIT) return RES_NOTRDY;

        SELECT();

        switch (ctrl)
        {
        case GET_SECTOR_COUNT:
            /* SEND_CSD */
            if ((SD_SendCmd(CMD9, 0) == 0) && SD_RxDataBlock(csd, 16))
            {
                if ((csd[0] >> 6) == 1) /* SDC V2 */
                {
                    // FIX 5: SDv2 CSD 파싱 및 용량 계산 로직 수정
                    // 기존 로직은 C_SIZE 필드를 일부만 사용하여 대용량 카드에서 용량을 잘못 계산합니다.
                    DWORD c_size;
                    c_size = (DWORD)(csd[7] & 0x3F) << 16 | (WORD)csd[8] << 8 | csd[9];
                    *(DWORD*)buff = (c_size + 1) << 10;
                }
                else /* MMC or SDC V1 */
                {
                    WORD csize;
                    n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
                    csize = (csd[8] >> 6) + ((WORD) csd[7] << 2) + ((WORD) (csd[6] & 3) << 10) + 1;
                    *(DWORD*) buff = (DWORD) csize << (n - 9);
                }
                res = RES_OK;
            }
            break;
        case GET_SECTOR_SIZE:
            *(WORD*) buff = 512;
            res = RES_OK;
            break;
        case CTRL_SYNC:
            if (SD_ReadyWait() == 0xFF) res = RES_OK;
            break;
        case MMC_GET_CSD:
            /* SEND_CSD */
            if (SD_SendCmd(CMD9, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
            break;
        case MMC_GET_CID:
            /* SEND_CID */
            if (SD_SendCmd(CMD10, 0) == 0 && SD_RxDataBlock(ptr, 16)) res = RES_OK;
            break;
        case MMC_GET_OCR:
            /* READ_OCR */
            if (SD_SendCmd(CMD58, 0) == 0)
            {
                for (n = 0; n < 4; n++)
                {
                    *ptr++ = SPI_RxByte();
                }
                res = RES_OK;
            }
            // FIX 4: 누락된 break 추가
            // break가 없어 default case로 넘어가 res값이 RES_PARERR로 덮어쓰이는 문제를 수정합니다.
            break; 
        default:
            res = RES_PARERR;
        }

        DESELECT();
        SPI_RxByte();
    }

    return res;
}

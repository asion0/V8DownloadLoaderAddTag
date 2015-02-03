/*******************************************************************************
 * Copyright (c) 2006 SkyTraq Technology, Inc. Hsinchu, Taiwan
 * All Rights Reserved.
 * 
 * file name: flashloader.h
 * Initial: 
 * Current: 
 * Description: 
 ******************************************************************************/

#ifndef FLASHLOADER_INCLUDED
#define FLASHLOADER_INCLUDED

//---------- head file inclusions --------------------------
#include "st_type.h"
#include "st_const.h"
#define	UART1_RBR			0x2400
#define	UART1_THR			0x2400
#define	UART1_IER			0x2404
#define	UART1_IIR			0x2408
#define	UART1_FCR			0x2408
#define	UART1_LCR			0x240c
#define	UART1_MCR			0x2410
#define	UART1_LSR			0x2414
#define	UART1_MSR			0x2418
#define	UART1_SCR			0x241c
#define	UART1_DLL			0x2400
#define	UART1_DLM			0x2404
#define	UART1_RBBC			0x2420
#define	UART1_TBBC			0x2424
#define	UART1_PISR			0x2428
#define	UART1_CESR			0x242c
//---------- constant definitions --------------------------
#define 		FLASHTESTSTARADDR		0x0 
#define MX_FLASH       0xC2 
#define	MX29LV400TT			0xB9
#define FJ_FLASH       0x4  /* manufacturing id */
#define	FJ29LV400B			0xBA	/* device code 29LV400B */
#define AMIC_FLASH     0x37
#define NUMONYX        0x0089
#define EON_FLASH      0x1C
#define SST_FLASH      0x20
#define M29W400DB      0xef
#define M29W400TB      0xee
#define A29L400ATV     0x34
#define A29LV400AT     0xB9
#define A29LV800AT     0xDA
#define A29LV400AB     0xBA
#define A29L400AUV     0xB5
//numonyx
#define JS28F256P30T   0x8919
#define	AT49BV163A			0xC0	/* device code 29LV400TT */
#define AT_FLASH       0x1F  /* manufacturing id */
//#define                 FLASH_WIDTH                     4
#define                 FLASH_WIDTH                     2
#define                 FLASH_CHIP_WIDTH                2

#if     (FLASH_WIDTH == 4)
#define FLASH_DEF      volatile unsigned int
#define FLASH_CAST      (volatile unsigned int *)
#define FLASH_T      unsigned long
#endif  /* FLASH_WIDTH */

#if     (FLASH_WIDTH == 2)
#define FLASH_DEF      volatile unsigned short
#define FLASH_CAST      (volatile unsigned short *)
#define FLASH_T      unsigned short
#endif  /* FLASH_WIDTH */

#define CAST_CH (char *)
#ifndef FLASH29_REG_ADRS
#define FLASH29_REG_ADRS(reg) (CAST_CH FLASHTESTSTARADDR + (reg * FLASH_WIDTH))
//#define FLASH29_REG_ADRS(reg) (FLASHTESTSTARADDR + (reg * FLASH_WIDTH))
#endif  /* FLASH29_REG_ADRS */

#define FLASH29_REG_FIRST_CYCLE		FLASH29_REG_ADRS (0x555)
#define FLASH29_REG_SECOND_CYCLE	FLASH29_REG_ADRS (0x2aa)
#define FLASH29_REG_ID_CYCLE	FLASH29_REG_ADRS (0x100)

#define	FLASH29_CMD_FIRST		(FLASH_DEF) 0xaaaaaaaa
#define	FLASH29_CMD_SECOND		(FLASH_DEF) 0x55555555
#define	FLASH29_CMD_FOURTH		(FLASH_DEF) 0xaaaaaaaa
#define	FLASH29_CMD_FIFTH		(FLASH_DEF) 0x55555555
#define	FLASH29_CMD_SIXTH		(FLASH_DEF) 0x10101010
#define	FLASH29_CMD_SECTOR		(FLASH_DEF) 0x30303030

#define	FLASH29_CMD_PROGRAM		(FLASH_DEF) 0xa0a0a0a0
#define	FLASH29_CMD_CHIP_ERASE		(FLASH_DEF) 0x80808080
#define	FLASH29_CMD_READ_RESET		(FLASH_DEF) 0xf0f0f0f0
#define	FLASH29_CMD_AUTOSELECT		(FLASH_DEF) 0x90909090

#define FLASH49_REG_FIRST_CYCLE		FLASH29_REG_ADRS (0x555)
#define FLASH49_REG_SECOND_CYCLE	FLASH29_REG_ADRS (0xaaa)

#define	FLASH49_CMD_FIRST		(FLASH_DEF) 0xaaaaaaaa
#define	FLASH49_CMD_SECOND		(FLASH_DEF) 0x55555555
#define	FLASH49_CMD_FOURTH		(FLASH_DEF) 0xaaaaaaaa
#define	FLASH49_CMD_FIFTH		(FLASH_DEF) 0x55555555
#define	FLASH49_CMD_SIXTH		(FLASH_DEF) 0x10101010
#define	FLASH49_CMD_SECTOR		(FLASH_DEF) 0x30303030

#define	FLASH49_CMD_PROGRAM		(FLASH_DEF) 0xa0a0a0a0
#define	FLASH49_CMD_CHIP_ERASE		(FLASH_DEF) 0x80808080
#define	FLASH49_CMD_READ_RESET		(FLASH_DEF) 0xf0f0f0f0
#define	FLASH49_CMD_AUTOSELECT		(FLASH_DEF) 0x90909090

//nuymonx
#define JS_CMD_READ_DID                  0x0090
#define JS_CMD_BLOCK_ERASE               0x0020
#define JS_DATA_BLOCK_ERASE              0x00d0
#define JS_CMD_CLEAR_STATUS_REG          0x0050
#define JS_CMD_PROGRAM_WORD              0x0040
#define JS_CMD_BLOCK_UNLOCK               0x0060
#define JS_DATA_BLOCK_UNLOCK               0x00d0

#define Q7(ix)      ((ix & 0x80) >> 7)  /* DQ7 bit */
#define Q5(ix)      ((ix & 0x20) >> 5)  /* DQ5 bit */
#define Q6(ix)      ((ix & 0x40) >> 6)  /* DQ6 bit */
#define Q4(ix)      ((ix & 0x10) >> 4)  /* DQ6 bit */
#define Q3(ix)      ((ix & 0x08) >> 3)  /* DQ6 bit */
#define Q2(ix)      ((ix & 0x04) >> 2)  /* DQ6 bit */
#define Q1(ix)      ((ix & 0x02) >> 1)  /* DQ6 bit */


#define REG_BASE_ADDR 0x80000000
#define U1DATA 0x70
#define U1STATUS 0x74
#define U1CONTROL 0x78
#define U1SCALER 0x7C
#define MCFG1 0x0
#define CACFG 0x14
//#define IMASK  0x90
#define IMASK  0x240
#define IMASK_LEON2  0x90

#define REG_BASE_MEMORYIO_ADDR 0x20000000
//#define FIFO2

#define I2C_S_GPREG0          (REG_BASE_MEMORYIO_ADDR+0x2300)
#define I2C_S_GPREG1          (REG_BASE_MEMORYIO_ADDR+0x2304)
#define I2C_S_GPREG2          (REG_BASE_MEMORYIO_ADDR+0x2308)
#define I2C_S_GPREG3          (REG_BASE_MEMORYIO_ADDR+0x230c)
#define I2C_S_GPREG4          (REG_BASE_MEMORYIO_ADDR+0x2310)
#define I2C_S_GPREG5          (REG_BASE_MEMORYIO_ADDR+0x2314)
#define I2C_S_GPREG6          (REG_BASE_MEMORYIO_ADDR+0x2318)
#define I2C_S_GPREG7          (REG_BASE_MEMORYIO_ADDR+0x231c)
#define I2C_S_CONTROL          (0x2320)
#define I2C_S_MAIN_CONTROL          (REG_BASE_MEMORYIO_ADDR+I2C_S_CONTROL)
#define IRQ_CTRL_I2C_SLAVE_IRQ_EN_BIT  4

 

//---------- macro definitions -----------------------------

//---------- type definitions ------------------------------

typedef struct flash_dev_s {
  U32		base;			/* Base address */
  U32		sectors;		/* Sector count */
  U32		lgSectorSize;		/* Log2(usable bytes/sector) */
  U32		vendorID;		/* Expected vendor ID */
  U32		deviceID;		/* Expected device ID */
} FL_DEV_T;



typedef struct {
  U32 addr;
  U32 size;
  U32 type;
  U08 *data;
  U08 *mem;
} RECBUF_T;

//qspi
//for SPI flash
#define GPIO_QSPI_ADDR 0x20001000UL

//spi flash
//spi

#define     SPIFLASHTESTSTARADDR    0x0
#define SPI_RX0 0x2000
#define SPI_TX0 0x2000
#define SPI_CTRL 0x2010
#define SPI_DIVIDER 0x2014
#define SPI_SS 0x2018

#define SPI_CS_ACTIVATE                 0x01
#define SPI_CS_DEACTIVATE                 0x0

//spi define
#define SPI_DELAY       0
#define CK_DIR          13
#define CSN_DIR         12
#define CK_DATA         5
#define CSN_DATA        4
#define SPI_DATA_DIR    8
#define SPI_DATA_BIT0   0
#define SW_CTL          31




#define EONMID              0x1C
#define	EON25Q40			0x3013	/* Serial FLASH device code  */
#define	EON25Q80			0x3014	/* Serial FLASH device code  */
#define WINBONDMID              0xEF
#define	W25Q80			0x4014	/* Serial FLASH device code  */


#define EON25Q40_CMD_WRENABLE   0x06
#define EON25Q40_CMD_READID   0x9F
#define EON25Q40_CMD_SECTOR_ERASE   0x20
#define EON25Q40_CMD_BLOCK_ERASE   0xd8
#define EON25Q40_CMD_RDSTATUS   0x05
#define EON25Q40_CMD_PAGE_PROGRAM   0x02
#define EON25Q40_CMD_FAST_READ   0x0B
#define EON25Q40_CMD_CHIP_ERASE  0xC7


//winbond
#define W25Q40_CMD_WRENABLE   0x06
#define W25Q40_CMD_WRENABLE_VOLATILE   0x50
#define W25Q40_CMD_READID   0x9F
#define W25Q40_CMD_READ_STATUS_REG_1 0x5
#define W25Q40_CMD_READ_STATUS_REG_2 0x35
#define W25Q40_CMD_WRITE_STATUS_REG 0x1
#define W25Q40_CMD_CHIP_ERASE  0xC7
#define W25Q40_CMD_QUAD_FAST_READ_QUAD   0x6B
#define W25Q40_CMD_QUAD_FAST_READ_QUAD_IO   0xEB
#define W25Q40_CMD_QUAD_FAST_WORD_READ_QUAD_IO   0xE7
#define W25Q40_CMD_SECTOR_ERASE   0x20
#define W25Q40_CMD_BLOCK_ERASE   0xd8
#define W25Q40_CMD_32K_BLOCK_ERASE   0x52
#define W25Q40_CMD_QUAD_PAGE_PROGRAM   0x32
#define W25Q80_CMD_ENABLE_RESET   0x66
#define W25Q80_CMD_RESET   0x99


#define QSPI_WAITTIME 2000000

#define SPI_S_MAIN          (0x20002200)
#define SPI_S_WBUF_SET      (0x20002201)
#define SPI_S_RBUF_STATUS   (0x20002202)
#define SPI_S_MISC          (0x20002203)
#define SPI_S_SW0           (0x20002204)
#define SPI_S_SW1           (0x20002205)
#define SPI_S_SW2           (0x20002206)
#define SPI_S_SW3           (0x20002207)
//#define SPI_S_RDBUF_BASE    (REG_BASE_MEMORYIO_ADDR+0x2210)
#define SPI_S_RDBUF_BASE    (0x20002210)
#define SPI_S_WRBUF_BASE    (0x20002220)


typedef struct spi_flash_dev_s {
  U32   base;     /* Base address */
  U32   sectors;    /* Sector count */
  U32   lgSectorSize;   /* Log2(usable bytes/sector) */
  U32   vendorID;   /* Expected vendor ID */
  U32   deviceID;   /* Expected device ID */
} SPI_FL_DEV_T;

typedef struct spi_flash_type_s {
  U32   sectors; 
  U32   size; 
} SPI_FL_TYPE_T;


SPI_FL_DEV_T spiflashDev[3] = 
{
    /* sector 1 - 128 (4KB) */
    {SPIFLASHTESTSTARADDR, 256, 0x1000, WINBONDMID, W25Q80},
    /* sector 1 - 128 (4KB) */
    {SPIFLASHTESTSTARADDR, 128, 0x1000, EONMID, EON25Q40},
    {SPIFLASHTESTSTARADDR, 256, 0x1000, EONMID, EON25Q80 }
};
int spiflashcount =(sizeof (spiflashDev) / sizeof (spiflashDev[0]));
U32 spiflashtype[2][3] = 
{
  // total sectors, size, poi entry
  {256, 0x100000, 400},
  // total sectors, size, poi entry
  {128, 0x80000, 400}

}; 

//spi
S08 QSPI_Init(U08 flashtype, U16 flashid);
U08 QSPI_chip_erase();
U08 QSPI_data_write(U32 addr, U08 *buf, U32 size);
//U08 QSPI_date_read(U08 *dest, U32 addr, U32 size);
int qspi_erase_sectors(unsigned long addr, signed int size);
U08 QSPI_sector_erase(unsigned long addr, U08 is_64k, U08 is_32k);
//U08 QSPI_non_quad_io_data_read(U08 *dest, U32 addr, U32 size);
U08 QSPI_non_quad_io_data_read_crc(U32 addr, U32 size);
U08 QSPI_date_read_crc(U32 addr, U32 size);

// --------- extern function declarations ------------------
#ifdef __cplusplus
extern "C" {
#endif

int SectorErase(FLASH_DEF * pFA,unsigned short flashType);
int check_flash(unsigned short *id, unsigned short *dev, U08 flashtype, U16 flashid);
int program_flash(unsigned long addr, unsigned long size, unsigned short id, unsigned short flashType,
                  unsigned long pattern);
int erase_sectors(unsigned long addr, signed int size, 
                  unsigned short id, unsigned short flashType);
U32 UART_poll1(volatile U32 *k);
U32 UART_poll2();
void UART_clearFIFO() ;
void gps1sgets(unsigned char *s, U32 mybaud) ;
void gps1sputs(const char *s,int s_len, U32 mybaud);
int gps1sgetsbin(unsigned char *s, int size, U32 mybaud) ;
VOID UART_init(U08 baud);
void sysDelay (int DelayCount);      
int new_fifouart_wr_buf (char* buf,int byte_cnt);
unsigned long iord (unsigned long addr);
void iowr (unsigned long addr, unsigned long data);   
void WD_disable();
#ifdef __cplusplus
}
#endif
#endif

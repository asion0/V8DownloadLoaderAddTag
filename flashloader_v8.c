/*******************************************************************************
 * Copyright (c) 2006 SkyTraq Technology, Inc. Hsinchu, Taiwan
 * All Rights Reserved.
 * 
 * file name: flashloader.c
 * Initial: 
 * Current: 
 ******************************************************************************/

//---------- head file inclusions -------------------------
#include "flashloader_v8.h"
//#define FIFOACK
#define DL_BYTES 8192
#define WAITTIME 200000
//#define WRITE_TEST
//#define WRITE_TAG
#define CUSTOMER_ID 0x0a
#define PRODUCT_ID 0x01
//#define UART_INIT_BAUDRATE  7 //0: 4800, 1:9600, 2:19200; 3:38400; 4:57600; 5:115200; 6: 230400: 7:460800, 8:921600
typedef volatile unsigned long Register;
#define FIRST_2BYTES_WRITE_LAST 1
//---------- local definitions ----------------------------

//---------- local types ----------------------------------

FL_DEV_T flashDev[][4] = {
  {
    /* sector 1 - 1 (16KB) */
    { FLASHTESTSTARADDR, 1, 14,
      AMIC_FLASH, A29L400AUV
    },
    /*sector 2 - 3 (8KB * 1) */
    { FLASHTESTSTARADDR + 0x4000, 3, 13,
      AMIC_FLASH, A29L400AUV
    },
    /* sector 4 - 4 (32KB ) */
    { FLASHTESTSTARADDR + 0x8000, 4, 15,
      AMIC_FLASH, A29L400AUV
    },
    /* sector 11-11 (64KB * 1) */
    { FLASHTESTSTARADDR + 0x10000, 11, 16,
      AMIC_FLASH, A29L400AUV
    }
  },
  {
    /* sector 1 - 7 (64KB) */
    { FLASHTESTSTARADDR, 7, 16,
    EON_FLASH, A29LV400AT
    },
    /*sector 8 - 8 (32KB * 1) */
    { FLASHTESTSTARADDR + 0x7000, 8, 15,
      EON_FLASH, A29LV400AT
    },
    /* sector 9 - 10 (8KB ) */
    { FLASHTESTSTARADDR + 0x7800, 10, 13,
      EON_FLASH, A29LV400AT
    },
    /* sector 11-11 (16KB * 1) */
    { FLASHTESTSTARADDR + 0x7C00, 11, 14,
      EON_FLASH, A29LV400AT
    }
  },
  {
    /* sector 1 - 7 (64KB) */
    { FLASHTESTSTARADDR, 15, 16,
      EON_FLASH, A29LV800AT
    },
    /*sector 16 - 16 (32KB * 1) */
    { FLASHTESTSTARADDR + 0x7000, 16, 15,
      EON_FLASH, A29LV800AT
    },
    /* sector 17 - 18 (8KB ) */
    { FLASHTESTSTARADDR + 0x7000, 18, 13,
      EON_FLASH, A29LV800AT
    },
    /* sector 19-19 (16KB * 1) */
    { FLASHTESTSTARADDR + 0x7000, 19, 14,
      EON_FLASH, A29LV800AT
    }
  },//numonyx
  {
    /* sector 1 - 7 (64KB) */
    { FLASHTESTSTARADDR, 255, 17,
      NUMONYX, JS28F256P30T
    },
    /*sector 8 - 8 (32KB * 1) */
    { FLASHTESTSTARADDR + 0x2000, 259, 15,
      NUMONYX, JS28F256P30T
    }    
  }
};  

#define BUILD_MONTH  (__DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? "01" : "06") \
                    : __DATE__ [2] == 'b' ? "02" \
                    : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? "03" : "04") \
                    : __DATE__ [2] == 'y' ? "05" \
                    : __DATE__ [2] == 'l' ? "07" \
                    : __DATE__ [2] == 'g' ? "08" \
                    : __DATE__ [2] == 'p' ? "09" \
                    : __DATE__ [2] == 't' ? "10" \
                    : __DATE__ [2] == 'v' ? "11" : "12")
                      
#define REV_ID "LDR02"

//---------- static global variables ----------------------
char _LOADER_VER_[30];

int flashDevCount =(sizeof (flashDev) / sizeof (flashDev[0]));
unsigned short norId = 0; 
unsigned short norDev = 0;

U32 uart_offset = 0;
unsigned char chipmode = 0;
S08 spi_entry = 0;
U32 mybaud = UART_INIT_BAUDRATE;  
int eraseTag = 0;
int writeTag = 0;
U32 tagAdd = 0;
U32 tagVal = 0; 

//---------- static function declarations -----------------
void InitSystem();
void InitFlashWriteMode();
void InitUart();
U32 GetInt(const int start, unsigned char* buf);
void ScanParameter(char* inBuf, int *binSize, U08 *chkSum);
void DebugOutput(const char *title, U32 data);
void DisableCache();
void ErrorEnd(int error);
void ScanParameter2(char* inBuf, int *binSize, U08 *chkSum,  U32 *baud,  U32 *tagAdd,  U32 *tagVal);
//void WriteTag(U32 tagAdd, U32 tagVal);
//void TestFlash();
//void EraseFlash(unsigned long *flashstart, int promSize);
void ScanParameter3(char* inBuf, int *binSize, U08 *chkSum, U32 *baud);
void ScanParameter4(char* inBuf, int *binSize, U08 *chkSum,  U32 *baud,  U32 *tagAdd,  U32 *tagVal);
static U08 QSPI_non_quad_write_status_value(U08 is_nonvolatile, U08 status1, U08 status2);
static void GetVersion(void);
static U08 QSPI_quad_write_status_eon(U08 status1);



//---------- functions definition --------------------------------------------//
int main()
{
  InitSystem();
  InitFlashWriteMode();
  
  //$LOADER,protocol type,year,month,day,sequence number
  //gps1sputs("$LOADER,B2,2013,09,18,1*7C\r\n", 0, mybaud);
  GetVersion();
  gps1sputs(_LOADER_VER_, 0, mybaud);	
  gps1sputs("END", 0, mybaud);

  int promSize = 0;
  unsigned char checkSum = 0;
  char buf[DL_BYTES];
  gps1sgets(buf, mybaud);
  
  if('2'==buf[6]) //BINSIZ2 command
  { // BINSIZ2 = promSize checkSum mybaud tagAdd tagVal checkCode
    //"BINSIZ2 = 563600 247 7 2862966102 1431696884 258944 "
    ScanParameter2(buf, &promSize, &checkSum, &mybaud, &tagAdd, &tagVal);
  }
  else if('3'==buf[6])
  { // BINSIZ3 = promSize checkSum mybaud checkCode
    //"BINSIZ2 = 563600 247 7 563854 "
    ScanParameter3(buf, &promSize, &checkSum, &mybaud);
  }
  else if('4'==buf[6]) //BINSIZ2 command
  { // BINSIZ2 = promSize checkSum mybaud tagAdd tagVal checkCode
    //"BINSIZ2 = 563600 247 7 2862966102 1431696884 258944 "
    ScanParameter4(buf, &promSize, &checkSum, &mybaud, &tagAdd, &tagVal);
  }
  else //Original BIZSIZE command
  { //"BINSIZE = 563600 Checksum = 247 563847 "
    ScanParameter(buf, &promSize, &checkSum);
  }
//  InitUart(mybaud);
//  DebugOutput("tagAdd ", tagAdd);
//  DebugOutput("tagVal ", tagVal);
//  DebugOutput("eraseTag ", eraseTag);
//  DebugOutput("writeTag ", writeTag);
//  DebugOutput("checkSum ", checkSum);

//TestFlash();
{
  U08 myflashtype = 0;
  U16 myflashid = 0;
  
  if(chipmode==4) //parallel flash
  {
    DisableCache();
    
    if(0 == check_flash(&norId, &norDev, (U08)myflashtype, (U16)myflashid))
    {
      ErrorEnd(1);
    }
  } //if(chipmode==4)
  else if(chipmode==1)//windbond serial flash
  {
    if(-1 == QSPI_Init((U08)myflashtype, (U16)myflashid))
    {
      ErrorEnd(1);
    }
  } //else if(chipmode==1)
  else if(chipmode==2) //eon serial flash
  {
    if(-1 == QSPI_Init((U08)myflashtype, (U16)myflashid))
    {
      ErrorEnd(1);
    }
  } //if(chipmode==2)
  else if(chipmode==0)//at ROM boot
  {
    
    //try windfirst qspi here
    chipmode = 1;
    if(-1 == QSPI_Init((U08)myflashtype, (U16)myflashid))
    {
      //again try eon qspi here
      chipmode = 2;
      if (-1 == QSPI_Init((U08)myflashtype, (U16)myflashid))
      {
        DisableCache();
        if(0 == check_flash(&norId, &norDev, (U08)myflashtype, (U16)myflashid))
        {
          ErrorEnd(1);
        }
      } //if (spi_entry == -1)
    } //if(spi_entry == -1)

  } //else if(chipmode==0)
} //TestFlash()

  unsigned long flashstart = 0;
//EraseFlash(&flashstart, promSize);
{
  unsigned char is1m = 0;
  const int eraseSize = 0x20;
  volatile int j;
  int i;
  FLASH_T dl; 
  
  //verify ID if 1M or 512K for parallel
  if(chipmode==4)
  {
    if(((norId== NUMONYX) && (norDev==JS28F256P30T)))
    {
        
    }
    else if(((norId== EON_FLASH) && ((norDev==A29LV400AT) || (norDev == A29LV800AT))))
    {
      if(norDev==A29LV800AT)
      {
        is1m = 1;
      }
    }
    else
    {
      norId = AMIC_FLASH;
      norDev = A29L400AUV;
    }

    flashstart = 0x78554;
    //size = 0x20;
    if(erase_sectors(flashstart, eraseSize, norId, norDev) == 0)
    {
      ErrorEnd(4);
    }
    
    
    for(i=0; i<4; i=i+2)
    {
      if(*((volatile unsigned short *)(flashstart + i)) != 0xffff)
      {
        ErrorEnd(5);
      }
    }
    for(i=0; i<4; i=i+2)
    {
      dl = (i==0) ? 0xaa55 : 0x55aa;
      if(program_flash(flashstart + i, 2 * (FLASH_WIDTH / FLASH_CHIP_WIDTH), norId, norDev, dl) == 0)
      {
         ErrorEnd(4);
      }
      if(*((volatile unsigned short *)(flashstart + i)) != dl)
      {
        ErrorEnd(4);
      }
    } //for(i=0; i<4; i=i+2)

    if(norId == NUMONYX)
    {
      erase_sectors(flashstart, eraseSize, norId, norDev);
      erase_sectors(0x1ff0000, eraseSize, norId, norDev);
      erase_sectors(0x1ff8000, eraseSize, norId, norDev);
    }
    else
    {
      int testoffset = (is1m == 1) ? 0x80000 : 0;
      erase_sectors(flashstart, eraseSize, norId, norDev);
      erase_sectors(0x70554 + testoffset, eraseSize, norId, norDev);
      erase_sectors(0x7a554 + testoffset, eraseSize, norId, norDev);
      erase_sectors(0x7c554 + testoffset, eraseSize, norId, norDev);
      if(eraseTag)
      {
        erase_sectors(0x77554 + testoffset, eraseSize, norId, norDev);
      }
    }
  } //if(chipmode==4)
  else if(chipmode==1)
  {
    flashstart = 0x78554;
    //size = 0x20;
    qspi_erase_sectors(0xfd000, eraseSize);
    qspi_erase_sectors(0xfe000, eraseSize);
    qspi_erase_sectors(0xff000, eraseSize);
    if(eraseTag)
    {
//      DebugOutput("EraseTag ", tagAdd & 0xFFFFF000);
      qspi_erase_sectors(tagAdd & 0xFFFFF000, eraseSize);
    }
  } //else if(chipmode==1)
  
  flashstart = 0x0;
  if(chipmode==4)
  {
    if(erase_sectors(flashstart, promSize, norId, norDev)==0)
    {
      ErrorEnd(4);
    }
  }
  else if((chipmode==2) || (chipmode==1))
  {
    if(qspi_erase_sectors(flashstart, promSize)==0)
    {
      ErrorEnd(4);
    }
  }
}

  //int size;
  int i, jj, offset = 0;
  FLASH_T dl; 
#if (1==FIRST_2BYTES_WRITE_LAST)
  unsigned char bootbyte[2];      //={0x88,0x10};
  unsigned char skip_first_2bytes = 1;
#endif

  WD_disable();//new image download launch a 60 seconds erase time 
  gps1sputs("OK", 0, mybaud);
  
  int size = promSize;
  unsigned long totalbytes = 0;
  do 
  {
    int recsize = (size>=DL_BYTES) ? DL_BYTES : size;
    unsigned long returnbytes = gps1sgetsbin(buf, recsize, mybaud);
    
    if(returnbytes == 0)
    { //timeout on uart
      gps1sputs("Error5", 0, mybaud); 
    }
    
    totalbytes = totalbytes + returnbytes;
    size = size - returnbytes;
#ifdef FIRST_2BYTES_WRITE_LAST
  	if(skip_first_2bytes == 1)
  	{
  	  bootbyte[0] = buf[0];
  	  bootbyte[1] = buf[1];
  	  buf[0] = 0xff;
  	  buf[1] = 0xff;
  	  skip_first_2bytes = 0;
  	}
#endif    
    if(chipmode == 4)
    {
      for(i=0; i<returnbytes; i = i + 2 * (FLASH_WIDTH / FLASH_CHIP_WIDTH))
      {
        dl = 0;
        for(jj=i; jj<(2*(FLASH_WIDTH/FLASH_CHIP_WIDTH)+i); jj++)
        {
          dl = (dl << 8) | buf[jj];
        }
        if(dl != 0xffff)
        {
          if(program_flash(flashstart+offset, 2*(FLASH_WIDTH/FLASH_CHIP_WIDTH), norId, norDev, dl)==0)
            ErrorEnd(4);
        }
        offset=offset+2*(FLASH_WIDTH/FLASH_CHIP_WIDTH);
      }
    } //if(chipmode == 4)
    else if((chipmode==2) || (chipmode==1))
    {
      for(i=0; i<returnbytes; i = i + 256)
      {
        jj = i;
        if(returnbytes < 256)
        {
          if(QSPI_data_write(flashstart+offset, &(buf[jj]), returnbytes)==0)
            ErrorEnd(4);
          offset = offset + returnbytes;
        }
        else
        {
          if(QSPI_data_write(flashstart+offset, &(buf[jj]), 256)==0)
            goto Error4;
          offset = offset + 256;
        }
      }
    } //else if((chipmode==2) || (chipmode==1))

    if(((totalbytes % DL_BYTES)==0) || size==0)
    {
      gps1sputs("OK", 0, mybaud);
    }

    if(totalbytes == promSize)
    {
      break;
    }
    
  } while(1);
  
#ifdef FIRST_2BYTES_WRITE_LAST
  if(chipmode==4)
  {
    dl = bootbyte[0] << 8 | bootbyte[1];
    if(program_flash(0x0, 2*(FLASH_WIDTH/FLASH_CHIP_WIDTH), norId, norDev, dl)==0)
      ErrorEnd(4); //flash test fail
  }
  else
  {
    if(QSPI_data_write(0x0, &(bootbyte[0]), 2)==0)
      ErrorEnd(4); //flash test fail
  }
#endif  
  /* checksum calcuation */

  int kkk, mmm, lll;
  unsigned char mycheck=0;
  unsigned short val;
  if(chipmode==4)
  {
    for(kkk=0;kkk<promSize;kkk=kkk+2)
    {
      val = *(unsigned short *)(kkk);
      mycheck=mycheck+(val>>8) + (val & 0xff);
    }
  }
  else if((chipmode==2) || (chipmode==1))
  {
    if(chipmode==1)
    {
      mycheck = QSPI_non_quad_io_data_read_crc(0x0, promSize);
    }
    else
    {
      mycheck=QSPI_date_read_crc(0x0,promSize);
    }
  }
  //DebugOutput("mck ", mycheck);
  //DebugOutput("ck  ", checkSum);

  if(mycheck == checkSum)
  {
    if(writeTag)
    {
      //WriteTag(tagAdd, tagVal);   
      {
        FLASH_T dl; 
        char buf[2];
        
        if(chipmode==4)
        {
          dl = tagVal;
          if(program_flash(tagAdd, 2 * (FLASH_WIDTH/FLASH_CHIP_WIDTH), norId, norDev, dl)==0)
          {
            ErrorEnd(4); //flash test fail
          }
        }
        else
        {
          buf[0] = (tagVal >> 8) & 0xFF;
          buf[1] = (tagVal) & 0xFF;
          
          //buf[0] = 0x0A;
          //buf[1] = 0x01;
         
          //DebugOutput("buf[0] ", (U32)buf[0]);
          //DebugOutput("buf[1] ", (U32)buf[1]);
          //DebugOutput("tagAdd ", tagAdd);
          //DebugOutput("tagVal ", tagVal);
          if(QSPI_data_write(tagAdd, &(buf[0]), 2)==0)
          {
            ErrorEnd(4);
          }
        } 
      }
 
    }
    if(chipmode==1)
      QSPI_non_quad_write_status_value(1, 0x0, 0x42);
    else
      QSPI_quad_write_status_eon(0x40);
    gps1sputs("END",0,mybaud);
  }
  else
  {
    ErrorEnd(2);
	}
  gps1sputs("END",0,mybaud); 

#if 1
  //volatile int aaa;
  if(((*(volatile unsigned long *)(0x2000f010))&0x80000000)!=0)//leon2
  {
    //*((volatile unsigned short *) (0x0)) = 0xf0;
		//volatile int aaa;
  //for(aaa=0;aaa<0x200000;aaa++);//need delay here or the END will not be sent out
  while(((*((volatile unsigned long *) (0x2000242c)))&0x8)==0);
  *((volatile unsigned short *) (0x20004c20)) = *((volatile unsigned short *) (0x20004c20))& 0xfffffffe;
  *((volatile unsigned long *) (0x8000004c)) = 0x10;
  //cause need some delay between counter & enable
    *((volatile unsigned long *) (0x800000b8))=*((volatile unsigned long *) (0x8000004c));
    *((volatile unsigned long *) (0x20001014)) = 
    ((*((volatile unsigned long *) (0x20001014))) & 0xfffffffe);
	} //if(((*(volatile unsigned long *)(0x2000f010)) & 0x80000000) != 0)//leon2
  else
  {
  if(mybaud<9)
  {
    while(((iord(UART1_CESR+uart_offset))&0x8)==0)
    {
    };
  }
  else
  {
     gps1sputs("END",0,mybaud); 
    while(( *((volatile unsigned long *) (SPI_S_MAIN))&0x4)==1){};
    //want to disable SPI_SLAVE
    //for(aaa=0;aaa<0x2000000;aaa++);//need delay here or the END will not be sent out
    *((volatile unsigned short *) (0x2000f004)) = *((volatile unsigned short *) (0x2000f004))& 0xfffffffa;
  }
  *((volatile unsigned short *) (0x20004c20)) = *((volatile unsigned short *) (0x20004c20))& 0xfffffffe;
  //below for leon2
    *((volatile unsigned long *) (0x8000004c)) = 0x10;
  //below for leon3
  *((volatile unsigned long *) (0x80000344)) = 0x10;
  *((volatile unsigned long *) (0x80000348)) = 0xd;

  //cause need some delay between counter & enable
    *((volatile unsigned long *) (0x800000b8))=*((volatile unsigned long *) (0x8000004c));
  //below for leon 3
    *((volatile unsigned long *) (0x20001014)) = 
    ((*((volatile unsigned long *) (0x20001014))) & 0xfffffffe);
  } //else | if(((*(volatile unsigned long *)(0x2000f010)) & 0x80000000) != 0) els
#else //#1
  volatile int aaa;
  for(aaa=0;aaa<0x200000;aaa++);//need delay here or the END will not be sent out
  prog = (void *)(0x0);
  prog();
#endif //#1 else

  while(1);
 
Error4:
  //while(1);
  gps1sputs("Error4",0,mybaud);
  while(1);
}

void WD_disable()
{
    *((volatile unsigned long *) (0x20001014)) =
    ((*((volatile unsigned long *) (0x20001014))) | 0x00000001);
}

unsigned long iord (unsigned long addr)
{
  volatile unsigned long *ioreg32 = (volatile unsigned long *) REG_BASE_MEMORYIO_ADDR;
  unsigned long   out;

  //disable_irq(7);

  out =  ioreg32[addr/4];


   //enable_irq(7); 
  return (out);
  
}
void iowr (unsigned long addr, unsigned long data)
{
  volatile unsigned long *ioreg32 = (volatile unsigned long *) REG_BASE_MEMORYIO_ADDR;

    //  disable_irq(7);
      
  ioreg32[addr/4]       = data ;
  
  
  //enable_irq(7);
  
}

//
#if 1 //add eon qspi
static void QUAD_SPI_ACTIVATE()
{ 
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  
  //cs low
  *lreg &= ~(0x1UL << CSN_DATA);
}

static void QUAD_SPI_DEACTIVATE()
{ 
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  
  //cs high
  *lreg |= (0x1UL << CSN_DATA); 
  //clk low
  *lreg &= ~(0x1UL << CK_DATA);
}
static U08 QUAD_SPI_CMD(U08 cmd)
{  

 
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  unsigned long int reg;
    
  U08 data_nibble;
  
  U08 i;
  //data dir output
  *lreg  &= ~(0xFUL << SPI_DATA_DIR);


  //clk low and data
  data_nibble=cmd>>4;
  reg = *lreg;
  reg &= ~((0x1UL << CK_DATA)|(0xf<<SPI_DATA_BIT0));
  reg |= data_nibble; 
  *lreg=reg;
  //clk high
  *lreg |= 0x1UL << CK_DATA; 
  //clk low and data
  data_nibble=cmd&0xf;
  reg = *lreg;
  reg &= ~((0x1UL << CK_DATA)|(0xf<<SPI_DATA_BIT0));
  reg |= data_nibble; 
  *lreg=reg;
  //clk high
   *lreg |= 0x1UL << CK_DATA; 

  return TRUE;
}


static U08 QUAD_SPI_DATA_BYTE_READ()
{  


  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  
    
  U08 data_nibble;
  
  U08 i;
  //data dir input
  *lreg  |= (0xFUL << SPI_DATA_DIR);
  //clk low
  *lreg &= ~(0x1UL << CK_DATA);
  //read data
  data_nibble=*lreg&0xf;
  //clk high
  *lreg  |=(0x1UL << CK_DATA); 
  asm("nop;");
  //clk low
  *lreg &= ~(0x1UL << CK_DATA);
  //read data
  data_nibble=(data_nibble<<4)| (*lreg&0xf);
  //clk high
  *lreg  |=(0x1UL << CK_DATA); 

  return (data_nibble);
}

static U08 QSPI_read_status()
{
  U08 val;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_RDSTATUS);//issue 
  val=QUAD_SPI_DATA_BYTE_READ();
  QUAD_SPI_DEACTIVATE();
  return val;
}

static U08 QUAD_SPI_NON_QUAD_CMD(U08 cmd)
{  


  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  unsigned long int reg;
    
  U08 data_nibble;
  
  U08 i;
  //data dir output
  *lreg  &= ~(0xFUL << SPI_DATA_DIR);

  for( i = 0; i <8 ; i++ )
  {
    *lreg &= ~(0x1UL << CK_DATA);
    if (cmd & 0x80)//high
    {
      //clk low
    
      reg = *lreg;
      //reg  |= (0x1UL << SPI_DATA_BIT0)|(0x1UL << CK_DATA);
      reg  |= (0x1UL << SPI_DATA_BIT0);
      *lreg = reg;
    }
    else
    {
      reg = *lreg;
      reg  &= ~(0x1UL << SPI_DATA_BIT0);
      //reg  |= (0x1UL << CK_DATA);
      *lreg = reg;
    }
    reg = *lreg;
    reg  |= 0x1UL << CK_DATA;
    *lreg = reg;
    asm("nop;");
    cmd = cmd<<1;
  } 


  return TRUE;
}

static U08 QUAD_SPI_DATA_NON_QUAD_BYTE_READ()
{  


  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  
    
  U08 data_nibble=0;
  
  U08 i;
  //data dir input
  *lreg  |= (0x2UL << SPI_DATA_DIR);

  for( i = 0; i <8 ; i++ )
  {
    *lreg &= ~(0x1UL << CK_DATA);
    
    data_nibble = ((*lreg&0x2)>>1) | (data_nibble<<1);
    *lreg  |=(0x1UL << CK_DATA); 
    asm("nop;");
  } 

  return (data_nibble);
}

U08 QSPI_readid(U08 *mid, U16 *did)
{
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_READID);//issue 0x9f
  *mid=QUAD_SPI_DATA_BYTE_READ();
  *did=QUAD_SPI_DATA_BYTE_READ();
  *did=(*did<<8) | (QUAD_SPI_DATA_BYTE_READ());
  QUAD_SPI_DEACTIVATE();
  return TRUE;
}

U08 QSPI_page_program(U32 addr, U08 *src, U32 size)
{
  U08 prog_addr;
  U32 SFA;
  U32 elapse;
  U08 value;
NextPage:
  SFA = addr;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_PAGE_PROGRAM);//issue 0x20
  prog_addr=(SFA>>16)&0xff;
  QUAD_SPI_CMD(prog_addr);
  prog_addr=(SFA>>8)&0xff;
  QUAD_SPI_CMD(prog_addr);
  prog_addr=SFA&0xff;
  QUAD_SPI_CMD(prog_addr);
  for ((SFA = addr); (SFA < size + addr ); SFA++, src++)
  {
      
    elapse=0;      
    value = *src;
    QUAD_SPI_CMD(value);
    if(((SFA+1)%0x100)==0)//not suitable for over 2 pages
    {
      SFA++;
      src++;
      size=size-(SFA-addr);
      addr=SFA;
      QUAD_SPI_DEACTIVATE();
      while(((QSPI_read_status()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
      {
        elapse++;
      }
      goto NextPage;
    }
       
  }
  QUAD_SPI_DEACTIVATE();
  return TRUE;
}





U08 QSPI_date_read_crc(U32 addr, U32 size)
{
  U08 read_addr;
  U32 i;
  U08 val;
  U08 crc=0;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_FAST_READ);//issue 0x0B
  read_addr=(addr>>16)&0xff;
  QUAD_SPI_CMD(read_addr);
  read_addr=(addr>>8)&0xff;
  QUAD_SPI_CMD(read_addr);
  read_addr=addr&0xff;
  QUAD_SPI_CMD(read_addr);
  //6 dummy cycles=3 write
  for(i=0;i<3;i++)
  QUAD_SPI_CMD(read_addr);
  for (i=0; i<size; i++)
  {
    val=QUAD_SPI_DATA_BYTE_READ();
    crc= crc + val;
        
  }
  QUAD_SPI_DEACTIVATE();
  return crc;
}

U08 QSPI_chip_erase()
{
  U32 elapse=0;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_WRENABLE);
  QUAD_SPI_DEACTIVATE();
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_CHIP_ERASE);//issue 0xc7
  QUAD_SPI_DEACTIVATE();
  while(((QSPI_read_status()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
  {
    elapse++;
  }
  if(elapse<QSPI_WAITTIME)
  return 1;
  else
  {
  return 0;
  }
}
static U08 QSPI_non_quad_readid(U08 *mid, U16 *did)
{
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_READID);//issue 0x9f
  *mid=QUAD_SPI_DATA_NON_QUAD_BYTE_READ();
  *did=QUAD_SPI_DATA_NON_QUAD_BYTE_READ();
  *did=(*did<<8) | (QUAD_SPI_DATA_NON_QUAD_BYTE_READ());
  QUAD_SPI_DEACTIVATE();
  return TRUE;
}

static U08 QSPI_non_quad_read_status_1()
{
  U08 val;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_READ_STATUS_REG_1);//issue 
  val=QUAD_SPI_DATA_NON_QUAD_BYTE_READ();
  QUAD_SPI_DEACTIVATE();
  return val;
}
static U08 QSPI_non_quad_page_program(U32 addr, U08 *src, U32 size)
{
  U08 prog_addr;
  U32 SFA;
  U32 elapse;
  U08 value;
NextPage:
  SFA = addr;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_QUAD_PAGE_PROGRAM);//issue 0x32
  prog_addr=(SFA>>16)&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  prog_addr=(SFA>>8)&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  prog_addr=SFA&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  for ((SFA = addr); (SFA < size + addr ); SFA++, src++)
  {
      
    elapse=0;      
    value = *src;
    
    //QUAD_SPI_CMD(value);
    QUAD_SPI_CMD(value);
    if(((SFA+1)%0x100)==0)//not suitable for over 2 pages
    {
      SFA++;
      src++;
      size=size-(SFA-addr);
      addr=SFA;
      QUAD_SPI_DEACTIVATE();
      while(((QSPI_non_quad_read_status_1()&0x1)!=0)&& (elapse<WAITTIME))
      {
        elapse++;
      }
      goto NextPage;
    }
       
  }
  QUAD_SPI_DEACTIVATE();
  return TRUE;
}

static U08 QSPI_non_quad_write_status_value(U08 is_nonvolatile, U08 status1, U08 status2)
{
  
  QUAD_SPI_ACTIVATE();
  if(is_nonvolatile==1)
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRENABLE);
  else
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRENABLE_VOLATILE);  
  QUAD_SPI_DEACTIVATE();
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRITE_STATUS_REG);
  QUAD_SPI_NON_QUAD_CMD(status1);
  QUAD_SPI_NON_QUAD_CMD(status2);
  QUAD_SPI_DEACTIVATE();
  while((QSPI_non_quad_read_status_1()&0x1)!=0);
  return TRUE;
}

static U08 QSPI_quad_write_status_eon(U08 status1)
{
  
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_WRENABLE);
  QUAD_SPI_DEACTIVATE();
  QUAD_SPI_ACTIVATE();
  //QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRITE_STATUS_REG);
  QUAD_SPI_CMD(W25Q40_CMD_WRITE_STATUS_REG);
  QUAD_SPI_CMD(status1);
  QUAD_SPI_DEACTIVATE();
  
  while((QSPI_read_status()&0x1)!=0);
  return TRUE;
}

S08 QSPI_Init(U08 flashtype, U16 flashid)
{
  U08 whichone;
  U08 mid;
  U16 did;
  S08 returnval=-1;
  // init spi
  
  volatile unsigned long int *lreg = (volatile unsigned long int *) GPIO_QSPI_ADDR;
  //unsigned long int reg = *lreg;
  //set swcfg && data all high
  *lreg  = 0x80000030;
  while((*lreg&0x80000000)==0);
  if(chipmode==1)
  {
    //write enable at quad mode
    //QSPI_WINBOND_NON_QUAD_enable();
    U32 reg;
    reg = *lreg;
    //enable non-used bit1~bit3
    reg  |= (0xcUL << SPI_DATA_BIT0);
    *lreg=reg;
    //winbond at this stage needs to make hold and wp output high
    QUAD_SPI_ACTIVATE();
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRENABLE);
    QUAD_SPI_DEACTIVATE();
    QSPI_non_quad_readid(&mid,&did);
  }
  else
  {
    //andrew 0508 add 12 cycles here
    volatile U08 cycle;
    QUAD_SPI_ACTIVATE();
    for(cycle=0;cycle<6;cycle++)
    QUAD_SPI_CMD(0x0);
    QUAD_SPI_DEACTIVATE();
  
    //set CLK as an output pin  
    //*lreg  &= ~(0x1UL << CK_DIR);
    //set CSN as an output pin
    //*lreg  &= ~(0x1UL << CSN_DIR);
  
    //write enable at quad mode
    QUAD_SPI_ACTIVATE();
    QUAD_SPI_CMD(EON25Q40_CMD_WRENABLE);
    QUAD_SPI_DEACTIVATE();
    QSPI_readid(&mid,&did);
  }
  
  for(whichone=0;whichone<spiflashcount;whichone++)//only 2 entries currently
  {
    //if((spiflashDev[whichone].vendorID==mid) &&(spiflashDev[whichone].deviceID==did))
    if(spiflashDev[whichone].vendorID==mid)
    {
      if(spiflashDev[whichone].deviceID==did)//exact match
      {
        returnval=whichone;
        break;
      }
      else
      if(flashtype==0)//auto, tolerate the match to this vendor
      {
        returnval=whichone;
        break;
      }
      else //check flash type too
      {
        if((flashid==did))
        {
          returnval=whichone;
          break;
        }
      }
    }
  }
  if(returnval!=-1)
  {
    if(chipmode==1)
    {
      QSPI_non_quad_write_status_value(0, 0x1c, 0x42);
      //DebugOutput("0x1c ", 0x42);
    }
    else
    {
      QSPI_quad_write_status_eon(0x40);
    }
  }

  return returnval;
}

U08 QSPI_data_write(U32 addr, U08 *buf, U32 size)
{
  U32 elapse=0;
  if(chipmode==2)
  {
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_WRENABLE);
  QUAD_SPI_DEACTIVATE();
  QSPI_page_program(addr, buf, size);
  while(((QSPI_read_status()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
  {
    elapse++;
  }
  if(elapse<QSPI_WAITTIME)
  return 1;
  else
  {
   return 0;
  }
  }
  else
  {
    QUAD_SPI_ACTIVATE();
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRENABLE);
    QUAD_SPI_DEACTIVATE();
    QSPI_non_quad_page_program(addr, buf, size);
    while(((QSPI_non_quad_read_status_1()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
    {
    elapse++;
    }
    if(elapse<QSPI_WAITTIME)
    return 1;
    else
    {
     return 0;
    }
  }
}


U08 QSPI_non_quad_io_data_read_crc(U32 addr, U32 size)
{
  U08 read_addr;
  U32 i;
  U08 crc=0;
  U08 val;
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_QUAD_FAST_WORD_READ_QUAD_IO);//issue 0x0B
  read_addr=(addr>>16)&0xff;
  //QUAD_SPI_CMD(read_addr);
  QUAD_SPI_CMD(read_addr);
  read_addr=(addr>>8)&0xff;
  //QUAD_SPI_CMD(read_addr);
  QUAD_SPI_CMD(read_addr);
  read_addr=addr&0xff;
  //QUAD_SPI_CMD(read_addr);
  QUAD_SPI_CMD(read_addr);
  read_addr=0x0;//m7-0, m5-4 needs 1,0
  //QUAD_SPI_CMD(read_addr);
  QUAD_SPI_CMD(read_addr);
  //1 dummy write
  //for(i=0;i<2;i++)
  //QUAD_SPI_CMD(read_addr);
  QUAD_SPI_CMD(read_addr);
  //int shift = 0;
  //U32 dbg = 0;
  for (i=0; i<size; i++)
  {
    val=QUAD_SPI_DATA_BYTE_READ();
    crc = crc+ val;
/*    
    dbg |= (val << shift);
    if(i % 4 == 3)
    {
      if(i>0x8f6a0)
        DebugOutput(" ", dbg);
      shift = 0;
      dbg = 0;
    }
    else
    {
      shift += 8;
    }
*/    
    //*(U08 *)dest = val;
        
  }
  QUAD_SPI_DEACTIVATE();
  return crc;
}



U08 QSPI_sector_erase(unsigned long addr, U08 is_64k, U08 is_32k)
{
  U32 elapse=0;
  U08 prog_addr;
  if(chipmode==2)
  {
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_CMD(EON25Q40_CMD_WRENABLE);
  QUAD_SPI_DEACTIVATE();
  QUAD_SPI_ACTIVATE();
  if(is_64k==0)
  QUAD_SPI_CMD(EON25Q40_CMD_SECTOR_ERASE);//issue 0xc7
  else
  QUAD_SPI_CMD(EON25Q40_CMD_BLOCK_ERASE);//issue 0xd8
  prog_addr=(addr>>16)&0xff;
  QUAD_SPI_CMD(prog_addr);
  prog_addr=(addr>>8)&0xff;
  QUAD_SPI_CMD(prog_addr);
  prog_addr=addr&0xff;
  QUAD_SPI_CMD(prog_addr);
  QUAD_SPI_DEACTIVATE();
  while(((QSPI_read_status()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
  {
    elapse++;
  }
  if(elapse<QSPI_WAITTIME)
  return 1;
  else
  {
  return 0;
  }
}
else//windbond
{
  QUAD_SPI_ACTIVATE();
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_WRENABLE);
  QUAD_SPI_DEACTIVATE();
  QUAD_SPI_ACTIVATE();
  if(is_64k==0)
  {
    if(is_32k==0)
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_SECTOR_ERASE);
    else
    QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_32K_BLOCK_ERASE);
  }
  else
  QUAD_SPI_NON_QUAD_CMD(W25Q40_CMD_BLOCK_ERASE);//64k
  prog_addr=(addr>>16)&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  prog_addr=(addr>>8)&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  prog_addr=addr&0xff;
  QUAD_SPI_NON_QUAD_CMD(prog_addr);
  QUAD_SPI_DEACTIVATE();
  while(((QSPI_non_quad_read_status_1()&0x1)!=0)&& (elapse<QSPI_WAITTIME))
  {
    elapse++;
  }
  if(elapse<QSPI_WAITTIME)
  return 1;
  else
  {
  return 0;
  }
}
}

int qspi_erase_sectors(unsigned long addr, signed int size)
{
  
  unsigned long i, index, tmpaddr=0;
  int sector=0;
  int sectorID=0;
  int retVal=1;
  int whichone;
  U08 is_64k;
  U08 is_32k;

  
  while (size>0)
  {
    if((size-0x10000)>=0)
    is_64k=1;
    else
    {
      if(((size-0x8000)>=0)&&(chipmode==1))
      is_32k=1;
      else
      is_32k=0;
      is_64k=0;
    }
    if (QSPI_sector_erase (addr, is_64k, is_32k) == 0)
    {
      retVal=0;
      break;
    }
    if(is_64k==0)
    {
      if(is_32k==1)
      {
        size=size-0x8000;
        addr += 0x8000;
      }
      else
      {
      size=size-0x1000;
      addr += 0x1000;
      }
    }
    else
    {
      size=size-0x10000;
      addr += 0x10000;
    }
  }

  return retVal;

}
#endif//end add
//Andrew
int sysFlashDataPoll
  (
    FLASH_DEF * pFA,   /* programmed address to poll */
    FLASH_DEF value,     /* data programmed to poll address */
    unsigned short flashType     /* type of flash memory on-board */
  )
{
  int retVal = 1;

  int vBit;      /* programmed value of DQ7 */

  /* is this big/little endian compatible ? */
  vBit = Q7(value);
  if(flashType==JS28F256P30T)
  {
    while (Q7(*pFA) != vBit)
    {
  
    if (Q6(*pFA) == 1)  /* suspend ? */
    {
       return 0;
    }

    }  
  }
  else
  {
    while (Q7(*pFA) != vBit)
  {
    if (Q5(*pFA) == 1)  /* timeout ? */
      break;
  }

  if (Q7(*pFA) != vBit)    /* check Q7 & Q5 race */
    retVal = 0;
  
  }              
        
  return (retVal);
}

int sysProgramDataPoll
  (
    FLASH_DEF * pFA,   /* programmed address to poll */
    FLASH_DEF value     /* data programmed to poll address */
  )
{
  int retVal = 1;

  int vBit;      /* programmed value of DQ7 */

  /* is this big/little endian compatible ? */
  vBit = 1;//Q7(value);


  while (Q7(*pFA) != vBit)
  {
    
    //if (Q6(*pFA) == 1)  /* suspend ? */
    //{
      // return 0;
    //}
  }
                 
        
  return (retVal);
}

int sysClearStatusReg(FLASH_DEF * pFA)
{
  *pFA = JS_CMD_CLEAR_STATUS_REG;
  //*pFA = value;    
}

int unlockblock
(
    FLASH_DEF * pFA,        /* Sector start address */
    unsigned short flashType     /* type of flash memory on-board */
    )
{
  int retVal = 1;
  //flashType=JS28F256P30T;
  switch (flashType)
  {
    

    case (A29L400AUV):
    case (FJ29LV400B):
    case (A29LV400AT):
    case (A29LV800AT):
    break;
    case (JS28F256P30T):
    //pFA = FLASH_CAST (addr);
    *pFA  = JS_CMD_BLOCK_UNLOCK;
    *pFA  = JS_DATA_BLOCK_UNLOCK;
    *pFA  = 0x00ff;





    break;

    default:
    retVal = 3;
  }
  return retVal;
}

int SectorErase
    (
    FLASH_DEF * pFA,        /* Sector start address */
    unsigned short flashType     /* type of flash memory on-board */
    )
{
  int retVal = 1;
  int i;
  unsigned char vBit;
  volatile unsigned long elapse=0;

  switch (flashType)
  {
    

    case (A29L400AUV):
    case (FJ29LV400B):
    case (A29LV400AT):
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_FIRST;
    *(FLASH_CAST FLASH29_REG_SECOND_CYCLE) = FLASH29_CMD_SECOND;
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_CHIP_ERASE;
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_FOURTH;
    *(FLASH_CAST FLASH29_REG_SECOND_CYCLE) = FLASH29_CMD_FIFTH;
    *pFA                   = FLASH29_CMD_SECTOR;

    do {
         retVal = sysFlashDataPoll (pFA, (FLASH_DEF) 0xffffffff,flashType);
         elapse++;
       } while (((*pFA != (FLASH_DEF) 0xffffffff) && (retVal == 1))&& (elapse<WAITTIME));
    break;
    case (JS28F256P30T):

    //pFA = FLASH_CAST (addr);
    *pFA  = JS_CMD_BLOCK_ERASE;
    *pFA  = JS_DATA_BLOCK_ERASE;


    do {
         retVal = sysFlashDataPoll (pFA, (FLASH_DEF) 0xffffffff,flashType);
         elapse++;         
       } while ((retVal != 1)&& (elapse<WAITTIME));
    

    sysClearStatusReg(pFA);

    break;

    default:
    retVal = 0;
  }
  #if 1
  if(elapse>=WAITTIME)
  retVal=0;
  #else
  if(elapse>=WAITTIME)
  retVal=2;
  if (Q5(*pFA) == 1)  /* suspend ? */
  retVal=0;
  sysClearStatusReg(pFA);
  #endif
  return (retVal);
}



int check_flash(unsigned short *id, unsigned short *dev, U08 flashtype, U16 flashid) //this name used by nymonyx
{
  FLASH_DEF *base_ptr, *ptr1;
  int ret = 0;

  //check nymonyx first
  volatile int i;
  static unsigned short old_id=0x9876;
  *((volatile unsigned short *)(2))=JS_CMD_READ_DID;
  *id = (*((volatile unsigned short *)(0x0))) & 0xFFFF;
  *dev = (*((volatile unsigned short *)(0x2))) & 0xFFFF;
  //*check if asynchronous mode in read configuration register
  unsigned short id3;
  id3 = (*((volatile unsigned short *)(0xa))) & 0x8000; 
  //if((*id==0x0089)&&(*dev==0x8919)&&(id3==0x8000))
  

  if((*id==0x0089)&&(id3==0x8000))
  {
    if(*dev==0x8919)//exact match return directly
    {     

        ret=1;
        return ret;
    }
    else
    {
      if(flashtype==0)//auto mode found the same manufacturer return directly
      {
        ret=1;
        *dev=0x8919;//force to 8919, broadly accept all flash
        return ret;
      }
      else
      {
        if(*dev==flashid)//check read id the same as passing
        {     

          ret=1;
          return ret;
        }
      }
    }
  }
  else
  {
    *((volatile unsigned short *)(0x0))=0x0;
    base_ptr = FLASH_CAST(FLASHTESTSTARADDR+FLASH29_REG_ID_CYCLE);
  *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_FIRST;
    *(FLASH_CAST FLASH29_REG_SECOND_CYCLE) = FLASH29_CMD_SECOND;
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_AUTOSELECT;
    *id = *(base_ptr) & 0xFF;
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE) = FLASH29_CMD_READ_RESET;
    //if((*id == MX_FLASH )|| (*id== FJ_FLASH) || (*id== AMIC_FLASH)|| (*id== EON_FLASH))
    if((*id== AMIC_FLASH)|| (*id== EON_FLASH) || (*id == MX_FLASH) || (*id == SST_FLASH))
    {     
      *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_FIRST;
      *(FLASH_CAST FLASH29_REG_SECOND_CYCLE) = FLASH29_CMD_SECOND;
      *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_AUTOSELECT;
      *dev = *(base_ptr+1) & 0xFF;
      *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_READ_RESET;
      if(((*id == AMIC_FLASH) && (*dev == A29L400ATV))||
         ((*id == SST_FLASH) && (*dev == M29W400TB)))
      {
        *id = EON_FLASH;
        *dev = A29LV400AT;
      }
      if((*dev== A29LV400AT) ||(*dev== A29LV400AB)|| (*dev== A29L400AUV)||
        (*dev== MX29LV400TT)|| (*dev== FJ29LV400B) || (*dev== M29W400DB) || (*dev==A29LV800AT)   ) //exact match
      {
        ret=1; //exact match
        return ret;
      }
      else
      {
        if(flashtype==0)//auto mode found the same manufacturer return directly
        {
          ret=1;
          *dev=A29LV800AT;
          return ret;
        }
        else
        {
          if(*dev==flashid)
          {
            ret=1;
            return ret;
          }
        }
      }
    }
  }

    
  return ret;

}

int program_flash(unsigned long addr, unsigned long size, unsigned short id, unsigned short flashType,
                  unsigned long pattern)
{

  FLASH_DEF i, n=0;
  FLASH_DEF *pFA;       /* flash address */
  FLASH_DEF value;
  int retVal = 1;
  volatile unsigned long elapse=0;
  
  switch (flashType)
  {

    case (A29L400AUV):
    case (FJ29LV400B):
    case (A29LV400AT):
    case (A29LV800AT):      
    pFA = FLASH_CAST (addr);
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_FIRST;
    *(FLASH_CAST FLASH29_REG_SECOND_CYCLE) = FLASH29_CMD_SECOND;
    *(FLASH_CAST FLASH29_REG_FIRST_CYCLE)  = FLASH29_CMD_PROGRAM;
    value = (FLASH_DEF)pattern;/*0xAAAA5555;*/
    *pFA = value;                       /* data to write */
    do{
         
         retVal = sysFlashDataPoll (pFA, (FLASH_DEF) value, flashType);
         elapse++;
      } while (((*pFA != value) && (retVal == 1)) && (elapse <WAITTIME));

    break;
    
    case (JS28F256P30T):     
    pFA = FLASH_CAST (addr);
    //*((volatile unsigned short *)(addr))=0x40;
    value = (FLASH_DEF)pattern;/*0xAAAA5555;*/
    //*((volatile unsigned short *)(addr))=value;
    *pFA = JS_CMD_PROGRAM_WORD;
    *pFA = value;                       /* data to write */
    do{
         retVal = sysProgramDataPoll (pFA, (FLASH_DEF) value);
         elapse++;
      //} while (((*pFA != value) && (retVal == 1)) && (elapse <WAITTIME));
     // } while ((retVal != 1) && (elapse <WAITTIME));
      } while ((retVal != 1));

    sysClearStatusReg(pFA);
    *((volatile unsigned short *)0x0)=0xff;
    break;


    default:
    retVal = 0;
  }
  #if 1
  if(elapse>=WAITTIME)
  retVal=0;
  #else
  if(elapse>=WAITTIME)
  retVal=2;
  if (Q4(*pFA) == 1)  /* failed ? */
  retVal=0;
  sysClearStatusReg(pFA);
  *((volatile unsigned short *)0x0)=0xff;
  #endif
  return (retVal);

}
int erase_sectors(unsigned long addr, signed int size, 
                  unsigned short id, unsigned short flashType)
{
  
  unsigned long i, index, tmpaddr=0;
  int sector=0;
  int sectorID=0;
  int retVal=1;
  int whichone;
  #if 1
  for(whichone=0;whichone<flashDevCount;whichone++)
  {
    if((flashDev[whichone][0].vendorID==id) &&(flashDev[whichone][0].deviceID==flashType))
        break;
  }
  if(whichone == flashDevCount)
    return retVal;
  #else
  whichone=0;
  #endif
  while(addr>tmpaddr)
  {
    sector=sector+1;
    tmpaddr=tmpaddr+(1<<flashDev[whichone][sectorID].lgSectorSize)*(FLASH_WIDTH/FLASH_CHIP_WIDTH);
    if(sector>=flashDev[whichone][sectorID].sectors)
    sectorID=sectorID+1;
  }    
  while (size>0)
  {
    if(flashType==JS28F256P30T)
    {
      if (unlockblock (FLASH_CAST (addr), flashType) == 0)
      {
        retVal=0;
        break;
      }
    }
    if (SectorErase (FLASH_CAST (addr), flashType) == 0)
    {
      retVal=0;
      break;
    }
  
  
      
    sector=sector+1;
    if(size<(1<<flashDev[whichone][sectorID].lgSectorSize)*(FLASH_WIDTH/FLASH_CHIP_WIDTH))
      return 1;
    size=size-(1<<flashDev[whichone][sectorID].lgSectorSize)*(FLASH_WIDTH/FLASH_CHIP_WIDTH);
    addr += (1<<flashDev[whichone][sectorID].lgSectorSize)*(FLASH_WIDTH/FLASH_CHIP_WIDTH);
    if(sector>=flashDev[whichone][sectorID].sectors)
      sectorID=sectorID+1;
  }

  return retVal;

}

static U08 SPI_S_wr_buff_read(U08 *buf, U08 cnt)
{
   U32 value;
   U08 idx=0;
   U08 j;
   U16 i;
   //cnt=cnt/4;
   if(cnt>16)
   return 0;
   for(i=0;i<cnt;i++)
   {
      if(i<idx)
      continue;
      value=*((volatile unsigned long *) (SPI_S_WRBUF_BASE+idx));
      for(j=0;j<4;j++)
      {
       if((idx+j)<cnt)
       buf[idx+j]=(value>>(8*j))&0xff;
      }
     //buf[idx+3]=value>>24;
     //buf[idx+2]=(value>>16)&0xff;
      //buf[idx+1]=(value>>8)&0xff;
     //buf[idx]=value&0xff;
      idx=idx+4;
   }
   return 1;
}

static void SPI_S_write_reg(U32 reg, U32 value)
{
   (*((volatile unsigned long *) (reg)))=value;
}

static U32 SPI_S_read_reg(U32 reg)
{
   return (*((volatile unsigned long *) (reg)));
}

void gps1sgets(unsigned char *s, U32 mybaud) 
{
  if(mybaud<9)
  {
    unsigned long i,j,k;
  unsigned char a;
  volatile int *ioreg = (volatile int *) REG_BASE_MEMORYIO_ADDR;
  unsigned char slength;
  U32  status;
  do 
  {
  
    status=iord (UART1_RBBC+uart_offset); 
    slength=status&0x1f;
    while (slength==0)
    {
      status=iord (UART1_RBBC+uart_offset); 
      slength=status&0x1f;
       };
       for(j=0;j<(slength);j++)
          {
      i=iord (UART1_RBR+uart_offset);
      *s=(unsigned char)(i&0xff);
      if ((*s == '\0')) 
      {
        return;
      }
      s++;
    }
  } while (1);
 }
 else
 {
   U32 value;
  U32 size;
  U08 inputbuf[16];
  U08 j;
  do
  {
    value=SPI_S_read_reg(SPI_S_MAIN);
    if((value&0x2)==0)
    {
     //s_rx_byte=value;
     size=(value>>8)&0xff;
     SPI_S_wr_buff_read(inputbuf, size);
     value=value|0x2;
     SPI_S_write_reg(SPI_S_MAIN, value);
     for(j=0;j<(size);j++)
     {
     *s=(unsigned char)(inputbuf[j]&0xff);
     if ((*s == '\0')) 
     {
      return;
     }
     s++;
     }
    }
    
  }while(1);
 }
}

static U08 SPI_S_rd_buff_write(const U08 *buf, U08 cnt)
{
   U16 i;
   
   U32 value;
   U08 idx=0;
   //cnt=cnt/4;
   if(cnt>16)
   return 0;
   for(i=0;i<cnt;i++)
   {
      if(i<idx)
      continue;
      value=buf[idx+3];
      value=value<<8 | buf[idx+2];
      value=value<<8 | buf[idx+1];
      value=value<<8 | buf[idx];
      *((volatile unsigned long *) (SPI_S_RDBUF_BASE+idx)) = value;
      idx=idx+4;
   }
   return 1;

}

static U08 SPI_S_set_rd_buff_bytes(U32 bytes)
{
   if(bytes>16)
   return 0;
   *((volatile unsigned long *) (SPI_S_MAIN)) = ((*((volatile unsigned long *) (SPI_S_MAIN)))&0xff00ffff)
   | (bytes<<16);
   return 1;
}

static void SPI_S_set_rd_buff_ready()
{
   *((volatile unsigned long *) (SPI_S_MAIN)) = *((volatile unsigned long *) (SPI_S_MAIN))
   | 0x4;
}

void gps1sputs(const char *s, int s_len, U32 mybaud)
{
  if(mybaud<9)
  {
    unsigned long len = 0;
    if(s_len>0)
    {
      for(len=0;len<s_len;len++)
      {
        //if(((iord(UART1_TBBC)&0x1f) !=0x0))
        while (((iord(UART1_TBBC+uart_offset)&0x1f) ==0x0));
          iowr(UART1_THR+uart_offset, *s++);
      }
      //while (((iord (UART1_TBBC)&0x1f)!=0x10))   ;
      //while((iord(UART1_CESR)&0x8)==0);
    }
    else
    {
        while(*s) 
        {
          if(((iord(UART1_TBBC+uart_offset)&0x1f) !=0x0))
          {
          //while (((iord (UART1_PISR)&0x02)==0))   ;
            iowr(UART1_THR+uart_offset, *s++);
          }
        } while (((iord (UART1_TBBC+uart_offset)&0x1f)==0))   ;
      iowr(UART1_THR+uart_offset, '\0');
    }
  } //if(mybaud<9)
  else
  {
    if(s_len>0)
    {
      SPI_S_rd_buff_write(s, s_len);
      SPI_S_set_rd_buff_bytes(s_len);
      SPI_S_set_rd_buff_ready();
    }
    else
    {
      U08 count=0;
      U08 outbuf[16];
      while(*s) 
      {
        outbuf[count]=*s++;
        count++;
      }
      outbuf[count]='\0';
      count++;
      SPI_S_rd_buff_write(outbuf, count);
      SPI_S_set_rd_buff_bytes(count);
      SPI_S_set_rd_buff_ready();
    }
  }
}

int gps1sgetsbin(unsigned char *s, int size, U32 mybaud) 
{
  if(mybaud<9)
  {
    unsigned long binlength = 0;
  do 
  {
      U32 status = iord(UART1_RBBC + uart_offset); 
      unsigned long slength = status & 0x1f;
    while (slength==0)
    {
      status=iord(UART1_RBBC+uart_offset); 
      slength=status&0x1f;
       }
       
       unsigned long i, j;
      for(j=0; j<slength; j++)
          {
      i=iord (UART1_RBR+uart_offset);
      *s=(unsigned char)(i&0xff);
          ++binlength;
      if (binlength == size) 
      {
      return binlength;
      }
            s++;
    }
  } while (1);
}
else
{
  U32 value;
  U32 spisize;
  U08 inputbuf[16];
  U08 j;
  unsigned long binlength=0;
  do
  {
    value=SPI_S_read_reg(SPI_S_MAIN);
    if((value&0x2)==0)
    {
     //s_rx_byte=value;
     spisize=(value>>8)&0xff;
     SPI_S_wr_buff_read(inputbuf, spisize);
     value=value|0x2;
     SPI_S_write_reg(SPI_S_MAIN, value);
     for(j=0;j<(spisize);j++)
     {
     *s=(unsigned char)(inputbuf[j]&0xff);
     s++;
     }
     binlength=binlength+spisize;
     if (binlength == size) 
      {
      return binlength;
      }
    }
  }while(1);
}
}

VOID UART_init(U08 baudidx)
{
  U32 register_baud;
  U32 baudrate[9]={4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
  //U32 register_baud;

  U32 reg_val2;
  U08 uart_boost;
  reg_val2=*(volatile unsigned long *)(0x2000f004);
  if((baudidx==6)||(baudidx==7))
  {
    reg_val2=(reg_val2&0x3fffffff) | 0x40000000;
    uart_boost=2;  
  }
  else
  if((baudidx==8))
  {
    
    reg_val2=(reg_val2&0x3fffffff) | 0x80000000;
    uart_boost=4;  
  }
  else  //set back
  {
    reg_val2=reg_val2&0x3fffffff;
    uart_boost=1;
  }
  *(volatile unsigned long *)(0x2000f004)=reg_val2;


  U32 f08_val;
  *((volatile unsigned long *)(0x2000f018))=0x20000;
  f08_val=*((volatile unsigned long *)(0x2000f018));
  while((f08_val&0x20000)!=0)
  {
     f08_val=*((volatile unsigned long *)(0x2000f018));
   };//the bit 17 case not zero should wait to go further
  if(((f08_val&0x20000)==0)&&((f08_val&0x10000)==0))
  {
   f08_val=61378751*uart_boost;
  }
  else
  if(((f08_val&0x20000)==0)&&((f08_val&0x10000)!=0))
  {
   f08_val=f08_val&0xffff;
   f08_val=f08_val/8;
   f08_val=f08_val*32768;
  }
  else
  {
  //should not come here
  f08_val=61378751*uart_boost;
  }
  
  //register_baud=((f08_val/115200)*reg_val)/16;
  register_baud=((f08_val/baudrate[baudidx]))/16;


  
  iowr (UART1_LCR+uart_offset, 0x83); // set DLAB=1, set transaction bits=8
  iowr (UART1_DLL+uart_offset, register_baud&0xff); // now uart uses 50MHz clk, so, 50M/(px51 * 0x10) = 38400Hz (baud rate) 
  iowr (UART1_DLM+uart_offset, (register_baud>>8)&0xff);
  iowr (UART1_LCR+uart_offset, 0x03); // set DLAB=0
  iowr (UART1_FCR+uart_offset, 0x86); // 1. reset TX/RX FIFO,  2. set RX FIFO trigger level to 8 bytes
  //iowr (UART1_IER, 0x03); // set interrupt enables, 0x03==> TX/RX fifo status

  while((iord(UART1_CESR+uart_offset)&0x6)!=0);

}

int new_fifouart_wr_buf (char* buf,int byte_cnt)
{
  int i;
  //for (i=0;i<byte_cnt;i++)
   {
  iowr(UART1_THR+uart_offset, buf[byte_cnt]);
}
}

void InitSystem() 
{
  Register tmp;
  volatile int *lreg = (volatile int *) REG_BASE_ADDR;
  
  tmp = lreg[MCFG1/4];
  lreg[MCFG1/4] = (tmp& 0xffff0000) | (FLASH_WIDTH/FLASH_CHIP_WIDTH)<<8 | 0x877;

  if(((*(volatile unsigned long *)(0x2000f010))&0x80000000)!=0)//leon2
  {
    lreg[IMASK_LEON2/4] = 0x0; /* mask all interrupt */
    //cache keep to backward to ML605 leon2 board
    tmp = lreg[CACFG/4]; /* disable and flush cache */
    lreg[CACFG/4] = (tmp | 0x600000) & 0xfffffff0;  
  }
  else
  {
    lreg[IMASK/4] = 0x0; /* mask all interrupt */
  }
  
  tmp = lreg[MCFG1/4]; 
  lreg[MCFG1/4] = (tmp&0xffff)| 0x10080000; // 15 wait state 
  
  //cpu boost
  if(((*(volatile unsigned long *)(0x2000f000))&0x1)!=0)//uart 0
  {
    uart_offset = 0;
  }
  else
  {
    uart_offset = 0x100;
  }
  
  //check boot from where
  tmp = (*(volatile unsigned long *)(0x2000f010));
  chipmode = (tmp >> 20) & 0x7;
} //InitSystem()

void InitFlashWriteMode  ()
{
  Register tmp;
  if(chipmode==4) //external flash, make it as old way, write Flash
  {
    tmp = *(volatile unsigned long *)(0x20004c18);
    *(volatile unsigned long *)(0x20004c18)= tmp & 0xfeffffff;
  
    tmp = *(volatile unsigned long *)(0x20004c1c);
    *(volatile unsigned long *)(0x20004c1c)= tmp | 0x10000;
  }
}

void InitUart(int baud)
{
  if(baud < 9)
  {
    while((iord(UART1_CESR+uart_offset) & 0x8)==0);
    UART_init(baud);
  }
}

U32 GetInt(const int start, unsigned char* buf)
{
  U32 data = 0;
  static U08 i = 0;

  if(-1 == start)
  {
    i = 0;
    return 0;
  }
  
  i += start;
  for(; i < 100; i++)
  {
    if (buf[i] != ' ')
    {
      data = data*10 + buf[i] - 0x30;
    }
    else
    {
      break;
    }
  }
  return data;
}

void ScanParameter(char* inBuf, int *binSize, U08 *chkSum)
{
  const int LenOfBINSIZE = 10; // strlen("BINSIZE = ");
  const int LenOfChecksum = 12; // strlen(" Checksum = ");
Resendbin:
  GetInt(-1, 0);
  // get bytes count
  *binSize = GetInt(LenOfBINSIZE, inBuf);

  // get checksum 
  *chkSum = (U08)GetInt(LenOfChecksum, inBuf);
  
  if((inBuf[0] != 'B') || (inBuf[1] != 'I') || (inBuf[2] != 'N')) //if BIN is not correctly received
  {
    ErrorEnd(3);
  }  
}

void ScanParameter2(char* inBuf, int *binSize, U08 *chkSum, U32 *baud, U32 *tagAdd, U32 *tagVal)
{  
  // BINSIZ2 = promSize checkSum mybaud tagAdd tagVal checkCode
  //"BINSIZ2 = 563600 247 7 2862966102 1431696884 258944 "

  const int LenOfBINSIZE = 10; // strlen("BINSIZE 2 ");
  GetInt(-1, 0);
  // get bytes count
  *binSize = GetInt(LenOfBINSIZE, inBuf);

  // get checksum 
  *chkSum = (U08)GetInt(1, inBuf);
  
  // get mybaud 
  *baud = GetInt(1, inBuf);
  
  // get tagAdd 
  *tagAdd = GetInt(1, inBuf);
  
  // get tagVal 
  *tagVal = GetInt(1, inBuf);
  
  // get checkCode 
  U32 checkCode = GetInt(1, inBuf);
  
  if(checkCode != (*binSize + *chkSum + *baud + *tagAdd + *tagVal))
  {
    ErrorEnd(2);    
  }
  
  eraseTag = 1;
  if(*tagAdd != 0)
  {
    writeTag = 1;
    *tagAdd = *tagAdd ^ 0xAAAAAAAA;
    *tagVal = *tagVal ^ 0x55555555;
  }
}

void ScanParameter3(char* inBuf, int *binSize, U08 *chkSum, U32 *baud)
{  
  // BINSIZ3 = promSize checkSum mybaud checkCode
  //"BINSIZ3 = 563600 247 7 563854 "

  const int LenOfBINSIZE = 10; // strlen("BINSIZE 2 ");
  GetInt(-1, 0);
  // get bytes count
  *binSize = GetInt(LenOfBINSIZE, inBuf);

  // get checksum 
  *chkSum = (U08)GetInt(1, inBuf);
  
  // get mybaud 
  *baud = GetInt(1, inBuf);
   
  // get checkCode 
  U32 checkCode = GetInt(1, inBuf);
  
  if(checkCode != (*binSize + *chkSum + *baud))
  {
    ErrorEnd(2);    
  }
  
  eraseTag = 0;
  writeTag = 0;
}

void ScanParameter4(char* inBuf, int *binSize, U08 *chkSum, U32 *baud, U32 *tagAdd, U32 *tagVal)
{  
  // BINSIZ2 = promSize checkSum mybaud tagAdd tagVal checkCode
  //"BINSIZ2 = 563600 247 7 2862966102 1431696884 258944 "

  const int LenOfBINSIZE = 10; // strlen("BINSIZE 2 ");
  GetInt(-1, 0);
  // get bytes count
  *binSize = GetInt(LenOfBINSIZE, inBuf);

  // get checksum 
  *chkSum = (U08)GetInt(1, inBuf);
  
  // get mybaud 
  *baud = GetInt(1, inBuf);
  
  // get tagAdd 
  *tagAdd = GetInt(1, inBuf);
  
  // get tagVal 
  *tagVal = GetInt(1, inBuf);
  
  // get checkCode 
  U32 checkCode = GetInt(1, inBuf);
  
  if(checkCode != (*binSize + *chkSum + *baud + *tagAdd + *tagVal))
  {
    ErrorEnd(2);    
  }
  
  eraseTag = 1;
  if(*tagAdd != 0)
  {
    writeTag = 0;
    *tagAdd = *tagAdd ^ 0xAAAAAAAA;
    *tagVal = *tagVal ^ 0x55555555;
  }
}

void DebugOutput(const char *title, U32 data)
{
  char buf[128];
  int i = 0;
  while(title[i])
  {
    buf[i] = title[i];
    ++i;
  }
  
  unsigned long long m = 0x0F;
  int j = 0;
  for(;j < 8; ++j)
  {
    char c = (data >> (7- j) * 4 ) & m;
    if(c > 9)
    {
      buf[i] = c - 10 + 'A'  ;
    }
    else
    {
      buf[i] = c + '0';
    }
    ++i;
  }
  buf[i] = 0;
  gps1sputs(buf, 0, mybaud);
}

void DisableCache()
{
  Register tmp;
  //disable cache
  asm volatile ("flush");
  asm volatile ("set 0x0, %g1");
  asm volatile ("sta %g1, [%g0] 2");
   
  //try parallel flash here
  tmp = *(volatile unsigned long *)(0x20004c18);
  *(volatile unsigned long *)(0x20004c18) = tmp & 0xfeffffff;
  tmp = *(volatile unsigned long *)(0x20004c1c);
  *(volatile unsigned long *)(0x20004c1c) = tmp | 0x10000;
  
}
 
void ErrorEnd(int error)
{
  
  char estring[8] = "Error0";
  
  if(error < 10)
  {
    estring[5] = '0' + error;
  }

  while(1)
  {
    gps1sputs(estring, 0, mybaud);
  }
}

/*
void WriteTag(U32 tagAdd, U32 tagVal)
{
  FLASH_T dl; 
  char buf[2];
  
  if(chipmode==4)
  {
    dl = tagVal;
    if(program_flash(tagAdd, 2 * (FLASH_WIDTH/FLASH_CHIP_WIDTH), norId, norDev, dl)==0)
    {
      ErrorEnd(4); //flash test fail
    }
  }
  else
  {
    buf[0] = (tagVal >> 8) & 0xFF;
    buf[1] = (tagVal) & 0xFF;
    
    //buf[0] = 0x0A;
    //buf[1] = 0x01;
   
    //DebugOutput("buf[0] ", (U32)buf[0]);
    //DebugOutput("buf[1] ", (U32)buf[1]);
    //DebugOutput("tagAdd ", tagAdd);
    //DebugOutput("tagVal ", tagVal);
    if(QSPI_data_write(tagAdd, &(buf[0]), 2)==0)
    {
      ErrorEnd(4);
    }
  } 
}

void TestFlash()
{
  U08 myflashtype = 0;
  U16 myflashid = 0;
  
  if(chipmode==4) //parallel flash
  {
    DisableCache();
    
    if(0 == check_flash(&norId, &norDev, myflashtype, myflashid))
    {
      ErrorEnd(1);
    }
  } //if(chipmode==4)
  else if(chipmode==1)//windbond serial flash
  {
    if(-1 == QSPI_Init(myflashtype, myflashid))
    {
      ErrorEnd(1);
    }
  } //else if(chipmode==1)
  else if(chipmode==2) //eon serial flash
  {
    if(-1 == QSPI_Init(myflashtype, myflashid))
    {
      ErrorEnd(1);
    }
  } //if(chipmode==2)
  else if(chipmode==0)//at ROM boot
  {
    //try windfirst qspi here
    chipmode = 1;
    if(-1 == QSPI_Init(myflashtype, myflashid))
    {
      //again try eon qspi here
      chipmode = 2;
      if (-1 == QSPI_Init(myflashtype, myflashid))
      {
        DisableCache();
        if(0 == check_flash(&norId, &norDev, myflashtype, myflashid))
        {
          ErrorEnd(1);
        }
      } //if (spi_entry == -1)
    } //if(spi_entry == -1)
  } //else if(chipmode==0)
}

void EraseFlash(unsigned long *flashstart, int promSize)
{
  unsigned char is1m = 0;
  const int eraseSize = 0x20;
  volatile int j;
  int i;
  FLASH_T dl; 
  
  //verify ID if 1M or 512K for parallel
  if(chipmode==4)
  {
    if(((norId== NUMONYX) && (norDev==JS28F256P30T)))
    {
        
    }
    else if(((norId== EON_FLASH) && ((norDev==A29LV400AT) || (norDev == A29LV800AT))))
    {
      if(norDev==A29LV800AT)
      {
        is1m = 1;
      }
    }
    else
    {
      norId = AMIC_FLASH;
      norDev = A29L400AUV;
    }

    *flashstart = 0x78554;
    //size = 0x20;
    if(erase_sectors(*flashstart, eraseSize, norId, norDev) == 0)
    {
      ErrorEnd(4);
    }
    
    
    for(i=0; i<4; i=i+2)
    {
      if(*((volatile unsigned short *)(*flashstart + i)) != 0xffff)
      {
        ErrorEnd(5);
      }
    }
    for(i=0; i<4; i=i+2)
    {
      dl = (i==0) ? 0xaa55 : 0x55aa;
      if(program_flash(*flashstart + i, 2 * (FLASH_WIDTH / FLASH_CHIP_WIDTH), norId, norDev, dl) == 0)
      {
         ErrorEnd(4);
      }
      if(*((volatile unsigned short *)(*flashstart + i)) != dl)
      {
        ErrorEnd(4);
      }
    } //for(i=0; i<4; i=i+2)

    if(norId == NUMONYX)
    {
      erase_sectors(*flashstart, eraseSize, norId, norDev);
      erase_sectors(0x1ff0000, eraseSize, norId, norDev);
      erase_sectors(0x1ff8000, eraseSize, norId, norDev);
    }
    else
    {
      int testoffset = (is1m == 1) ? 0x80000 : 0;
      erase_sectors(*flashstart, eraseSize, norId, norDev);
      erase_sectors(0x70554 + testoffset, eraseSize, norId, norDev);
      erase_sectors(0x7a554 + testoffset, eraseSize, norId, norDev);
      erase_sectors(0x7c554 + testoffset, eraseSize, norId, norDev);
      if(eraseTag)
      {
        erase_sectors(0x77554 + testoffset, eraseSize, norId, norDev);
      }
    }
  } //if(chipmode==4)
  else if(chipmode==1)
  {
    *flashstart = 0x78554;
    //size = 0x20;
    qspi_erase_sectors(0xfd000, eraseSize);
    qspi_erase_sectors(0xfe000, eraseSize);
    qspi_erase_sectors(0xff000, eraseSize);
    if(eraseTag)
    {
//      DebugOutput("EraseTag ", tagAdd & 0xFFFFF000);
 
      qspi_erase_sectors(tagAdd & 0xFFFFF000, eraseSize);
    }
  } //else if(chipmode==1)
  
  *flashstart = 0x0;
  if(chipmode==4)
  {
    if(erase_sectors(*flashstart, promSize, norId, norDev)==0)
    {
      ErrorEnd(4);
    }
  }
  else if((chipmode==2) || (chipmode==1))
  {
    if(qspi_erase_sectors(*flashstart, promSize)==0)
    {
      ErrorEnd(4);
    }
  }
}

//*/
static void GetVersion(void)
{
  _LOADER_VER_[0] = '$';
  _LOADER_VER_[1] = 'L';
  _LOADER_VER_[2] = 'O';
  _LOADER_VER_[3] = 'A';
  _LOADER_VER_[4] = 'D';
  _LOADER_VER_[5] = 'E';
  _LOADER_VER_[6] = 'R';
  _LOADER_VER_[7] = ',';
  _LOADER_VER_[8] = REV_ID[0];
  _LOADER_VER_[9] = REV_ID[1];
  _LOADER_VER_[10] = REV_ID[2];
  _LOADER_VER_[11] = REV_ID[3];
  _LOADER_VER_[12] = REV_ID[4];
  _LOADER_VER_[13] = ',';
  _LOADER_VER_[14] = __DATE__[9];
  _LOADER_VER_[15] = __DATE__[10];
  _LOADER_VER_[16] = BUILD_MONTH[0];
  _LOADER_VER_[17] = BUILD_MONTH[1];
  _LOADER_VER_[18] = (__DATE__[4]==' ') ? '0' : __DATE__[4];
  _LOADER_VER_[19] = __DATE__[5];
  _LOADER_VER_[20] = __TIME__[0];
  _LOADER_VER_[21] = __TIME__[1];
  _LOADER_VER_[22] = __TIME__[3];
  _LOADER_VER_[23] = __TIME__[4];
  _LOADER_VER_[24] = __TIME__[6];
  _LOADER_VER_[25] = __TIME__[7];
  _LOADER_VER_[26] = 0x0d;
  _LOADER_VER_[27] = 0x0a;
  _LOADER_VER_[28] = 0;
}

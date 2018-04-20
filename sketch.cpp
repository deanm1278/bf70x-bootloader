/*
This is a simple openocd based bootloader for easily writing to the external flash
on a bf706 board.
*/

#include <Arduino.h>

#include <SPI.h>
#include <Adafruit_SPIFlash.h>

#define FLASH_TYPE     SPIFLASHTYPE_W25Q16BV

#define FLASH_SS       SS1                    // Flash chip SS pin.
#define FLASH_SPI_PORT _SPI1                   // What SPI port is Flash on?

Adafruit_SPIFlash flash(FLASH_SS, &FLASH_SPI_PORT);

#define EZ_LOAD_BASE_ADDR       (0x11900000ul)

//the command register
#define EZ_LOAD_CMD_OFFSET      (0x00ul)
#define EZ_LOAD_CMD             *((volatile uint32_t *)(EZ_LOAD_BASE_ADDR + EZ_LOAD_CMD_OFFSET))

//the address register
#define EZ_LOAD_ADDR_OFFSET     (0x04ul)
#define EZ_LOAD_ADDR            *((volatile uint32_t *)(EZ_LOAD_BASE_ADDR + EZ_LOAD_ADDR_OFFSET))

//the count register
#define EZ_LOAD_COUNT_OFFSET    (0x08ul)
#define EZ_LOAD_COUNT           *((volatile uint32_t *)(EZ_LOAD_BASE_ADDR + EZ_LOAD_COUNT_OFFSET))

//the status register
#define EZ_LOAD_STATUS_OFFSET   (0x0Cul)
#define EZ_LOAD_STATUS          *((volatile uint32_t *)(EZ_LOAD_BASE_ADDR + EZ_LOAD_STATUS_OFFSET))

//the page buffer
#define EZ_LOAD_PAGE_OFFSET     (0x10ul)
#define EZ_LOAD_PAGE            *((volatile uint32_t *)(EZ_LOAD_BASE_ADDR + EZ_LOAD_PAGE_OFFSET))

//the high byte must be set to this to execute
#define EZ_CMD_EXECUTE          (0xA5ul)

#define EZ_CMD_CHIP_ERASE       (0x01ul)
#define EZ_CMD_ERASE_BLOCK      (0x02ul)
#define EZ_CMD_WRITE_BLOCK      (0x03ul)
#define EZ_CMD_READ_BLOCK       (0x04ul)
#define EZ_CMD_INFO             (0x05ul)

#define EZ_STATUS_OK            (0x00ul)
#define EZ_STATUS_BUSY          (0x01ul)
#define EZ_STATUS_ERROR         (0x02ul)

#define EZ_LOAD_IDCODE          (0x56ul)

#define BUF_MAX                 ((uint32_t)(16 * 1024))

//#define AUTO_VERIFY

#ifdef AUTO_VERIFY
L2DATA uint8_t verifyBuf[BUF_MAX];
#endif

void setup() {
  Serial.begin(9600);

  Serial.println("bf706 ez-kit openocd bootloader");

  // Initialize flash library and check its chip ID.
  if (!flash.begin(FLASH_TYPE)) {
    Serial.println("could not start flash");
    while(1);
  }

  uint32_t id = flash.GetJEDECID();
  if(id & 0xFF000000 != 0xEF){
      Serial.println("incorrect manufacturer id returned");
      while(1);
  }
}

static inline void setOutData(uint32_t *buf, uint32_t count){
    volatile uint32_t *ptr = &EZ_LOAD_PAGE;
    for(uint32_t i=0; i<count; i++){
        *ptr++ = *buf++;
    }
    EZ_LOAD_COUNT = count;
}

void loop() {
    if( EZ_LOAD_CMD & (EZ_CMD_EXECUTE << 8) == (EZ_CMD_EXECUTE << 8)){
        //command has been received.
        EZ_LOAD_STATUS = EZ_STATUS_BUSY;

        uint32_t status = EZ_STATUS_OK;
        switch((uint8_t)EZ_LOAD_CMD & 0xFF){
        case EZ_CMD_INFO: {
            Serial.println("info command received");

            uint32_t c = EZ_LOAD_IDCODE;
            setOutData(&c, 1);

            break;
        }
        case EZ_CMD_CHIP_ERASE: {
            Serial.println("performing chip erase...");
            flash.eraseChip();
            Serial.println("chip erase finished.");
            break;
        }
        case EZ_CMD_WRITE_BLOCK: {
            uint32_t count = EZ_LOAD_COUNT;
            uint32_t addr = EZ_LOAD_ADDR;
            uint8_t *buf = (uint8_t *)&EZ_LOAD_PAGE;
            Serial.print("writing ");
            Serial.print(count);
            Serial.print(" words at addr 0x");
            Serial.println(addr, HEX);

            flash.writeBuffer(addr, buf, count*4);
#ifdef AUTO_VERIFY
            flash.readBuffer(addr, verifyBuf, count*4);

            for(uint32_t i=0; i<count; i++){
                if(verifyBuf[i] != *(buf + i)){
                    Serial.print("mismatch found at 0x");
                    Serial.print(addr + i, HEX);
                    Serial.print(" expected 0x");
                    Serial.print(*(buf + i), HEX);
                    Serial.print(" received 0x");
                    Serial.println(verifyBuf[i], HEX);
                    status = EZ_STATUS_ERROR;
                    break;
                }
            }
#endif

            break;
        }
        default:{
            Serial.println("unknown command received!");
            status = EZ_STATUS_ERROR;
            break;
        }
        }

        EZ_LOAD_CMD = 0;
        EZ_LOAD_STATUS = status;
    }
}
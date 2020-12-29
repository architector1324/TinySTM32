#ifndef _SDHC_H_
#define _SDHC_H_

#include "hal.h"


////////////////////////////////
//          SETTINGS          //
////////////////////////////////


////////////////////////////////
//         DEFINITION         //
////////////////////////////////

typedef struct {
    hal_spi_t spi;
    hal_gpio_pin_t cs;
} sdhc_t;

typedef enum {
    SD_OK,
    SD_FAIL,
    SD_FAIL_BUSY,
    SD_FAIL_RESET,
    SD_FAIL_VOLTAGE,
    SD_FAIL_NOT_SDHC,
    SD_FAIL_OFF_CRC,
    SD_FAIL_BLOCK_LEN
} sdhc_status_t;

static bool sdhc_not_bsy(const sdhc_t* sd);
static uint8_t sdhc_cmd(uint8_t cmd, uint32_t arg, uint8_t crc, const sdhc_t* sd);
static void sdhc_cmd_r7(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t res[5], const sdhc_t* sd);

sdhc_status_t sdhc_init(const sdhc_t* sd);

sdhc_status_t sdhc_w(const uint8_t block512[512], size_t block_id, const sdhc_t* sd);
sdhc_status_t sdhc_r(uint8_t block512[512], size_t block_id, const sdhc_t* sd);

////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

static bool sdhc_not_bsy(const sdhc_t* sd) {
    uint8_t tmp;
    for(size_t i = 0; i < 1000; i++) {
        tmp = hal_spi_r(sd->spi);

        if(tmp == 0xff) return true;
    }
    return false;
}

static uint8_t sdhc_cmd(uint8_t cmd, uint32_t arg, uint8_t crc, const sdhc_t* sd) {
    hal_spi_sel(&sd->cs);

    // cmd
    hal_spi_w(cmd | 0x40, sd->spi);

    // arg
    hal_spi_w((uint8_t)(arg >> 24), sd->spi);
    hal_spi_w((uint8_t)(arg >> 16), sd->spi);
    hal_spi_w((uint8_t)(arg >> 8), sd->spi);
    hal_spi_w((uint8_t)(arg), sd->spi);

    // crc
    hal_spi_w(crc | 0x1, sd->spi);

    // response
    uint8_t res = 0;

    for(size_t i = 0; i < 1000; i++) {
        res = hal_spi_r(sd->spi);
        if((res & 0x80) == 0) break;
    }
    hal_spi_desel(&sd->cs);

    return res;
}

static void sdhc_cmd_r7(uint8_t cmd, uint32_t arg, uint8_t crc, uint8_t res[5], const sdhc_t* sd) {
    hal_spi_sel(&sd->cs);

    // cmd
    hal_spi_w(cmd | 0x40, sd->spi);

    // arg
    hal_spi_w((uint8_t)(arg >> 24), sd->spi);
    hal_spi_w((uint8_t)(arg >> 16), sd->spi);
    hal_spi_w((uint8_t)(arg >> 8), sd->spi);
    hal_spi_w((uint8_t)(arg), sd->spi);

    // crc
    hal_spi_w(crc | 0x1, sd->spi);

    // response
    for(size_t i = 0; i < 1000; i++) {
        res[0] = hal_spi_r(sd->spi);
        if((res[0] & 0x80) == 0) break;
    }
    if(res[0] > 1) {
        hal_spi_desel(&sd->cs);
        return;
    }

    for(size_t i = 1; i < 5; i++)
        res[i] = hal_spi_r(sd->spi);

    hal_spi_desel(&sd->cs);
}

sdhc_status_t sdhc_init(const sdhc_t* sd) {
    hal_delay(250);
    hal_spi_desel(&sd->cs);

    // 80 times 1
    for(size_t i = 0; i < 10; i++)
        hal_spi_w(0xff, sd->spi);

    // cmd0 (reset sd card)
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    uint8_t tmp = sdhc_cmd(0, 0, 0x95, sd);
    if(tmp != 1) return SD_FAIL_RESET;

    // cmd8 (check voltage)
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    uint8_t tmp2[5];
    sdhc_cmd_r7(8, 0x000001aa, 0x87, tmp2, sd);

    if(((tmp2[0] & 0x4) == 0x4) || ((tmp2[1] & 0x40) != 0)) return SD_FAIL_NOT_SDHC;
    if((tmp2[0] != 1) || (tmp2[4] != 0xaa)) return SD_FAIL_VOLTAGE;

    // cmd55 / acmd41 (init)
    for(size_t i = 0; i < 1000; i++) {
        if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

        // cmd55
        tmp = sdhc_cmd(55, 0, 0x95, sd);
        if(tmp != 1) return SD_FAIL;

        if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

        // acmd41
        tmp = sdhc_cmd(41, 0x40000000, 0x95, sd);
        if(tmp == 0) break;
        if(tmp != 1) return SD_FAIL;
    }
    if(tmp != 0) return SD_FAIL;

    // cmd58 (read ocr)
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    sdhc_cmd_r7(58, 0, 0x95, tmp2, sd);
    if(tmp2[0] > 1)
        return SD_FAIL;

    // cmd59 (off crc)
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;
    
    tmp = sdhc_cmd(59, 0, 0x95, sd);
    if(tmp != 0) return SD_FAIL_OFF_CRC;

    // cmd16 (set block)
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    tmp = sdhc_cmd(16, 512, 0x95, sd);
    if(tmp != 0) return SD_FAIL_BLOCK_LEN;

    return SD_OK;
}

sdhc_status_t sdhc_w(const uint8_t block512[512], size_t block_id, const sdhc_t* sd) {
    // cmd
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    uint8_t tmp = sdhc_cmd(24, block_id, 0x95, sd);
    if(tmp != 0) return SD_FAIL;

    hal_spi_sel(&sd->cs);

    // data
    hal_spi_w(0xff, sd->spi);  // start
    hal_spi_w(0xfe, sd->spi);

    for(size_t i = 0; i < 512; i++)
        hal_spi_w(block512[i], sd->spi);

    hal_spi_w(0xff, sd->spi);
    hal_spi_w(0xff, sd->spi);  // crc

    // response
    tmp = hal_spi_r(sd->spi);
    if((tmp & 0x5) != 0x5) {
        hal_spi_desel(&sd->cs);
        return SD_FAIL;
    }

    for(size_t i = 0; i < 100; i++) {
        tmp = hal_spi_r(sd->spi);
        if(tmp == 0xff) break;
        hal_delay(10);
    }
    if(tmp != 0xff) {
        hal_spi_desel(&sd->cs);
        return SD_FAIL;
    }

    hal_spi_desel(&sd->cs);

    return SD_OK;
}

sdhc_status_t sdhc_r(uint8_t block512[512], size_t block_id, const sdhc_t* sd) {
    hal_spi_sel(&sd->cs);

    // cmd
    if(!sdhc_not_bsy(sd)) return SD_FAIL_BUSY;

    uint8_t tmp = sdhc_cmd(17, block_id, 0x95, sd);
    if(tmp != 0) return SD_FAIL;

    hal_spi_sel(&sd->cs);
    hal_spi_w(0xff, sd->spi);

    for(size_t i = 0; i < 100; i++) {
        tmp = hal_spi_r(sd->spi);
        if(tmp == 0xfe) break;
        hal_delay(10);
    }
    if(tmp != 0xfe) {
        hal_spi_desel(&sd->cs);
        return SD_FAIL;
    }

    // data
    for(size_t i = 0; i < 512; i++)
        block512[i] = hal_spi_r(sd->spi);

    // crc
    hal_spi_w(0xff, sd->spi);
    hal_spi_w(0xff, sd->spi);

    hal_spi_desel(&sd->cs);

    return SD_OK;
}

#endif
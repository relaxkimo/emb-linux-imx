#ifndef _ETH_SPI_PROTOCOL_H_
#define _ETH_SPI_PROTOCOL_H_

#include <linux/types.h>
#include <linux/byteorder/generic.h>
#include <linux/build_bug.h>
#include <linux/crc-itu-t.h>

#define ETH_SPI_FRAG_LEN    (128u)
#define ETH_SPI_MAX_MTU     (16u * ETH_SPI_FRAG_LEN)

#define ETH_SPI_CRC_SIZE   2u

struct eth_spi_frame {
    u8 type;
    u8 frag_idx : 4;
    u8 frag_tot : 4;
    u16 len;
    u8 buf[ETH_SPI_FRAG_LEN];
    u16 crc;
} __packed;

static_assert(sizeof(struct eth_spi_frame) == (ETH_SPI_FRAG_LEN + 4u + ETH_SPI_CRC_SIZE));

static inline void eth_spi_init_frame(struct eth_spi_frame* frame, u8 frag_idx, u8 frag_tot, u16 len)
{
    u16 crc;

    frame->type = 0;
    frame->frag_idx = frag_idx;
    frame->frag_tot = frag_tot;
    frame->len = cpu_to_be16(len);
    crc = crc_itu_t(0, (const u8 *)frame, (sizeof(struct eth_spi_frame) - ETH_SPI_CRC_SIZE));
    frame->crc = cpu_to_be16(crc);
}

static inline void eth_spi_reset_frame(struct eth_spi_frame* frame)
{
    frame->type = 0;
    frame->frag_idx = 0;
    frame->frag_tot = 0;
    frame->len = cpu_to_be16(0);
    frame->crc = cpu_to_be16(0);
}

static inline bool eth_spi_crc_check(const struct eth_spi_frame *frame)
{
#ifdef ETH_SPI_SW_CRC
    return crc16_itu_t(0, (const uint8_t *)frame, sizeof(struct eth_spi_frame)) ==
           CRC16_XMODEM_MAGIC;
#else
    return true;
#endif
}


static inline bool eth_spi_frame_valid(const struct eth_spi_frame* frame)
{
    return (frame != NULL) && (be16_to_cpu(frame->len) <= ETH_SPI_FRAG_LEN) &&
           eth_spi_crc_check(frame) && (frame->frag_idx <= frame->frag_tot);
}

#endif

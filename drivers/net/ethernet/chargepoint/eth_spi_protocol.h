#ifndef _ETH_SPI_PROTOCOL_H_
#define _ETH_SPI_PROTOCOL_H_

#include <linux/types.h>
#include <linux/byteorder/generic.h>
#include <linux/build_bug.h>

#define ETH_SPI_FRAG_LEN    (128u)
#define ETH_SPI_MTU         (16 * ETH_SPI_FRAG_LEN)

#define ETH_SPI_SOF         0x55AAu
#define ETH_SPI_EOF         0xAA55u

struct eth_spi_frame {
    u16 sof;
    u8 type;
    u8 frag_idx : 4;
    u8 frag_tot : 4;
    u16 len;
    u16 reserved_0;
    u8 buf[ETH_SPI_FRAG_LEN];
    u16 eof;
} __packed;

static_assert(sizeof(struct eth_spi_frame) == (ETH_SPI_FRAG_LEN + 10u));

static inline void eth_spi_init_frame(struct eth_spi_frame* frame, u8 frag_idx, u8 frag_tot, u16 len)
{
    frame->sof = cpu_to_le16(ETH_SPI_SOF);
    frame->type = 0;
    frame->frag_idx = frag_idx;
    frame->frag_tot = frag_tot;
    frame->len = cpu_to_le16(len);
    frame->eof = cpu_to_le16(ETH_SPI_EOF);
}

static inline void eth_spi_reset_frame(struct eth_spi_frame* frame)
{
    frame->sof = cpu_to_le16(0);
    frame->type = 0;
    frame->frag_idx = 0;
    frame->frag_tot = 0;
    frame->len = cpu_to_le16(0);
    frame->eof = cpu_to_le16(0);
}

static inline bool eth_spi_frame_valid(const struct eth_spi_frame* frame)
{
    return (frame != NULL) && (le16_to_cpu(frame->sof) == ETH_SPI_SOF) &&
        (le16_to_cpu(frame->eof) == ETH_SPI_EOF) &&
        (le16_to_cpu(frame->len) <= ETH_SPI_FRAG_LEN) && 
        (frame->frag_idx <= frame->frag_tot);
}

#endif


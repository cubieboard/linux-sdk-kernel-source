#ifndef __SUNXI_SS__ 
#define __SUNXI_SS__ 
 
#define SUNXI_SS_REG_BASE        0x01C15000 
 
#define SUNXI_SS_IRQ                86 
 
#define SUNXI_SS_CTL                0x00 
#define SUNXI_SS_KEY0                0x04 
#define SUNXI_SS_KEY1                0x08 
#define SUNXI_SS_KEY2                0x0C 
#define SUNXI_SS_KEY3                0x10 
#define SUNXI_SS_KEY4                0x14 
#define SUNXI_SS_KEY5                0x18 
#define SUNXI_SS_KEY6                0x1C 
#define SUNXI_SS_KEY7                0x20 
 
#define SUNXI_SS_IV0                0x24 
#define SUNXI_SS_IV1                0x28 
#define SUNXI_SS_IV2                0x2C 
#define SUNXI_SS_IV3                0x30 
 
#define SUNXI_SS_CNT0                0x34 
#define SUNXI_SS_CNT1                0x38 
#define SUNXI_SS_CNT2                0x3C 
#define SUNXI_SS_CNT3                0x40 
 
#define SUNXI_SS_FCSR                0x44 
#define SUNXI_SS_ICSR                0x48 
 
#define SUNXI_SS_MD0                0x4C 
#define SUNXI_SS_MD1                0x50 
#define SUNXI_SS_MD2                0x54 
#define SUNXI_SS_MD3                0x58 
#define SUNXI_SS_MD4                0x5C 
 
#define SUNXI_SS_RXFIFO                0x200 
#define SUNXI_SS_TXFIFO                0x204 
 
 
/* SUNXI_SS_CTL configuration values */ 
 
/* AES/DES/3DES key select - bits 24-27 */ 
#define SUNXI_KEYSELECT_KEYN            (0 << 24) 
#define SUNXI_KEYSELECT_SID_RKEYN       (1 << 24) 
#define SUNXI_KEYSELECT_IKEY(n)         ((3 + n) << 24) 
 
/* PRNG generator mode - bit 15 */ 
#define SUNXI_PRNG_ONESHOT              (0 << 15) 
#define SUNXI_PRNG_CONTINUE             (1 << 15) 
 
/* IV Steady of SHA-1/MD5 constants - bit 14 */ 
#define SUNXI_IV_CONSTANTS              (0 << 14) 
#define SUNXI_IV_ARBITRARY              (1 << 14) 
 
/* SS operation mode - bits 12-13 */ 
#define SUNXI_SS_ECB                    (0 << 12) 
#define SUNXI_SS_CBC                    (1 << 12) 
#define SUNXI_SS_CNT                    (2 << 12) 
 
/* Counter width for CNT mode - bits 10-11 */ 
#define SUNXI_CNT_16BITS                (0 << 10) 
#define SUNXI_CNT_32BITS                (1 << 10) 
#define SUNXI_CNT_64BITS                (2 << 10) 
 
/* Key size for AES - bits 8-9 */ 
#define SUNXI_AES_128BITS               (0 << 8) 
#define SUNXI_AES_192BITS               (1 << 8) 
#define SUNXI_AES_256BITS               (2 << 8) 
 
/* Operation direction - bit 7 */ 
#define SUNXI_SS_ENCRYPTION             (0 << 7) 
#define SUNXI_SS_DECRYPTION             (1 << 7) 
 
/* SS Method - bits 4-6 */ 
#define SUNXI_OP_AES                    (0 << 4) 
#define SUNXI_OP_DES                    (1 << 4) 
#define SUNXI_OP_3DES                   (2 << 4) 
#define SUNXI_OP_SHA1                   (3 << 4) 
#define SUNXI_OP_MD5                    (4 << 4) 
#define SUNXI_OP_PRNG                   (5 << 4) 
 
/* Data end bit - bit 2 */ 
#define SUNXI_SS_DATA_END                BIT(2) 
 
/* PRNG start bit - bit 1 */ 
#define SUNXI_PRNG_START                BIT(1) 
 
/* SS Enable bit - bit 0 */ 
#define SUNXI_SS_DISABLED               (0 << 0) 
#define SUNXI_SS_ENABLED                (1 << 0) 
 
/* RX FIFO status - bit 30 */ 
#define SUNXI_RXFIFO_FREE               BIT(30) 
 
/* RX FIFO empty spaces - bits 24-29 */ 
#define SUNXI_RXFIFO_SPACES(val)        (((val) >> 24) & 0x3f) 
 
/* TX FIFO status - bit 22 */ 
#define SUNXI_TXFIFO_AVAILABLE          BIT(22) 
 
/* TX FIFO available spaces - bits 16-21 */ 
#define SUNXI_TXFIFO_SPACES(val)        (((val) >> 16) & 0x3f) 
 
#define SUNXI_RXFIFO_EMP_INT_PENDING        BIT(2) 
#define SUNXI_TXFIFO_AVA_INT_PENDING        BIT(1) 
 
#define SUNXI_SS_ICS_DRA_ENABLE                BIT(4) 
 
#endif 
 

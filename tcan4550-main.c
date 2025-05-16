// SPDX-License-Identifier: GPL-2.0
// CAN driver for TI TCAN4550
// Driver is optimized to work on systems with high SPI bus latencies
// writing large SPI packages when possible
// Copyright (C) 2023 CrossControl

#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/netlink.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>
#include <linux/version.h>

// 32-bit SPI transfers are a little faster as there is no delay between the
// bytes in a word. However, for instance Raspberry Pi 4 only support 8-bit
// transfers. Depending on SPI controller activating MSB-LSB swap might also
// be necessary when running in 32-bit mode.
#define USE_32BIT_SPI_TRANSFERS
#define MSB_LSB_SWAP

// Message RAM (MRAM) config. User adjustable.
const static uint32_t RX_SLOT_SIZE          = 16;  // size of one element in the rx fifo
const static uint32_t TX_SLOT_SIZE          = 16;  // size of one element in the tx fifo
const static uint32_t TX_FIFO_SIZE          = 32;  // possible values = 0 - 32
const static uint32_t TX_FIFO_START_ADDRESS = 0x0; // position in MRAM where tx fifo start (excluding MRAM base address)
const static uint32_t RX_FIFO_SIZE          = 64;  // possible values = 0 - 64
const static uint32_t RX_FIFO_START_ADDRESS =
    0x200; // position in MRAM where rx fifo start (excluding MRAM base address)
const static uint32_t EVENT_FIFO_START_ADDRESS =
    0x600; // position in MRAM where tx event fifo start (excluding MRAM base address)
const static uint32_t EVENT_FIFO_SIZE      = 0; // elements in the event FIFO
const static uint32_t EVENT_FIFO_WATERMARK = 0; // watermark level to generate interrupt

// SPI burst settings. User adjustable.
#define MAX_SPI_BURST_TX_MESSAGES \
    8 // Max CAN messages in a SPI write. A high value gives better TX throughput but can lead to lost rx messages due
      // to blocking rx too long.
#define MAX_SPI_BURST_RX_MESSAGES 48 // Max CAN messages in a SPI read

// Buffer configuration. User adjustable.
#define TX_BUFFER_SIZE \
    (16 + 1) // size of tx-buffer used between Linux networking stack and SPI. One slot is reserved to be able to keep
             // track of if queue is full
#define ECHO_BUFFERS 1 // Number of buffers allocated for local echo of can msgs

#define NAPI_BUDGET 64 // maximum number of messages that NAPI will request
#define RX_BUFFER_SIZE \
    (64 + 1) // size of buffer to store messages from chip until fetched by NAPI. One slot is reserved to be able to
             // keep track of if queue is full.

// TCAN4550 Registers
const static uint32_t DEVICE_ID1 = 0x0;
const static uint32_t DEVICE_ID2 = 0x4;
const static uint32_t STATUS     = 0x0C;
const static uint32_t SPI_MASK   = 0x10;

const static uint32_t MODES_OF_OPERATION = 0x0800;
const static uint32_t INTERRUPT_FLAGS    = 0x0820;
const static uint32_t INTERRUPT_ENABLE   = 0x0830;

const static uint32_t TEST  = 0x1010; // test register
const static uint32_t CCCR  = 0x1018; // cc control register
const static uint32_t NBTP  = 0x101C; // nominal bit timing & prescaler register
const static uint32_t ECR   = 0x1040; // error counter register
const static uint32_t PSR   = 0x1044; // protocol status register
const static uint32_t IR    = 0x1050; // interrupt register
const static uint32_t IE    = 0x1054; // interrupt enable
const static uint32_t ILE   = 0x105C; // interrupt line enable
const static uint32_t RXF0C = 0x10A0; // rx FIFO 0 configuration
const static uint32_t RXF0S = 0x10A4; // rx FIFO 0 status
const static uint32_t RXF0A = 0x10A8; // rx FIFO 0 acknowledge
const static uint32_t TXBC  = 0x10C0; // tx buffer configuration
const static uint32_t TXESC = 0x10C8; // tx buffer element size configuration
const static uint32_t RXESC = 0x10BC; // rx buffer element size configuration
const static uint32_t TXQFS = 0x10C4; // tx FIFO/queue status
const static uint32_t TXBAR = 0x10D0; // tx buffer add request
const static uint32_t TXEFC = 0x10F0; // tx event fifo configuration

const static uint32_t TX_8_BYTES = 0; // tx message length
const static uint32_t RX_8_BYTES = 0; // rx message length

const static uint32_t RF0N  = (0x1UL << 0);  // rx fifo 0 new data
const static uint32_t RF0LE = (0x1UL << 3);  // rx fifo 0 message lost
const static uint32_t TC    = (0x1UL << 9);  // transmission complete
const static uint32_t TFE   = (0x1UL << 11); // transmit fifo empty
const static uint32_t TEFW  = (0x1UL << 13); // event fifo watermark
const static uint32_t EP    = (0x1UL << 23); // error passive
const static uint32_t EW    = (0x1UL << 24); // error warning
const static uint32_t BO    = (0x1UL << 25); // bus off

const static uint32_t INIT    = (0x1UL << 0); // init
const static uint32_t CCE     = (0x1UL << 1); // configuration change enable
const static uint32_t CSR     = (0x1UL << 4); // clock stop request
const static uint32_t MON     = (0x1UL << 5); // bus monitoring mode
const static uint32_t DAR     = (0x1UL << 6); // disable automatic retransmission
const static uint32_t TEST_EN = (0x1UL << 7); // test mode

const static uint32_t MODESEL_1 = (0x1UL << 6);
const static uint32_t MODESEL_2 = (0x1UL << 7);

const static uint32_t LBCK = (0x1UL << 4); // loopback mode

const static uint32_t ERROR_PASSIVE = (0x1UL << 5);
const static uint32_t ERROR_WARNING = (0x1UL << 6);
const static uint32_t BUS_OFF       = (0x1UL << 7);

const static uint32_t TCAN_EXTENDED_FLAG = (0x1UL << 30);

// Message RAM (MRAM) constants. Do not change.
const static uint32_t MRAM_BASE       = 0x8000;
const static uint32_t MRAM_SIZE_WORDS = 0x200;

// Identifiers for TCAN4550 chip. TCAN4550 in ascii.
const static uint32_t TCAN_ID  = 0x4E414354;
const static uint32_t TCAN_ID2 = 0x30353534;

// byte ordering for sending 32-bit transfers over SPI
#ifdef MSB_LSB_SWAP
#define BYTE_0 3
#define BYTE_1 2
#define BYTE_2 1
#define BYTE_3 0
#else
#define BYTE_0 0
#define BYTE_1 1
#define BYTE_2 2
#define BYTE_3 3
#endif

const static uint32_t SPI_READ_COMMAND  = 0x41;
const static uint32_t SPI_WRITE_COMMAND = 0x61;

const static struct can_bittiming_const tcan4550_bittiming_const = {
    .name      = KBUILD_MODNAME,
    .tseg1_min = 2,
    .tseg1_max = 256,
    .tseg2_min = 2,
    .tseg2_max = 128,
    .sjw_max   = 128,
    .brp_min   = 1,
    .brp_max   = 512,
    .brp_inc   = 1,
};

struct tcan_raw
{
    uint32_t data[4];
};

struct tcan4550_priv
{
    struct can_priv can; // must be located first in private struct
    struct device *dev;
    struct net_device *ndev;
    struct spi_device *spi;
    struct gpio_desc *reset_gpio;

    struct workqueue_struct *wq;
    struct work_struct tx_work;
    struct work_struct restart_work;

    struct sk_buff *tx_skb_buf[TX_BUFFER_SIZE];
    int tx_skb_buf_head;
    int tx_skb_buf_tail;

    struct tcan_raw rx_skb_buf[RX_BUFFER_SIZE];
    int rx_skb_buf_head;
    int rx_skb_buf_tail;

    uint32_t rxBuffer[MAX_SPI_BURST_RX_MESSAGES * 4];
    uint32_t txBuffer[MAX_SPI_BURST_TX_MESSAGES * 4];

    unsigned char read_txBuf[4 + (MAX_SPI_BURST_RX_MESSAGES * 16)];
    unsigned char read_rxBuf[4 + (MAX_SPI_BURST_RX_MESSAGES * 16)];

    unsigned char write_txBuf[4 + (MAX_SPI_BURST_TX_MESSAGES * 16)];
    unsigned char write_rxBuf[4 + (MAX_SPI_BURST_TX_MESSAGES * 16)];

    spinlock_t tx_skb_lock; // spinlock protecting tx skb buffer
    spinlock_t rx_skb_lock; // spinlock protecting rx skb buffer
    struct mutex spi_lock;  // mutex protecting SPI access

    struct napi_struct napi;
};

// SPI helper function headers
static int spi_transfer(struct spi_device *spi, int lenBytes, unsigned char *rxBuf, unsigned char *txBuf);
static uint32_t spi_read32(struct spi_device *spi, uint32_t address);
static int spi_write32(struct spi_device *spi, uint32_t address, uint32_t data);
static int spi_write_msgs(struct tcan4550_priv *priv, uint32_t address, int32_t msgs, uint32_t *data);
static int spi_read_msgs(struct tcan4550_priv *priv, uint32_t address, int32_t msgs, uint32_t *data);

// TCAN function headers
static void tcan4550_init(struct net_device *dev);
static void tcan4550_set_normal_mode(struct spi_device *spi);
static void tcan4550_set_standby_mode(struct spi_device *spi);
static void tcan4550_configure_mram(struct spi_device *spi);
static void tcan4550_clear_sw_buffers(struct tcan4550_priv *priv);
static void tcan4550_unlock(struct spi_device *spi);
static bool tcan4550_read_identification(struct spi_device *spi);
static void tcan4550_set_bit_rate(struct spi_device *spi, uint32_t bitRateReg);
static void tcan4550_setup_interrupts(struct spi_device *spi);
static void tcan4550_hw_reset(struct net_device *dev);
static void tcan4550_setup_io(struct net_device *dev);
static void tcan4550_skbuff_to_tcan_msg(struct sk_buff *skb, uint32_t *buffer);
static int tcan4550_set_mode(struct net_device *net, enum can_mode mode);
static void tcan4550_configure_control_modes(struct net_device *dev);
static void tcan4550_handle_bus_status_change(void *dev);
static irqreturn_t tcan4550_handle_interrupts(int irq, void *dev);
static void tcan4550_tx_work_handler(struct work_struct *ws);
static void tcan4550_send_msgs(struct tcan4550_priv *priv);
static uint32_t tcan4550_rec_msgs(struct net_device *dev);
static int tcan4550_poll(struct napi_struct *napi, int budget);

/*------------------------------------------------------------*/
/* SPI helper functions                                       */
/*------------------------------------------------------------*/
static int spi_transfer(struct spi_device *spi, int lenBytes, unsigned char *rxBuf, unsigned char *txBuf)
{
    struct net_device *ndev    = spi_get_drvdata(spi);
    struct tcan4550_priv *priv = netdev_priv(ndev);
    struct spi_transfer t      = {
             .tx_buf    = txBuf,
             .rx_buf    = rxBuf,
             .len       = lenBytes,
             .cs_change = 0,
    };

    struct spi_message m;
    int ret;

    if (spi == 0 || rxBuf == 0 || txBuf == 0)
    {
        return -EINVAL;
    }

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);

    // only one thread can access SPI at the same time
    mutex_lock(&priv->spi_lock);

    ret = spi_sync(spi, &m);
    if (ret)
    {
        dev_err(&spi->dev, "spi transfer failed: ret = %d\n", ret);
    }

    mutex_unlock(&priv->spi_lock);

    return ret;
}

static uint32_t spi_read32(struct spi_device *spi, uint32_t address)
{
    unsigned char txBuf[8];
    unsigned char rxBuf[8];

    txBuf[BYTE_0] = SPI_READ_COMMAND;
    txBuf[BYTE_1] = address >> 8;
    txBuf[BYTE_2] = address & 0xFF;
    txBuf[BYTE_3] = 1;

    spi_transfer(spi, 8, rxBuf, txBuf);

    return (rxBuf[4 + BYTE_0] << 24) + (rxBuf[4 + BYTE_1] << 16) + (rxBuf[4 + BYTE_2] << 8) + rxBuf[4 + BYTE_3];
}

static int spi_read_msgs(struct tcan4550_priv *priv, uint32_t address, int32_t msgs, uint32_t *data)
{
    uint32_t i, j;
    int ret;

    if (msgs > MAX_SPI_BURST_RX_MESSAGES)
    {
        return -EINVAL;
    }

    priv->read_txBuf[BYTE_0] = SPI_READ_COMMAND;
    priv->read_txBuf[BYTE_1] = address >> 8;
    priv->read_txBuf[BYTE_2] = address & 0xFF;
    priv->read_txBuf[BYTE_3] = msgs * 4;

    ret = spi_transfer(priv->spi, 4 + (msgs * 16), priv->read_rxBuf, priv->read_txBuf);

    for (i = 0; i < msgs; i++)
    {
        for (j = 0; j < 4; j++)
        {
            data[j + (i * 4)] = priv->read_rxBuf[4 + BYTE_3 + (j * 4) + (i * 16)] +
                                (priv->read_rxBuf[4 + BYTE_2 + (j * 4) + (i * 16)] << 8) +
                                (priv->read_rxBuf[4 + BYTE_1 + (j * 4) + (i * 16)] << 16) +
                                (priv->read_rxBuf[4 + BYTE_0 + (j * 4) + (i * 16)] << 24);
        }
    }

    return ret;
}

static int spi_write32(struct spi_device *spi, uint32_t address, uint32_t data)
{
    unsigned char txBuf[8];
    unsigned char rxBuf[8];

    int ret;

    txBuf[BYTE_0]     = SPI_WRITE_COMMAND;
    txBuf[BYTE_1]     = address >> 8;
    txBuf[BYTE_2]     = address & 0xFF;
    txBuf[BYTE_3]     = 1;
    txBuf[4 + BYTE_0] = (data >> 24) & 0xFF;
    txBuf[4 + BYTE_1] = (data >> 16) & 0xFF;
    txBuf[4 + BYTE_2] = (data >> 8) & 0xFF;
    txBuf[4 + BYTE_3] = data & 0xFF;

    ret = spi_transfer(spi, 8, rxBuf, txBuf);

    return ret;
}

static int spi_write_msgs(struct tcan4550_priv *priv, uint32_t address, int32_t msgs, uint32_t *data)
{
    uint32_t i;
    int ret;

    if (msgs > MAX_SPI_BURST_TX_MESSAGES)
    {
        return -EINVAL;
    }

    priv->write_txBuf[BYTE_0] = SPI_WRITE_COMMAND;
    priv->write_txBuf[BYTE_1] = address >> 8;
    priv->write_txBuf[BYTE_2] = address & 0xFF;
    priv->write_txBuf[BYTE_3] = msgs * 4;

    for (i = 0; i < (msgs * 4); i++)
    {
        priv->write_txBuf[4 + BYTE_0 + (i * 4)] = ((data[i] >> 24) & 0xFF);
        priv->write_txBuf[4 + BYTE_1 + (i * 4)] = ((data[i] >> 16) & 0xFF);
        priv->write_txBuf[4 + BYTE_2 + (i * 4)] = ((data[i] >> 8) & 0xFF);
        priv->write_txBuf[4 + BYTE_3 + (i * 4)] = (data[i] & 0xFF);
    }

    ret = spi_transfer(priv->spi, 4 + (msgs * 16), priv->write_rxBuf, priv->write_txBuf);

    return ret;
}

/*------------------------------------------------------------*/
/* TCAN4550 functions                                         */
/*------------------------------------------------------------*/
static void tcan4550_set_standby_mode(struct spi_device *spi)
{
    uint32_t val;

    val = spi_read32(spi, MODES_OF_OPERATION);

    val |= MODESEL_1;
    val &= ~((uint32_t)MODESEL_2);

    spi_write32(spi, MODES_OF_OPERATION, val);
}

static void tcan4550_set_normal_mode(struct spi_device *spi)
{
    uint32_t val;

    val = spi_read32(spi, MODES_OF_OPERATION);

    val |= MODESEL_2;
    val &= ~((uint32_t)MODESEL_1);

    spi_write32(spi, MODES_OF_OPERATION, val);
}

static bool tcan4550_read_identification(struct spi_device *spi)
{
    uint32_t id1, id2;

    id1 = spi_read32(spi, DEVICE_ID1);
    id2 = spi_read32(spi, DEVICE_ID2);

    // TCAN4550 in ascii
    if ((id1 == TCAN_ID) && (id2 == TCAN_ID2))
    {
        return true;
    }

    return false;
}

static void tcan4550_set_bit_rate(struct spi_device *spi, uint32_t bitRateReg)
{
    spi_write32(spi, NBTP, bitRateReg);
}

static void tcan4550_configure_mram(struct spi_device *spi)
{
    uint32_t i;

    // clear MRAM to avoid risk of ECC errors 2kB = 512 words
    for (i = 0; i < MRAM_SIZE_WORDS; i++)
    {
        spi_write32(spi, MRAM_BASE + (i * 4), 0);
    }

    // configure tx-fifo
    spi_write32(spi, TXBC, TX_FIFO_START_ADDRESS + (TX_FIFO_SIZE << 24));

    // configure rx-fifo
    spi_write32(spi, RXF0C, RX_FIFO_START_ADDRESS + (RX_FIFO_SIZE << 16));

    // size of one tx message
    spi_write32(spi, TXESC, TX_8_BYTES);

    // size of one rx message
    spi_write32(spi, RXESC, RX_8_BYTES);

    // setup tx event-fifo
    spi_write32(spi, TXEFC, EVENT_FIFO_START_ADDRESS + (EVENT_FIFO_SIZE << 16) + (EVENT_FIFO_WATERMARK << 24));
}

static void tcan4550_unlock(struct spi_device *spi)
{
    uint32_t val = spi_read32(spi, CCCR);

    val |= (CCE + INIT);     // set CCE and INIT bits
    val &= ~((uint32_t)CSR); // clear CSR

    spi_write32(spi, CCCR, val);
}

static void tcan4550_clear_sw_buffers(struct tcan4550_priv *priv)
{
    unsigned long flags;

    spin_lock_irqsave(&priv->tx_skb_lock, flags);
    while (priv->tx_skb_buf_head != priv->tx_skb_buf_tail)
    {
        dev_kfree_skb(priv->tx_skb_buf[priv->tx_skb_buf_tail]);
        priv->tx_skb_buf_tail = (priv->tx_skb_buf_tail + 1) % TX_BUFFER_SIZE;
    }
    priv->tx_skb_buf_head = 0;
    priv->tx_skb_buf_tail = 0;
    spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

    spin_lock_irqsave(&priv->rx_skb_lock, flags);
    priv->rx_skb_buf_head = 0;
    priv->rx_skb_buf_tail = 0;
    spin_unlock_irqrestore(&priv->rx_skb_lock, flags);
}

// convert a struct sk_buff (socket buffer) to a tcan4550 msg and store in buffer
void tcan4550_skbuff_to_tcan_msg(struct sk_buff *skb, uint32_t *buffer)
{
    struct can_frame *frame = (struct can_frame *)skb->data;
    bool extended           = (frame->can_id & CAN_EFF_FLAG) ? true : false;
    bool rtr                = (frame->can_id & CAN_RTR_FLAG) ? true : false;
    uint32_t len            = (frame->len <= 8) ? frame->len : 8;
    uint32_t id;

    if (extended)
    {
        id        = frame->can_id & CAN_EFF_MASK;
        buffer[0] = id + (rtr << 29) + TCAN_EXTENDED_FLAG; // add extended + rtr flag
    }
    else
    {
        id        = frame->can_id & CAN_SFF_MASK;
        buffer[0] = (id << 18) + (rtr << 29); // add rtr flag
    }

    buffer[1] = (len << 16);
    buffer[2] = frame->data[0] + (frame->data[1] << 8) + (frame->data[2] << 16) + (frame->data[3] << 24);
    buffer[3] = frame->data[4] + (frame->data[5] << 8) + (frame->data[6] << 16) + (frame->data[7] << 24);
}

// called from work queue
static void tcan4550_tx_work_handler(struct work_struct *ws)
{
    struct tcan4550_priv *priv = container_of(ws, struct tcan4550_priv, tx_work);
    uint32_t i;

    // as tx-work handler already might be executing when queue_work is called
    // from tcan_start_xmit (in that case no new item will be put on the queue) we
    // need to check once more if there is any messages to send before leaving
    // work handler
    for (i = 0; i < 2; i++)
    {
        tcan4550_send_msgs(priv);
    }
}

// copy messages from sw tx fifo to tx fifo in CAN controller and request transmission
static void tcan4550_send_msgs(struct tcan4550_priv *priv)
{
    struct net_device_stats *stats = &(priv->ndev->stats);
    uint32_t txqfs                 = spi_read32(priv->spi, TXQFS);
    uint32_t freeBuffers           = txqfs & 0x3F;
    uint32_t writeIndex            = (txqfs >> 16) & 0x1F;
    uint32_t requestMask           = 0;
    uint32_t msgs                  = 0;
    unsigned long flags;

    uint32_t startAddress = MRAM_BASE + TX_FIFO_START_ADDRESS + (writeIndex * TX_SLOT_SIZE);

    uint32_t maxMsgsToTransmit = freeBuffers;
    if (maxMsgsToTransmit > MAX_SPI_BURST_TX_MESSAGES)
    {
        maxMsgsToTransmit = MAX_SPI_BURST_TX_MESSAGES;
    }

    // Make sure TX buffer does not wrap around
    if ((writeIndex + maxMsgsToTransmit) > TX_FIFO_SIZE)
    {
        maxMsgsToTransmit = (TX_FIFO_SIZE - writeIndex);
    }

    spin_lock_irqsave(&priv->tx_skb_lock, flags);

    // build an SPI message consisting of several CAN msgs
    while ((priv->tx_skb_buf_head != priv->tx_skb_buf_tail) && (msgs < maxMsgsToTransmit))
    {
        int len;
        struct can_frame *frame = (struct can_frame *)priv->tx_skb_buf[priv->tx_skb_buf_tail]->data;

        tcan4550_skbuff_to_tcan_msg(priv->tx_skb_buf[priv->tx_skb_buf_tail], &priv->txBuffer[msgs * 4]);

        // put message on echo stack
        can_put_echo_skb(priv->tx_skb_buf[priv->tx_skb_buf_tail], priv->ndev, 0, frame->len);

        // loop back the message
        // TODO: this should preferably be done when we are sure the message is
        // actually sent in tx interrupt
        len = can_get_echo_skb(priv->ndev, 0, 0);

        // as we loop back the message, we also need to increase rx stats
        // Note: The original TCAN driver and also flexcan driver does this using
        // rx_offload, other drivers such as Kvaser does not
        stats->rx_packets++;
        stats->rx_bytes += frame->len;

        requestMask += (1 << writeIndex); // add current message to request mask

        msgs++;
        writeIndex++;

        priv->tx_skb_buf_tail = (priv->tx_skb_buf_tail + 1) % TX_BUFFER_SIZE;

        // update statistics
        stats->tx_packets++;
        stats->tx_bytes += frame->len;
    }

    spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

    if (msgs > 0)
    {
        if (spi_write_msgs(priv, startAddress, msgs, priv->txBuffer) == 0)
        {
            spi_write32(priv->spi, TXBAR, requestMask); // request buffer transmission
        }
        else
        {
            dev_err(priv->dev, "spi_write_msgs failed\n");
        }
    }
}

// this function is called from NAPI (soft-irq context) and is not allowed to
// sleep or call functions that might sleep like SPI access
static int tcan4550_poll(struct napi_struct *_napi, int budget)
{
    struct tcan4550_priv *priv     = container_of(_napi, struct tcan4550_priv, napi);
    struct net_device_stats *stats = &(priv->ndev->stats);
    uint32_t msgs                  = 0;
    unsigned long flags;

    if (budget == 0)
    {
        return 0;
    }

    spin_lock_irqsave(&priv->rx_skb_lock, flags);

    while ((priv->rx_skb_buf_head != priv->rx_skb_buf_tail) && (msgs < budget))
    {
        struct can_frame *cf;
        struct sk_buff *skb = alloc_can_skb(priv->ndev, &cf);

        if (skb)
        {
            uint32_t *data = (uint32_t *)&priv->rx_skb_buf[priv->rx_skb_buf_tail].data;
            cf->len        = (data[1] >> 16) & 0x0F;

            if (cf->len > 8)
            {
                cf->len = 8;
                dev_info(priv->dev, "invalid frame length\n");
            }

            if (data[0] & TCAN_EXTENDED_FLAG) // extended
            {
                cf->can_id = (data[0] & CAN_EFF_MASK) | CAN_EFF_FLAG;
            }
            else
            {
                cf->can_id = (data[0] >> 18) & CAN_SFF_MASK;
            }

            cf->data[0] = data[2] & 0xFF;
            cf->data[1] = (data[2] >> 8) & 0xFF;
            cf->data[2] = (data[2] >> 16) & 0xFF;
            cf->data[3] = (data[2] >> 24) & 0xFF;
            cf->data[4] = data[3] & 0xFF;
            cf->data[5] = (data[3] >> 8) & 0xFF;
            cf->data[6] = (data[3] >> 16) & 0xFF;
            cf->data[7] = (data[3] >> 24) & 0xFF;

            // send message to Linux networking stack
            netif_receive_skb(skb);

            priv->rx_skb_buf_tail = (priv->rx_skb_buf_tail + 1) % RX_BUFFER_SIZE;
            msgs++;

            stats->rx_packets++;
            stats->rx_bytes += cf->len;
        }
        else
        {
            // we could try to allocate the skb again a little later but that might
            // also fail so we drop the packet
            priv->rx_skb_buf_tail = (priv->rx_skb_buf_tail + 1) % RX_BUFFER_SIZE;
            msgs++;

            stats->rx_dropped++;
            dev_info(priv->dev, "could not allocate frame\n");
        }
    }

    // If all messages did fit within budget, tell NAPI we are ready. If
    // msgs=budget, we shall NOT call napi_complete_done
    if (msgs < budget)
    {
        napi_complete_done(&priv->napi, msgs);
    }

    spin_unlock_irqrestore(&priv->rx_skb_lock, flags);

    return msgs;
}

uint32_t tcan4550_rec_msgs(struct net_device *dev)
{
    struct net_device_stats *stats = &(dev->stats);
    struct tcan4550_priv *priv     = netdev_priv(dev);
    uint32_t rxf0s                 = spi_read32(priv->spi, RXF0S);
    uint32_t fillLevel             = ((rxf0s & 0x7F) < 64) ? (rxf0s & 0x7F) : 64; // 0-64
    uint32_t getIndex              = ((rxf0s >> 8) & 0x3F);                       // 0-63
    uint32_t totalMsgsToGet;
    uint32_t msgsToGet[2];
    uint32_t startAddress[2];
    uint32_t spiPackages = 1;
    uint32_t i, spiPackage;
    uint32_t msgsReceived = 0;

    if (fillLevel == 0)
    {
        return 0;
    }

    totalMsgsToGet = fillLevel;

    startAddress[0] = MRAM_BASE + RX_FIFO_START_ADDRESS + (getIndex * RX_SLOT_SIZE);
    startAddress[1] = MRAM_BASE + RX_FIFO_START_ADDRESS;

    if (totalMsgsToGet > MAX_SPI_BURST_RX_MESSAGES)
    {
        totalMsgsToGet = MAX_SPI_BURST_RX_MESSAGES;
    }

    msgsToGet[0] = totalMsgsToGet;

    // if hw rx buffer wraps around, we need to make two SPI requests to get all data
    if (totalMsgsToGet > (RX_FIFO_SIZE - getIndex))
    {
        msgsToGet[0] = (RX_FIFO_SIZE - getIndex);     // request as much as possible in SPI package 1
        msgsToGet[1] = totalMsgsToGet - msgsToGet[0]; // request the rest in SPI package 2

        if (msgsToGet[1] > 0)
        {
            spiPackages = 2;
        }
    }

    for (spiPackage = 0; spiPackage < spiPackages; spiPackage++)
    {
        if (spi_read_msgs(priv, startAddress[spiPackage], msgsToGet[spiPackage], priv->rxBuffer) == 0)
        {
            uint32_t ack;
            ack = (spiPackage == 0) ? (getIndex + msgsToGet[0] - 1) : (msgsToGet[1] - 1);

            // acknowledge the last message we have read, that will automatically free all messages up until that
            // message
            spi_write32(priv->spi, RXF0A, ack);

            for (i = 0; i < msgsToGet[spiPackage]; i++)
            {
                unsigned long flags;
                uint32_t tmpHead;
                uint32_t *data = (uint32_t *)&priv->rxBuffer[0 + (i * 4)];

                // store skb in rx buffer
                spin_lock_irqsave(&priv->rx_skb_lock, flags);

                tmpHead = (priv->rx_skb_buf_head + 1) % RX_BUFFER_SIZE;
                if (tmpHead != priv->rx_skb_buf_tail)
                {
                    priv->rx_skb_buf[priv->rx_skb_buf_head].data[0] = data[0];
                    priv->rx_skb_buf[priv->rx_skb_buf_head].data[1] = data[1];
                    priv->rx_skb_buf[priv->rx_skb_buf_head].data[2] = data[2];
                    priv->rx_skb_buf[priv->rx_skb_buf_head].data[3] = data[3];

                    priv->rx_skb_buf_head = tmpHead;

                    msgsReceived++;
                }
                else
                {
                    stats->rx_dropped++;
                }

                spin_unlock_irqrestore(&priv->rx_skb_lock, flags);
            }
        }
        else
        {
            dev_err(priv->dev, "spi_read_msgs failed\n");
        }
    }

    return msgsReceived;
}

// go through errors in priority order (most severe error first)
static void tcan4550_handle_bus_status_change(void *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    uint32_t psr               = spi_read32(priv->spi, PSR);
    uint32_t err               = spi_read32(priv->spi, ECR);
    uint32_t tx_err            = err & 0xFF;
    uint32_t rx_err            = (err >> 8) & 0x7F;
    struct sk_buff *skb;
    struct can_frame *cf;

    // bus off
    if (psr & BUS_OFF)
    {
        priv->can.state = CAN_STATE_BUS_OFF;
        spi_write32(priv->spi, ILE, 0); // disable all interrupts to avoid flooding
        priv->can.can_stats.bus_off++;
        can_bus_off(dev); // tell Linux networking stack that we are bus off

        skb = alloc_can_err_skb((struct net_device *)dev, &cf);
        if (skb)
        {
            cf->can_id |= CAN_ERR_BUSOFF;

            netif_rx(skb);
        }
    }

    // error passive
    if (psr & ERROR_PASSIVE)
    {
        if (priv->can.state != CAN_STATE_ERROR_PASSIVE)
        {
            priv->can.can_stats.error_passive++;
            priv->can.state = CAN_STATE_ERROR_PASSIVE;

            skb = alloc_can_err_skb((struct net_device *)dev, &cf);
            if (skb)
            {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
                cf->can_id |= CAN_ERR_CRTL;
#else
                cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
#endif
                if (err & (1 << 15))
                    cf->data[1] |= CAN_ERR_CRTL_RX_PASSIVE;
                if (tx_err > 127)
                    cf->data[1] |= CAN_ERR_CRTL_TX_PASSIVE;
                cf->data[6] = tx_err;
                cf->data[7] = rx_err;

                netif_rx(skb);
            }
        }

        return;
    }

    // error warning
    if (psr & ERROR_WARNING)
    {
        if (priv->can.state != CAN_STATE_ERROR_WARNING)
        {
            priv->can.can_stats.error_warning++;
            priv->can.state = CAN_STATE_ERROR_WARNING;

            skb = alloc_can_err_skb((struct net_device *)dev, &cf);
            if (skb)
            {
#if LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0)
                cf->can_id |= CAN_ERR_CRTL;
#else
                cf->can_id |= CAN_ERR_CRTL | CAN_ERR_CNT;
#endif
                cf->data[1] = (tx_err > rx_err) ? CAN_ERR_CRTL_TX_WARNING : CAN_ERR_CRTL_RX_WARNING;
                cf->data[6] = tx_err;
                cf->data[7] = rx_err;

                netif_rx(skb);
            }
        }

        return;
    }

    // no errors found, set error active (bus is active - no errors)
    priv->can.state = CAN_STATE_ERROR_ACTIVE;
}

// interrupt handler - run as an irq thread
static irqreturn_t tcan4550_handle_interrupts(int irq, void *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    uint32_t ir;

    // NOTE: This call might be blocked for a pretty long time due to long SPI
    // burst transfers
    ir = spi_read32(priv->spi, IR);
    spi_write32(priv->spi, IR, ir); // acknowledge interrupts

    if (ir == 0)
    {
        return IRQ_NONE;
    }

    // rx fifo 0 new message
    if (ir & RF0N)
    {
        tcan4550_rec_msgs(dev);

        // disable bottom halves when calling napi_schedule to
        // avoid error message "NOHZ tick-stop error: Non-RCU
        // local softirq work is pending, handler #08!!!"
        local_bh_disable();
        napi_schedule(&priv->napi);
        local_bh_enable();
    }

    // rx fifo 0 message lost
    if (ir & RF0LE)
    {
        priv->ndev->stats.rx_errors++;
        priv->ndev->stats.rx_over_errors++;
    }

    // tx fifo empty
    if (ir & TFE)
    {
        // note that queue can only contain one item of the tx_work type so if tx_work is already on queue, no new item
        // will be added
        queue_work(priv->wq, &priv->tx_work);

        netif_wake_queue(dev);
    }

    // handle bus errors (error warning, error passive or bus off)
    if ((ir & EW) || (ir & EP) || (ir & BO))
    {
        tcan4550_handle_bus_status_change(dev);
    }

    return IRQ_HANDLED;
}

void tcan4550_setup_interrupts(struct spi_device *spi)
{
    // rx fifo 0 new message + tx fifo empty + bus off + error warning + error passive + rx fifo 0 msg lost
    spi_write32(spi, IE, RF0N + TFE + BO + EW + EP + RF0LE);
    spi_write32(spi, ILE, 0x1); // enable interrupt line 1

    // mask all spi errors
    spi_write32(spi, SPI_MASK, 0xFFFFFFFF);

    // clear spi status register
    spi_write32(spi, STATUS, 0xFFFFFFFF);

    // clear interrupts
    spi_write32(spi, INTERRUPT_FLAGS, 0xFFFFFFFF);

    // interrupt enables
    spi_write32(spi, INTERRUPT_ENABLE, 0);
}

void tcan4550_setup_io(struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);

    // get reset gpio from devicetree reset-gpio property, set to output low
    priv->reset_gpio = devm_gpiod_get_optional(priv->dev, "reset", GPIOD_OUT_LOW);

    if (IS_ERR(priv->reset_gpio))
    {
        dev_err(priv->dev, "could not get reset GPIO\n");

        priv->reset_gpio = NULL;
    }
}

void tcan4550_hw_reset(struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);

    gpiod_set_value(priv->reset_gpio, 1);

    // according to spec we need to toggle pin for at least 30us
    usleep_range(30, 100);
    gpiod_set_value(priv->reset_gpio, 0);

    // according to spec we need to wait at least 700us for chip to become ready
    usleep_range(700, 1000);
}

static void tcan4550_configure_control_modes(struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    uint32_t cccr              = spi_read32(priv->spi, CCCR);
    uint32_t test              = spi_read32(priv->spi, TEST);

    if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
    {
        cccr |= TEST_EN | MON;
        test |= LBCK;
    }

    if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
    {
        cccr |= MON;
    }

    if (priv->can.ctrlmode & CAN_CTRLMODE_ONE_SHOT)
    {
        cccr |= DAR;
    }

    cccr &= ~CSR; // clock stop should never be written 1 to even if reading returns 1

    spi_write32(priv->spi, CCCR, cccr);
    spi_write32(priv->spi, TEST, test);
}

// initialize TCAN4550 hardware
static void tcan4550_init(struct net_device *dev)
{
    struct tcan4550_priv *priv     = netdev_priv(dev);
    const struct can_bittiming *bt = &priv->can.bittiming;
    uint32_t bitRateReg = (bt->phase_seg2 - 1) + ((bt->prop_seg + bt->phase_seg1 - 1) << 8) + ((bt->brp - 1) << 16) +
                          ((bt->sjw - 1) << 25);

    tcan4550_set_standby_mode(priv->spi);
    tcan4550_unlock(priv->spi);
    tcan4550_set_bit_rate(priv->spi, bitRateReg);
    tcan4550_configure_mram(priv->spi);
    tcan4550_configure_control_modes(dev);
    tcan4550_setup_interrupts(priv->spi);

    // after this call, the TCAN chip is ready to send/receive messages
    tcan4550_set_normal_mode(priv->spi);
}

/*------------------------------------------------------------*/
/* Linux CAN Driver standard functions                        */
/*------------------------------------------------------------*/

// called if user performs ifconfig CANx up
static int tcan_open(struct net_device *ndev)
{
    struct tcan4550_priv *priv = netdev_priv(ndev);
    int err;

    // open the can device
    err = open_candev(ndev);
    if (err)
    {
        netdev_err(ndev, "failed to open can device\n");
        return err;
    }

    tcan4550_init(ndev);
    tcan4550_clear_sw_buffers(priv);
    priv->can.state = CAN_STATE_ERROR_ACTIVE;

    // start interrupt handler, as SPI is slow, run as threaded irq in one-shot
    // mode (hw interrupt is disabled when running irq thread function)
    err = request_threaded_irq(priv->spi->irq, NULL, tcan4550_handle_interrupts, IRQF_ONESHOT, ndev->name, ndev);
    if (err)
    {
        netdev_err(ndev, "failed to register interrupt\n");

        close_candev(ndev);

        return err;
    }

    dev_info(priv->dev, "hw rx buffers %d\n", RX_FIFO_SIZE);
    dev_info(priv->dev, "hw tx buffers %d\n", TX_FIFO_SIZE);
    dev_info(priv->dev, "max rx SPI burst %d\n", MAX_SPI_BURST_RX_MESSAGES);
    dev_info(priv->dev, "max tx SPI burst %d\n", MAX_SPI_BURST_TX_MESSAGES);

    napi_enable(&priv->napi);

    netif_start_queue(ndev); // This will make Linux network stack start send us packages

    return 0;
}

// called if user performs ifconfig CANx down
static int tcan_close(struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);

    netif_stop_queue(dev);
    napi_disable(&priv->napi);

    free_irq(priv->spi->irq, dev);
    tcan4550_set_standby_mode(priv->spi);
    close_candev(dev);

    priv->can.state = CAN_STATE_STOPPED;

    return 0;
}

// Called from Linux network stack to request sending of a CAN message
// Linux call start_xmit from soft-irq context so we are not allowed to sleep
// here. We do not actually send anything here, just copy frame to our sw tx
// buffer. If sw tx buffer is full, stop queue with netif_stop_queue.
static netdev_tx_t tcan_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    struct tcan4550_priv *priv = netdev_priv(dev);
    uint32_t tmpHead;
    unsigned long flags;

    // drop invalid CAN msgs
    if (can_dropped_invalid_skb(dev, skb))
    {
        return NETDEV_TX_OK;
    }

    spin_lock_irqsave(&priv->tx_skb_lock, flags);

    tmpHead = priv->tx_skb_buf_head;
    tmpHead = (tmpHead + 1) % TX_BUFFER_SIZE;

    // Stop network queue and return busy if we cannot buffer anyhing more. We try
    // to stop queue already when last empty slot is used but we can still
    // get here if TFE interrupt wakes queue up when our queue is still full
    if (tmpHead == priv->tx_skb_buf_tail)
    {
        // queue will be started again from TFE interrupt
        netif_stop_queue(dev);
        spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

        return NETDEV_TX_BUSY;
    }

    priv->tx_skb_buf[priv->tx_skb_buf_head] = skb;
    priv->tx_skb_buf_head                   = tmpHead;

    // check if queue can hold one more item, if not - stop queue
    tmpHead = (tmpHead + 1) % TX_BUFFER_SIZE;
    if (tmpHead == priv->tx_skb_buf_tail)
    {
        // queue will be started again from TFE interrupt
        netif_stop_queue(dev);
    }

    spin_unlock_irqrestore(&priv->tx_skb_lock, flags);

    queue_work(priv->wq, &priv->tx_work);

    return NETDEV_TX_OK;
}

// restart controller after bus off
static void tcan4550_restart_work_handler(struct work_struct *ws)
{
    struct tcan4550_priv *priv = container_of(ws, struct tcan4550_priv, restart_work);

    tcan4550_clear_sw_buffers(priv);
    priv->can.state = CAN_STATE_ERROR_ACTIVE;

    // NOTE! when this call returns we will get interrupts again so be very
    // careful what is done after this call
    tcan4550_init(priv->ndev);
    netif_wake_queue(priv->ndev);
}

// Called automatically from Linux can device if bus off is detected and
// restart-ms is set or manually by calling 'ip link set CANx type can restart'
static int tcan4550_set_mode(struct net_device *dev, enum can_mode mode)
{
    struct tcan4550_priv *priv = netdev_priv(dev);

    switch (mode)
    {
    case CAN_MODE_START:
        queue_work(priv->wq, &priv->restart_work);
        break;

    default:
        return -EOPNOTSUPP;
    }

    return 0;
}

static const struct net_device_ops m_can_netdev_ops = {
    .ndo_open       = tcan_open,
    .ndo_stop       = tcan_close,
    .ndo_start_xmit = tcan_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

// called by Linux if it matches our driver to a entry in device tree or if we
// manually call dtoverlay
static int tcan_probe(struct spi_device *spi)
{
    struct net_device *ndev;
    int err;
    struct tcan4550_priv *priv;
    struct spi_delay delay = {.unit = SPI_DELAY_UNIT_USECS, .value = 0};

    ndev = alloc_candev(sizeof(struct tcan4550_priv), ECHO_BUFFERS);
    if (!ndev)
    {
        dev_err(&spi->dev, "could not allocate candev\n");
        return -ENOMEM;
    }

    priv = netdev_priv(ndev);
    spi_set_drvdata(spi, ndev);
    SET_NETDEV_DEV(ndev, &spi->dev);

    priv->dev                 = &spi->dev;
    priv->ndev                = ndev;
    priv->spi                 = spi;
    priv->can.bittiming_const = &tcan4550_bittiming_const;
    priv->can.clock.freq      = 40000000;
    priv->can.do_set_mode     = tcan4550_set_mode;

    priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_ONE_SHOT;

    ndev->netdev_ops = &m_can_netdev_ops;

    // Tell Linux we support local echo
    ndev->flags |= IFF_ECHO;

#ifdef USE_32BIT_SPI_TRANSFERS
    spi->bits_per_word = 32;
#else
    spi->bits_per_word = 8;
#endif
    spi->max_speed_hz = 18000000;
    spi->cs_setup     = delay;
    spi->cs_hold      = delay;
    spi->cs_inactive  = delay;
    spi->word_delay   = delay;

    spin_lock_init(&priv->tx_skb_lock);
    spin_lock_init(&priv->rx_skb_lock);
    mutex_init(&priv->spi_lock);

    err = spi_setup(spi);
    if (err)
    {
        dev_err(&spi->dev, "could not setup SPI\n");
        goto exit_free;
    }

    err = register_candev(ndev);
    if (err)
    {
        dev_err(&spi->dev, "registering candev failed\n");
        goto exit_free;
    }

    tcan4550_setup_io(ndev);
    usleep_range(1000, 2000);
    tcan4550_hw_reset(ndev);

    // TODO: this dummy read is needed to get SPI working. Can we remove this?
    tcan4550_read_identification(spi);

    // read chip identification to verify correct chip is there
    if (tcan4550_read_identification(spi))
    {
        dev_info(&spi->dev, "TCAN4550 identification read ok\n");
    }
    else
    {
        dev_err(&spi->dev, "failed to read TCAN4550 identification\n");
        err = -ENODEV;
        goto exit_unregister;
    }

    priv->wq = alloc_workqueue("tcan4550_wq", WQ_FREEZABLE | WQ_MEM_RECLAIM, 1);
    if (!priv->wq)
    {
        dev_err(&spi->dev, "could not allocate workqueue\n");
        err = -ENOMEM;
        goto exit_unregister;
    }
    INIT_WORK(&priv->tx_work, tcan4550_tx_work_handler);
    INIT_WORK(&priv->restart_work, tcan4550_restart_work_handler);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 19, 0)
    netif_napi_add_weight(priv->ndev, &(priv->napi), tcan4550_poll, NAPI_BUDGET);
#else
    netif_napi_add(priv->ndev, &(priv->napi), tcan4550_poll, NAPI_BUDGET);
#endif

    dev_info(&spi->dev, "device registered\n");

    return 0;

exit_unregister:
    unregister_candev(ndev);
exit_free:
    free_candev(ndev);

    return err;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 18, 0)
int tcan_remove(struct spi_device *spi)
#else
void tcan_remove(struct spi_device *spi)
#endif
{
    struct net_device *ndev    = spi_get_drvdata(spi);
    struct tcan4550_priv *priv = netdev_priv(ndev);

    unregister_candev(ndev);
    free_candev(ndev);
    destroy_workqueue(priv->wq);
    netif_napi_del(&priv->napi);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 18, 0)
    return 0;
#endif
}

// Note! During suspend Linux will freeze our work-queues and disable interrupts
static __maybe_unused int tcan4550_suspend(struct device *dev)
{
    struct spi_device *spi     = to_spi_device(dev);
    struct net_device *ndev    = spi_get_drvdata(spi);
    struct tcan4550_priv *priv = netdev_priv(ndev);

    if (netif_running(ndev))
    {
        netif_stop_queue(ndev);
        netif_device_detach(ndev);

        spi_write32(priv->spi, ILE, 0);
        tcan4550_set_standby_mode(spi);
        tcan4550_clear_sw_buffers(priv);
    }
    priv->can.state = CAN_STATE_SLEEPING;

    return 0;
}

static __maybe_unused int tcan4550_resume(struct device *dev)
{
    struct spi_device *spi     = to_spi_device(dev);
    struct net_device *ndev    = spi_get_drvdata(spi);
    struct tcan4550_priv *priv = netdev_priv(ndev);

    priv->can.state = CAN_STATE_ERROR_ACTIVE;

    if (netif_running(ndev))
    {
        tcan4550_hw_reset(ndev);
        tcan4550_init(ndev);

        netif_device_attach(ndev);
        netif_start_queue(ndev);
    }

    return 0;
}

static int __maybe_unused tcan4550_runtime_nop(struct device *dev)
{
    return 0;
}

static const struct dev_pm_ops tcan4550_dev_pm_ops = {SET_SYSTEM_SLEEP_PM_OPS(
    tcan4550_suspend, tcan4550_resume) SET_RUNTIME_PM_OPS(tcan4550_runtime_nop, tcan4550_runtime_nop, NULL)};

static const struct of_device_id tcan4550_of_match[] = {{
                                                            .compatible = "ti,tcan4x5x",
                                                        },
                                                        {}};
MODULE_DEVICE_TABLE(of, tcan4550_of_match);

static const struct spi_device_id tcan4550_id_table[] = {{
                                                             .name = "tcan4x5x",
                                                         },
                                                         {}};
MODULE_DEVICE_TABLE(spi, tcan4550_id_table);

static struct spi_driver tcan4550_can_driver = {
    .driver =
        {
            .name           = "tcan4x5x",
            .of_match_table = tcan4550_of_match,
            .pm             = &tcan4550_dev_pm_ops,
        },
    .id_table = tcan4550_id_table,
    .probe    = tcan_probe,
    .remove   = tcan_remove,
};
module_spi_driver(tcan4550_can_driver);

MODULE_AUTHOR("Carl-Magnus Moon <info@crosscontrol.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN bus driver for TI TCAN4550 controller");

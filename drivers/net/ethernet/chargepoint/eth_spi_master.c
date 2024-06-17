#include "linux/gfp_types.h"
#include <linux/delay.h>
#include <linux/skbuff.h>
#include <linux/if_ether.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>
#include <linux/semaphore.h>
#include <linux/jiffies.h>

#include "eth_spi_protocol.h"


#define ETH_SPI_DRV_NAME    "chpt-eth-spi"
#define TX_RING_MAX_LEN     10
#define TX_RING_MIN_LEN     2

#define ETH_SPI_CLK_SPEED_MIN   (1000000u)
#define ETH_SPI_CLK_SPEED_MAX   (25000000u)
#define ETH_SPI_CLK_SPEED       (1000000u)
static int eth_spi_clkspeed;
module_param(eth_spi_clkspeed, int, 0);
MODULE_PARM_DESC(eth_spi_clkspeed, "SPI bus clock speed (Hz) (1000000 - 25000000).");

struct eth_spi_stats {
    u64 reset_timeout;
    u64 spi_err;
    u64 ring_full;
    u64 write_err;
};

struct tx_ring {
    struct sk_buff *skb[TX_RING_MAX_LEN];
    u16 head;
    u16 tail;
    u16 size;
    u16 count;
};

struct chpt_eth_spi {
    struct net_device* net_dev;
    struct spi_device* spi_dev;
    struct eth_spi_stats stats;
    struct task_struct *spi_thread;

    struct gpio_desc* slave_select;
    struct gpio_desc* slave_data;
    struct gpio_desc* slave_sync;

    struct semaphore slave_sync_sem;

    unsigned int intr_req;
    unsigned int intr_svc;

    struct tx_ring txr;

    struct eth_spi_frame *tx_frame;
    struct eth_spi_frame *rx_frame;

    u8* rx_buffer;
    struct sk_buff* rx_skb;
};

static int chpt_eth_spi_transceive_frame(struct chpt_eth_spi* espi, struct sk_buff* skb)
{
    struct spi_message msg;
    struct spi_transfer transfer[1];
    int ret;

    memset(&transfer, 0, sizeof(transfer));

    // Ensure semaphore is reset!
    //up(&espi->slave_sync_sem);
    sema_init(&espi->slave_sync_sem, 0);
    gpiod_set_value(espi->slave_select, 1);
    dev_info(&espi->net_dev->dev, "slave select: 1");
    if(down_timeout(&espi->slave_sync_sem, msecs_to_jiffies(100))) {
        dev_warn(&espi->net_dev->dev, "Slave sync timeout");
    } else {
        dev_info(&espi->net_dev->dev, "slave ack'd sync signal");
    }

    gpiod_set_value(espi->slave_select, 0);

    return 0;

    spi_message_init(&msg);

    // semaphore ack 

    memcpy(espi->tx_frame->buf, skb->data, skb->len);

    espi->tx_frame->sof = 0x55aa;

    transfer[0].tx_buf = espi->tx_frame;
    transfer[0].rx_buf = espi->rx_frame;
    transfer[0].len = sizeof(*espi->tx_frame);
    netdev_info(espi->net_dev, "tx_frame: %d", skb->len);

    spi_message_add_tail(&transfer[0], &msg);
    ret = spi_sync(espi->spi_dev, &msg);


    gpiod_set_value(espi->slave_select, 0);

    if(ret) {
        return -1;
    }

    return 0;
}

static int chpt_eth_spi_transceive(struct chpt_eth_spi* espi) 
{
    u16 packets = 0;

    struct net_device_stats *n_stats = &espi->net_dev->stats;
    if(espi->txr.skb[espi->txr.head] == NULL)
        return 0;

    while(espi->txr.skb[espi->txr.head]) {
        if(chpt_eth_spi_transceive_frame(espi, espi->txr.skb[espi->txr.head]) == -1) {
            espi->stats.write_err++;
            return -1;
        }

        packets++;
        n_stats->tx_packets++;
        n_stats->tx_bytes += espi->txr.skb[espi->txr.head]->len;
        
        netif_tx_lock_bh(espi->net_dev);
        dev_kfree_skb(espi->txr.skb[espi->txr.head]);
        espi->txr.skb[espi->txr.head] = NULL;

        u16 new_head = espi->txr.head + 1;
        if(new_head >= espi->txr.count)
            new_head = 0;

        espi->txr.head = new_head;

        if(netif_queue_stopped(espi->net_dev))
            netif_wake_queue(espi->net_dev);

        netif_tx_unlock_bh(espi->net_dev);
    }

    return 0;
}

static int chpt_eth_spi_netdev_init(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);

    dev_info(&espi->spi_dev->dev, "netdev_init");
    
    dev->mtu = ETH_SPI_MTU;
    dev->type = ARPHRD_ETHER;

    memset(&espi->stats, 0, sizeof(struct eth_spi_stats));

    espi->tx_frame = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);
    espi->rx_frame = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);

    espi->rx_buffer = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);
    if(!espi->rx_buffer)
        return -ENOBUFS;

    espi->rx_skb = netdev_alloc_skb_ip_align(dev, ETH_SPI_MTU + VLAN_ETH_HLEN);
    if(!espi->rx_skb) {
        kfree(espi->rx_buffer);
        netdev_info(espi->net_dev, "Failed to allocation RX sk_buff.\n");
        return -ENOBUFS;
    }

    return 0;
}

static void chpt_eth_spi_netdev_uninit(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_uninit");

    kfree(espi->tx_frame);
    kfree(espi->rx_frame);

    kfree(espi->rx_buffer);
    dev_kfree_skb(espi->rx_skb);
}

static int chpt_eth_spi_thread(void* data)
{
    struct chpt_eth_spi* espi = data;
    netdev_info(espi->net_dev, "SPI thread created\n");

    while(!kthread_should_stop()) {
        set_current_state(TASK_INTERRUPTIBLE);
        if((espi->intr_req != espi->intr_svc) && !espi->txr.skb[espi->txr.head])
            schedule();
        
        

        set_current_state(TASK_RUNNING);

        netdev_info(espi->net_dev, "have work to do. int: , tx_skb: %p\n", espi->txr.skb[espi->txr.head]);

        chpt_eth_spi_transceive(espi);
        msleep(500);
    }

    set_current_state(TASK_RUNNING);
    netdev_info(espi->net_dev, "SPI thread exit\n");

    return 0;
}

static irqreturn_t eth_spi_intr_handler(int irq, void* data)
{
    struct chpt_eth_spi* espi = data;
    
    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);

    return IRQ_HANDLED;
}

static int chpt_eth_spi_netdev_open(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_open");

    int ret = 0;

    espi->intr_req = 1;
    espi->intr_svc = 0;

    espi->spi_thread = kthread_run(chpt_eth_spi_thread, espi, "%s", dev->name);
    if(IS_ERR(espi->spi_thread)) {
        netdev_err(dev, "%s: unable to start kernel thread.\n", ETH_SPI_DRV_NAME); 
        return PTR_ERR(espi->spi_thread);
    }

    ret = request_irq(espi->spi_dev->irq, eth_spi_intr_handler, 0, dev->name, espi);
    if(ret) {
        netdev_err(dev, "%s: unable to get IRQ %d (irqval=%d).\n", ETH_SPI_DRV_NAME, espi->spi_dev->irq, ret);
        kthread_stop(espi->spi_thread);
        return ret;
    }

    netif_carrier_on(dev);

    return 0;
}

static int chpt_eth_spi_netdev_stop(struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_stop");

    netif_stop_queue(dev);

    free_irq(espi->spi_dev->irq, espi);

    kthread_stop(espi->spi_thread);
    espi->spi_thread = NULL;

    // flush rx_ring.
    return 0;
}

static int chpt_eth_spi_ring_has_space(struct tx_ring* txr)
{
    if(txr->skb[txr->tail])
        return 0;

    return 1;
}

static netdev_tx_t chpt_eth_spi_netdev_xmit(struct sk_buff* skb, struct net_device* dev)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_xmit");


    if(espi->txr.skb[espi->txr.tail]) {
        netdev_warn(espi->net_dev, "queue was unexpectedly full!\n");
        netif_stop_queue(espi->net_dev);
        espi->stats.ring_full++;
        return NETDEV_TX_BUSY;
    }

    netdev_dbg(espi->net_dev, "Tx-ing packet: Size: 0x%08x\n", skb->len);

    u16 new_tail = espi->txr.tail + 1;
    if(new_tail >= espi->txr.count)
        new_tail = 0;

    espi->txr.skb[espi->txr.tail] = skb;
    espi->txr.tail = new_tail;

    if(!chpt_eth_spi_ring_has_space(&espi->txr)) {
        netdev_info(espi->net_dev, "TXring is full!");
        netif_stop_queue(espi->net_dev);
        espi->stats.ring_full++;
    }

    netif_trans_update(dev);

    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);


    return NETDEV_TX_OK;
}

static void chpt_eth_spi_netdev_tx_timeout(struct net_device* dev, unsigned int txqueue)
{
    struct chpt_eth_spi* espi = netdev_priv(dev);
    dev_info(&espi->spi_dev->dev, "netdev_tx_timeout");

    if(espi->spi_thread)
        wake_up_process(espi->spi_thread);
}

static const struct net_device_ops chpt_eth_spi_netdev_ops = {
    .ndo_init = chpt_eth_spi_netdev_init,
    .ndo_uninit = chpt_eth_spi_netdev_uninit,
    .ndo_open = chpt_eth_spi_netdev_open,
    .ndo_stop = chpt_eth_spi_netdev_stop,
    .ndo_start_xmit = chpt_eth_spi_netdev_xmit,
    .ndo_set_mac_address = eth_mac_addr,
    .ndo_tx_timeout = chpt_eth_spi_netdev_tx_timeout,
    .ndo_validate_addr = eth_validate_addr,
};

static const struct ethtool_ops chpt_eth_spi_ethtool_ops = {
};

static void chpt_eth_spi_netdev_setup(struct net_device *dev) {

    struct chpt_eth_spi* espi = NULL;

    dev->netdev_ops = &chpt_eth_spi_netdev_ops;
    dev->ethtool_ops = &chpt_eth_spi_ethtool_ops;
    //
    dev->watchdog_timeo = 250;
    dev->priv_flags &= ~IFF_TX_SKB_SHARING;
    dev->tx_queue_len = 100;

    dev->min_mtu = ETH_SPI_MTU;
    dev->max_mtu = ETH_SPI_MTU;

    espi = netdev_priv(dev);
    memset(espi, 0, sizeof(*espi));
    memset(&espi->txr, 0, sizeof(espi->txr));
    espi->txr.count = TX_RING_MAX_LEN;
};

static const struct of_device_id chpt_eth_spi_of_match[] = {
    { .compatible = "chpt,eth-spi" },
    { /* sentinel */ }
};

static irqreturn_t espi_slave_sync_handler(int irq, void* data)
{
    struct chpt_eth_spi* espi = data;

    down(&espi->slave_sync_sem);

    return IRQ_HANDLED;
}

static int chpt_eth_spi_probe(struct spi_device* spi)
{
    struct chpt_eth_spi *espi = NULL;
    struct net_device* espi_netdev = NULL;

    dev_info(&spi->dev, "Probing ChargePoint Eth SPI device...");

    if(!spi->dev.of_node) {
        dev_err(&spi->dev, "Missing device tree\n");
        return -EINVAL;
    }
    
    if(eth_spi_clkspeed == 0) {
        if(spi->max_speed_hz)
            eth_spi_clkspeed = spi->max_speed_hz;
        else
            eth_spi_clkspeed = ETH_SPI_CLK_SPEED;
    }

    if((eth_spi_clkspeed < ETH_SPI_CLK_SPEED_MIN) || 
    (eth_spi_clkspeed > ETH_SPI_CLK_SPEED_MAX)) {
        dev_err(&spi->dev, "Invalid clkspeed: %d\n", eth_spi_clkspeed);
        return -EINVAL;
    }

    dev_info(&spi->dev, "clkspeed=%d\n", eth_spi_clkspeed);

    spi->mode = SPI_MODE_3;
    spi->max_speed_hz = eth_spi_clkspeed;
    if(spi_setup(spi) < 0) {
        dev_err(&spi->dev, "Unable to setup SPI device\n");
        return -EFAULT;
    }

    espi_netdev = alloc_etherdev(sizeof(struct chpt_eth_spi));
    if(!espi_netdev)
        return -ENOMEM;

    chpt_eth_spi_netdev_setup(espi_netdev);
    espi = netdev_priv(espi_netdev);
    
    espi->slave_data = devm_gpiod_get(&spi->dev, "slave-data", 0);
    espi->slave_sync = devm_gpiod_get(&spi->dev, "slave-sync", 0);
    espi->slave_select = devm_gpiod_get(&spi->dev, "slave-select", 0);

    dev_info(&spi->dev, "slave_data: %p", espi->slave_data);
    dev_info(&spi->dev, "slave_sync: %p", espi->slave_sync);
    dev_info(&spi->dev, "slave_select: %p", espi->slave_select);
	struct pinctrl* pinctrl = devm_pinctrl_get_select_default(&spi->dev);
    dev_info(&spi->dev, "pinctrl default: %d", IS_ERR(pinctrl));
    gpiod_direction_input(espi->slave_sync);
    gpiod_direction_input(espi->slave_data);
    gpiod_direction_output(espi->slave_select, 0);

    if(request_irq(gpiod_to_irq(espi->slave_sync), espi_slave_sync_handler, IRQF_SHARED | IRQF_TRIGGER_RISING, ETH_SPI_DRV_NAME, espi)) {
        dev_err(&spi->dev, "Failed to request_irq: slave_data_ready");

    }

    sema_init(&espi->slave_sync_sem, 1);

    espi->net_dev = espi_netdev;
    espi->spi_dev = spi;

    spi_set_drvdata(spi, espi_netdev);

    int ret = of_get_ethdev_address(spi->dev.of_node, espi->net_dev);
    if(ret) {
        eth_hw_addr_random(espi->net_dev);
        dev_info(&spi->dev, "Using random MAC address: %pM\n", espi->net_dev->dev_addr);
    }

    netif_carrier_off(espi_netdev);

    espi->net_dev->ethtool_ops = &chpt_eth_spi_ethtool_ops;

    if(register_netdev(espi_netdev)) {
        dev_err(&spi->dev, "Unable to register net device %s\n", espi_netdev->name);
        free_netdev(espi_netdev);
        return -EFAULT;
    }
    // init debugfs

    return 0;
}

static void chpt_eth_spi_remove(struct spi_device* spi)
{
    struct net_device* espi_devs = spi_get_drvdata(spi);
    //struct chpt_eth_spi* espi = netdev_priv(espi_devs);

    // remove debugfs
    //
    unregister_netdev(espi_devs);
    free_netdev(espi_devs);

}

static const struct spi_device_id chpt_eth_spi_id[] = {
    {"eth-spi", 0 },
    { /* sentinel */ }
};

static struct spi_driver chpt_eth_spi_driver = {
    .driver = {
        .name = ETH_SPI_DRV_NAME,
        .of_match_table = chpt_eth_spi_of_match,
    },
    .id_table = chpt_eth_spi_id,
    .probe = chpt_eth_spi_probe,
    .remove = chpt_eth_spi_remove,
};
module_spi_driver(chpt_eth_spi_driver);

MODULE_DESCRIPTION("ChargePoint Eth SPI Driver");
MODULE_AUTHOR("James Walmsley <james.walmsley@chargepoint.com");
MODULE_LICENSE("GPL");

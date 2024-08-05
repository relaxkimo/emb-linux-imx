#include <linux/circ_buf.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio/consumer.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <linux/pinctrl/consumer.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/version.h>

#include "eth_spi_protocol.h"

#define ETH_SPI_DRV_NAME "chpt-eth-spi"
#define TX_RING_NUM 16 /* power-of-2 */
#define ETH_SPI_TX_TIMEOUT 250

#define ETH_SPI_CLK_SPEED_MIN (1000000u)
#define ETH_SPI_CLK_SPEED_MAX (25000000u)
#define ETH_SPI_CLK_SPEED (1000000u)
static int eth_spi_clkspeed;
module_param(eth_spi_clkspeed, int, 0);
MODULE_PARM_DESC(eth_spi_clkspeed,
		 "SPI bus clock speed (Hz) (1000000 - 25000000).");

struct eth_spi_stats {
	u64 reset_timeout;
	u64 spi_err;
	u64 ring_full;
	u64 write_err;
	u64 out_of_mem;
};

struct tx_ring {
	struct sk_buff *skb[TX_RING_NUM];
	u16 head; /* write index (producer) */
	u16 tail; /* read index (consumer) */
};

struct chpt_eth_spi {
	struct net_device *net_dev;
	struct spi_device *spi_dev;
	struct eth_spi_stats stats;
	struct task_struct *spi_thread;

	struct gpio_desc *slave_select;
	struct gpio_desc *slave_data;
	struct gpio_desc *slave_sync;
	int slave_sync_irq;
	struct completion slave_sync_comp;

	unsigned int intr_req;
	unsigned int intr_svc;

	struct tx_ring txr;

	struct eth_spi_frame *tx_frame;
	struct eth_spi_frame *rx_frame;

	struct sk_buff *rx_skb;
};

static int chpt_eth_spi_transceive_frame(struct chpt_eth_spi *espi)
{
	struct spi_message msg;
	struct spi_transfer transfer = {
		.tx_buf = espi->tx_frame,
		.rx_buf = espi->rx_frame,
		.len = sizeof(struct eth_spi_frame),
	};
	int ret;

	reinit_completion(&espi->slave_sync_comp);

	gpiod_set_value(espi->slave_select, 1);
	netdev_vdbg(espi->net_dev, "slave select: 1");
	if (wait_for_completion_interruptible_timeout(
		    &espi->slave_sync_comp, msecs_to_jiffies(100)) == 0) {
		netdev_warn(espi->net_dev, "slave ack timeout");
	} else {
		netdev_vdbg(espi->net_dev, "slave ready");
	}

	spi_message_init(&msg);

	eth_spi_reset_frame(espi->rx_frame);

	spi_message_add_tail(&transfer, &msg);

	reinit_completion(&espi->slave_sync_comp);
	ret = spi_sync(espi->spi_dev, &msg);

	if (wait_for_completion_interruptible_timeout(
		    &espi->slave_sync_comp, msecs_to_jiffies(10)) == 0) {
		netdev_warn(espi->net_dev, "slave ack timeout");
	} else {
		netdev_vdbg(espi->net_dev, "slave done");
	}

	gpiod_set_value(espi->slave_select, 0);
	netdev_vdbg(espi->net_dev, "slave select: 0");
	usleep_range(20, 100);

	if (ret) {
		netdev_warn(espi->net_dev, "spi transfer failed");
		return -1;
	}

	if (eth_spi_frame_valid(espi->rx_frame)) {
		netdev_dbg(espi->net_dev, "rx frame valid %d/%d (len=%d)",
			   espi->rx_frame->frag_idx + 1,
			   espi->rx_frame->frag_tot + 1,
			   eth_spi_frame_len(espi->rx_frame));
		if (eth_spi_frame_len(espi->rx_frame) > 0) {
			if (espi->rx_frame->frag_idx == 0) {
				// first fragment

				if (espi->rx_skb) {
					netdev_warn(
						espi->net_dev,
						"drop incomplete package\n");
					kfree_skb(espi->rx_skb);
					espi->rx_skb = NULL;
				}
				espi->rx_skb = netdev_alloc_skb_ip_align(
					espi->net_dev,
					(espi->rx_frame->frag_tot + 1) *
						ETH_SPI_FRAG_LEN);
				if (!espi->rx_skb) {
					netdev_warn(
						espi->net_dev,
						"out of RX resources: no buffer\n");
					espi->stats.out_of_mem++;
					return -1;
				}
			}

			if (espi->rx_skb) {
				u16 frame_len =
					eth_spi_frame_len(espi->rx_frame);
				if (skb_tailroom(espi->rx_skb) < frame_len) {
					netdev_warn(
						espi->net_dev,
						"out of RX resources: buffer to small\n");
					kfree_skb(espi->rx_skb);
					espi->rx_skb = NULL;
					espi->stats.out_of_mem++;
					return -1;
				};
				skb_put_data(espi->rx_skb, espi->rx_frame->buf,
					     frame_len);
			}

			if (espi->rx_frame->frag_idx ==
			    espi->rx_frame->frag_tot) {
				struct net_device_stats *n_stats =
					&espi->net_dev->stats;

				// last frame
				espi->rx_skb->protocol = eth_type_trans(
					espi->rx_skb, espi->rx_skb->dev);
				skb_checksum_none_assert(espi->rx_skb);

				n_stats->rx_packets++;
				n_stats->rx_bytes += espi->rx_skb->len;
				netdev_dbg(espi->net_dev,
					   "Rx-ing packet: Size: %d\n",
					   espi->rx_skb->len);
				netif_rx(espi->rx_skb);
				espi->rx_skb = NULL;
			}
		}
	} else {
		netdev_warn(espi->net_dev, "invalid frame");
		return -1;
	}

	return 0;
}

static int chpt_eth_spi_transceive(struct chpt_eth_spi *espi)
{
	struct net_device_stats *n_stats = &espi->net_dev->stats;

	while (CIRC_CNT(espi->txr.head, espi->txr.tail, TX_RING_NUM) ||
	       gpiod_get_value(espi->slave_data)) {
		// transceive as long as we have data to send or the slave has data ready

		if (CIRC_CNT(espi->txr.head, espi->txr.tail, TX_RING_NUM) > 0) {
			// data to send
			u8 frag_idx;
			struct sk_buff *skb = espi->txr.skb[espi->txr.tail];

			u8 frag_tot = ((skb->len - 1u) / ETH_SPI_FRAG_LEN) + 1u;
			for (frag_idx = 0; frag_idx < frag_tot; frag_idx++) {
				u16 len = min(128u,
					      (skb->len -
					       (frag_idx * ETH_SPI_FRAG_LEN)));
				memcpy(espi->tx_frame->buf,
				       &skb->data[frag_idx * ETH_SPI_FRAG_LEN],
				       len);
				eth_spi_init_frame(espi->tx_frame, frag_idx,
						   frag_tot - 1, len);
				netdev_dbg(espi->net_dev,
					   "tx frame %d/%d (len=%d)",
					   frag_idx + 1, frag_tot, len);
				if (chpt_eth_spi_transceive_frame(espi) == -1) {
					espi->stats.write_err++;
					return -1;
				}
			}
			n_stats->tx_packets++;
			n_stats->tx_bytes += espi->txr.skb[espi->txr.tail]->len;

			dev_kfree_skb(espi->txr.skb[espi->txr.tail]);
			espi->txr.skb[espi->txr.tail] = NULL;
			espi->txr.tail =
				((espi->txr.tail + 1) & (TX_RING_NUM - 1));

			if (netif_queue_stopped(espi->net_dev)) {
				netif_wake_queue(espi->net_dev);
			}
		} else {
			// no date to send, receive only
			memset(espi->tx_frame->buf, 0, ETH_SPI_FRAG_LEN);
			eth_spi_init_frame(espi->tx_frame, 0, 0, 0);
			if (chpt_eth_spi_transceive_frame(espi) == -1) {
				espi->stats.write_err++;
				return -1;
			}
		}
	}
	return 0;
}

static int chpt_eth_spi_thread(void *data)
{
	struct chpt_eth_spi *espi = data;
	netdev_info(espi->net_dev, "SPI thread created\n");

	while (!kthread_should_stop()) {
		set_current_state(TASK_INTERRUPTIBLE);
		if (netif_carrier_ok(espi->net_dev) &&
		    (espi->intr_req == espi->intr_svc) &&
		    !CIRC_CNT(espi->txr.head, espi->txr.tail, TX_RING_NUM)) {
			// no intr to serve and no skb to send
			schedule();
		}

		set_current_state(TASK_RUNNING);

		netdev_dbg(
			espi->net_dev,
			"have work to do. int: %d, tx_skb: %p, carrier: %s\n",
			espi->intr_req - espi->intr_svc,
			espi->txr.skb[espi->txr.tail],
			netif_carrier_ok(espi->net_dev) ? "ok" : "nok");

		if (!netif_carrier_ok(espi->net_dev)) {
			memset(espi->tx_frame->buf, 0, ETH_SPI_FRAG_LEN);
			eth_spi_init_frame(espi->tx_frame, 0, 0, 0);
			if (chpt_eth_spi_transceive_frame(espi)) {
				// failed to sync
				msleep(500); // wait for retry
			} else {
				// sync ready
				netif_wake_queue(espi->net_dev);
				netif_carrier_on(espi->net_dev);
			}
		}

		if (netif_carrier_ok(espi->net_dev)) {
			espi->intr_svc = espi->intr_req;
			chpt_eth_spi_transceive(espi);
		}
	}

	// flush ring buffer
	while (CIRC_CNT(espi->txr.head, espi->txr.tail, TX_RING_NUM) > 0) {
		dev_kfree_skb(espi->txr.skb[espi->txr.tail]);
		espi->txr.skb[espi->txr.tail] = NULL;
		espi->txr.tail = ((espi->txr.tail + 1) & (TX_RING_NUM - 1));
	}

	set_current_state(TASK_RUNNING);
	netdev_info(espi->net_dev, "SPI thread exit\n");

	return 0;
}

static irqreturn_t eth_spi_intr_handler(int irq, void *data)
{
	struct chpt_eth_spi *espi = data;

	espi->intr_req++;
	if (espi->spi_thread) {
		wake_up_process(espi->spi_thread);
	}

	return IRQ_HANDLED;
}

static irqreturn_t espi_slave_sync_handler(int irq, void *data)
{
	struct chpt_eth_spi *espi = data;

	complete(&espi->slave_sync_comp);

	return IRQ_HANDLED;
}

static int chpt_eth_spi_netdev_open(struct net_device *dev)
{
	int ret;

	struct chpt_eth_spi *espi = netdev_priv(dev);
	netdev_info(espi->net_dev, "netdev_open");

	espi->intr_req = 1;
	espi->intr_svc = 0;

	espi->spi_thread =
		kthread_run(chpt_eth_spi_thread, espi, "%s", dev->name);
	if (IS_ERR(espi->spi_thread)) {
		netdev_err(dev, "%s: unable to start kernel thread.\n",
			   ETH_SPI_DRV_NAME);
		return PTR_ERR(espi->spi_thread);
	}

	ret = request_irq(espi->spi_dev->irq, eth_spi_intr_handler, 0,
			  dev->name, espi);
	if (ret) {
		netdev_err(dev, "%s: unable to get IRQ %d (irqval=%d).\n",
			   ETH_SPI_DRV_NAME, espi->spi_dev->irq, ret);
		kthread_stop(espi->spi_thread);
		return ret;
	}

	/* CHECK: is IRQF_TRIGGER_* is required as it's define in DT */
	ret = request_irq(espi->slave_sync_irq, espi_slave_sync_handler,
			  IRQF_SHARED | IRQF_TRIGGER_RISING |
				  IRQF_TRIGGER_FALLING,
			  ETH_SPI_DRV_NAME, espi);
	if (ret) {
		netdev_err(dev, "Failed to request_irq: slave_sync");
	}

	return 0;
}

static int chpt_eth_spi_netdev_stop(struct net_device *dev)
{
	struct chpt_eth_spi *espi = netdev_priv(dev);
	netdev_info(espi->net_dev, "netdev_stop");

	netif_stop_queue(dev);

	free_irq(espi->spi_dev->irq, espi);
	free_irq(espi->slave_sync_irq, espi);

	kthread_stop(espi->spi_thread);
	espi->spi_thread = NULL;

	return 0;
}

static netdev_tx_t chpt_eth_spi_netdev_xmit(struct sk_buff *skb,
					    struct net_device *dev)
{
	struct chpt_eth_spi *espi = netdev_priv(dev);
	netdev_vdbg(espi->net_dev, "netdev_xmit");

	if (CIRC_SPACE(espi->txr.head, espi->txr.tail, TX_RING_NUM) == 0) {
		netdev_warn(espi->net_dev, "queue was unexpectedly full!\n");
		netif_stop_queue(espi->net_dev);
		espi->stats.ring_full++;
		return NETDEV_TX_BUSY;
	}

	netdev_dbg(espi->net_dev, "Tx-ing packet: Size: %d\n", skb->len);

	espi->txr.skb[espi->txr.head] = skb;
	espi->txr.head = (espi->txr.head + 1) & (TX_RING_NUM - 1);

	if (CIRC_SPACE(espi->txr.head, espi->txr.tail, TX_RING_NUM) == 0) {
		netdev_info(espi->net_dev, "TXring is full!");
		netif_stop_queue(espi->net_dev);
		espi->stats.ring_full++;
	}

	netif_trans_update(dev);

	if (espi->spi_thread) {
		wake_up_process(espi->spi_thread);
	}

	return NETDEV_TX_OK;
}

static void chpt_eth_spi_netdev_tx_timeout(struct net_device *dev,
					   unsigned int txqueue)
{
	struct chpt_eth_spi *espi = netdev_priv(dev);
	netdev_info(espi->net_dev, "Transmit timeout at %ld, latency %ld\n",
		    jiffies, jiffies - dev_trans_start(dev));
	espi->net_dev->stats.tx_errors++;

	if (espi->spi_thread) {
		wake_up_process(espi->spi_thread);
	}
}

static int chpt_eth_spi_netdev_init(struct net_device *dev)
{
	struct chpt_eth_spi *espi = netdev_priv(dev);

	netdev_info(espi->net_dev, "netdev_init");

	dev->mtu = ETH_DATA_LEN;
	dev->type = ARPHRD_ETHER;
	espi->spi_thread = NULL;

	memset(&espi->stats, 0, sizeof(struct eth_spi_stats));

	espi->tx_frame = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);
	if (!espi->tx_frame) {
		return -ENOBUFS;
	}
	espi->rx_frame = kmalloc(sizeof(struct eth_spi_frame), GFP_KERNEL);
	if (!espi->rx_frame) {
		kfree(espi->tx_frame);
		return -ENOBUFS;
	}

	espi->rx_skb = NULL; // CHECK if allready null-ed

	return 0;
}

static void chpt_eth_spi_netdev_uninit(struct net_device *dev)
{
	struct chpt_eth_spi *espi = netdev_priv(dev);
	netdev_info(espi->net_dev, "netdev_uninit");

	kfree(espi->tx_frame);
	kfree(espi->rx_frame);

	dev_kfree_skb(espi->rx_skb);
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

static const struct ethtool_ops chpt_eth_spi_ethtool_ops = {};

static void chpt_eth_spi_netdev_setup(struct net_device *dev)
{
	struct chpt_eth_spi *espi = NULL;

	dev->netdev_ops = &chpt_eth_spi_netdev_ops;
	dev->ethtool_ops = &chpt_eth_spi_ethtool_ops;
	dev->watchdog_timeo = ETH_SPI_TX_TIMEOUT;
	dev->priv_flags &= ~IFF_TX_SKB_SHARING;
	dev->tx_queue_len = 100;

	dev->min_mtu = ETH_MIN_MTU;
	dev->max_mtu = ETH_SPI_MAX_MTU;

	espi = netdev_priv(dev);
	memset(espi, 0, sizeof(*espi));
};

static const struct of_device_id chpt_eth_spi_of_match[] = {
	{ .compatible = "chpt,eth-spi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, chpt_eth_spi_of_match);

static int chpt_eth_spi_probe(struct spi_device *spi)
{
	int ret;
	struct chpt_eth_spi *espi = NULL;
	struct net_device *espi_netdev = NULL;
	struct pinctrl *pinctrl;

	dev_info(&spi->dev, "Probing ChargePoint Eth SPI device...");

	if (!spi->dev.of_node) {
		dev_err(&spi->dev, "Missing device tree\n");
		return -EINVAL;
	}

	if (eth_spi_clkspeed == 0) {
		if (spi->max_speed_hz) {
			eth_spi_clkspeed = spi->max_speed_hz;
		} else {
			eth_spi_clkspeed = ETH_SPI_CLK_SPEED;
		}
	}

	if ((eth_spi_clkspeed < ETH_SPI_CLK_SPEED_MIN) ||
	    (eth_spi_clkspeed > ETH_SPI_CLK_SPEED_MAX)) {
		dev_err(&spi->dev, "Invalid clkspeed: %d\n", eth_spi_clkspeed);
		return -EINVAL;
	}

	dev_info(&spi->dev, "clkspeed=%d\n", eth_spi_clkspeed);

	spi->max_speed_hz = eth_spi_clkspeed;
	if (spi_setup(spi) < 0) {
		dev_err(&spi->dev, "Unable to setup SPI device\n");
		return -EFAULT;
	}

	espi_netdev = alloc_etherdev(sizeof(struct chpt_eth_spi));
	if (!espi_netdev)
		return -ENOMEM;

	chpt_eth_spi_netdev_setup(espi_netdev);
	SET_NETDEV_DEV(espi_netdev, &spi->dev);

	espi = netdev_priv(espi_netdev);
	if (!espi) {
		free_netdev(espi_netdev);
		dev_err(&spi->dev, "Fail to retrieve private structure\n");
		return -ENOMEM;
	}
	espi->net_dev = espi_netdev;
	espi->spi_dev = spi;

	espi->slave_data = devm_gpiod_get(&spi->dev, "slave-data", 0);
	espi->slave_sync = devm_gpiod_get(
		&spi->dev, "slave-sync",
		0); // slave sync gpio not used (only as interrupt source)
	espi->slave_select = devm_gpiod_get(&spi->dev, "slave-select", 0);

	dev_info(&spi->dev, "slave_data: %p", espi->slave_data);
	dev_info(&spi->dev, "slave_sync: %p", espi->slave_sync);
	dev_info(&spi->dev, "slave_select: %p", espi->slave_select);
	pinctrl = devm_pinctrl_get_select_default(&spi->dev);
	dev_info(&spi->dev, "pinctrl default: %d", IS_ERR(pinctrl));
	gpiod_direction_input(espi->slave_sync);
	gpiod_direction_input(espi->slave_data);
	gpiod_direction_output(espi->slave_select, 0);

	espi->slave_sync_irq = gpiod_to_irq(espi->slave_sync);

	init_completion(&espi->slave_sync_comp);

	spi_set_drvdata(spi, espi_netdev);

	ret = of_get_ethdev_address(spi->dev.of_node, espi->net_dev);
	if (ret) {
		eth_hw_addr_random(espi->net_dev);
		dev_info(&spi->dev, "Using random MAC address: %pM\n",
			 espi->net_dev->dev_addr);
	}

	netif_carrier_off(espi_netdev);

	if (register_netdev(espi_netdev)) {
		dev_err(&spi->dev, "Unable to register net device %s\n",
			espi_netdev->name);
		free_netdev(espi_netdev);
		return -EFAULT;
	}

	return 0;
}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 18, 0)
static int chpt_eth_spi_remove(struct spi_device *spi)
#else
// make remove callback a void function: a0386bba70934d42f586eaf68b21d5eeaffa7bd0
static void chpt_eth_spi_remove(struct spi_device *spi)
#endif
{
	struct net_device *espi_devs = spi_get_drvdata(spi);

	unregister_netdev(espi_devs);
	free_netdev(espi_devs);
#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 18, 0)
	return 0;
#endif
}

/* clang-format off */
static const struct spi_device_id chpt_eth_spi_id[] = {
	{ "eth-spi", 0 },
	{ /* sentinel */ }
};
/* clang-format on */
MODULE_DEVICE_TABLE(spi, chpt_eth_spi_id);

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

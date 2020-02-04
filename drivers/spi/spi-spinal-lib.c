#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/gpio.h>

#define DRV_NAME "spinal-lib,spi-1.0"

#define SPI_CMD_WRITE (1 << 8)
#define SPI_CMD_READ (1 << 9)
#define SPI_CMD_SS (1 << 11)

#define SPI_RSP_VALID (1 << 31)

#define SPI_STATUS_CMD_INT_ENABLE = (1 << 0)
#define SPI_STATUS_RSP_INT_ENABLE = (1 << 1)
#define SPI_STATUS_CMD_INT_FLAG = (1 << 8)
#define SPI_STATUS_RSP_INT_FLAG = (1 << 9)


#define SPI_MODE_CPOL (1 << 0)
#define SPI_MODE_CPHA (1 << 1)


#define SPI_SPINAL_LIB_DATA        0x00
#define SPI_SPINAL_LIB_BUFFER      0x04
#define SPI_SPINAL_LIB_CONFIG      0x08
#define SPI_SPINAL_LIB_INTERRUPT   0x0C
#define SPI_SPINAL_LIB_CLK_DIVIDER 0x20
#define SPI_SPINAL_LIB_SS_SETUP    0x24
#define SPI_SPINAL_LIB_SS_HOLD     0x28
#define SPI_SPINAL_LIB_SS_DISABLE  0x2C
#define SPI_SPINAL_LIB_SS_ACTIVE_HIGH  0x30



struct spi_spinal_lib {
	void __iomem *base;
	s32 irq;
	u32 len;
	u32 count, txCount;
	u32 bytes_per_word;
	u32 ssActiveHigh;
	u32 hz;
	u32 cmdFifoDepth;
	u32 rspFifoDepth;

	/* data buffers */
	const u8 *tx;
	u8 *rx;
};

static inline struct spi_spinal_lib *spi_spinal_lib_to_hw(struct spi_device *sdev)
{
	return spi_master_get_devdata(sdev->master);
}

static u32 spi_spinal_lib_cmd_availability(struct spi_spinal_lib *hw){
	return readl(hw->base + SPI_SPINAL_LIB_BUFFER) & 0xFFFF;
}

static u32 spi_spinal_lib_rsp_occupancy(struct spi_spinal_lib *hw){
	return readl(hw->base + SPI_SPINAL_LIB_BUFFER) >> 16;
}

static void spi_spinal_lib_cmd(struct spi_spinal_lib *hw, u32 cmd){
	writel(cmd, hw->base + SPI_SPINAL_LIB_DATA);
}

static u32 spi_spinal_lib_rsp(struct spi_spinal_lib *hw){
	return readl(hw->base + SPI_SPINAL_LIB_DATA);
}

static void spi_spinal_lib_cmd_wait(struct spi_spinal_lib *hw){
	while(spi_spinal_lib_cmd_availability(hw) == 0) cpu_relax();
}

static void spi_spinal_lib_rsp_wait(struct spi_spinal_lib *hw){
	while(spi_spinal_lib_rsp_occupancy(hw) == 0) cpu_relax();
}

static u32 spi_spinal_lib_rsp_pull(struct spi_spinal_lib *hw){
	u32 rsp;
	while(((s32)(rsp = spi_spinal_lib_rsp(hw))) < 0) cpu_relax();
	return rsp;
}

static void spi_spinal_lib_set_cs(struct spi_device *spi, bool high)
{
	struct spi_spinal_lib *hw = spi_spinal_lib_to_hw(spi);
	spi_spinal_lib_cmd(hw, spi->chip_select | ((high != 0) ^ ((spi->mode & SPI_CS_HIGH) != 0) ? 0x00 : 0x80) | SPI_CMD_SS);
	spi_spinal_lib_cmd_wait(hw);

//	printk("CS %d %d\n",spi->chip_select, disable);
}

static void spi_spinal_lib_speed(struct spi_spinal_lib *hw, u32 speed_hz){
	u32 clk_divider = (hw->hz/speed_hz/2)-1;
	writel(clk_divider, hw->base + SPI_SPINAL_LIB_CLK_DIVIDER);
}

static int spi_spinal_lib_txrx(struct spi_master *master, struct spi_device *spi, struct spi_transfer *t)
{
	struct spi_spinal_lib *hw = spi_master_get_devdata(master);


	spi_spinal_lib_speed(hw, t->speed_hz);

	hw->tx = t->tx_buf;
	hw->rx = t->rx_buf;
	hw->count = 0;
	hw->txCount = 0;
	hw->bytes_per_word = DIV_ROUND_UP(t->bits_per_word, 8);
	hw->len = t->len / hw->bytes_per_word;

	if (hw->irq >= 0) {
		dev_info(&master->dev, "Interrupt not implemented\n");
		/* enable receive interrupt */
//		hw->imr |= spi_spinal_lib_CONTROL_IRRDY_MSK;
//		writel(hw->imr, hw->base + spi_spinal_lib_CONTROL);

		/* send the first byte */
//		spi_spinal_lib_tx_word(hw);
	} else {
		if(hw->cmdFifoDepth > 1 && hw->rspFifoDepth > 1){
			u32 cmd = SPI_CMD_WRITE | SPI_CMD_READ;
			while (hw->count < hw->len) {
				{	//rsp
					u32 burst;
					u8 *ptr, *end;

					burst = spi_spinal_lib_rsp_occupancy(hw);
					ptr = hw->rx + hw->count;
					end = ptr + burst;
					if(hw->rx) {while(ptr != end) {*ptr++ = spi_spinal_lib_rsp(hw);}}
					else	   {while(ptr != end) { ptr++;  spi_spinal_lib_rsp(hw);}}
					hw->count += burst;
				}

				{	//cmd
					u32 burst;
					const u8 *ptr, *end;
					u32 cmdAvailability = spi_spinal_lib_cmd_availability(hw);
					u32 cmdOccupancy = hw->cmdFifoDepth - cmdAvailability;
					u32 rxAvailability = hw->rspFifoDepth - spi_spinal_lib_rsp_occupancy(hw);
					u32 cmdMax = rxAvailability < cmdOccupancy ? 0 : rxAvailability - cmdOccupancy;
					burst = min(hw->len - hw->txCount, min(cmdAvailability, cmdMax));
					ptr = hw->tx + hw->txCount;
					end = ptr + burst;
					if(hw->tx) {while(ptr != end) {writel(cmd | *ptr++, hw->base + SPI_SPINAL_LIB_DATA);}}
					else	   {while(ptr != end) {ptr++; writel(cmd, hw->base + SPI_SPINAL_LIB_DATA);}}
					hw->txCount += burst;
				}
			}
		} else {
			u32 cmd = (hw->tx ? SPI_CMD_WRITE : 0) | SPI_CMD_READ;
			while (hw->count < hw->len) {
				u32 data = hw->tx ? hw->tx[hw->count] : 0;
				writel(cmd | data, hw->base + SPI_SPINAL_LIB_DATA);
				data = spi_spinal_lib_rsp_pull(hw);
				if (hw->rx) hw->rx[hw->count] = data;

				hw->count++;
			}
		}

		spi_finalize_current_transfer(master);
	}


	return t->len;
}

//static irqreturn_t spi_spinal_lib_irq(int irq, void *dev)
//{
//	struct spi_master *master = dev;
//	struct spi_spinal_lib *hw = spi_master_get_devdata(master);
//
//	spi_spinal_lib_rx_word(hw);
//
//	if (hw->count < hw->len) {
//		spi_spinal_lib_tx_word(hw);
//	} else {
//		/* disable receive interrupt */
//		hw->imr &= ~spi_spinal_lib_CONTROL_IRRDY_MSK;
//		writel(hw->imr, hw->base + spi_spinal_lib_CONTROL);
//
//		spi_finalize_current_transfer(master);
//	}
//
//	return IRQ_HANDLED;
//}

static int spi_spinal_lib_setup(struct spi_device *spi)
{
	struct spi_spinal_lib *hw = spi_master_get_devdata(spi->controller);
	u32 config = 0;

	if (gpio_is_valid(spi->cs_gpio)){
		gpio_direction_output(spi->cs_gpio, spi->mode & SPI_CS_HIGH ? 0 : 1);
	} else {
		if(spi->mode & SPI_CS_HIGH)
			hw->ssActiveHigh |= 1 << spi->chip_select;
		else
			hw->ssActiveHigh &= ~(1 << spi->chip_select);
		writel(hw->ssActiveHigh, hw->base + SPI_SPINAL_LIB_SS_ACTIVE_HIGH);
	}


	if (spi->mode & SPI_CPOL)
		config |= SPI_MODE_CPOL;
	if (spi->mode & SPI_CPHA)
		config |= SPI_MODE_CPHA;
	writel(config, hw->base + SPI_SPINAL_LIB_CONFIG);


//	printk("Setup %d %d\n", hw->ssActiveHigh, config);
	return 0;
}


static int spi_spinal_lib_probe(struct platform_device *pdev)
{
	struct spi_spinal_lib *hw;
	struct spi_master *master;
	struct resource *res;
	u32 hz;
	int err = -ENODEV;

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_spinal_lib));
	if (!master)
		return err;

	/* setup the master state. */
	master->bus_num = pdev->id;
	master->num_chipselect = 16; //TODO
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH;
	master->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 8);
	master->dev.of_node = pdev->dev.of_node;
	master->transfer_one = spi_spinal_lib_txrx;
	master->set_cs = spi_spinal_lib_set_cs;
	master->setup = spi_spinal_lib_setup;
	hz = 100000000; //TODO
//	hz = devm_clk_get(&pdev->dev, "hz");
//	if (IS_ERR(hz))
//		return PTR_ERR(hz);


	hw = spi_master_get_devdata(master);
	hw->hz = hz;
	if(of_property_read_u32(pdev->dev.of_node, "rsp_fifo_depth", &hw->rspFifoDepth)){
		dev_info(&pdev->dev, "Missing rsp_fifo_depth in DTS\n");
		goto exit;
	}
	if(of_property_read_u32(pdev->dev.of_node, "cmd_fifo_depth", &hw->cmdFifoDepth)){
		dev_info(&pdev->dev, "Missing cmd_fifo_depth in DTS\n");
		goto exit;
	}

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hw->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hw->base)) {
		err = PTR_ERR(hw->base);
		goto exit;
	}
	/* program defaults into the registers */
	hw->ssActiveHigh = 0;
	writel(0, hw->base + SPI_SPINAL_LIB_CONFIG);
	writel(3, hw->base + SPI_SPINAL_LIB_INTERRUPT);
	writel(3, hw->base + SPI_SPINAL_LIB_CLK_DIVIDER);
	writel(3, hw->base + SPI_SPINAL_LIB_SS_DISABLE);
	writel(3, hw->base + SPI_SPINAL_LIB_SS_SETUP);
	writel(3, hw->base + SPI_SPINAL_LIB_SS_HOLD);
	while(spi_spinal_lib_rsp_occupancy(hw)) spi_spinal_lib_rsp(hw); //Flush rsp
	//TODO all chipselect disable


	/* Request GPIO CS lines, if any */
	if (master->cs_gpios) {
		u32 i;
		for (i = 0; i < master->num_chipselect; i++) {
			if (!gpio_is_valid(master->cs_gpios[i]))
				continue;

			err = devm_gpio_request(&pdev->dev,
						master->cs_gpios[i],
						DRV_NAME);
			if (err) {
				dev_err(&pdev->dev, "Can't get CS GPIO %i\n",
					master->cs_gpios[i]);
				goto exit;
			}
		}
	}


	/* irq is optional */
	hw->irq = platform_get_irq(pdev, 0);
	if (hw->irq >= 0) {
//		err = devm_request_irq(&pdev->dev, hw->irq, spi_spinal_lib_irq, 0,
//				       pdev->name, master);
//		if (err)
//			goto exit;
		dev_info(&pdev->dev, "Interrupt not supported %d\n", hw->irq);
		goto exit;
	}

	err = devm_spi_register_master(&pdev->dev, master);
	if (err)
		goto exit;
	dev_info(&pdev->dev, "base %p, irq %d\n", hw->base, hw->irq);

	return 0;
exit:
	spi_master_put(master);
	return err;
}


static const struct of_device_id spi_spinal_lib_match[] = {
	{ .compatible = "spinal-lib,spi-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, spi_spinal_lib_match);

static struct platform_driver spi_spinal_lib_driver = {
	.probe = spi_spinal_lib_probe,
	.driver = {
		.name = DRV_NAME,
		.pm = NULL,
		.of_match_table = of_match_ptr(spi_spinal_lib_match),
	},
	.prevent_deferred_probe = 1,
};


module_platform_driver(spi_spinal_lib_driver);

MODULE_DESCRIPTION("spinal lib SPI driver");
MODULE_AUTHOR("Charles Papon <charles.papon.90@gmail.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);

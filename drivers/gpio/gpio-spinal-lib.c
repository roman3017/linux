/*
 * Copyright (C) 2013 Altera Corporation
 * Based on gpio-mpc8xxx.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/gpio/driver.h>
#include <linux/of_gpio.h> /* For of_mm_gpio_chip */
#include <linux/platform_device.h>

#define SPINAL_LIB_GPIO_MAX_NGPIO		32
#define SPINAL_LIB_GPIO_INPUT		    0x0
#define SPINAL_LIB_GPIO_OUTPUT			0x4
#define SPINAL_LIB_GPIO_OUTPUT_ENABLE	0x8

/**
* struct spinal_lib_gpio_chip
* @mmchip		: memory mapped chip structure.
* @gpio_lock		: synchronization lock so that new irq/set/get requests
			  will be blocked until the current one completes.
* @interrupt_trigger	: specifies the hardware configured IRQ trigger type
			  (rising, falling, both, high)
* @mapped_irq		: kernel mapped irq number.
*/
struct spinal_lib_gpio_chip {
	struct of_mm_gpio_chip mmchip;
	raw_spinlock_t gpio_lock;
//	int interrupt_trigger;
//	int mapped_irq;
};

//static void spinal_lib_gpio_irq_unmask(struct irq_data *d)
//{
//	struct spinal_lib_gpio_chip *spinal_lib_gc;
//	struct of_mm_gpio_chip *mm_gc;
//	unsigned long flags;
//	u32 intmask;
//
//	spinal_lib_gc = gpiochip_get_data(irq_data_get_irq_chip_data(d));
//	mm_gc = &spinal_lib_gc->mmchip;
//
//	raw_spin_lock_irqsave(&spinal_lib_gc->gpio_lock, flags);
//	intmask = readl(mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK);
//	/* Set SPINAL_LIB_GPIO_IRQ_MASK bit to unmask */
//	intmask |= BIT(irqd_to_hwirq(d));
//	writel(intmask, mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK);
//	raw_spin_unlock_irqrestore(&spinal_lib_gc->gpio_lock, flags);
//}
//
//static void spinal_lib_gpio_irq_mask(struct irq_data *d)
//{
//	struct spinal_lib_gpio_chip *spinal_lib_gc;
//	struct of_mm_gpio_chip *mm_gc;
//	unsigned long flags;
//	u32 intmask;
//
//	spinal_lib_gc = gpiochip_get_data(irq_data_get_irq_chip_data(d));
//	mm_gc = &spinal_lib_gc->mmchip;
//
//	raw_spin_lock_irqsave(&spinal_lib_gc->gpio_lock, flags);
//	intmask = readl(mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK);
//	/* Clear SPINAL_LIB_GPIO_IRQ_MASK bit to mask */
//	intmask &= ~BIT(irqd_to_hwirq(d));
//	writel(intmask, mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK);
//	raw_spin_unlock_irqrestore(&spinal_lib_gc->gpio_lock, flags);
//}

/**
 * This controller's IRQ type is synthesized in hardware, so this function
 * just checks if the requested set_type matches the synthesized IRQ type
 */
//static int spinal_lib_gpio_irq_set_type(struct irq_data *d,
//				   unsigned int type)
//{
//	struct spinal_lib_gpio_chip *spinal_lib_gc;
//
//	spinal_lib_gc = gpiochip_get_data(irq_data_get_irq_chip_data(d));
//
//	if (type == IRQ_TYPE_NONE) {
//		irq_set_handler_locked(d, handle_bad_irq);
//		return 0;
//	}
//	if (type == spinal_lib_gc->interrupt_trigger) {
//		if (type == IRQ_TYPE_LEVEL_HIGH)
//			irq_set_handler_locked(d, handle_level_irq);
//		else
//			irq_set_handler_locked(d, handle_simple_irq);
//		return 0;
//	}
//	irq_set_handler_locked(d, handle_bad_irq);
//	return -EINVAL;
//}
//
//static unsigned int spinal_lib_gpio_irq_startup(struct irq_data *d)
//{
//	spinal_lib_gpio_irq_unmask(d);
//
//	return 0;
//}
//
//static struct irq_chip spinal_lib_irq_chip = {
//	.name		= "spinal_lib-gpio",
//	.irq_mask	= spinal_lib_gpio_irq_mask,
//	.irq_unmask	= spinal_lib_gpio_irq_unmask,
//	.irq_set_type	= spinal_lib_gpio_irq_set_type,
//	.irq_startup	= spinal_lib_gpio_irq_startup,
//	.irq_shutdown	= spinal_lib_gpio_irq_mask,
//};

static int spinal_lib_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc;

	mm_gc = to_of_mm_gpio_chip(gc);

	return !!(readl(mm_gc->regs + SPINAL_LIB_GPIO_INPUT) & BIT(offset));
}

static void spinal_lib_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc;
	struct spinal_lib_gpio_chip *chip;
	unsigned long flags;
	unsigned int data_reg;

	mm_gc = to_of_mm_gpio_chip(gc);
	chip = gpiochip_get_data(gc);

	raw_spin_lock_irqsave(&chip->gpio_lock, flags);
	data_reg = readl(mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT);
	if (value)
		data_reg |= BIT(offset);
	else
		data_reg &= ~BIT(offset);
	writel(data_reg, mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT);
	raw_spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int spinal_lib_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc;
	struct spinal_lib_gpio_chip *chip;
	unsigned long flags;
	unsigned int gpio_ddr;

	mm_gc = to_of_mm_gpio_chip(gc);
	chip = gpiochip_get_data(gc);

	raw_spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as input, assumes software controlled IP */
	gpio_ddr = readl(mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT_ENABLE);
	gpio_ddr &= ~BIT(offset);
	writel(gpio_ddr, mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT_ENABLE);
	raw_spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int spinal_lib_gpio_direction_output(struct gpio_chip *gc,
		unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc;
	struct spinal_lib_gpio_chip *chip;
	unsigned long flags;
	unsigned int data_reg, gpio_ddr;

	mm_gc = to_of_mm_gpio_chip(gc);
	chip = gpiochip_get_data(gc);

	raw_spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Sets the GPIO value */
	data_reg = readl(mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT);
	if (value)
		data_reg |= BIT(offset);
	else
		data_reg &= ~BIT(offset);
	writel(data_reg, mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT);

	/* Set pin as output, assumes software controlled IP */
	gpio_ddr = readl(mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT_ENABLE);
	gpio_ddr |= BIT(offset);
	writel(gpio_ddr, mm_gc->regs + SPINAL_LIB_GPIO_OUTPUT_ENABLE);
	raw_spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

//static void spinal_lib_gpio_irq_edge_handler(struct irq_desc *desc)
//{
//	struct spinal_lib_gpio_chip *spinal_lib_gc;
//	struct irq_chip *chip;
//	struct of_mm_gpio_chip *mm_gc;
//	struct irq_domain *irqdomain;
//	unsigned long status;
//	int i;
//
//	spinal_lib_gc = gpiochip_get_data(irq_desc_get_handler_data(desc));
//	chip = irq_desc_get_chip(desc);
//	mm_gc = &spinal_lib_gc->mmchip;
//	irqdomain = spinal_lib_gc->mmchip.gc.irq.domain;
//
//	chained_irq_enter(chip, desc);
//
//	while ((status =
//	      (readl(mm_gc->regs + SPINAL_LIB_GPIO_EDGE_CAP) &
//	      readl(mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK)))) {
//		writel(status, mm_gc->regs + SPINAL_LIB_GPIO_EDGE_CAP);
//		for_each_set_bit(i, &status, mm_gc->gc.ngpio) {
//			generic_handle_irq(irq_find_mapping(irqdomain, i));
//		}
//	}
//
//	chained_irq_exit(chip, desc);
//}

//static void spinal_lib_gpio_irq_leveL_high_handler(struct irq_desc *desc)
//{
//	struct spinal_lib_gpio_chip *spinal_lib_gc;
//	struct irq_chip *chip;
//	struct of_mm_gpio_chip *mm_gc;
//	struct irq_domain *irqdomain;
//	unsigned long status;
//	int i;
//
//	spinal_lib_gc = gpiochip_get_data(irq_desc_get_handler_data(desc));
//	chip = irq_desc_get_chip(desc);
//	mm_gc = &spinal_lib_gc->mmchip;
//	irqdomain = spinal_lib_gc->mmchip.gc.irq.domain;
//
//	chained_irq_enter(chip, desc);
//
//	status = readl(mm_gc->regs + SPINAL_LIB_GPIO_DATA);
//	status &= readl(mm_gc->regs + SPINAL_LIB_GPIO_IRQ_MASK);
//
//	for_each_set_bit(i, &status, mm_gc->gc.ngpio) {
//		generic_handle_irq(irq_find_mapping(irqdomain, i));
//	}
//	chained_irq_exit(chip, desc);
//}

static int spinal_lib_gpio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int reg, ret;
	struct spinal_lib_gpio_chip *spinal_lib_gc;

	spinal_lib_gc = devm_kzalloc(&pdev->dev, sizeof(*spinal_lib_gc), GFP_KERNEL);
	if (!spinal_lib_gc)
		return -ENOMEM;

	raw_spin_lock_init(&spinal_lib_gc->gpio_lock);

	if (of_property_read_u32(node, "spinal-lib,ngpio", &reg))
		/* By default assume maximum ngpio */
		spinal_lib_gc->mmchip.gc.ngpio = SPINAL_LIB_GPIO_MAX_NGPIO;
	else
		spinal_lib_gc->mmchip.gc.ngpio = reg;

	if (spinal_lib_gc->mmchip.gc.ngpio > SPINAL_LIB_GPIO_MAX_NGPIO) {
		dev_warn(&pdev->dev,
			"ngpio is greater than %d, defaulting to %d\n",
			SPINAL_LIB_GPIO_MAX_NGPIO, SPINAL_LIB_GPIO_MAX_NGPIO);
		spinal_lib_gc->mmchip.gc.ngpio = SPINAL_LIB_GPIO_MAX_NGPIO;
	}

	spinal_lib_gc->mmchip.gc.direction_input	= spinal_lib_gpio_direction_input;
	spinal_lib_gc->mmchip.gc.direction_output	= spinal_lib_gpio_direction_output;
	spinal_lib_gc->mmchip.gc.get		= spinal_lib_gpio_get;
	spinal_lib_gc->mmchip.gc.set		= spinal_lib_gpio_set;
	spinal_lib_gc->mmchip.gc.owner		= THIS_MODULE;
	spinal_lib_gc->mmchip.gc.parent		= &pdev->dev;

	ret = of_mm_gpiochip_add_data(node, &spinal_lib_gc->mmchip, spinal_lib_gc);
	if (ret) {
		dev_err(&pdev->dev, "Failed adding memory mapped gpiochip\n");
		return ret;
	}

	platform_set_drvdata(pdev, spinal_lib_gc);

	writel(0, spinal_lib_gc->mmchip.regs + SPINAL_LIB_GPIO_OUTPUT);
	writel(0, spinal_lib_gc->mmchip.regs + SPINAL_LIB_GPIO_OUTPUT_ENABLE);

//	spinal_lib_gc->mapped_irq = platform_get_irq(pdev, 0);
//
//	if (spinal_lib_gc->mapped_irq < 0)
//		goto skip_irq;
//
//	if (of_property_read_u32(node, "altr,interrupt-type", &reg)) {
//		ret = -EINVAL;
//		dev_err(&pdev->dev,
//			"altr,interrupt-type value not set in device tree\n");
//		goto teardown;
//	}
//	spinal_lib_gc->interrupt_trigger = reg;
//
//	ret = gpiochip_irqchip_add(&spinal_lib_gc->mmchip.gc, &spinal_lib_irq_chip, 0,
//		handle_bad_irq, IRQ_TYPE_NONE);
//
//	if (ret) {
//		dev_err(&pdev->dev, "could not add irqchip\n");
//		goto teardown;
//	}
//
//	gpiochip_set_chained_irqchip(&spinal_lib_gc->mmchip.gc,
//		&spinal_lib_irq_chip,
//		spinal_lib_gc->mapped_irq,
//		spinal_lib_gc->interrupt_trigger == IRQ_TYPE_LEVEL_HIGH ?
//		spinal_lib_gpio_irq_leveL_high_handler :
//		spinal_lib_gpio_irq_edge_handler);

skip_irq:
	return 0;
teardown:
	of_mm_gpiochip_remove(&spinal_lib_gc->mmchip);
	pr_err("%pOF: registration failed with status %d\n",
		node, ret);

	return ret;
}

static int spinal_lib_gpio_remove(struct platform_device *pdev)
{
	struct spinal_lib_gpio_chip *spinal_lib_gc = platform_get_drvdata(pdev);

	of_mm_gpiochip_remove(&spinal_lib_gc->mmchip);

	return 0;
}

static const struct of_device_id spinal_lib_gpio_of_match[] = {
	{ .compatible = "spinal-lib,gpio-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, spinal_lib_gpio_of_match);

static struct platform_driver spinal_lib_gpio_driver = {
	.driver = {
		.name	= "spinal_lib_gpio",
		.of_match_table = of_match_ptr(spinal_lib_gpio_of_match),
	},
	.probe		= spinal_lib_gpio_probe,
	.remove		= spinal_lib_gpio_remove,
};

static int __init spinal_lib_gpio_init(void)
{
	return platform_driver_register(&spinal_lib_gpio_driver);
}
subsys_initcall(spinal_lib_gpio_init);

static void __exit spinal_lib_gpio_exit(void)
{
	platform_driver_unregister(&spinal_lib_gpio_driver);
}
module_exit(spinal_lib_gpio_exit);

MODULE_AUTHOR("Charles Papon <charles.papon.90@gmail.com>");
MODULE_DESCRIPTION("Spinal lib GPIO driver");
MODULE_LICENSE("GPL");

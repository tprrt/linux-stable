/*
 *  Freescale FlexTimer Module (FTM) PWM Driver
 *
 *  Copyright 2012-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define FTM_SC		0x00
#define FTM_SC_CLK_MASK_SHIFT	3
#define FTM_SC_CLK_MASK	(3 << FTM_SC_CLK_MASK_SHIFT)
#define FTM_SC_CLK(c)	(((c) + 1) << FTM_SC_CLK_MASK_SHIFT)
#define FTM_SC_PS_MASK	0x7

#define FTM_CNT		0x04
#define FTM_MOD		0x08

#define FTM_CSC_BASE	0x0C
#define FTM_CSC_CHIE	BIT(6)
#define FTM_CSC_MSB	BIT(5)
#define FTM_CSC_MSA	BIT(4)
#define FTM_CSC_ELSB	BIT(3)
#define FTM_CSC_ELSA	BIT(2)
#define FTM_CSC(_channel)	(FTM_CSC_BASE + ((_channel) * 8))

#define FTM_CV_BASE	0x10
#define FTM_CV(_channel)	(FTM_CV_BASE + ((_channel) * 8))

#define FTM_CNTIN	0x4C
#define FTM_STATUS	0x50

#define FTM_MODE	0x54
#define FTM_MODE_FTMEN	BIT(0)
#define FTM_MODE_INIT	BIT(2)
#define FTM_MODE_PWMSYNC	BIT(3)

#define FTM_SYNC	0x58
#define FTM_OUTINIT	0x5C
#define FTM_OUTMASK	0x60
#define FTM_COMBINE	0x64
#define FTM_DEADTIME	0x68
#define FTM_EXTTRIG	0x6C
#define FTM_POL		0x70
#define FTM_FMS		0x74
#define FTM_FILTER	0x78
#define FTM_FLTCTRL	0x7C
#define FTM_QDCTRL	0x80
#define FTM_CONF	0x84
#define FTM_FLTPOL	0x88
#define FTM_SYNCONF	0x8C
#define FTM_INVCTRL	0x90
#define FTM_SWOCTRL	0x94
#define FTM_PWMLOAD	0x98

enum fsl_pwm_clk {
	FSL_PWM_CLK_SYS,
	FSL_PWM_CLK_FIX,
	FSL_PWM_CLK_EXT,
	FSL_PWM_CLK_CNTEN,
	FSL_PWM_CLK_MAX
};

struct fsl_cpt_ddata {
	u32 snapshot[3];
	unsigned int index;
	struct mutex lock;
	wait_queue_head_t wait;
};

struct fsl_pwm_chip {
	struct pwm_chip chip;

	struct mutex lock;

	unsigned int cnt_select;
	unsigned int clk_ps;

	struct regmap *regmap;

	int period_ns;
	bool has_pwmen;
	struct pwm_device *cpt_dev;

	struct clk *ipg_clk;
	struct clk *clk[FSL_PWM_CLK_MAX];
};

static inline struct fsl_pwm_chip *to_fsl_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct fsl_pwm_chip, chip);
}

static inline int fsl_pwm_mode_enable(struct fsl_pwm_chip *fpc)
{
	if (!fpc)
		return -ENODEV;

	if (fpc->ipg_clk)
		clk_prepare_enable(fpc->ipg_clk);

	return clk_prepare_enable(fpc->clk[FSL_PWM_CLK_SYS]);
}

static inline void fsl_pwm_mode_disable(struct fsl_pwm_chip *fpc)
{
	if (!fpc)
		return;

	clk_disable_unprepare(fpc->clk[FSL_PWM_CLK_SYS]);
	if (fpc->ipg_clk)
		clk_disable_unprepare(fpc->ipg_clk);
}

static int fsl_pwm_request(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);

	return fsl_pwm_mode_enable(fpc);
}

static void fsl_pwm_free(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);

	fsl_pwm_mode_disable(fpc);
}

static int fsl_pwm_calculate_default_ps(struct fsl_pwm_chip *fpc,
					enum fsl_pwm_clk index)
{
	unsigned long sys_rate, cnt_rate;
	unsigned long long ratio;

	sys_rate = clk_get_rate(fpc->clk[FSL_PWM_CLK_SYS]);
	if (!sys_rate)
		return -EINVAL;

	cnt_rate = clk_get_rate(fpc->clk[fpc->cnt_select]);
	if (!cnt_rate)
		return -EINVAL;

	switch (index) {
	case FSL_PWM_CLK_SYS:
		fpc->clk_ps = 1;
		break;
	case FSL_PWM_CLK_FIX:
		ratio = 2 * cnt_rate - 1;
		do_div(ratio, sys_rate);
		fpc->clk_ps = ratio;
		break;
	case FSL_PWM_CLK_EXT:
		ratio = 4 * cnt_rate - 1;
		do_div(ratio, sys_rate);
		fpc->clk_ps = ratio;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static unsigned long fsl_pwm_calculate_cycles(struct fsl_pwm_chip *fpc,
					      unsigned long period_ns)
{
	unsigned long long c, c0;

	c = clk_get_rate(fpc->clk[fpc->cnt_select]);
	c = c * period_ns;
	do_div(c, 1000000000UL);

	do {
		c0 = c;
		do_div(c0, (1 << fpc->clk_ps));
		if (c0 <= 0xFFFF)
			return (unsigned long)c0;
	} while (++fpc->clk_ps < 8);

	return 0;
}

static unsigned long fsl_pwm_calculate_period_cycles(struct fsl_pwm_chip *fpc,
						     unsigned long period_ns,
						     enum fsl_pwm_clk index)
{
	int ret;

	ret = fsl_pwm_calculate_default_ps(fpc, index);
	if (ret) {
		dev_err(fpc->chip.dev,
			"failed to calculate default prescaler: %d\n",
			ret);
		return 0;
	}

	return fsl_pwm_calculate_cycles(fpc, period_ns);
}

static unsigned long fsl_pwm_calculate_period(struct fsl_pwm_chip *fpc,
					      unsigned long period_ns)
{
	enum fsl_pwm_clk m0, m1;
	unsigned long fix_rate, ext_rate, cycles;

	cycles = fsl_pwm_calculate_period_cycles(fpc, period_ns,
			FSL_PWM_CLK_SYS);
	if (cycles) {
		fpc->cnt_select = FSL_PWM_CLK_SYS;
		return cycles;
	}

	fix_rate = clk_get_rate(fpc->clk[FSL_PWM_CLK_FIX]);
	ext_rate = clk_get_rate(fpc->clk[FSL_PWM_CLK_EXT]);

	if (fix_rate > ext_rate) {
		m0 = FSL_PWM_CLK_FIX;
		m1 = FSL_PWM_CLK_EXT;
	} else {
		m0 = FSL_PWM_CLK_EXT;
		m1 = FSL_PWM_CLK_FIX;
	}

	cycles = fsl_pwm_calculate_period_cycles(fpc, period_ns, m0);
	if (cycles) {
		fpc->cnt_select = m0;
		return cycles;
	}

	fpc->cnt_select = m1;

	return fsl_pwm_calculate_period_cycles(fpc, period_ns, m1);
}

static unsigned long fsl_pwm_calculate_duty(struct fsl_pwm_chip *fpc,
					    unsigned long period_ns,
					    unsigned long duty_ns)
{
	unsigned long long duty;
	u32 val;

	regmap_read(fpc->regmap, FTM_MOD, &val);
	duty = (unsigned long long)duty_ns * (val + 1);
	do_div(duty, period_ns);

	return (unsigned long)duty;
}

static int fsl_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			  int duty_ns, int period_ns)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);
	u32 period, duty;

	mutex_lock(&fpc->lock);

	/*
	 * The Freescale FTM controller supports only a single period for
	 * all PWM channels, therefore incompatible changes need to be
	 * refused.
	 */
	if (fpc->period_ns && fpc->period_ns != period_ns) {
		dev_err(fpc->chip.dev,
			"conflicting period requested for PWM %u\n",
			pwm->hwpwm);
		mutex_unlock(&fpc->lock);
		return -EBUSY;
	}

	if (!fpc->period_ns && duty_ns) {
		period = fsl_pwm_calculate_period(fpc, period_ns);
		if (!period) {
			dev_err(fpc->chip.dev, "failed to calculate period\n");
			mutex_unlock(&fpc->lock);
			return -EINVAL;
		}

		regmap_update_bits(fpc->regmap, FTM_SC, FTM_SC_PS_MASK,
				   fpc->clk_ps);
		regmap_write(fpc->regmap, FTM_MOD, period - 1);
		fpc->period_ns = period_ns;
	}

	mutex_unlock(&fpc->lock);

	duty = fsl_pwm_calculate_duty(fpc, period_ns, duty_ns);

	regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm),
		     FTM_CSC_MSB | FTM_CSC_ELSB);
	regmap_write(fpc->regmap, FTM_CV(pwm->hwpwm), duty);

	return 0;
}

static int fsl_pwm_set_polarity(struct pwm_chip *chip,
				struct pwm_device *pwm,
				enum pwm_polarity polarity)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);
	u32 val;

	regmap_read(fpc->regmap, FTM_POL, &val);

	if (polarity == PWM_POLARITY_INVERSED)
		val |= BIT(pwm->hwpwm);
	else
		val &= ~BIT(pwm->hwpwm);

	regmap_write(fpc->regmap, FTM_POL, val);

	return 0;
}

static int fsl_counter_clock_enable(struct fsl_pwm_chip *fpc)
{
	int ret;

	/* select counter clock source */
	regmap_update_bits(fpc->regmap, FTM_SC, FTM_SC_CLK_MASK,
			   FTM_SC_CLK(fpc->cnt_select));

	ret = clk_prepare_enable(fpc->clk[fpc->cnt_select]);
	if (ret)
		return ret;

	ret = clk_prepare_enable(fpc->clk[FSL_PWM_CLK_CNTEN]);
	if (ret) {
		clk_disable_unprepare(fpc->clk[fpc->cnt_select]);
		return ret;
	}

	return 0;
}

static int fsl_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);
	int ret;

	mutex_lock(&fpc->lock);
	regmap_update_bits(fpc->regmap, FTM_OUTMASK, BIT(pwm->hwpwm), 0);
	if (fpc->has_pwmen)
		regmap_update_bits(fpc->regmap, FTM_SC,
				BIT(pwm->hwpwm + 16), BIT(pwm->hwpwm + 16));

	ret = fsl_counter_clock_enable(fpc);
	mutex_unlock(&fpc->lock);

	return ret;
}

static void fsl_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);
	u32 val;

	mutex_lock(&fpc->lock);

	if (fpc->has_pwmen)
		regmap_update_bits(fpc->regmap, FTM_SC, BIT(pwm->hwpwm + 16), 0);

	regmap_update_bits(fpc->regmap, FTM_OUTMASK, BIT(pwm->hwpwm),
			   BIT(pwm->hwpwm));

	clk_disable_unprepare(fpc->clk[FSL_PWM_CLK_CNTEN]);
	clk_disable_unprepare(fpc->clk[fpc->cnt_select]);

	regmap_read(fpc->regmap, FTM_OUTMASK, &val);
	if ((val & 0xFF) == 0xFF)
		fpc->period_ns = 0;

	mutex_unlock(&fpc->lock);
}

static int fsl_pwm_capture(struct pwm_chip *chip, struct pwm_device *pwm,
			   struct pwm_capture *result, unsigned long timeout)
{
	struct fsl_pwm_chip *fpc = to_fsl_chip(chip);
	struct fsl_cpt_ddata *ddata = pwm_get_chip_data(pwm);
	unsigned int effective_ticks;
	unsigned long long high, low;
	int ret;
	u32 val;

	fpc->cpt_dev = pwm;

	mutex_lock(&ddata->lock);
	ddata->index = 0;

	/* Initialize the counter */
	mutex_lock(&fpc->lock);
	regmap_update_bits(fpc->regmap, FTM_SC, FTM_SC_PS_MASK, 1);
	regmap_write(fpc->regmap, FTM_MOD, 0xFFFF);
	regmap_write(fpc->regmap, FTM_CNTIN, 0x0000);
	mutex_unlock(&fpc->lock);

	/* Select input capture mode */
	regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
	val &= ~(FTM_CSC_MSB | FTM_CSC_MSA);
	regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);

	/* Enable interrupt on rising edge only */
	regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
	val &= ~(FTM_CSC_ELSB);
	val |= FTM_CSC_ELSA;
	val |= FTM_CSC_CHIE;
	regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);

	//regmap_write(fpc->regmap, FTM_CV(pwm->hwpwm), 0);

	mutex_lock(&fpc->lock);
	/* Select counter clock source */
	regmap_update_bits(fpc->regmap, FTM_SC, FTM_SC_CLK_MASK,
			   FTM_SC_CLK(FSL_PWM_CLK_SYS));
	/* Enable capture */
	clk_prepare_enable(fpc->clk[FSL_PWM_CLK_SYS]);
	clk_prepare_enable(fpc->clk[FSL_PWM_CLK_CNTEN]);

	mutex_unlock(&fpc->lock);

	/* Wait */
	ret = wait_event_interruptible_timeout(ddata->wait, ddata->index > 1,
					       msecs_to_jiffies(timeout));

	/* Disable interrupt */
	regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
	val &= ~(FTM_CSC_ELSB | FTM_CSC_ELSA | FTM_CSC_CHIE);
	regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);

	if (ret == -ERESTARTSYS)
		goto out;

	/* Compute the result */
	switch (ddata->index) {
	case 0:
	case 1:
		/*
		 * Getting here could mean:
		 *  - input signal is constant of less than 1 Hz
		 *  - there is no input signal at all
		 *
		 * In such case the frequency is rounded down to 0
		 */
		result->period = 0;
		result->duty_cycle = 0;
		break;

	case 2:
		/* We have everying we need */
		high = ddata->snapshot[1] - ddata->snapshot[0];
		low = ddata->snapshot[2] - ddata->snapshot[1];

		effective_ticks = clk_get_rate(fpc->clk[fpc->cnt_select]);

		result->period = (high + low) * NSEC_PER_SEC;
		result->period /= effective_ticks;

		result->duty_cycle = high * NSEC_PER_SEC;
		result->duty_cycle /= effective_ticks;
		break;

	default:
		dev_err(fpc->chip.dev, "internal error\n");
		break;
	}

out:
	/* Disable capture */
	mutex_lock(&fpc->lock);
	clk_disable_unprepare(fpc->clk[FSL_PWM_CLK_CNTEN]);
	clk_disable_unprepare(fpc->clk[FSL_PWM_CLK_SYS]);
	mutex_unlock(&fpc->lock);

	mutex_unlock(&ddata->lock);
	return ret;
}

static const struct pwm_ops fsl_pwm_ops = {
	.capture = fsl_pwm_capture,
	.request = fsl_pwm_request,
	.free = fsl_pwm_free,
	.config = fsl_pwm_config,
	.set_polarity = fsl_pwm_set_polarity,
	.enable = fsl_pwm_enable,
	.disable = fsl_pwm_disable,
	.owner = THIS_MODULE,
};

static int fsl_pwm_init(struct fsl_pwm_chip *fpc)
{
	int ret;

	ret = fsl_pwm_mode_enable(fpc);
	if (ret)
		return ret;

	regmap_write(fpc->regmap, FTM_CNTIN, 0x00);
	regmap_write(fpc->regmap, FTM_OUTINIT, 0x00);
	regmap_write(fpc->regmap, FTM_OUTMASK, 0xFF);
	regmap_write(fpc->regmap, FTM_FILTER, 0x0000);

	fsl_pwm_mode_disable(fpc);

	return 0;
}

static bool fsl_pwm_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case FTM_CNT:
		return true;
	}
	return false;
}

static const struct regmap_config fsl_pwm_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,

	.max_register = FTM_PWMLOAD,
	.volatile_reg = fsl_pwm_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static irqreturn_t fsl_pwm_interrupt(int irq, void *data)
{
	struct fsl_pwm_chip *fpc = data;
	u32 val;

	struct pwm_device *pwm = fpc->cpt_dev;
	struct fsl_cpt_ddata *ddata = pwm_get_chip_data(pwm);

	/*
	 * Capture input:
	 *    _______                   _______
	 *   |       |                 |       |
	 * __|       |_________________|       |________
	 *   ^0      ^1                ^2
	 *
	 * Capture start by the first available rising edge. When a
	 * capture event occurs, capture value (FTMx_CnV) is stored,
	 * index incremented, capture edge changed.
	 *
	 * After the capture, if the index > 1, we have collected the
	 * necessary data so we signal the thread waiting for it and
	 * disable the capture by setting capture edge to none
	 */

	/* Read the counter value */
	regmap_read(fpc->regmap, FTM_CV(pwm->hwpwm), &ddata->snapshot[ddata->index]);
	regmap_read(fpc->regmap, FTM_CNT, &val);
	regmap_write(fpc->regmap, FTM_STATUS, 0x00);

	switch (ddata->index) {
	case 0:
		ddata->index++;
		/* Enable interrupt on failling edge only */
		regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
		val &= ~(FTM_CSC_ELSA);
		val |= FTM_CSC_ELSB;
		regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);
		break;

	case 1:
		ddata->index++;
		/* Enable interrupt on rising edge only */
		regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
		val &= ~(FTM_CSC_ELSB);
		val |= FTM_CSC_ELSA;
		regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);
		break;

	case 2:
		/* Disable interrupt */
		regmap_read(fpc->regmap, FTM_CSC(pwm->hwpwm), &val);
		val &= ~(FTM_CSC_ELSB | FTM_CSC_ELSA | FTM_CSC_CHIE);
		regmap_write(fpc->regmap, FTM_CSC(pwm->hwpwm), val);
		/* Wake up threads blocked */
		wake_up(&ddata->wait);
		break;

	default:
		dev_err(fpc->chip.dev, "internal error\n");
	}

	return IRQ_HANDLED;
}

static int fsl_pwm_probe(struct platform_device *pdev)
{
	struct fsl_pwm_chip *fpc;
	struct resource *res;
	void __iomem *base;
	int i, irq, ret;

	fpc = devm_kzalloc(&pdev->dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;

	mutex_init(&fpc->lock);

	fpc->chip.dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	fpc->regmap = devm_regmap_init_mmio_clk(&pdev->dev, "ftm_sys", base,
						&fsl_pwm_regmap_config);
	if (IS_ERR(fpc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(fpc->regmap);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to obtain IRQ\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, fsl_pwm_interrupt, 0,
			       pdev->name, fpc);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		return ret;
	}

	fpc->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(fpc->ipg_clk))
		fpc->ipg_clk = 0;

	fpc->clk[FSL_PWM_CLK_SYS] = devm_clk_get(&pdev->dev, "ftm_sys");

	if (IS_ERR(fpc->clk[FSL_PWM_CLK_SYS])) {
		dev_err(&pdev->dev, "failed to get \"ftm_sys\" clock\n");
		return PTR_ERR(fpc->clk[FSL_PWM_CLK_SYS]);
	}

	fpc->clk[FSL_PWM_CLK_FIX] = devm_clk_get(fpc->chip.dev, "ftm_fix");
	if (IS_ERR(fpc->clk[FSL_PWM_CLK_FIX]))
		return PTR_ERR(fpc->clk[FSL_PWM_CLK_FIX]);

	fpc->clk[FSL_PWM_CLK_EXT] = devm_clk_get(fpc->chip.dev, "ftm_ext");
	if (IS_ERR(fpc->clk[FSL_PWM_CLK_EXT]))
		return PTR_ERR(fpc->clk[FSL_PWM_CLK_EXT]);

	fpc->clk[FSL_PWM_CLK_CNTEN] =
				devm_clk_get(fpc->chip.dev, "ftm_cnt_clk_en");
	if (IS_ERR(fpc->clk[FSL_PWM_CLK_CNTEN]))
		return PTR_ERR(fpc->clk[FSL_PWM_CLK_CNTEN]);

	fpc->chip.ops = &fsl_pwm_ops;
	fpc->chip.of_xlate = of_pwm_xlate_with_flags;
	fpc->chip.of_pwm_n_cells = 3;
	fpc->chip.base = -1;
	fpc->chip.npwm = 8;
	fpc->has_pwmen = of_property_read_bool(pdev->dev.of_node,
						"ftm-has-pwmen-bits");

	ret = pwmchip_add(&fpc->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	for (i = 0; i < fpc->chip.npwm; i++) {
		struct fsl_cpt_ddata *ddata;

		ddata = devm_kzalloc(&pdev->dev, sizeof(*ddata), GFP_KERNEL);
		if (!ddata)
			return -ENOMEM;

		init_waitqueue_head(&ddata->wait);
		mutex_init(&ddata->lock);

		pwm_set_chip_data(&fpc->chip.pwms[i], ddata);
	}

	platform_set_drvdata(pdev, fpc);

	return fsl_pwm_init(fpc);
}

static int fsl_pwm_remove(struct platform_device *pdev)
{
	struct fsl_pwm_chip *fpc = platform_get_drvdata(pdev);

	return pwmchip_remove(&fpc->chip);
}

#ifdef CONFIG_PM_SLEEP
static int fsl_pwm_suspend(struct device *dev)
{
	struct fsl_pwm_chip *fpc = dev_get_drvdata(dev);
	int i;

	regcache_cache_only(fpc->regmap, true);
	regcache_mark_dirty(fpc->regmap);

	for (i = 0; i < fpc->chip.npwm; i++) {
		struct pwm_device *pwm = &fpc->chip.pwms[i];

		if (!test_bit(PWMF_REQUESTED, &pwm->flags))
			continue;

		fsl_pwm_mode_disable(fpc);

		if (!pwm_is_enabled(pwm))
			continue;

		clk_disable_unprepare(fpc->clk[FSL_PWM_CLK_CNTEN]);
		clk_disable_unprepare(fpc->clk[fpc->cnt_select]);
	}

	return 0;
}

static int fsl_pwm_resume(struct device *dev)
{
	struct fsl_pwm_chip *fpc = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < fpc->chip.npwm; i++) {
		struct pwm_device *pwm = &fpc->chip.pwms[i];

		if (!test_bit(PWMF_REQUESTED, &pwm->flags))
			continue;

		fsl_pwm_mode_enable(fpc);

		if (!pwm_is_enabled(pwm))
			continue;

		clk_prepare_enable(fpc->clk[fpc->cnt_select]);
		clk_prepare_enable(fpc->clk[FSL_PWM_CLK_CNTEN]);
	}

	/* restore all registers from cache */
	regcache_cache_only(fpc->regmap, false);
	regcache_sync(fpc->regmap);

	return 0;
}
#endif

static const struct dev_pm_ops fsl_pwm_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(fsl_pwm_suspend, fsl_pwm_resume)
};

static const struct of_device_id fsl_pwm_dt_ids[] = {
	{ .compatible = "fsl,vf610-ftm-pwm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fsl_pwm_dt_ids);

static struct platform_driver fsl_pwm_driver = {
	.driver = {
		.name = "fsl-ftm-pwm",
		.of_match_table = fsl_pwm_dt_ids,
		.pm = &fsl_pwm_pm_ops,
	},
	.probe = fsl_pwm_probe,
	.remove = fsl_pwm_remove,
};
module_platform_driver(fsl_pwm_driver);

MODULE_DESCRIPTION("Freescale FlexTimer Module PWM Driver");
MODULE_AUTHOR("Xiubo Li <Li.Xiubo@freescale.com>");
MODULE_ALIAS("platform:fsl-ftm-pwm");
MODULE_LICENSE("GPL");

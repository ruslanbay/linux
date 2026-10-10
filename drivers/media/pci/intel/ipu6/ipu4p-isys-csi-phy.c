// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2026 Intel Corporation */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/io.h>

#include "ipu6.h"
#include "ipu6-bus.h"
#include "ipu6-isys.h"
#include "ipu6-platform-buttress-regs.h"
#include "ipu4p-isys-csi2-regs.h"

struct ipu4p_phy_bb_config {
	u8 bb;
	u8 crc;
	u8 drc;
	u8 afe;
};

#define IPU4P_CSI2_GPREG_CR_PORT_CONFIG_DEFAULT	0x3895
#define IPU4P_CSI_BSCAN_EXCLUDE			(BIT(9) | BIT(18) | BIT(27))

static const struct ipu4p_phy_bb_config ipu4p_phy_bb_configs[] = {
	{ 4, 13, 32, 0x0f },
	{ 6, 13, 32, 0x15 },
	/* BB10 is part of the verified IPU4P PHY baseline. */
	{ 10, 13, 32, 0x15 },
	{ 12, 13, 32, 0x0f },
	{ 14, 13, 32, 0x15 },
};

static void ipu4p_isys_configure_phy_bb(struct ipu6_device *isp,
					const struct ipu4p_phy_bb_config *config)
{
	void __iomem *base = isp->base;
	u32 value;

	value = readl(base + IPU4P_BUTTRESS_REG_CPHYX_DLL_OVRD(config->bb));
	value &= ~GENMASK(6, 1);
	value |= FIELD_PREP(GENMASK(6, 1), config->crc) | BIT(0);
	writel(value, base + IPU4P_BUTTRESS_REG_CPHYX_DLL_OVRD(config->bb));

	value = readl(base + IPU4P_BUTTRESS_REG_DPHYX_DLL_OVRD(config->bb));
	value &= ~GENMASK(6, 1);
	value |= FIELD_PREP(GENMASK(6, 1), config->drc) | BIT(0);
	writel(value, base + IPU4P_BUTTRESS_REG_DPHYX_DLL_OVRD(config->bb));

	value = config->afe | FIELD_PREP(GENMASK(30, 29), 2);
	writel(value, base + IPU4P_BUTTRESS_REG_BBX_AFE_CONFIG(config->bb));
}

void ipu4p_isys_phy_setup(struct ipu6_isys *isys)
{
	struct ipu6_device *isp = isys->adev->isp;
	void __iomem *isys_base = isys->pdata->base;

	writel(IPU4P_CSI2_GPREG_CR_PORT_CONFIG_DEFAULT,
	       isys_base + IPU4P_ISYS_GPOFFSET +
	       IPU4P_CSI2_GPREG_CR_PORT_CONFIG);
	writel(IPU4P_CSI2_GPREG_CR_PORT_CONFIG_DEFAULT,
	       isys_base + IPU4P_ISYS_COMBO_GPOFFSET +
	       IPU4P_CSI2_GPREG_CR_PORT_CONFIG);
	writel(IPU4P_CSI_BSCAN_EXCLUDE,
	       isp->base + IPU4P_BUTTRESS_REG_CSI_BSCAN_EXCLUDE);

	for (unsigned int i = 0; i < ARRAY_SIZE(ipu4p_phy_bb_configs); i++)
		ipu4p_isys_configure_phy_bb(isp, &ipu4p_phy_bb_configs[i]);
}

/*
 * Use fixed timing observed in Windows for the SP7 front camera:
 * IPU4P CSI-2 port 2, firmware source 7, two-lane D-PHY.
 */
static void ipu4p_isys_apply_fixed_2lane_timing(struct ipu6_isys_csi2 *csi2)
{
	writel(0, csi2->base + IPU4P_CSI2_RX_DLY_CNT_SETTLE_CLANE);
	writel(1155, csi2->base + IPU4P_CSI2_RX_DLY_CNT_TERMEN_DLANE(0));

	for (unsigned int i = 0; i < 8; i++) {
		writel(0, csi2->base + IPU4P_CSI2_RX_DLY_CNT_SETTLE_DLANE(i));
		writel(1269,
		       csi2->base + IPU4P_CSI2_RX_DLY_CNT_TERMEN_DLANE(i + 1));
	}
}

static void ipu4p_isys_csi_irq_enable(struct ipu6_isys *isys,
				      unsigned int port)
{
	void __iomem *base = isys->pdata->base;
	u32 offset = IPU4P_ISYS_CSI_IRQ_CTRL_BASE(port);
	u32 offset0 = IPU4P_ISYS_CSI_IRQ_CTRL0_BASE(port);
	u32 mask = IPU4P_ISYS_CSI_ERROR_MASK;

	writel(IPU4P_ISYS_CSI_IRQ_ACTIVE,
	       base + offset);
	writel(0, base + offset + IPU4P_ISYS_IRQ_LEVEL_NOT_PULSE_OFFSET);
	writel(0xffffffff, base + offset + IPU4P_ISYS_IRQ_CLEAR_OFFSET);
	writel(IPU4P_ISYS_CSI_IRQ_ACTIVE,
	       base + offset + IPU4P_ISYS_IRQ_MASK_OFFSET);
	writel(IPU4P_ISYS_CSI_IRQ_ACTIVE,
	       base + offset + IPU4P_ISYS_IRQ_ENABLE_OFFSET);

	writel(mask, base + offset0);
	writel(0, base + offset0 + IPU4P_ISYS_IRQ_LEVEL_NOT_PULSE_OFFSET);
	writel(0xffffffff, base + offset0 + IPU4P_ISYS_IRQ_CLEAR_OFFSET);
	writel(mask, base + offset0 + IPU4P_ISYS_IRQ_MASK_OFFSET);
	writel(mask, base + offset0 + IPU4P_ISYS_IRQ_ENABLE_OFFSET);
}

static void ipu4p_isys_csi_irq_disable(struct ipu6_isys *isys,
				       unsigned int port)
{
	void __iomem *base = isys->pdata->base;
	u32 offset = IPU4P_ISYS_CSI_IRQ_CTRL_BASE(port);
	u32 offset0 = IPU4P_ISYS_CSI_IRQ_CTRL0_BASE(port);

	writel(0, base + offset + IPU4P_ISYS_IRQ_MASK_OFFSET);
	writel(0, base + offset + IPU4P_ISYS_IRQ_ENABLE_OFFSET);
	writel(0xffffffff, base + offset + IPU4P_ISYS_IRQ_CLEAR_OFFSET);

	writel(0, base + offset0 + IPU4P_ISYS_IRQ_MASK_OFFSET);
	writel(0, base + offset0 + IPU4P_ISYS_IRQ_ENABLE_OFFSET);
	writel(0xffffffff, base + offset0 + IPU4P_ISYS_IRQ_CLEAR_OFFSET);
}

/* IPU4P PHY building blocks remain configured until the ISYS power cycle. */
int ipu4p_isys_phy_set_power(struct ipu6_isys *isys,
			     struct ipu6_isys_csi2_config *cfg,
			     const struct ipu6_isys_csi2_timing *timing,
			     bool on)
{
	struct ipu6_isys_csi2 *csi2;
	void __iomem *base;
	u32 value;
	bool fixed_timing;

	if (!cfg || cfg->port >= isys->pdata->ipdata->csi2.nports)
		return -EINVAL;

	csi2 = &isys->csi2[cfg->port];
	base = csi2->base;

	if (!on) {
		value = readl(base + IPU4P_CSI2_RX_CONFIG);
		value &= ~(IPU4P_CSI2_RX_CONFIG_DISABLE_BYTE_CLK_GATING |
			   IPU4P_CSI2_RX_CONFIG_RELEASE_LP11);
		writel(value, base + IPU4P_CSI2_RX_CONFIG);
		writel(0, base + IPU4P_CSI2_RX_ENABLE);

		ipu4p_isys_csi_irq_disable(isys, cfg->port);

		return 0;
	}

	if (cfg->nlanes != 1 && cfg->nlanes != 2 && cfg->nlanes != 4)
		return -EINVAL;

	if (csi2->phy_mode != PHY_MODE_DPHY)
		return -EOPNOTSUPP;

	if (!timing)
		return -EINVAL;

	writel(timing->ctermen,
	       base + IPU4P_CSI2_RX_DLY_CNT_TERMEN_CLANE);

	fixed_timing = csi2->port == 2 && cfg->nlanes == 2 &&
		       csi2->asd.source ==
		       IPU6_FW_ISYS_STREAM_SRC_CSI2_PORT0 + 7;
	if (fixed_timing) {
		ipu4p_isys_apply_fixed_2lane_timing(csi2);
		dev_dbg(&isys->adev->auxdev.dev,
			"using fixed SP7 timing for CSI-2 port %u, source 7, two lanes\n",
			csi2->port);

		/* Windows configures the lane count before RX_CONFIG. */
		writel(cfg->nlanes, base + IPU4P_CSI2_RX_NOF_ENABLED_LANES);
	} else {
		writel(timing->csettle,
		       base + IPU4P_CSI2_RX_DLY_CNT_SETTLE_CLANE);

		for (unsigned int lane = 0; lane < cfg->nlanes; lane++) {
			writel(timing->dtermen,
			       base + IPU4P_CSI2_RX_DLY_CNT_TERMEN_DLANE(lane));
			writel(timing->dsettle,
			       base + IPU4P_CSI2_RX_DLY_CNT_SETTLE_DLANE(lane));
		}
	}

	value = readl(base + IPU4P_CSI2_RX_CONFIG);
	value |= IPU4P_CSI2_RX_CONFIG_DISABLE_BYTE_CLK_GATING |
		 IPU4P_CSI2_RX_CONFIG_RELEASE_LP11;
	writel(value, base + IPU4P_CSI2_RX_CONFIG);

	writel(cfg->nlanes, base + IPU4P_CSI2_RX_NOF_ENABLED_LANES);

	writel(0, base + IPU4P_CSI2_RX_ENABLE);

	ipu4p_isys_csi_irq_enable(isys, cfg->port);

	writel(IPU4P_CSI2_RX_ENABLE_ENABLE,
	       base + IPU4P_CSI2_RX_ENABLE);

	return 0;
}

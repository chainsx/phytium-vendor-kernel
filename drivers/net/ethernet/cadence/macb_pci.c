// SPDX-License-Identifier: GPL-2.0-only
/**
 * DOC: Cadence GEM PCI wrapper.
 *
 * Copyright (C) 2016 Cadence Design Systems - https://www.cadence.com
 *
 * Authors: Rafal Ozieblo <rafalo@cadence.com>
 *	    Bartosz Folta <bfolta@cadence.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include "macb.h"

#define PCI_DRIVER_NAME "macb_pci"
#define PLAT_DRIVER_NAME "macb"

#define CDNS_VENDOR_ID 0x17cd
#define CDNS_DEVICE_ID 0xe007
#define PCI_DEVICE_ID_GMAC_3P0 0xdc3b
#define PCI_SUBDEVICE_ID_SGMII				0x1000
#define PCI_SUBDEVICE_ID_1000BASEX			0x1001
#define PCI_SUBDEVICE_ID_USXGMII			0x1004
#define PCI_SUBDEVICE_ID_10GBASER			0x1005

#define GEM_PCLK_RATE 50000000
#define GEM_HCLK_RATE 50000000
#define GEM_TXCLK_RATE 25000000
#define GEM_RXCLK_RATE 25000000
#define GEM_TSUCLK_RATE 300000000

static const u32 fixedlink[][5] = {
	{0, 1, 1000, 1, 0},
	{0, 1, 2500, 1, 0},
	{0, 1, 5000, 1, 0},
	{0, 1, 10000, 1, 0},
};

static const struct property_entry fl_properties[][2] = {
	{PROPERTY_ENTRY_U32_ARRAY("fixed-link", fixedlink[0]), {}},
	{PROPERTY_ENTRY_U32_ARRAY("fixed-link", fixedlink[1]), {}},
	{PROPERTY_ENTRY_U32_ARRAY("fixed-link", fixedlink[2]), {}},
	{PROPERTY_ENTRY_U32_ARRAY("fixed-link", fixedlink[3]), {}},
};

static const struct phytium_platform_pdata phytium_sgmii_pdata = {
	.phytium_dev_type = PHYTIUM_DEV_3P0,
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP |
			MACB_CAPS_BD_RD_PREFETCH |
			MACB_CAPS_USRIO_DISABLED |
			MACB_CAPS_TAILPTR,
	.phy_interface = PHY_INTERFACE_MODE_SGMII,
};

static const struct phytium_platform_pdata phytium_1000basex_pdata = {
	.phytium_dev_type = PHYTIUM_DEV_3P0,
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP |
			MACB_CAPS_BD_RD_PREFETCH |
			MACB_CAPS_USRIO_DISABLED |
			MACB_CAPS_TAILPTR,
	.phy_interface = PHY_INTERFACE_MODE_SGMII,
	.properties = fl_properties[0],
};

static const struct phytium_platform_pdata phytium_usxgmii_pdata = {
	.phytium_dev_type = PHYTIUM_DEV_3P0,
	.caps = MACB_CAPS_GIGABIT_MODE_AVAILABLE |
			MACB_CAPS_JUMBO |
			MACB_CAPS_GEM_HAS_PTP |
			MACB_CAPS_BD_RD_PREFETCH |
			MACB_CAPS_USRIO_DISABLED |
			MACB_CAPS_TAILPTR,
	.phy_interface = PHY_INTERFACE_MODE_USXGMII,
	.properties = fl_properties[3],
};

static int phytium_macb_pci_init(struct pci_dev *pdev, struct macb_platform_data *plat_data,
				 struct platform_device_info *plat_info,
				 struct phytium_platform_pdata *phytium_data)
{
	int i;
	int err;
	char clkname[20];

	err = pci_alloc_irq_vectors(pdev, 4, 4, PCI_IRQ_MSI);
	if (err < 0) {
		dev_err(&pdev->dev, "err=%d, fialed to allocate MSI entry", err);
		plat_data->phytium_macb_pdata.irq_type = IRQ_TYPE_INTX;
		plat_data->phytium_macb_pdata.irq[0] = pdev->irq;

	} else {
		plat_data->phytium_macb_pdata.irq_type = IRQ_TYPE_MSI;
		for (i = 0; i < 4; i++)
			plat_data->phytium_macb_pdata.irq[i] = pci_irq_vector(pdev, i);
	}

	plat_data->phytium_macb_pdata.phytium_dev_type = phytium_data->phytium_dev_type;
	plat_data->phytium_macb_pdata.caps = phytium_data->caps;
	plat_data->phytium_macb_pdata.phy_interface = phytium_data->phy_interface;
	if (phytium_data && phytium_data->properties) {
		plat_info->fwnode = NULL;
		plat_info->properties = phytium_data->properties;
		plat_data->phytium_macb_pdata.properties = phytium_data->properties;
	}

	snprintf(clkname, 20, "txclk:%02x", plat_info->id);
	plat_data->phytium_macb_pdata.txclk =
		clk_register_fixed_rate(&pdev->dev, clkname, NULL, 0, GEM_TXCLK_RATE);
	if (IS_ERR(plat_data->phytium_macb_pdata.txclk)) {
		err = PTR_ERR(plat_data->phytium_macb_pdata.txclk);
		goto err_txclk_register;
	}

	snprintf(clkname, 20, "rxclk:%02x", plat_info->id);
	plat_data->phytium_macb_pdata.rxclk =
		clk_register_fixed_rate(&pdev->dev, clkname, NULL, 0, GEM_RXCLK_RATE);
	if (IS_ERR(plat_data->phytium_macb_pdata.rxclk)) {
		err = PTR_ERR(plat_data->phytium_macb_pdata.rxclk);
		goto err_rxclk_register;
	}

	snprintf(clkname, 20, "tsuclk:%02x", plat_info->id);
	plat_data->phytium_macb_pdata.tsu_clk =
		clk_register_fixed_rate(&pdev->dev, clkname, NULL, 0, GEM_TSUCLK_RATE);
	if (IS_ERR(plat_data->phytium_macb_pdata.tsu_clk)) {
		err = PTR_ERR(plat_data->phytium_macb_pdata.tsu_clk);
		goto err_tsuclk_register;
	}

	return 0;

err_tsuclk_register:
	clk_unregister(plat_data->phytium_macb_pdata.rxclk);

err_rxclk_register:
	clk_unregister(plat_data->phytium_macb_pdata.txclk);

err_txclk_register:

	return err;
}

static void phytium_macb_pci_uninit(struct pci_dev *pdev)
{
	struct platform_device *plat_dev = pci_get_drvdata(pdev);
	struct macb_platform_data *plat_data = dev_get_platdata(&plat_dev->dev);

	plat_dev->dev.dma_ops = NULL;
	plat_dev->dev.iommu = NULL;
	plat_dev->dev.iommu_group = NULL;
	plat_dev->dev.dma_range_map = NULL;

	clk_unregister(plat_data->phytium_macb_pdata.txclk);
	clk_unregister(plat_data->phytium_macb_pdata.rxclk);
	clk_unregister(plat_data->phytium_macb_pdata.tsu_clk);

	if (plat_data->phytium_macb_pdata.properties) {
		struct fwnode_handle *fw_node = dev_fwnode(&plat_dev->dev);

		if (fw_node)
			fwnode_remove_software_node(fw_node);
		fw_node = NULL;
	}
}

static int macb_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int err;
	struct platform_device *plat_dev;
	struct platform_device_info plat_info;
	struct phytium_platform_pdata *phytium_data = NULL;
	struct macb_platform_data plat_data;
	struct resource res[2];
	char pclk_name[20] = "pclk";
	char hclk_name[20] = "hclk";

	/* enable pci device */
	err = pcim_enable_device(pdev);
	if (err < 0) {
		dev_err(&pdev->dev, "Enabling PCI device has failed: %d", err);
		return err;
	}

	pci_set_master(pdev);

	/* set up resources */
	memset(res, 0x00, sizeof(struct resource) * ARRAY_SIZE(res));
	res[0].start = pci_resource_start(pdev, 0);
	res[0].end = pci_resource_end(pdev, 0);
	res[0].name = PCI_DRIVER_NAME;
	res[0].flags = IORESOURCE_MEM;

	dev_info(&pdev->dev, "EMAC physical base addr: %pa\n",
		 &res[0].start);

	/* set up macb platform data */
	memset(&plat_data, 0, sizeof(plat_data));

	/* set up platform device info */
	memset(&plat_info, 0, sizeof(plat_info));
	plat_info.parent = &pdev->dev;
	plat_info.fwnode = pdev->dev.fwnode;
	plat_info.name = PLAT_DRIVER_NAME;
	plat_info.id = pdev->devfn;
	plat_info.res = res;
	plat_info.num_res = ARRAY_SIZE(res);
	plat_info.data = &plat_data;
	plat_info.size_data = sizeof(plat_data);
	plat_info.dma_mask = pdev->dma_mask;
	if (pdev->vendor == PCI_VENDOR_ID_PHYTIUM)
		phytium_data = (struct phytium_platform_pdata *)id->driver_data;

	/* initialize clocks */
	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0) {
		plat_info.id = (pdev->bus->number << 8) | pdev->devfn;
		snprintf(pclk_name, 20, "pclk:%02x", plat_info.id);
		snprintf(hclk_name, 20, "hclk:%02x", plat_info.id);
	}
	plat_data.pclk = clk_register_fixed_rate(&pdev->dev, pclk_name, NULL, 0,
						 GEM_PCLK_RATE);
	if (IS_ERR(plat_data.pclk)) {
		err = PTR_ERR(plat_data.pclk);
		goto err_pclk_register;
	}

	plat_data.hclk = clk_register_fixed_rate(&pdev->dev, hclk_name, NULL, 0,
						 GEM_HCLK_RATE);
	if (IS_ERR(plat_data.hclk)) {
		err = PTR_ERR(plat_data.hclk);
		goto err_hclk_register;
	}

	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0) {
		if (phytium_macb_pci_init(pdev, &plat_data, &plat_info, phytium_data))
			goto err_phytium_clk_register;
	}

	res[1].start = pci_irq_vector(pdev, 0);
	res[1].name = PCI_DRIVER_NAME;
	res[1].flags = IORESOURCE_IRQ;

	/* register platform device */
	plat_dev = platform_device_register_full(&plat_info);
	if (IS_ERR(plat_dev)) {
		err = PTR_ERR(plat_dev);
		goto err_plat_dev_register;
	}

	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0) {
		plat_dev->dev.dma_ops = (&pdev->dev)->dma_ops;
		plat_dev->dev.iommu = (&pdev->dev)->iommu;
		plat_dev->dev.iommu_group = (&pdev->dev)->iommu_group;
		plat_dev->dev.dma_range_map = (&pdev->dev)->dma_range_map;
	}
	pci_set_drvdata(pdev, plat_dev);

	return 0;

err_plat_dev_register:
	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0)
		clk_unregister(plat_data.phytium_macb_pdata.tsu_clk);

err_phytium_clk_register:
	clk_unregister(plat_data.hclk);

err_hclk_register:
	clk_unregister(plat_data.pclk);

err_pclk_register:
	return err;
}

static void macb_remove(struct pci_dev *pdev)
{
	struct platform_device *plat_dev = pci_get_drvdata(pdev);
	struct macb_platform_data *plat_data = dev_get_platdata(&plat_dev->dev);

	clk_unregister(plat_data->pclk);
	clk_unregister(plat_data->hclk);

	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0)
		phytium_macb_pci_uninit(pdev);
	platform_device_unregister(plat_dev);

	if (pdev->device == PCI_DEVICE_ID_GMAC_3P0)
		pci_free_irq_vectors(pdev);
}

static const struct pci_device_id dev_id_table[] = {
	{ PCI_DEVICE(CDNS_VENDOR_ID, CDNS_DEVICE_ID), },
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_PHYTIUM, PCI_DEVICE_ID_GMAC_3P0,
				PCI_VENDOR_ID_PHYTIUM, PCI_SUBDEVICE_ID_SGMII),
		.driver_data = (kernel_ulong_t)&phytium_sgmii_pdata},
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_PHYTIUM, PCI_DEVICE_ID_GMAC_3P0,
				PCI_VENDOR_ID_PHYTIUM, PCI_SUBDEVICE_ID_1000BASEX),
		.driver_data = (kernel_ulong_t)&phytium_1000basex_pdata},
	{ PCI_DEVICE_SUB(PCI_VENDOR_ID_PHYTIUM, PCI_DEVICE_ID_GMAC_3P0,
				PCI_VENDOR_ID_PHYTIUM, PCI_SUBDEVICE_ID_USXGMII),
		.driver_data = (kernel_ulong_t)&phytium_usxgmii_pdata},
	{ 0, }
};

static struct pci_driver macb_pci_driver = {
	.name     = PCI_DRIVER_NAME,
	.id_table = dev_id_table,
	.probe    = macb_probe,
	.remove	  = macb_remove,
};

module_pci_driver(macb_pci_driver);
MODULE_DEVICE_TABLE(pci, dev_id_table);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cadence NIC PCI wrapper");

/*
 * Homogeneous Remote Processor Control Driver
 *
 * Copyright (C) 2022 Phytium Technology Co., Ltd. - All Rights Reserved
 * Author: Shaojun Yang <yangshaojun@phytium.com.cn>
 *
 * This program is free software; you can redistribute it and/or modify it under the terms
 * of the GNU General Public License version 2 as published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/remoteproc.h>
#include <linux/arm-smccc.h>
#include <linux/kthread.h>
#include <linux/platform_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/delay.h>
#include <asm/cacheflush.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <linux/kallsyms.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <asm/arch_gicv3.h>
#include <linux/cpu.h>

#include <asm/smp_plat.h>
#include <linux/device.h>
#include <linux/psci.h>

#include "remoteproc_internal.h"

#define RPROC_RESOURCE_ENTRIES      8

#define PSCI_VERSION                0x84000000
#define CPU_SUSPEND                 0xc4000001
#define CPU_OFF                     0x84000002
#define CPU_ON                      0xc4000003
#define AFFINITY_INFO               0xc4000004
#define MIGRATE                     0xc4000005

static typeof(ioremap_page_range) *ioremap_page_range_sym;
static typeof(__flush_dcache_area) *__flush_dcache_area_sym;

/* Resource table for the homo remote processors */
struct homo_resource_table {
	unsigned int version;
	unsigned int num;
	unsigned int reserved[2];
	unsigned int offset[RPROC_RESOURCE_ENTRIES];

	/* Note: linux kenrel 'struct fw_rsc_vdev' has no 'type' field, here add to align data structre */
	uint32_t type;
	/* rpmsg vdev entry */
	struct fw_rsc_vdev rpmsg_vdev;
	struct fw_rsc_vdev_vring rpmsg_vring0;
	struct fw_rsc_vdev_vring rpmsg_vring1;
};

struct homo_rproc {
	struct rproc *rproc;
	struct homo_resource_table *rsc;

	u64 phys_addr;
	void *addr;
	u64 size;

	int irq;
	int cpu;
};

static struct homo_rproc *g_priv;
static struct work_struct workqueue;

static unsigned int cpuid = 3;
module_param(cpuid, uint, 0);
MODULE_PARM_DESC(cpuid, "Cpu logical number used exclusively by the remote processor. default is 3.");

static unsigned int sgi = 9;
module_param(sgi, uint, 0);
MODULE_PARM_DESC(sgi, "GIC SGI interrupt number for communication with remote processor. default is 9");

#define MPIDR_TO_SGI_AFFINITY(cluster_id, level)        (MPIDR_AFFINITY_LEVEL(cluster_id, level) << ICC_SGI1R_AFFINITY_## level ## _SHIFT)

void gicv3_ipi_send_single(int irq, u64 mpidr)
{
	u16 tlist = 0;
	u64 cluster_id;
	u64 sgi1r;

	/* Ensure stores to Normal memory are visible to other CPUs before sending the IPI. */
	wmb();

	cluster_id = mpidr & ~0xffUL;
	tlist |= 1 << (mpidr & 0xf);

	/* Send the IPIs for the target list of this cluster */
	sgi1r = (MPIDR_TO_SGI_AFFINITY(cluster_id, 3) |
			MPIDR_TO_SGI_AFFINITY(cluster_id, 2) |
			irq << 24 |
			MPIDR_TO_SGI_AFFINITY(cluster_id, 1) | tlist);
	gic_write_sgi1r(sgi1r);

	/* Force the above writes to ICC_SGI1R_EL1 to be executed */
	isb();
}

static void homo_rproc_vq_irq(struct work_struct *work)
{
	struct homo_rproc *priv = g_priv;
	struct homo_resource_table *rsc = priv->rsc;
	struct rproc *rproc = priv->rproc;

	rproc_vq_interrupt(rproc, rsc->rpmsg_vring0.notifyid);
}

static void homo_rproc_interrupt(void)
{
	schedule_work(&workqueue);
}

static int homo_rproc_start(struct rproc *rproc)
{
	int err;
	struct homo_rproc *priv = rproc->priv;
	int phys_cpuid = cpu_logical_map(priv->cpu);
	struct arm_smccc_res smc_res;

	err = psci_ops.affinity_info(phys_cpuid, 0);
	if (err == 0)
		cpu_down(priv->cpu);

	INIT_WORK(&workqueue, homo_rproc_vq_irq);

	priv->rsc = (struct homo_resource_table *)rproc->table_ptr;

	/* ARMv8 requires to clean D-cache and invalidate I-cache for memory containing new instructions. */
	flush_icache_range((unsigned long)priv->addr, (unsigned long)(priv->addr + priv->size));

	arm_smccc_smc(CPU_ON, phys_cpuid, (unsigned long long)priv->phys_addr, 0, 0, 0, 0, 0, &smc_res);

	return smc_res.a0;
}

static int homo_rproc_stop(struct rproc *rproc)
{
	int err;
	struct homo_rproc *priv = rproc->priv;

	err = psci_ops.affinity_info(cpu_logical_map(priv->cpu), 0);
	if (err == 1)
		cpu_up(priv->cpu);

	return 0;
}

static void *homo_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct homo_rproc *priv = rproc->priv;

	return priv->addr + (da - rproc->bootaddr);
}

static void homo_rproc_kick(struct rproc *rproc, int vqid)
{
	struct homo_rproc *priv = rproc->priv;

	if (rproc->state == RPROC_RUNNING)
		gicv3_ipi_send_single(priv->irq, cpu_logical_map(priv->cpu));

	return ;
}

static const struct rproc_ops homo_rproc_ops = {
	.start = homo_rproc_start,
	.stop = homo_rproc_stop,
	.kick = homo_rproc_kick,
	.da_to_va = homo_rproc_da_to_va,
};

static void *homo_rproc_ioremap(phys_addr_t phys,
		unsigned long virt, unsigned long size)
{
	struct vm_struct *vma;

	size = PAGE_ALIGN(size);

	if (virt)
		vma = __get_vm_area(size, VM_IOREMAP, virt, virt + size + PAGE_SIZE);
	else
		vma = __get_vm_area(size, VM_IOREMAP, VMALLOC_START, VMALLOC_END);
	if (!vma)
		return NULL;

	vma->phys_addr = phys;

	if (ioremap_page_range_sym((unsigned long)vma->addr, (unsigned long)vma->addr + size, phys, PAGE_KERNEL_EXEC)) {
		vunmap(vma->addr);
		return NULL;
	}

	return vma->addr;
}

static int homo_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *np_mem;
	struct rproc *rproc;
	struct homo_rproc *priv;
	u32 phandle;
	int ret;

	rproc = rproc_alloc(dev, np->name, &homo_rproc_ops,
			"openamp_core0.elf", sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rproc);

	priv = g_priv = rproc->priv;
	priv->rproc = rproc;

	/* Lookup kernel symbol that is not exported out. */
	ioremap_page_range_sym = (void *)kallsyms_lookup_name("ioremap_page_range");
	if (ioremap_page_range_sym == NULL) {
		dev_err(dev, "Symbol 'ioremap_page_range' not found.\n");
		return -1;
	}

	__flush_dcache_area_sym = (void *)kallsyms_lookup_name("__flush_dcache_area");
	if (__flush_dcache_area_sym == NULL) {
		dev_err(dev, "Symbol '__flush_dcache_area' not found.\n");
		return -1;
	}

	ret = of_property_read_u32(np, "memory-region", &phandle);
	if (ret) {
		dev_err(dev, "Can't find memory-region for Baremetal\n");
		return ret;
	}

	np_mem = of_find_node_by_phandle(phandle);
	if (!np_mem) {
		dev_err(dev, "Cant' find corresponding memory-region\n");
		return -EINVAL;
	} else {
		int n = of_property_count_elems_of_size(np_mem, "reg", sizeof(u64));

		if (n != 2) {
			dev_err(dev, "Memory address and size not found in devicetree!\n");
			return -EINVAL;
		}

		of_property_read_u64_index(np_mem, "reg", 0, &priv->phys_addr);
		of_property_read_u64_index(np_mem, "reg", 1, &priv->size);

		dev_info(dev, "Baremetal Address: %llx, size: %llx\n",
				priv->phys_addr, priv->size);
	}

	priv->rsc = NULL;
	priv->addr = NULL;

	/* The following values can be modified through module parameters */
	priv->irq = sgi;
	priv->cpu = cpuid;

	/* Map physical memory region reserved for homo remote processor. */
	priv->addr = homo_rproc_ioremap(priv->phys_addr, 0, priv->size);
	if (!priv->addr) {
		dev_err(dev, "ioremap failed\n");
		return -1;
	}
	dev_info(dev, "ioremap: phys_addr = %016llx, addr = %llx, size = %lld\n",
			priv->phys_addr, (u64)(priv->addr), priv->size);

	rproc_set_handle_irq(homo_rproc_interrupt);

	rproc->auto_boot = false;
	rproc->has_iommu = false;

	rproc_add(rproc);

	return 0;
}

static int homo_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct homo_rproc *priv = rproc->priv;

	rproc_del(rproc);
	of_reserved_mem_device_release(&pdev->dev);
	rproc_free(rproc);

	if (priv->addr) {
		vunmap(priv->addr);
		priv->addr = NULL;
	}

	return 0;
}

static const struct of_device_id homo_rproc_ids[] = {
	{ .compatible = "phytium,rproc", },
	{ }
};
MODULE_DEVICE_TABLE(of, homo_rproc_ids);

static struct platform_driver homo_rproc_driver = {
	.probe = homo_rproc_probe,
	.remove = homo_rproc_remove,
	.driver = {
		.name = "homo-rproc",
		.of_match_table = of_match_ptr(homo_rproc_ids),
	},
};
module_platform_driver(homo_rproc_driver);

MODULE_DESCRIPTION("Homogeneous Remote Processor Control Driver");
MODULE_AUTHOR("Shaojun Yang <yangshaojun@phytium.com.cn>");
MODULE_LICENSE("GPL v2");

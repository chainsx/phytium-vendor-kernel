dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-demo-ddr4.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-demo-ddr4-local.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2202-demo-ddr4.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2202-demo-ddr4-local.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2201-demo-ddr4.dtb

dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-miniitx-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2202-miniitx-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-vpx-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-edu-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-come-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2202-power-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2204-hanwei-board.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += phytiumpi_firefly.dtb
dtb-$(CONFIG_ARCH_PHYTIUM) += pe2202-chillipi-edu-board.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb

#Enable support for device-tree overlay
DTC_FLAGS_pe2204-come-board += -@
DTC_FLAGS_pe2204-demo-ddr4 += -@
DTC_FLAGS_pe2204-edu-board += -@
DTC_FLAGS_pe2204-hanwei-board += -@
DTC_FLAGS_pe2204-miniitx-board += -@
DTC_FLAGS_pe2204-vpx-board += -@

.PHONY: menuconfig

ROOT_LOC := $(CURDIR)
ROOT_KCONFIG := $(ROOT_LOC)/Kconfig
KCONFIG_CONFIG_LOC := $(ROOT_LOC)/configs
DEFCONFIG_FILE := $(KCONFIG_CONFIG_LOC)/dpp_defconfig
KCONFIG_CONFIG := $(KCONFIG_CONFIG_LOC)/.config
KCONFIG_CONFIG_OUTPUT_LOC := $(KCONFIG_CONFIG_LOC)/generated
export KCONFIG_CONFIG

menuconfig:
	menuconfig $(ROOT_KCONFIG)
	cd $(KCONFIG_CONFIG_OUTPUT_LOC) && genconfig $(ROOT_KCONFIG) && mv config.h autoconf.h

savedefconfig:
	cp $(KCONFIG_CONFIG_LOC)/.config $(DEFCONFIG_FILE)

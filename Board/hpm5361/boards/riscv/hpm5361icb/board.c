/*
 * Copyright (c) 2026 HPMicro
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <hpm_common.h>
#include <hpm_iomux.h>
#include <hpm_soc.h>

#if defined(CONFIG_HPM_BOOT_HEADER)
/* BootROM reads this block from the NOR flash offset 0x400. */
__attribute__((section(".nor_cfg_option"), used, aligned(4)))
const uint32_t hpm5361_nor_cfg_option[4] = {
	DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_hdr),
	DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_opt0),
	DT_PROP(DT_CHOSEN(zephyr_flash), nor_cfg_opt_opt1),
	0U,
};
#endif

/* Keep the debugger pins in their dedicated function before Zephyr starts. */
void c_startup(void)
{
	HPM_IOC->PAD[IOC_PAD_PA04].FUNC_CTL = IOC_PA04_FUNC_CTL_JTAG_TDO;
	HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_JTAG_TDI;
	HPM_IOC->PAD[IOC_PAD_PA06].FUNC_CTL = IOC_PA06_FUNC_CTL_JTAG_TCK;
	HPM_IOC->PAD[IOC_PAD_PA07].FUNC_CTL = IOC_PA07_FUNC_CTL_JTAG_TMS;
	HPM_IOC->PAD[IOC_PAD_PA08].FUNC_CTL = IOC_PA08_FUNC_CTL_JTAG_TRST;
}

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	HPM_PPOR->RESET_ENABLE = (1UL << 31);
	HPM_PPOR->SOFTWARE_RESET = 1000U;
	while (1) {
	}
}

/*
 * Copyright (C) 2009 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2009 PetaLogix
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <asm/prom.h>
#include <microblaze_fsl.h>

#ifdef CONFIG_PPC
#  define SYNCHRONIZE_IO __asm__ volatile ("eieio") /* should be 'mbar' ultimately */
#else
#  define SYNCHRONIZE_IO
#endif
static inline u32 in_32(u32 *InputPtr) { u32 v = (*(volatile u32 *)(InputPtr)); SYNCHRONIZE_IO; return v; }
static inline void out_32(u32 *OutputPtr, u32 Value) { (*(volatile u32 *)(OutputPtr) = Value); SYNCHRONIZE_IO; return; }

/* Trigger specific functions */
#ifdef CONFIG_GPIOLIB

#include <linux/of_gpio.h>

static int handle; /* reset pin handle */
static unsigned int reset_val;

static int of_reset_gpio_handle(void)
{
	int ret; /* variable which stored handle reset gpio pin */
	struct device_node *root; /* root node */
	struct device_node *gpio; /* gpio node */
	struct of_gpio_chip *of_gc = NULL;
	enum of_gpio_flags flags ;
	const void *gpio_spec;

	/* find out root node */
	root = of_find_node_by_path("/");

	/* give me handle for gpio node to be possible allocate pin */
	ret = of_parse_phandles_with_args(root, "hard-reset-gpios",
				"#gpio-cells", 0, &gpio, &gpio_spec);
	if (ret) {
		pr_debug("%s: can't parse gpios property\n", __func__);
		goto err0;
	}

	of_gc = gpio->data;
	if (!of_gc) {
		pr_debug("%s: gpio controller %s isn't registered\n",
			 root->full_name, gpio->full_name);
		ret = -ENODEV;
		goto err1;
	}

	ret = of_gc->xlate(of_gc, root, gpio_spec, &flags);
	if (ret < 0)
		goto err1;

	ret += of_gc->gc.base;
err1:
	of_node_put(gpio);
err0:
	pr_debug("%s exited with status %d\n", __func__, ret);
	return ret;
}

void of_platform_reset_gpio_probe(void)
{
	int ret;
	handle = of_reset_gpio_handle();

	if (!gpio_is_valid(handle)) {
		printk(KERN_INFO "Skipping unavailable RESET gpio %d (%s)\n",
				handle, "reset");
	}

	ret = gpio_request(handle, "reset");
	if (ret < 0) {
		printk(KERN_INFO "GPIO pin is already allocated\n");
		return;
	}

	/* get current setup value */
	reset_val = gpio_get_value(handle);
	/* FIXME maybe worth to perform any action */
	pr_debug("Reset: Gpio output state: 0x%x\n", reset_val);

	/* Setup GPIO as output */
	ret = gpio_direction_output(handle, 0);
	if (ret < 0)
		goto err;

	/* Setup output direction */
	gpio_set_value(handle, 0);

	printk(KERN_INFO "RESET: Registered gpio device: %d, current val: %d\n",
							handle, reset_val);
	return;
err:
	gpio_free(handle);
	return;
}


static void gpio_system_reset(void)
{
	gpio_set_value(handle, 1 - reset_val);
}
#else
#define gpio_system_reset() do {} while (0)
void of_platform_reset_gpio_probe(void)
{
	return;
}
#endif

static void icap_reset(u32 gp5)
{
	// ICAP behavior is described (poorly) in Xilinx specification UG380.  Brave
	// souls may look there for detailed guidance on what is being done here.
#if 1
	// It has been empirically determined that ICAP FSL doesn't always work
	// the first time, but if retried enough times it does eventually work.
	// Thus we keep hammering the operation we want and checking for failure
	// until we finally succeed.  Somebody please fix ICAP!! <sigh>
	u32 val;
	// Abort anything in progress
	do {
		putfslx(0x0FFFF, 0, FSL_CONTROL); // Control signal aborts, NOP doesn't matter
	    udelay(1000);
	    getfsl(val, 0); // Read the ICAP result
	} while ((val & ICAP_FSL_FAILED) != 0);

	// Synchronize command bytes
	do {
		putfsl(0x0FFFF, 0); // Pad words
		putfsl(0x0FFFF, 0);
		putfsl(0x0AA99, 0); // SYNC
		putfsl(0x05566, 0); // SYNC

        // Set the Mode register so that fallback images will be manipulated
        // correctly.  Use bitstream mode instead of physical mode (required
        // for configuration fallback) and set boot mode for BPI
        putfsl(0x03301, 0); // Write MODE_REG
        putfsl(0x02000, 0); // Value 0 allows u-boot to use production image
		// Write the reconfiguration FPGA offset; the base address of the
		// "run-time" FPGA is #defined as a byte address, but the ICAP needs
		// a 16-bit half-word address, so we shift right by one extra bit.
		putfsl(0x03261, 0); // Write GENERAL1
		putfsl(((BOOT_FPGA_BASE >> 1) & 0x0FFFF), 0); // "golden" FPGA start address[15:0]
		putfsl(0x03281, 0); // Write GENERAL2
		putfsl(((BOOT_FPGA_BASE >> 17) & 0x0FF), 0); // Opcode 0x00 and address[23:16]

		// Write the fallback FPGA offset (this image)
		putfsl(0x032A1, 0); // Write GENERAL3
		putfsl((BOOT_FPGA_BASE & 0x0FFFF), 0);
		putfsl(0x032C1, 0); // Write GENERAL4
		putfsl(((BOOT_FPGA_BASE >> 16) & 0x0FF), 0);
		putfsl(0x032E1, 0); // Write GENERAL5
		putfsl(gp5, 0); // Value 1 forces u-boot to try firmware update

		// Write IPROG command
		putfsl(0x030A1, 0); // Write CMD
		putfsl(0x0000E, 0); // IPROG Command
		// Add some safety noops
		putfsl(0x02000, 0); // Type 1 NOP
		putfsl(FINISH_FSL_BIT | 0x02000, 0); // Type 1 NOP, and Trigger the FSL peripheral to drain the FIFO into the ICAP
		udelay(1000);
		getfsl(val, 0); // Read the ICAP result
	} while ((val & ICAP_FSL_FAILED) != 0);

#else

#define XPAR_XPS_HWICAP_0_BASEADDR 0x80004000 // TODO: Get this from device tree
/* ICAP peripheral controller */
#define XPAR_ICAP_CR_ABORT		BIT(4)
#define XPAR_ICAP_CR_RESET		BIT(3)
#define XPAR_ICAP_CR_FIFO_CLEAR	BIT(2)
#define XPAR_ICAP_CR_READ		BIT(1)
#define XPAR_ICAP_CR_WRITE		BIT(0)

#define XPAR_ICAP_SR_CFGERR		BIT(8)
#define XPAR_ICAP_SR_DALIGN		BIT(7)
#define XPAR_ICAP_SR_READ_IP	BIT(6)
#define XPAR_ICAP_SR_IN_ABORT	BIT(5)
#define XPAR_ICAP_SR_DONE		BIT(0)

#define	CONFIG_SYS_ICAP_ADDR	XPAR_XPS_HWICAP_0_BASEADDR
#define	CONFIG_SYS_ICAP_GIE		(CONFIG_SYS_ICAP_ADDR + 0x01C)
#define	CONFIG_SYS_ICAP_IPISR	(CONFIG_SYS_ICAP_ADDR + 0x020)
#define	CONFIG_SYS_ICAP_IPIER	(CONFIG_SYS_ICAP_ADDR + 0x028)
#define	CONFIG_SYS_ICAP_WF		(CONFIG_SYS_ICAP_ADDR + 0x100)
#define	CONFIG_SYS_ICAP_RF		(CONFIG_SYS_ICAP_ADDR + 0x104)
#define	CONFIG_SYS_ICAP_SZ		(CONFIG_SYS_ICAP_ADDR + 0x108)
#define	CONFIG_SYS_ICAP_CR		(CONFIG_SYS_ICAP_ADDR + 0x10C)
#define	CONFIG_SYS_ICAP_SR		(CONFIG_SYS_ICAP_ADDR + 0x110)
#define	CONFIG_SYS_ICAP_WFV		(CONFIG_SYS_ICAP_ADDR + 0x114)
#define	CONFIG_SYS_ICAP_RFO		(CONFIG_SYS_ICAP_ADDR + 0x118)

	out_32(CONFIG_SYS_ICAP_CR, XPAR_ICAP_CR_ABORT);
	while ((in_32(CONFIG_SYS_ICAP_CR) & (XPAR_ICAP_CR_ABORT | XPAR_ICAP_CR_RESET |
    			XPAR_ICAP_CR_FIFO_CLEAR | XPAR_ICAP_CR_READ | XPAR_ICAP_CR_WRITE)) != 0)
		;
	// Synchronize command bytes
	out_32(CONFIG_SYS_ICAP_WF, 0x0FFFF); // Pad words
	out_32(CONFIG_SYS_ICAP_WF, 0x0FFFF);
	out_32(CONFIG_SYS_ICAP_WF, 0x0AA99); // SYNC
	out_32(CONFIG_SYS_ICAP_WF, 0x05566); // SYNC

	// Write the reconfiguration FPGA offset; the base address of the
	// "run-time" FPGA is #defined as a byte address, but the ICAP needs
	// a 16-bit half-word address, so we shift right by one extra bit.
	out_32(CONFIG_SYS_ICAP_WF, 0x03261); // Write GENERAL1
	out_32(CONFIG_SYS_ICAP_WF, ((BOOT_FPGA_BASE >> 1) & 0x0FFFF)); // Multiboot start address[15:0]
	out_32(CONFIG_SYS_ICAP_WF, 0x03281); // Write GENERAL2
	out_32(CONFIG_SYS_ICAP_WF, ((BOOT_FPGA_BASE >> 17) & 0x0FF)); // Opcode 0x00 and address[23:16]
	// Write the fallback FPGA offset (this image)
	out_32(CONFIG_SYS_ICAP_WF, 0x032A1); // Write GENERAL3
	out_32(CONFIG_SYS_ICAP_WF, ((BOOT_FPGA_BASE >> 1) & 0x0FFFF)); // Multiboot start address[15:0]
	out_32(CONFIG_SYS_ICAP_WF, 0x032C1); // Write GENERAL4
	out_32(CONFIG_SYS_ICAP_WF, ((BOOT_FPGA_BASE >> 17) & 0x0FF)); // Opcode 0x00 and address[23:16]
	out_32(CONFIG_SYS_ICAP_WF, 0x032E1); // Write GENERAL5
	out_32(CONFIG_SYS_ICAP_WF, val); // Value 0 allows u-boot to use production image
	// Write IPROG command
	out_32(CONFIG_SYS_ICAP_WF, 0x030A1); // Write CMD
	out_32(CONFIG_SYS_ICAP_WF, 0x0000E); // IPROG Command
	// Add some safety noops
	out_32(CONFIG_SYS_ICAP_WF, 0x02000); // Type 1 NOP
	out_32(CONFIG_SYS_ICAP_WF, 0x02000); // Type 1 NOP
	// Trigger the FSL peripheral to drain the FIFO into the ICAP
	out_32(CONFIG_SYS_ICAP_CR, XPAR_ICAP_CR_WRITE);
	while ((in_32(CONFIG_SYS_ICAP_CR) & XPAR_ICAP_CR_WRITE) != 0)
		;
#endif
	while(1);
}

void machine_restart(char *cmd)
{
	printk(KERN_NOTICE "Machine restart...\n");
#ifdef CONFIG_SPARTAN6_RESET
	{
		u32 val = 0;
		if (cmd == NULL || (val = simple_strtoul(cmd, NULL, 0)) > 65536) {
			val = 0;
		}
	printk(KERN_NOTICE "ICAP reset %d ...\n", val);
	icap_reset(val); //reboot using ICAP
	}
#else
	gpio_system_reset();
#endif
	dump_stack();
	while (1)
		;
}

void machine_shutdown(void)
{
	printk(KERN_NOTICE "Machine shutdown...\n");
	while (1)
		;
}

void machine_halt(void)
{
	printk(KERN_NOTICE "Machine halt...\n");
	while (1)
		;
}

void machine_power_off(void)
{
	printk(KERN_NOTICE "Machine power off...\n");
	while (1)
		;
}

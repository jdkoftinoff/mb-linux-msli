/*
 * platform-specific setup routines
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/autoconf.h>
#include <linux/platform_device.h>
#include <net/labx_avb/packet_engine_defs.h>
#include <net/labx_ptp/labx_ptp_defs.h>
#include <linux/labx_dma_coprocessor_defs.h>
#include <linux/init.h>
#include <linux/resource.h>
#include <linux/xilinx_devices.h>

/*
 * Declare AVB packet engines
 */
#if 0
static struct resource audio_packetizer_resources[PACKET_ENGINE_NUM_RESOURCES];

static struct platform_device audio_packetizer_device = {
  .name	         = "labx_audio_packetizer",
  .id	         = 0,
  .resource	     = audio_packetizer_resources,
  .num_resources = ARRAY_SIZE(audio_packetizer_resources)
};

static struct resource audio_depacketizer_resources[PACKET_ENGINE_NUM_RESOURCES];

static struct platform_device audio_depacketizer_device = {
  .name	         = "labx_audio_depacketizer",
  .id	         = 0,
  .resource	     = audio_depacketizer_resources,
  .num_resources = ARRAY_SIZE(audio_depacketizer_resources)
};

static struct resource serializer_dma_resources[LABX_DMA_NUM_RESOURCES];

static struct platform_device serializer_dma_device = {
  .name	         = "labx_dma",
  .id	         = 0,
  .resource	     = serializer_dma_resources,
  .num_resources = ARRAY_SIZE(serializer_dma_resources)
};

/* Declare the PTP hardware device */
static struct resource ptp_resources[] = {
  {
    .start	= CONFIG_XILINX_LABX_PTP_0_BASEADDR,
    .end	= CONFIG_XILINX_LABX_PTP_0_HIGHADDR,
    .flags	= IORESOURCE_MEM
  }, 
  {
    .start	= CONFIG_XILINX_LABX_PTP_0_IRQ,
    .end	= CONFIG_XILINX_LABX_PTP_0_IRQ,
    .flags	= IORESOURCE_IRQ
  }
};

static struct platform_device ptp_device = {
  .name	         = "labx_ptp",
  .id	         = 0,
  .resource	     = ptp_resources,
  .num_resources = ARRAY_SIZE(ptp_resources),
  .dev.platform_data = &(PtpPlatformData) {
    /* The event timer should be firing roughly 100 times a second for proper timing
     * of packet transmissions, timeouts, etc.  This is run from the host clock, which
     * is at 125 MHz.
     */
    .timerPrescaler = 2000,
    .timerDivider   = 625,
 
    /* The RTC is clocked off of a 125 MHz clock, so the nominal RTC increment
     * is eight nanoseconds
     */
    .nominalIncrement = {
      .mantissa = 8,
      .fraction = 0
    },

    /* A full-scale 1.0 coefficient is 0x80000000; this coefficient represents
     * the value (0xFFC00000 = -0.001953125)
     */
    .rtcPCoefficient = 0xFFC00000
  }
};

/* Initialize the Lab X Ethernet / LocalLink legacy MAC using hardware
 * definitions specific to the Meyer Sound D'Mitri IO Link platform
 */

/* Address of the PHY on the MDIO bus on the Spartan-3A DSP 1800 Board */
#define PHY_MDIO_ADDRESS  (0x01)

/* Operate in SDMA mode; on this platform, we have port four (4) of the MPMC
 * configured as an SDMA PIM to shuttle data for the legacy traffic.
 *
 * The MPMC is configured to have all SDMA PIMs share the same base address, so
 * compute the address of PIM three.  Also sanity-check the configuration; this
 * is one case where the definitions in the autoconf file aren't conducive to
 * extracting the value we really need ("four").
 */
#define MPMC_LLTEMAC_SDMA_PORT    (4)
#define MPMC_SDMA_BASE_TYPE       (3)
#if (CONFIG_XILINX_MPMC_0_PIM4_BASETYPE != MPMC_SDMA_BASE_TYPE)
#  error "uClinux <linux/auto-config.h> conflicts with expected SDMA PIM!"
#endif

#define LABX_ETH_LLINK_SDMA_CTRL_BASEADDR  (CONFIG_XILINX_MPMC_0_SDMA_CTRL_BASEADDR + (MPMC_LLTEMAC_SDMA_PORT * SDMA_PIM_REGS_SIZE))

/* Interrupts assigned to Tx and Rx on the SDMA PIM used for legacy Ethernet */
#define LABX_ETH_LLINK_TX_IRQ  (CONFIG_XILINX_MPMC_0_SDMA4_TX_INTOUT_IRQ)
#define LABX_ETH_LLINK_RX_IRQ  (CONFIG_XILINX_MPMC_0_SDMA4_RX_INTOUT_IRQ)

static struct platform_device labx_eth_locallink_device = {
  .name = "labx_eth_llink",
  .id = 0,
  .num_resources = 2,
  .resource = (struct resource[]) {
    {
      .start = CONFIG_XILINX_LABX_ETH_LOCALLINK_0_BASEADDR,
      .end	 = CONFIG_XILINX_LABX_ETH_LOCALLINK_0_BASEADDR,
      .flags = IORESOURCE_MEM
    },
    {
      .start = CONFIG_XILINX_LABX_ETH_LOCALLINK_0_IRQ,
      .end	 = CONFIG_XILINX_LABX_ETH_LOCALLINK_0_IRQ,
      .flags = IORESOURCE_IRQ
    }
  },
  .dev.platform_data = &(struct xlltemac_platform_data) {
    .tx_csum            = 0,
    .rx_csum            = 0,
    .phy_type           = XTE_PHY_TYPE_GMII,
    .phy_addr           = PHY_MDIO_ADDRESS,
    .ll_dev_type        = XPAR_LL_DMA,
    .ll_dev_baseaddress = LABX_ETH_LLINK_SDMA_CTRL_BASEADDR,
    .ll_dev_dma_rx_irq  = LABX_ETH_LLINK_RX_IRQ,
    .ll_dev_dma_tx_irq  = LABX_ETH_LLINK_TX_IRQ,
    
    /* locally administered default MAC address */
    .mac_addr = {0x00, 0x0A, 0x35, 5, 5, ((1))<<3}
  }
};

static struct platform_device *iolink_devices[] __initdata = {
  &audio_packetizer_device,
  &audio_depacketizer_device,
  &serializer_dma_device,
  &ptp_device,
  &labx_eth_locallink_device
};

/* Add all platform devices we defined */
static int __init iolink_init(void)
{
  /* Initialize resources for the audio packetizer */
  struct resource *resource;

  /* Configure packetizer resources */

  /* Address range for the instance */
  resource = &audio_packetizer_resources[PACKET_ENGINE_ADDRESS_RANGE_RESOURCE];
  resource->start = CONFIG_XILINX_LABX_AUDIO_PACKETIZER_0_BASEADDR;
  resource->end   = CONFIG_XILINX_LABX_AUDIO_PACKETIZER_0_HIGHADDR;
  resource->flags = IORESOURCE_MEM;
  resource->name  = "labx_audio_packetizer.0";

  /* Configure depacketizer resources */

  /* Address range for the instance */
  resource = &audio_depacketizer_resources[PACKET_ENGINE_ADDRESS_RANGE_RESOURCE];
  resource->start = CONFIG_XILINX_LABX_AUDIO_DEPACKETIZER_0_BASEADDR;
  resource->end   = CONFIG_XILINX_LABX_AUDIO_DEPACKETIZER_0_HIGHADDR;
  resource->flags = IORESOURCE_MEM;
  resource->name  = "labx_audio_depacketizer.0";

  /* Configure serializer dma resources */

  /* Address range for the instance */
  resource = &serializer_dma_resources[LABX_DMA_ADDRESS_RANGE_RESOURCE];
  resource->start = CONFIG_XILINX_LABX_LOCAL_AUDIO_0_BASEADDR;
  resource->end   = CONFIG_XILINX_LABX_LOCAL_AUDIO_0_HIGHADDR;
  resource->flags = IORESOURCE_MEM;
  resource->name  = "labx_local_audio.0";

  /* Add all of the platform devices */
  return(platform_add_devices(iolink_devices, ARRAY_SIZE(iolink_devices)));
}
subsys_initcall(iolink_init);
#endif

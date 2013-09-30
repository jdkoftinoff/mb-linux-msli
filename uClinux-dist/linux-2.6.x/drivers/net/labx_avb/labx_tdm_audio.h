/*
 *  linux/drivers/net/labx_avb/labx_tdm_audio.h
 *
 *  Lab X Technologies Audio time division multiplexing driver
 *
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2012 Lab X Technologies LLC, All Rights Reserved.
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
#include "labx_tdm_analyzer.h"
#endif

#define NAME_MAX_SIZE    (256)

#if 0
#define DBG(f, x...) printk(DRIVER_NAME " [%s()]: " f, __func__,## x)
#else
#define DBG(f, x...)
#endif

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->virtualAddress | (offset << 2))

// There's a single register, at offset 0x00000008 (byte address, is actually register 0x02 in 32-bit offset-speak)
   // Bits[23:20]  - MCLK divisor
   // Bits[19:18]  - Sample rate
   //                "00" - Single rate
   //                "01" - Double rate
   //                "10" - Quad rate                                 
   // Bits[17:13]  - Burst length (4, 8, 16 are the only valid burst sizes)
   // Bit[12]      - Sample depth
   //                '0' 24-bits
   //                '1' 16-bits
   // Bit[11]      - Tx master / slave
   //                '0' Master
   //                '1' Slave
   // Bit[10]      - Rx master / slave
   //                '0' Master
   //                '1' Slave
   // Bit[9]       - Bit alignment:
   //                '0' - Left-justified, or "normal" mode
   //                '1' - I2S-like mode, MSB of channel 0 lags the LRCK edge by one clock
   // Bit[8]       - LRCK mode:
   //                '0' - Normal operation as a clock
   //                '1' - Pulse indicating frame start
   // Bit[7]       - Sample edge:
   //                '0' - Rising edge signifies channel zero
   //                '1' - Falling edge signifies channel zero
   // Bits[6:0]    - Number of channels per lane (2, 4, 8 or 16, 32, 64 are the only valid number of TDM modes)
                
/* Global control registers */
#define TDM_CONTROL_REG             (0x02)
#  define TDM_SLOT_DENSITY_MASK     (0x7F)
#  define TDM_BURST_LENGTH_MUL_MASK (0xE000)
#  define TDM_BURST_LENGTH_MUL_BITS (13)
#  define TDM_SAMPLE_RATE_MASK      (0xC0000)
#  define TDM_SAMPLE_RATE_BITS      (18)
#  define TDM_MODULE_OWNER_MASK     (0xC00)
#  define TDM_MODULE_OWNER_BITS     (10)
#  define TDM_MCLK_DIVIDER_MASK     (0xF00000)
#  define TDM_MCLK_DIVIDER_BITS     (20)
#define TDM_STREAM_MAP_REG          (0x03)
#  define MAP_SWAP_BANK             (0x80000000)
#  define MAP_CHANNEL_SHIFT         (16)
#  define MAP_STREAM_MASK           (0x0000003F)
#define TDM_IRQ_MASK_REG            (0x009)
#define TDM_IRQ_FLAGS_REG           (0x00A)
#  define DMA_ERROR_IRQ             (0x001)
#  define ANALYSIS_ERROR_IRQ        (0x002)

/* Base address of analyzer/generator registers
 * (within TDM module), in bytes. */
#define TDM_ANALYZER_BASE_ADDRESS (0x010)

/* Number of physical banks used for auto-mute mapping */
#define STREAM_MAP_BANKS  (2)

/* Minimum slot density supported by hardware */
#define LABX_TDM_MIN_SLOT_DENSITY (2)

/* Driver definitions for operating contexts */
#  define TDM_LRCLK_RISING_EDGE_CH0          (0x0)
#  define TDM_LRCLK_FALLING_EDGE_CH0         (0x80)
#  define TDM_LRCLK_MODE_NORMAL              (0x0)
#  define TDM_LRCLK_MODE_PULSE               (0x100)
#  define TDM_BIT_ALIGNMENT_LEFT_JUSTIFIED   (0x0)
#  define TDM_BIT_ALIGNMENT_I2S_DELAYED      (0x200)
#  define TDM_MODULE_MASTER_MODE             (0x0)
#  define TDM_MODULE_SLAVE_MODE              (0xC00)
#  define TDM_RX_MASTER_MODE                 (0x0)
#  define TDM_RX_SLAVE_MODE                  (0x400)
#  define TDM_TX_MASTER_MODE                 (0x0)
#  define TDM_TX_SLAVE_MODE                  (0x800)
#  define TDM_SAMPLE_DEPTH_24BIT             (0x0)
#  define TDM_SAMPLE_DEPTH_16BIT             (0x1000)

/* Sample rate constants */
#  define SINGLE_SAMPLE_RATE (0x00)
#  define DOUBLE_SAMPLE_RATE (0x01)
#  define QUAD_SAMPLE_RATE   (0x02)

#  define SAMPLE_RATE_32_KHZ    (0x00)
#  define SAMPLE_RATE_44_1_KHZ  (0x01)
#  define SAMPLE_RATE_48_KHZ    (0x02)
#  define SAMPLE_RATE_88_2_KHZ  (0x03)
#  define SAMPLE_RATE_96_KHZ    (0x04) 
#  define SAMPLE_RATE_176_4_KHZ (0x05)
#  define SAMPLE_RATE_192_KHZ   (0x06)

#define NO_IRQ_SUPPLIED   (-1)

/* Structures for storing physical hardware attributes */
struct labx_tdm_platform_data {
  uint8_t num_transmitters;
  uint8_t num_receivers;
  uint8_t lane_count;
  uint8_t num_streams;
  uint8_t slot_density;
  uint8_t burst_length;
  uint8_t burst_length_multiple;
  uint32_t mclk_ratio;
  uint8_t has_loopback;
  uint8_t slave_manager;
#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  uint8_t analyzer;
#endif
  uint8_t has_dynamic_sample_rates;
};

typedef struct {
  u32 TdmSampleRate;
  u32 TdmSlotDensity;
  u32 TdmChannelBits;
} labx_tdm_operating_Config;

struct audio_tdm {
  /* Misc device */
  struct miscdevice miscdev;  

  /* Pointer back to the platform device */
  struct platform_device *pdev;
  struct class tdmclass;

  /* Mutex for the device instance */
  spinlock_t mutex;
  bool opened;
  
  /* File operations and private data for a polymorphic
   * driver to use
   */
  struct file_operations *derivedFops;
  void *derivedData;
    
  /* Name for use in identification */
  char name[NAME_MAX_SIZE];
  /* Device version */
  uint32_t version;
      
  /* Physical and virtual base address */
  uintptr_t      physicalAddress;
  uintptr_t      addressRangeSize;
  void __iomem  *virtualAddress;

  /* Interrupt request number */
  int32_t irq;
             
  uint32_t initialVal;

  /* Hardware configuration */
  TdmCaps tdmCaps;

  /* Operating modes */
  labx_tdm_operating_Config opConfig;

#ifdef CONFIG_LABX_AUDIO_TDM_ANALYZER
  /* Analyzer, if enabled */
  struct tdm_analyzer analyzer;
#endif

};

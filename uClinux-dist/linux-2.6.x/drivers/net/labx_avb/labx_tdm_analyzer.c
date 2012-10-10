/*
 *  linux/drivers/net/labx_avb/labx_tdm_analyzer.c
 *
 *  Lab X Technologies AVB TDM analyzer driver
 *
 *  Written by Eldridge M. Mount IV (eldridge.mount@labxtechnologies.com)
 *  Written by Albert M. Hajjar (albert.hajjar@labxtechnologies.com)
 *
 *  Copyright (C) 2011 Lab X Technologies LLC, All Rights Reserved.
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

#include "linux/labx_tdm_analyzer_defs.h"
#include <xio.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

#define NAME_MAX_SIZE    (256)

struct tdm_analyzer {
  /* Name for used for identification */
  char tdmName[NAME_MAX_SIZE];
  
  /* Base address of analyzer register set */
  void __iomem  *baseAddress;

  /* IRQ associated with errors */
  int32_t        errorIrq;

  /* Register addresses for interrupts */
  uint32_t       irqMaskReg;
  uint32_t       irqFlagsReg;
};

#  define TDM_ANALYZER_ENABLE     (0x80000000)
#  define TDM_DEBUG_PATTERN       (0x40000000)
#  define ANALYZER_RAMP           (0x20000000)
#  define AUTO_MUTE_ACTIVE        (0x10000000)
#  define ANALYZER_LANE_MASK      (0x00F)
#  define ANALYZER_SLOT_MASK      (0x3F0)
#  define ANALYZER_SLOT_SHIFT     (4)
#  define GENERATOR_PATTERN_MASK  (0x78000000)
#  define GENERATOR_PATTERN_SHIFT (27)
#  define ANALYZER_LFSR_MODE      (0x40000000)

#define GENERATOR_CONTROL_REG (0x000)
#define ANALYZER_CONTROL_REG  (0x001)
#define ERROR_COUNT_REG       (0x002)
#define ERROR_PREDICT_REG     (0x003)
#define ERROR_ACTUAL_REG      (0x004)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->baseAddress | (offset << 2))

/* Configures the pseudorandom generator */
static void configure_generator(struct tdm_analyzer *analyzer,
                                AnalyzerConfig *analyzerConfig) {
  uint32_t controlRegister;

  controlRegister = XIo_In32(REGISTER_ADDRESS(analyzer, GENERATOR_CONTROL_REG));
  if(analyzerConfig->enable == ANALYZER_ENABLE) {
    // Enable the analyzer on the appropriate channel 
    controlRegister &= ~(ANALYZER_LANE_MASK | ANALYZER_SLOT_MASK);
    controlRegister |= (analyzerConfig->tdmLane & ANALYZER_LANE_MASK);
    controlRegister |= ((analyzerConfig->tdmSlot << ANALYZER_SLOT_SHIFT) & ANALYZER_SLOT_MASK);
    controlRegister |= ((analyzerConfig->signalControl << GENERATOR_PATTERN_SHIFT) & GENERATOR_PATTERN_MASK);
    controlRegister |= TDM_ANALYZER_ENABLE;

  } else {
    // Just disable the generator  
    controlRegister &= ~TDM_ANALYZER_ENABLE;
  }
  XIo_Out32(REGISTER_ADDRESS(analyzer, GENERATOR_CONTROL_REG), controlRegister);
}

/* Configures the pseudorandom generator */
static void configure_analyzer(struct tdm_analyzer *analyzer,
                                AnalyzerConfig *analyzerConfig) {
  uint32_t irqMask;
  uint32_t controlRegister;

  controlRegister = XIo_In32(REGISTER_ADDRESS(analyzer, ANALYZER_CONTROL_REG));
  irqMask = XIo_In32(REGISTER_ADDRESS(analyzer, analyzer->irqMaskReg));
  if(analyzerConfig->enable == ANALYZER_ENABLE) {
    // Enable the analyzer on the appropriate channel 
    controlRegister &= ~(ANALYZER_LANE_MASK | ANALYZER_SLOT_MASK);
    controlRegister |= (analyzerConfig->tdmLane & ANALYZER_LANE_MASK);
    controlRegister |= ((analyzerConfig->tdmSlot << ANALYZER_SLOT_SHIFT) & ANALYZER_SLOT_MASK);
    if (analyzerConfig->signalControl == ANALYZE_MODE_RAMP) {
      controlRegister &= ~ANALYZER_LFSR_MODE;
    } else {
      controlRegister |= ANALYZER_LFSR_MODE;
    }
    controlRegister |= TDM_ANALYZER_ENABLE;

    /* Enable the analysis error interrupt as a "one-shot" */
    irqMask |= analyzer->errorIrq;
    XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqFlagsReg), analyzer->errorIrq);
  } else {
    /* Just disable the analyzer and its IRQ */
    irqMask &= ~analyzer->errorIrq;
    controlRegister &= ~ANALYZER_ENABLE;
  }
  XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqMaskReg), irqMask);
  XIo_Out32(REGISTER_ADDRESS(analyzer, GENERATOR_CONTROL_REG), controlRegister);
}

static void get_analyzer_results(struct tdm_analyzer *analyzer,
                                 AnalyzerResults *analyzerResults) {
  // Fetch the most recent snapshot of analysis results 
  analyzerResults->errorCount = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_COUNT_REG));
  analyzerResults->predictedSample = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_PREDICT_REG));
  analyzerResults->actualSample = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_ACTUAL_REG));
}

/* Opens the instance for use, placing the hardware into a known state */
int32_t tdm_analyzer_open(struct tdm_analyzer *analyzer) {
  AnalyzerConfig analyzerConfig;

  /* Disable the pseudorandom analyzer */
  analyzerConfig.enable = ANALYZER_DISABLE;
  configure_analyzer(analyzer, &analyzerConfig);
  configure_generator(analyzer, &analyzerConfig);
  return(0);
}
EXPORT_SYMBOL(tdm_analyzer_open);

/* I/O control operations for the driver */
int tdm_analyzer_ioctl(struct tdm_analyzer* analyzer, 
                       unsigned int command, 
                       unsigned long arg) {
  // Switch on the request
  int returnValue = 0;

  switch(command) {
  
  case IOC_CONFIG_GENERATOR:
    {
      AnalyzerConfig analyzerConfig;

      if(copy_from_user(&analyzerConfig, (void __user*)arg, sizeof(analyzerConfig)) != 0) {
        return(-EFAULT);
      }
      configure_analyzer(analyzer, &analyzerConfig);
    }
    break;
  
  case IOC_CONFIG_ANALYZER:
    {
      AnalyzerConfig analyzerConfig;

      if(copy_from_user(&analyzerConfig, (void __user*)arg, sizeof(analyzerConfig)) != 0) {
        return(-EFAULT);
      }
      configure_analyzer(analyzer, &analyzerConfig);
    }
    break;
  
  case IOC_GET_ANALYZER_RESULTS:
    {
      AnalyzerResults analyzerResults;

      get_analyzer_results(analyzer, &analyzerResults);
      if(copy_to_user((void __user*)arg, &analyzerResults, sizeof(analyzerResults)) != 0) {
        return(-EFAULT);
      }
    }
    break;

  default:
    returnValue = -EINVAL;
    break;

  }
  /* Return an error code appropriate to the command */
  return(returnValue);
}
EXPORT_SYMBOL(tdm_analyzer_ioctl);

/* Interrupt service routine for the driver */
static irqreturn_t tdm_analyzer_interrupt(struct tdm_analyzer *analyzer, 
                                          uint32_t irqMask) {

  /* TEMPORARY - Just announce this and treat it as a one-shot.
   *             Ultimately this should be communicated via generic Netlink.
   */
  irqMask &= ~analyzer->errorIrq;
  XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqFlagsReg), irqMask);
  printk("%s: Analysis error!\n", analyzer->tdmName);
  
  return(IRQ_HANDLED);
}

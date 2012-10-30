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
#include "labx_tdm_analyzer.h"
#include <xio.h>
#include <linux/highmem.h>
#include <linux/platform_device.h>

#ifdef CONFIG_OF
#include <linux/of_device.h>
#include <linux/of_platform.h>
#endif // CONFIG_OF

#define _LABXDEBUG

#define NAME_MAX_SIZE    (256)

#define GENERATOR_CONTROL_REG (0x000)
#  define TDM_GENERATOR_ENABLE        (0x80000000)
#  define TDM_ANALYZER_ENABLE         (0x80000000)
#  define TDM_GENERATOR_PATTERN_MASK  (0x78000000)
#  define TDM_GENERATOR_PATTERN_SHIFT (27)
#  define TDM_GENERATOR_DEBUG_PATTERN (0x40000000)
#  define TDM_GENERATOR_LANE_MASK     (0x00F)
#  define TDM_GENERATOR_SLOT_MASK     (0x3F0)
#  define TDM_GENERATOR_SLOT_SHIFT    (4)
#define ANALYZER_CONTROL_REG  (0x001)
#  define TDM_ANALYZER_RAMP           (0x20000000)
#  define TDM_ANALYZER_LANE_MASK      (0x00F)
#  define TDM_ANALYZER_SLOT_MASK      (0x3F0)
#  define TDM_ANALYZER_SLOT_SHIFT     (4)
#  define TDM_ANALYZER_LFSR_MODE      (0x40000000)

#define ERROR_COUNT_REG       (0x002)
#define ERROR_PREDICT_REG     (0x003)
#define ERROR_ACTUAL_REG      (0x004)

#define REGISTER_ADDRESS(device, offset) \
  ((uintptr_t)device->baseAddress + (offset << 2))

/* Configures the pseudorandom generator */
static void configure_generator(struct tdm_analyzer *analyzer,
                                GeneratorConfig *generatorConfig) {
  uint32_t controlRegister;

  controlRegister = XIo_In32(REGISTER_ADDRESS(analyzer, GENERATOR_CONTROL_REG));
  if(generatorConfig->enable == LFSR_GENERATOR_ENABLE) {
    // Enable the analyzer on the appropriate channel 
    controlRegister &= ~(TDM_GENERATOR_LANE_MASK | TDM_GENERATOR_SLOT_MASK);
    controlRegister |= (generatorConfig->tdmLane & TDM_GENERATOR_LANE_MASK);
    controlRegister |= ((generatorConfig->tdmChannel << TDM_GENERATOR_SLOT_SHIFT) & TDM_GENERATOR_SLOT_MASK);
    controlRegister |= ((generatorConfig->signalControl << TDM_GENERATOR_PATTERN_SHIFT) & TDM_GENERATOR_PATTERN_MASK);
    controlRegister |= TDM_GENERATOR_ENABLE;

#ifdef _LABXDEBUG
    printk("TDM: enabled generator on lane %u, channel %u\n", generatorConfig->tdmLane, generatorConfig->tdmChannel);
#endif
  } else {
    // Just disable the generator  
    controlRegister &= ~TDM_GENERATOR_ENABLE;
#ifdef _LABXDEBUG
    printk("TDM: disabled generator\n");
#endif
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
  if(analyzerConfig->enable == LFSR_ANALYZER_ENABLE) {
    // Enable the analyzer on the appropriate channel 
    controlRegister &= ~(TDM_ANALYZER_LANE_MASK | TDM_ANALYZER_SLOT_MASK);
    controlRegister |= (analyzerConfig->tdmLane & TDM_ANALYZER_LANE_MASK);
    controlRegister |= ((analyzerConfig->tdmChannel << TDM_ANALYZER_SLOT_SHIFT) & TDM_ANALYZER_SLOT_MASK);
    if (analyzerConfig->signalControl == ANALYSIS_PSEUDORANDOM) {
      controlRegister &= ~TDM_ANALYZER_RAMP;
#ifdef _LABXDEBUG
      printk("TDM: enabled pseudorandom analyzer on lane %u, channel %u\n", analyzerConfig->tdmLane, analyzerConfig->tdmChannel);
#endif
    } else {
      controlRegister |= TDM_ANALYZER_RAMP;
#ifdef _LABXDEBUG
      printk("TDM: enabled ramp analyzer on lane %u, channel %u\n", analyzerConfig->tdmLane, analyzerConfig->tdmChannel);
#endif
    }
    controlRegister |= TDM_ANALYZER_ENABLE;

    /* Enable the analysis error interrupt as a "one-shot" */
    irqMask |= analyzer->errorIrq;
    XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqFlagsReg), analyzer->errorIrq);
  } else {
    /* Just disable the analyzer and its IRQ */
    irqMask &= ~analyzer->errorIrq;
    controlRegister &= ~TDM_ANALYZER_ENABLE;
#ifdef _LABXDEBUG
    printk("TDM: disabled analyzer\n");
#endif
  }
  XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqMaskReg), irqMask);
  XIo_Out32(REGISTER_ADDRESS(analyzer, ANALYZER_CONTROL_REG), controlRegister);
}

static void get_analyzer_results(struct tdm_analyzer *analyzer,
                                 AnalyzerResults *analyzerResults) {
  // Fetch the most recent snapshot of analysis results 
  analyzerResults->errorCount = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_COUNT_REG));
  analyzerResults->predictedSample = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_PREDICT_REG));
  analyzerResults->actualSample = XIo_In32(REGISTER_ADDRESS(analyzer, ERROR_ACTUAL_REG));
#ifdef _LABXDEBUG
  printk("TDM: analyzer results: %u errors, most recent error: (predicted=0x%X,actual=0x%X)\n",
         analyzerResults->errorCount, analyzerResults->predictedSample, analyzerResults->actualSample);
#endif
}

/* Resets the instance, placing the hardware into a known state */
int32_t labx_tdm_analyzer_reset(struct tdm_analyzer *analyzer) {
  GeneratorConfig generatorConfig;
  AnalyzerConfig analyzerConfig;

  /* Disable the pseudorandom analyzer */
  generatorConfig.enable = LFSR_GENERATOR_DISABLE;
  analyzerConfig.enable = LFSR_ANALYZER_DISABLE;
  configure_generator(analyzer, &generatorConfig);
  configure_analyzer(analyzer, &analyzerConfig);
  return(0);
}
EXPORT_SYMBOL(labx_tdm_analyzer_reset);

/* I/O control operations for the driver */
int labx_tdm_analyzer_ioctl(struct tdm_analyzer* analyzer, 
                       unsigned int command, 
                       unsigned long arg) {
  // Switch on the request
  int returnValue = 0;

  switch(command) {
  
  case IOC_CONFIG_GENERATOR:
    {
      GeneratorConfig generatorConfig;

      if(copy_from_user(&generatorConfig, (void __user*)arg, sizeof(generatorConfig)) != 0) {
        return(-EFAULT);
      }
      configure_generator(analyzer, &generatorConfig);
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
EXPORT_SYMBOL(labx_tdm_analyzer_ioctl);

/* Interrupt service routine for the driver */
irqreturn_t labx_tdm_analyzer_interrupt(struct tdm_analyzer *analyzer, 
                                          uint32_t irqMask) {

  /* TEMPORARY - Just announce this and treat it as a one-shot.
   *             Ultimately this should be communicated via generic Netlink.
   */
  irqMask &= ~analyzer->errorIrq;
  XIo_Out32(REGISTER_ADDRESS(analyzer, analyzer->irqFlagsReg), irqMask);
  printk("%s: Analysis error!\n", analyzer->tdmName);
  
  return(IRQ_HANDLED);
}
EXPORT_SYMBOL(labx_tdm_analyzer_interrupt);


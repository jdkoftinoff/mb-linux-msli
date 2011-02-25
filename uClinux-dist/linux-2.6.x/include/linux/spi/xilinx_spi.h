/*
 * xilinx_spi.h
 *
 * Xilinx SPI controller driver extended functionality
 *
 * Author: Lab X Technologies, LLC
 *  scott.wagner@labxtechnologies.com
 *
 * 2011 (c) Lab X Technologies, LLC.  This file is licensed under the
 * terms of the GNU General Public License version 2.  This program is licensed
 * "as is" without any warranty of any kind, whether express or implied.
 */

#ifndef __LINUX_SPI_XILINX_SPI_H
#define __LINUX_SPI_XILINX_SPI_H
#include <linux/spi/spi.h>
//#define SPI_SELECT_PULSEWIDTH       3       /* mS slave select pulse width */

/* Strobe slave select: Generate an active pulse on the device's slave select
 * line for SPI_SELECT_PULSEWIDTH mS, or if SPI_SELECT_PULSEWIDTH is undefined,
 * for about 100 nS.
 */
void spi_strobe_ssel(struct spi_device *spi);
/* Reset transmit buffer: In slave mode only, reset the transmit buffer pointer
 * to the beginning of the transmit buffer.  This will be done automatically by
 * the assertion of SLAVE_SELECT, or may be done explicitly if SLAVE_SELECT is
 * not used.
 */
void spi_reset_transmit_buffer(struct spi_device *spi);

#endif // not defined __LINUX_SPI_XILINX_SPI_H

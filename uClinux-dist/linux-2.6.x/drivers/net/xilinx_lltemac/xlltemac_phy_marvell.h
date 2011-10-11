#ifndef XLLTEMAC_PHY_MARVELL_H
#define XLLTEMAC_PHY_MARVELL_H

#include "xlltemac_common.h"

#ifdef CONFIG_XILINX_LLTEMAC_MARVELL_88E1112_GMII

/* Ethernet LED control, currently only implemented for 88e1112 */

typedef enum {
  LED_STATE_OFF,
  LED_STATE_10M,
  LED_STATE_100M,
  LED_STATE_1G,
  LED_STATE_NSTATES
} led_state;

#define  LED_STATE_FLAG_BLINK          0x01
#define  LED_STATE_FLAG_BLINK_ACTIVITY 0x02
#define  LED_STATE_FLAG_BLINK_SLOW     0x04
#define  LED_STATE_FLAG_BLINK_FLIP     0x08
#define  LED_STATE_FLAG_GREEN          0x10
#define  LED_STATE_FLAG_YELLOW         0x20

int xlltemac_eth_leds_setup(char *s);
void xlltemac_leds_initialize(struct xlltemac_net_local *lp,int phy_addr,
			      led_state state);
#endif

/* PHY initialization */
void xlltemac_phy_setup_marvell(struct xlltemac_net_local *lp);

#endif

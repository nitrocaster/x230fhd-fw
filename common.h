#ifndef COMMON_H
#define COMMON_H

#include "stm8s.h"
#include <iostm8s003f3.h>

#define PWMCAP_TTL (((u16)64)*1)

extern volatile u16 pwmcap_dc_num, pwmcap_dc_denom;
extern volatile u8 pwmcap_st;
extern volatile u16 pwmcap_alive;
extern volatile u16 tim4_tout;
extern volatile u16 tim2_dc;

#endif

#ifndef __COMPILER_H
#define __COMPILER_H

#include <arch/cpu.h>
//#include <aarch32/cortex_m/exc.h>
#include <device.h>
#include <init.h>
#include <kernel.h>
#include <soc.h>
#include <mrepeat.h>

#define SAML21 1
#define SAML21XXXB 1

enum status_code {
    STATUS_OK = 0,
    STATUS_ERR_INVALID_ARG,
};

#  define Assert(expr) ((void) 0)

#endif

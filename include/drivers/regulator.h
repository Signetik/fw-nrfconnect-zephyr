#ifndef __REGULATORS_H
#define __REGULATORS_H

__syscall void regulator_enable(struct device *dev);
__syscall void regulator_disable(struct device *dev);

#include <syscalls/regulator.h>

#endif /* __REGULATORS_H */

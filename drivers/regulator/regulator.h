#ifndef __REGULATORS_H
#define __REGULATORS_H

struct regulator_data {
    int use_count;
    struct device *gpio;
};

struct regulator_config {
    const char * en_cont;
    int en_pin;
    int flags;
    int use_init;
};

struct regulator_api {
    void (*na)(void);
};

int regulator_init(struct device *dev);

#endif /* __REGULATORS_H */

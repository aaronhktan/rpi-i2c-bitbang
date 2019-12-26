static int _i2c_pull_low(const int pin);
static int _i2c_set_floating(const int pin);
static int _i2c_set_floating_wait(const int pin);

int i2c_start(const int sda, const int scl);
int i2c_stop(const int sda, const int scl);

static int _i2c_reset(const int sda, const int scl);

#include "i2c.h"

#include "bcm2835.h"

#include <stdio.h>

#define SDA_PIN 17
#define SCL_PIN 27
#define DELAY_US 5

int started = 0;

static int _i2c_pull_low(const int pin) {
  // In I2C, assumption is that we are using pull-up resistors.
  // So we must actively pull low to indicate 0.
  bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_clr(pin);
  bcm2835_delayMicroseconds(DELAY_US);
  return 0;
}

static int _i2c_set_floating(const int pin) {
  // In I2C, assumption is that we use pull-up resistors.
  // So the pin should float high if we "disconnect" by
  // setting the pin as input.
  bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
  bcm2835_delayMicroseconds(DELAY_US);
  return bcm2835_gpio_lev(pin);
}

static int _i2c_set_floating_wait(const int pin) {
  // I2C supports clock stretching.
  // Here, we expect to float high after setting the pin
  // as input. Then, we wait up to five seconds, in case
  // the slave needs more time (indicated by the slave 
  // driving the pin low).
  int counter = 0;

  bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
  bcm2835_delayMicroseconds(DELAY_US);
  while (!bcm2835_gpio_lev(pin)) {
    if (++counter >= 50) {
      fprintf(stderr, "Clock stretched for too long!\n");
      return -1;
    }
    bcm2835_delayMicroseconds(100000);
    bcm2835_gpio_fsel(pin, BCM2835_GPIO_FSEL_INPT);
  }
  bcm2835_delayMicroseconds(DELAY_US);
  return 0;
}

int i2c_start(const int sda, const int scl) {
  // I2C start condition: SCL high, SDA high -> low.
  // First, try to set SDA high; if it doesn't succeed,
  // means that there is another master or slave using the bus.
  // Reset the bus, then set the SCL high to prepare for
  // the start condition.
  if (!_i2c_set_floating(sda)) {
    _i2c_reset(sda, scl);
    _i2c_set_floating_wait(scl);
  }

  _i2c_pull_low(sda);
  _i2c_pull_low(scl);
  return 0;
}

// Pre-req: SDA and SCL must both be pulled low
// before this function is called
int i2c_stop(const int sda, const int scl) {
  // I2C stop condition: SCL high, SDA low -> high.
  _i2c_set_floating_wait(scl);
  if (!_i2c_set_floating(sda)) {
    _i2c_reset(sda, scl);
  }
  return 0;
}

static int _i2c_reset(const int sda, const int scl) {
  // I2C bus recovery: https://bits4device.wordpress.com/2017/07/28/i2c-bus-recovery/
  // - Send nine clock pulses on SCL (8 data + 1 ack),
  //   then nine more for clock stretching.
  // - SDA is kept floating (high) to indicate NACK to slave
  //   so slave will stop sending data if it is stuck.
  // - Send stop command to terminate the bus transfer
  //   that slave was stuck on.
  _i2c_set_floating(sda);
  
  do {
    for (int i = 0; i < 18; i++) {
      _i2c_pull_low(scl);
      _i2c_set_floating(scl);
    }
    bcm2835_delayMicroseconds(1000);
  } while (!bcm2835_gpio_lev(sda));
  
  _i2c_pull_low(scl);
  _i2c_pull_low(sda);
  
  i2c_stop(sda, scl);

  return 0;
}

int i2c_init(const int sda, const int scl) {
  bcm2835_init();
  
  bcm2835_gpio_set_pud(sda, BCM2835_GPIO_PUD_UP);
  bcm2835_gpio_set_pud(scl, BCM2835_GPIO_PUD_UP);

  bcm2835_gpio_fsel(sda, BCM2835_GPIO_FSEL_OUTP);
  bcm2835_gpio_fsel(scl, BCM2835_GPIO_FSEL_OUTP);  

  _i2c_reset(sda, scl); 
 
  return 0;
}

int i2c_deinit(void) {
  bcm2835_close();
  return 0;
}

int i2c_check_address(const int sda,
                      const int scl,
                      const uint8_t address) {
  // Send I2C start condition
  i2c_start(sda, scl); 
  started = 1;

  uint8_t current_address = address;
  // Start transmitting bits
  for (int i = 0; i < 8; i++) {
    if (current_address & 0x80) {
      _i2c_set_floating(sda);
    } else {
      _i2c_pull_low(sda);
    }
    _i2c_set_floating_wait(scl);
    
    // Get ready for next bit to be transmitted 
    _i2c_pull_low(scl);     
    //_i2c_pull_low(sda);

    current_address <<= 1;
  }

  // Check for ACK/NACK
  _i2c_set_floating(sda); // Let go of SDA so slave can write
  _i2c_set_floating_wait(scl); // Wait until SCL floats high; slave indicates we can read
  int nack = bcm2835_gpio_lev(sda);
  if (nack) {
    fprintf(stderr, "NACK received for address 0x%02x\n", address >> 1);
    return -1;
  }

  // Then send stop condition
  _i2c_pull_low(scl);
  _i2c_pull_low(sda);
  i2c_stop(sda, scl);

  started = 0;
  printf("Contacted address 0x%x\n", (address & 0xFE) >> 1);
  return 0;
}

int main(const int argc, const char **argv) {
  i2c_init(SDA_PIN, SCL_PIN); 

  size_t size = 4;
  uint8_t addresses[4] = {0x48, 0x58, 0x77, 0x10}; 
 
  for (int i = 0; i < size; i++) {
    uint8_t address = (addresses[i] << 1 | 1);
    printf("Address: %08x\n", address);
    i2c_check_address(SDA_PIN, SCL_PIN, address);
  }

  i2c_deinit();
}

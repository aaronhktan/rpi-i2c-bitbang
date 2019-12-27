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
    printf("Resetting bus\n"); 
    _i2c_reset(sda, scl);
    _i2c_set_floating_wait(scl);
  }

  if (started) {
    _i2c_set_floating(sda);
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
  started = 0;
  return 0;
}

static int _i2c_reset(const int sda, const int scl) {
  // I2C bus recovery: https://bits4device.wordpress.com/2017/07/28/i2c-bus-recovery/
  // - Send nine clock pulses on SCL (8 data + 1 ack),
  //   then nine more for clock stretching.
  // - SDA is kept floating (high) to indicate NACK to slave
  //   so slave will stop sending data if it is stuck.
  // - Send stop condition to terminate the bus transfer
  //   that slave was stuck on.
  // - Since the stop condition is sent, after this function
  //   is called, both lines will be floating high.
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

int i2c_write_bit(const int sda,
                  const int scl,
                  const uint8_t bit) {
  // To write a bit, set SDA to correct value
  // Then float SCL to indicate to device that it should read
  // Then pull it back low to prepare for next bit to write
  _i2c_pull_low(scl);
  if (bit & 0x1) {
    _i2c_set_floating(sda);
  } else {
    _i2c_pull_low(sda);
  }
  _i2c_set_floating_wait(scl);

  _i2c_pull_low(scl);
  //_i2c_pull_low(sda);
  return 0;
}

int i2c_read_bit(const int sda,
                 const int scl) {
  // To read a bit, relinquish control of SDA
  // Then wait until SCL is high; this indicates slave is ready
  // Then sample SDA
  // And prepare for next bit by pulling SCL low.
  _i2c_set_floating(sda);
  _i2c_set_floating_wait(scl);
  int ret = bcm2835_gpio_lev(sda);
  _i2c_pull_low(scl);
  //_i2c_pull_low(sda);
  return ret;
}

int i2c_write_byte(const int sda,
                   const int scl,
                   const uint8_t byte) {
  // To write a byte, write 8 bits, then read ACK/NACK
  uint8_t copied_byte = byte;
  for (int i = 0; i < 8; i++) {
    i2c_write_bit(sda, scl, !!(copied_byte & 0x80));
    copied_byte <<= 1;
  }
  int nack = i2c_read_bit(sda, scl);
  if (nack) {
    fprintf(stderr, "NACK received for byte 0x%02x\n", byte);
  }
  return nack;
}

int i2c_write_bytes(const int sda,
                    const int scl,
                    const uint8_t *bytes,
                    const size_t length) {
  // To write many bytes, loop through i2c_write_byte()
  int err = 0;
  for (size_t i = 0; i < length; i++) {
    err |= i2c_write_byte(sda, scl, bytes[i]);
    if (err) {
      fprintf(stderr, "An error occurred while writing byte %d: 0x%02x\n", i, bytes[i]);
      return -1;
    }
  }
  return err;
}

uint8_t i2c_read_byte(const int sda,
                      const int scl) {
  // Bytes are transmitted MSB in I2C.
  // So right shift value read by 1, then bitwise OR it with the next bit read.
  uint8_t byte = 0;
  for (int i = 0; i < 8; i++) {
    byte = byte << 1 | i2c_read_bit(sda, scl);
  }
  // After receiving a bit, send ACK
  i2c_write_bit(sda, scl, 0x0);
  return byte;
}

int i2c_read_bytes(const int sda,
                   const int scl,
                   uint8_t *bytes,
                   const size_t length) {
  // To read multiple bytes, loop through i2c_read_byte()
  for (size_t i = 0; i < length; i++) {
    bytes[i] = i2c_read_byte(sda, scl);
  }
  return 0;
}

int i2c_check_address(const int sda,
                      const int scl,
                      const uint8_t address) {
  // Function to check whether device is present on I2C bus
  // Send I2C start condition
  i2c_start(sda, scl); 
  started = 1;

  int ret = i2c_write_byte(sda, scl, address); 

  // Then send stop condition
  _i2c_pull_low(scl);
  _i2c_pull_low(sda);
  i2c_stop(sda, scl);

  started = 0;
  if (ret) {
    fprintf(stderr, "Couldn't contact address 0x%x\n", (address & 0xFE) >> 1);
  } else {
    printf("Contacted address 0x%x\n", (address & 0xFE) >> 1);
  }
  return 0;
}

int main(const int argc, const char **argv) {
  i2c_init(SDA_PIN, SCL_PIN); 

  //size_t size = 4;
  //uint8_t addresses[4] = {0x48, 0x58, 0x77, 0x10}; 
 
  //for (int i = 0; i < size; i++) {
  //  uint8_t address = (addresses[i] << 1 | 1);
  //  printf("Address: %08x\n", address);
  //  i2c_check_address(SDA_PIN, SCL_PIN, address);
  //}

  // Check try reading serial number of BMP280
  
  uint8_t commands[2] = {0x77 << 1 | 0, 0xD0};
  i2c_start(SDA_PIN, SCL_PIN);
//  for (int i = 0; i < 2; i++) {
//    int err = i2c_write_byte(SDA_PIN, SCL_PIN, commands[i]);
//  }
  i2c_write_bytes(SDA_PIN, SCL_PIN, commands, 3); 
  _i2c_pull_low(SCL_PIN);
  _i2c_pull_low(SDA_PIN);
  i2c_stop(SDA_PIN, SCL_PIN);
//  bcm2835_delay(1);
  uint8_t response[1];
  i2c_start(SDA_PIN, SCL_PIN);
  uint8_t command = 0x77 << 1 | 1;
  i2c_write_byte(SDA_PIN, SCL_PIN, command);
  i2c_read_bytes(SDA_PIN, SCL_PIN, response, 1);
  i2c_stop(SDA_PIN, SCL_PIN);
  fprintf(stdout, "%s", "Serial ID: ");
  for (int i = 0; i < 1; i++) {
    fprintf(stdout, "0x%02x", response[i]);
  }
  fprintf(stdout, "%s", "\n");

  i2c_deinit();
}

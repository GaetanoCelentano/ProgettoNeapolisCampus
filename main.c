/*
 ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include "ch.h"
#include "hal.h"

#include "chprintf.h"
#include "hts221.h"
#include "lps22hb.h"
#include "lsm303agr.h"
#include "lsm6dsl.h"

#define cls(chp)                            chprintf(chp, "\033[2J\033[1;1H")
#define MAX_AXIS_NUMBER                     3U

/* Array for data storage. */
static float cooked[MAX_AXIS_NUMBER];
/* Axis identifiers. */
static char axis_id[MAX_AXIS_NUMBER] = {'X', 'Y', 'Z'};

/* Generic I2C configuration for every MEMS. */
static const I2CConfig i2ccfg = {OPMODE_I2C, 400000, FAST_DUTY_CYCLE_2};

static uint32_t i;
static BaseSequentialStream *chp = (BaseSequentialStream*)&SD2;

/*===========================================================================*/
/* LSM6DSL related.                                                          */
/*===========================================================================*/

/* LSM6DSL Driver: This object represent an LSM6DSL instance */
static LSM6DSLDriver LSM6DSLD1;

static const LSM6DSLConfig lsm6dslcfg = {&I2CD1, &i2ccfg, LSM6DSL_SAD_VCC, NULL,
                                         NULL, LSM6DSL_ACC_FS_2G,
                                         LSM6DSL_ACC_ODR_416Hz, NULL, NULL,
                                         LSM6DSL_GYRO_FS_250DPS,
                                         LSM6DSL_GYRO_ODR_104Hz};

/*
 * Setup ICU for ultrasonic
 * */

#define LINE_TRIGGER                PAL_LINE(GPIOC, 7U)
#define LINE_ECHO                   PAL_LINE(GPIOA, 8U)
#define LINE_BUZZ                   PAL_LINE(GPIOC, 1U)
#define LED_FALL                    PAL_LINE(GPIOA, 4U)

#define ANSI_ESCAPE_CODE_ALLOWED    FALSE


#define ICU_TIM_FREQ                1000000
#define M_TO_CM                     100.0f
#define SPEED_OF_SOUND              343.2f

static float lastdistance = 0.0;
static int state = 0;
static void icuwidthcb(ICUDriver *icup) {
  icucnt_t width = icuGetWidthX(icup);
  lastdistance = (SPEED_OF_SOUND * width * M_TO_CM) / (ICU_TIM_FREQ * 2);
}

static ICUConfig icucfg =
    {ICU_INPUT_ACTIVE_HIGH,
    ICU_TIM_FREQ,
     icuwidthcb, NULL, NULL, ICU_CHANNEL_1, 0, 0xFFFFFFFFU};

/*===========================================================================*/
/* Application related code. Easy way for writing and reading registers      */
/*===========================================================================*/

msg_t writeRegister(uint8_t register_address, uint8_t register_value) {
  uint8_t buff[2];
  buff[0] = register_address; //Register Address
  buff[1] = register_value;

  return i2cMasterTransmitTimeout(&I2CD1, LSM6DSL_SAD_VCC, buff, 2, NULL, 0,
  TIME_INFINITE);
}

msg_t readRegister(uint8_t register_address, uint8_t *out) {
  return i2cMasterTransmitTimeout(&I2CD1, LSM6DSL_SAD_VCC, &register_address, 1,
                                  out, 1, TIME_INFINITE);
}

/*
 * Enable free fall detection on LMS6DSL sensor of the shield
 * */
void enableFreeFall() {

  /*
   * Enable interrupts, probably not needed
   * */
  writeRegister(LSM6DSL_AD_TAP_CFG, 0x81);
  /*
   * Reset wake up dur
   * */
  writeRegister(LSM6DSL_AD_WAKE_UP_DUR, 0x00);
  /*
   * Set FREE FALL duration and threshold
   * */
  // 0x30 => Duration: 00110, Treshold: 000
  writeRegister(LSM6DSL_AD_FREE_FALL, 0x08);

  /*
   * Needed for driving event interrupt to PIN1, but probably not useful
   * */
  writeRegister(LSM6DSL_AD_MD1_CFG, 0x20);
}
/*
 * Testing write and read to and from registers
 * */
void test() {
  msg_t write_output = writeRegister(LSM6DSL_AD_WAKE_UP_SRC, 0x33);
  chprintf(chp, "Write Output: %d\r\n", write_output);

  uint8_t out = 7;
  msg_t msg = readRegister(LSM6DSL_AD_WAKE_UP_SRC, &out);

  chprintf(chp, "Valore registro: %d\r\n", out);
  chprintf(chp, "Messaggio: %d\r\n", msg);
}

// This should become a thread
void checkFreeFall() {
  uint8_t out;

  msg_t msg = readRegister(LSM6DSL_AD_WAKE_UP_SRC, &out);

  if (msg == MSG_OK) {
    if (out == 47) {
      chprintf(chp, "CADUTA!!!\r\n");
      //chprintf(chp, "Value for redgister : %d\r\n", out);
      //chprintf(chp, "Message out: %d\r\n", msg);
      palSetLine(LED_FALL);
    }else{
      palClearLine(LED_FALL);
    }

  }
  else if (msg == MSG_TIMEOUT) {
    chprintf(chp, "Error reading register: TIMEOUT\r\n");
  }
  else if (msg == MSG_RESET) {
    chprintf(chp, "Error reading register: RESET\r\n");
  }else{
    palClearLine(LED_FALL);
  }
}

/*===========================================================================*/
/* Generic code.                                                             */
/*===========================================================================*/

/*
 * Green LED blinker thread, times are in milliseconds.
 */
static THD_WORKING_AREA(waThread1, 128);
static THD_FUNCTION(Thread1, arg) {

  (void)arg;
  chRegSetThreadName("blinker");
  while (true) {
    palClearPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
    palSetPad(GPIOA, GPIOA_LED_GREEN);
    chThdSleepMilliseconds(500);
  }
}

/*
 * Thread buzzer
 * */

static THD_WORKING_AREA(waStateThread1, 256);
static THD_FUNCTION(ThreadSt1, arg) {
  (void)arg;
  chRegSetThreadName("buzz1");
  int currentState = 0;
  while (true) {
    chSysLock();
    currentState = state;
    chSysUnlock();
    if (currentState == 1) {
      palWriteLine(LINE_BUZZ, PAL_HIGH);

    }
    else if (currentState == 2) {
      palWriteLine(LINE_BUZZ, PAL_HIGH);
      chThdSleepMilliseconds(500);
      palWriteLine(LINE_BUZZ, PAL_LOW);
      chThdSleepMilliseconds(500);
    }
    else if (currentState == 3) {
      palWriteLine(LINE_BUZZ, PAL_HIGH);
      chThdSleepMilliseconds(1000);
      palWriteLine(LINE_BUZZ, PAL_LOW);
      chThdSleepMilliseconds(1000);
    }
    else if (currentState == 0) {
      palWriteLine(LINE_BUZZ, PAL_LOW);
    }
    else {
      chThdSleepMilliseconds(50);
    }
    chThdSleepMilliseconds(50);
  }
}

int main(void) {

  halInit();
  chSysInit();

  /* Configuring I2C SCK and I2C SDA related GPIOs .*/
  palSetLineMode(
      LINE_ARD_D15,
      PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);
  palSetLineMode(
      LINE_ARD_D14,
      PAL_MODE_ALTERNATE(4) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_OTYPE_OPENDRAIN);

  /*
   * Setup lines for buzzer and sensor
   * */
  palSetLineMode(LINE_ECHO, PAL_MODE_ALTERNATE(1));
  icuStart(&ICUD1, &icucfg);
  icuStartCapture(&ICUD1);
  icuEnableNotifications(&ICUD1);

  palSetLineMode(LINE_TRIGGER, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LINE_BUZZ, PAL_MODE_OUTPUT_PUSHPULL);
  palSetLineMode(LED_FALL, PAL_MODE_OUTPUT_PUSHPULL);

  palClearLine(LINE_BUZZ);
  palClearLine(LED_FALL);

  //palSetPadMode(GPIOA, GPIOA_ARD_D13, PAL_MODE_OUTPUT_PUSHPULL);
  /*
   * Activates the serial driver 2 using the driver default configuration.
   */
  sdStart(&SD2, NULL);

  /*
   * Creates the blinker thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1), NORMALPRIO + 1, Thread1,
                    NULL);

  chThdCreateStatic(waStateThread1, sizeof(waStateThread1), NORMALPRIO + 1, ThreadSt1,
                     NULL);

  /* MEMS Driver Objects Initialization.*/
  lsm6dslObjectInit(&LSM6DSLD1);

  /* Activates all the MEMS related drivers.*/
  lsm6dslStart(&LSM6DSLD1, &lsm6dslcfg);

  chprintf(chp, "Starting Free Fall Detection\r\n");

  enableFreeFall();

  int currentState;

  while (true) {
    palWriteLine(LINE_TRIGGER, PAL_HIGH);
    chThdSleepMicroseconds(10);
    palWriteLine(LINE_TRIGGER, PAL_LOW);
#if ANSI_ESCAPE_CODE_ALLOWED
      chprintf(chp, "\033[2J\033[1;1H");
  #endif
    chprintf(chp, "Distance: %.2f cm\n\r", lastdistance);

    if (lastdistance < 30.0 && lastdistance > 20.0) {
      currentState = 3;
    }
    else if (lastdistance < 20.0 && lastdistance > 10.0) {
      currentState = 2;
    }
    else if (lastdistance < 10.0) {
      currentState = 1;
    }
    else {
      currentState = 0;
    }
    chSysLock();
    state = currentState;
    chSysUnlock();

    checkFreeFall();

    chThdSleepMilliseconds(100);
  }
  icuStopCapture(&ICUD1);
  icuStop(&ICUD1);
}

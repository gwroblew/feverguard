#include <atmel_start.h>
#include <hri_port_d10.h>

#include "mlx90632.c"
// #include "mlx90632_extended_meas.c"

#define CHIP_ADDRESS 0x3a

void set_led(int id)
{
  static const int ids[] = {
      0b00011110,
      0b00101110,
      0b01001110,
      0b00011101,
      0b00101101,
      0b01001101,
      0b10001101,
      0b00011011,
      0b00101011,
      0b01001011,
      0b00010111,
      0b00100111,
      0b01000111,
      0b10000111,
      0b11111111,
      0b11111111,
  };
  int v = ((~ids[id & 15]) << 2);
  hri_port_set_OUT_reg(PORT_IOBUS, 0, v);
  hri_port_clear_OUT_reg(PORT_IOBUS, 0, ~v & 0x3FC);
}

int32_t i2c_cmd_read(struct i2c_m_sync_desc *i2c, uint16_t reg, uint8_t *buffer, uint8_t length)
{
  struct _i2c_m_msg msg;
  int32_t ret;

  reg = (reg >> 8) | (reg << 8);

  msg.addr = i2c->slave_addr;
  msg.len = 2;
  msg.flags = 0;
  msg.buffer = &reg;

  ret = _i2c_m_sync_transfer(&i2c->device, &msg);

  if (ret != 0)
  {
    /* error occurred */
    return ret;
  }

  msg.flags = I2C_M_STOP | I2C_M_RD;
  msg.buffer = buffer;
  msg.len = length;

  return _i2c_m_sync_transfer(&i2c->device, &msg);
}

/** Read the register_address value from the mlx90632
 *
 * i2c read is processor specific and this function expects to have address of mlx90632 known, as it operates purely on
 * register addresses.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[out] *value pointer to where read data can be written

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
  uint8_t data[2];
  int32_t ret = i2c_cmd_read(&I2C_0, register_address, data, sizeof(data));
  // Endianness
  *value = data[1] | (data[0] << 8);
  return ret;
}

/** Read the register_address value from the mlx90632
 *
 * i2c read is processor specific and this function expects to have address of mlx90632 known, as it operates purely on
 * register addresses.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[out] *value pointer to where read data can be written

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_read32(int16_t register_address, uint32_t *value)
{
  uint8_t data[4];
  int32_t ret = i2c_cmd_read(&I2C_0, register_address, data, sizeof(data));
  // Endianness
  *value = data[2] << 24 | data[3] << 16 | data[0] << 8 | data[1];
  return ret;
}

/** Write value to register_address of the mlx90632
 *
 * i2c write is processor specific and this function expects to have address of mlx90632 known, as it operates purely
 * on register address and value to be written to.
 *
 * @note Needs to be implemented externally
 * @param[in] register_address Address of the register to be read from
 * @param[in] value value to be written to register address of mlx90632

 * @retval 0 for success
 * @retval <0 for failure
 */
int32_t mlx90632_i2c_write(int16_t reg, uint16_t value)
{
  uint8_t data[4];
  data[0] = reg >> 8;
  data[1] = reg;
  data[2] = value >> 8;
  data[3] = value;
  struct _i2c_m_msg msg;

  msg.addr = I2C_0.slave_addr;
  msg.len = 4;
  msg.flags = I2C_M_STOP;
  msg.buffer = &data;

  return _i2c_m_sync_transfer(&I2C_0.device, &msg);
}

/** Blocking function for sleeping in microseconds
 *
 * Range of microseconds which are allowed for the thread to sleep. This is to avoid constant pinging of sensor if the
 * data is ready.
 *
 * @note Needs to be implemented externally
 * @param[in] min_range Minimum amount of microseconds to sleep
 * @param[in] max_range Maximum amount of microseconds to sleep
 */
void usleep(int min_range, int max_range)
{
  delay_us(min_range);
}

/** Blocking function for sleeping in milliseconds
 *
 * milliseconds which are allowed for the thread to sleep. This is to avoid constant pinging of sensor
 * while the measurement is ongoing in sleeping step mode.
 *
 * @note Needs to be implemented externally
 * @param[in] msecs Amount of milliseconds to sleep
 */
void msleep(int msecs)
{
  delay_ms(msecs);
}

void debug(int val, int nd)
{
  while (1)
  {
    int v = val;
    for (int i = 0; i < nd; i++)
    {
      for (int j = 0; j < 200; j++)
      {
        for (int k = 0; k < 8; k++)
        {
          if (v & (1 << k))
            set_led(7 - k);
          delay_us(100);
        }
      }
      v >>= 8;
      set_led(15);
    }
    delay_ms(500);
    break;
  }
}

void debug2(int val, int nd)
{
  while (1)
  {
    int d = 1;
    for (int i = 0; i < nd - 1; i++)
      d *= 10;
    for (int i = 0; i < nd; i++)
    {
      set_led((val / d) % 10);
      d /= 10;
      delay_ms(500);
    }
    set_led(15);
    delay_ms(500);
    break;
  }
}

double pre_ambient, pre_object, ambient, object;

static void alarm_cb(struct calendar_dev *const dev)
{
}

void sleep_ms(int ms)
{
  if (ms == 0)
    return;
  calendar_enable(&CALENDAR_0);
  _calendar_set_counter(&CALENDAR_0.device, 0);
  _calendar_set_comp(&CALENDAR_0.device, ms);
  _calendar_register_callback(&CALENDAR_0.device, alarm_cb);
  sleep(3);
}

void blink(int led, int delay)
{
  set_led(led);
  sleep_ms(delay);
  set_led(15);
  sleep_ms(delay);
}

void pulse(int led, int delay)
{
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < delay; j++)
    {
      set_led(led);
      sleep_ms(i + 1);
      set_led(15);
      sleep_ms(9 - i);
    }
  }
  for (int i = 0; i < 10; i++)
  {
    for (int j = 0; j < delay; j++)
    {
      set_led(led);
      sleep_ms(9 - i);
      set_led(15);
      sleep_ms(i + 1);
    }
  }
}

void turn_off()
{
  int i = 0;
  while (1)
  {
    sleep_ms(900);
    set_led(i++);
    sleep_ms(100);
    set_led(15);
  }
}

void dead_battery()
{
  for (int i = 0; i < 10; i++)
    blink(13, 250);
  turn_off();
}

void broken_sensor()
{
  for (int i = 0; i < 10; i++)
    blink(12, 250);
  turn_off();
}

int temp_to_led(int temp)
{
  if (temp > 320 && temp < 350)
    // standard forehead temperature, assume 36.6
    return 3;

  if (temp < 300)
    return 0;
  if (temp < 310)
    return 1;
  if (temp < 320)
    return 2;

  int led = (temp - 350) / 5 + 4;
  if (led > 13)
    led = 13;
  return led;
}

void battery_check()
{
  uint8_t buffer[2];

  adc_sync_enable_channel(&ADC_0, 0);

  adc_sync_read_channel(&ADC_0, 0, buffer, 1);
  if (buffer[0] < 80)
  {
    // set_led(buffer[0] / 10);
    // sleep_ms(10000);
    //  Battery near dead.
    dead_battery();
  }
  adc_sync_disable_channel(&ADC_0, 0);
}

int main(void)
{
  /* Initializes MCU, drivers and middleware */
  atmel_start_init();
  sleep_ms(100);
  battery_check();

  for (int i = 0; i < 14; i++)
    blink(i, 15);
  for (int i = 0; i < 14; i++)
    blink(13 - i, 15);
  // for (int i = 0; i < 14; i++)
  //   blink(i, 10);

  if (i2c_m_sync_enable(&I2C_0) != 0)
  {
    // Sensor broken.
    broken_sensor();
  }
  i2c_m_sync_set_slaveaddr(&I2C_0, CHIP_ADDRESS, I2C_M_SEVEN);

  /* Check the internal version and prepare a clean start */
  if (mlx90632_init() < 0)
  {
    // Sensor broken.
    broken_sensor();
  }

  uint16_t reg_ctrl;

  mlx90632_i2c_read(MLX90632_REG_CTRL, &reg_ctrl);
  if ((reg_ctrl & 6) != 6)
  {
    mlx90632_i2c_write(MLX90632_REG_CTRL, reg_ctrl & 0xFFF9);
    mlx90632_write_eeprom(MLX90632_EE_CTRL, (reg_ctrl & 0xFFF9) + 6);
    blink(2, 250);
    blink(5, 250);
    blink(8, 250);
  }
  // reg_ctrl |= 6;
  // mlx90632_i2c_write(MLX90632_REG_CTRL, reg_ctrl);
  sleep_ms(100);

  battery_check();

  /* Definition of MLX90632 calibration parameters */
  int16_t ambient_new_raw;
  int16_t ambient_old_raw;
  int16_t object_new_raw;
  int16_t object_old_raw;
  int32_t PR = 0x00587f5b;
  int32_t PG = 0x04a10289;
  int32_t PT = 0xfff966f8;
  int32_t PO = 0x00001e0f;
  int32_t Ea = 4859535;
  int32_t Eb = 5686508;
  int32_t Fa = 53855361;
  int32_t Fb = 42874149;
  int32_t Ga = -14556410;
  int16_t Ha = 16384;
  int16_t Hb = 0;
  int16_t Gb = 9728;
  int16_t Ka = 10752;

  /* Read EEPROM calibration parameters */
  mlx90632_i2c_read32(MLX90632_EE_P_R, &PR);
  mlx90632_i2c_read32(MLX90632_EE_P_G, &PG);
  mlx90632_i2c_read32(MLX90632_EE_P_O, &PO);
  mlx90632_i2c_read32(MLX90632_EE_P_T, &PT);
  mlx90632_i2c_read32(MLX90632_EE_Ea, &Ea);
  mlx90632_i2c_read32(MLX90632_EE_Eb, &Eb);
  mlx90632_i2c_read32(MLX90632_EE_Fa, &Fa);
  mlx90632_i2c_read32(MLX90632_EE_Fb, &Fb);
  mlx90632_i2c_read32(MLX90632_EE_Ga, &Ga);
  mlx90632_i2c_read(MLX90632_EE_Gb, &Gb);
  mlx90632_i2c_read(MLX90632_EE_Ha, &Ha);
  mlx90632_i2c_read(MLX90632_EE_Hb, &Hb);
  mlx90632_i2c_read(MLX90632_EE_Ka, &Ka);

  /*while (1)
  {
    mlx90632_read_temp_raw_extended(&ambient_new_raw, &ambient_old_raw, &object_new_raw);
    pre_ambient = mlx90632_preprocess_temp_ambient_extended(ambient_new_raw, ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object_extended(object_new_raw, ambient_new_raw, ambient_old_raw, Ka);
    mlx90632_set_emissivity(1.0);
    ambient = mlx90632_calc_temp_ambient_extended(ambient_new_raw, ambient_old_raw, PT, PR, PG, PO, Gb);
    object = mlx90632_calc_temp_object_extended(pre_object, pre_ambient, ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

    int temp = (int)(object * 10.0);
    int th = (temp / 100) + (((temp / 10) % 10) << 4) + ((temp % 10) << 8);
    debug(th, 2);
  }*/

  int temp = 0;

  for (int i = 0; i < 10; i++)
  {
    /* Get raw data from MLX90632 */
    mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
    /* Pre-calculations for ambient and object temperature calculation */
    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, Gb);
    pre_object = mlx90632_preprocess_temp_object(object_new_raw, object_old_raw, ambient_new_raw, ambient_old_raw, Ka);
    // mlx90632_set_emissivity(0.98);
    /* Calculate ambient and object temperature */
    ambient = mlx90632_calc_temp_ambient(ambient_new_raw, ambient_old_raw, PT, PR, PG, PO, Gb);
    object = mlx90632_calc_temp_object(pre_object, pre_ambient, Ea, Eb, Ga, Fa, Fb, Ha, Hb);

    int ntemp = (int)(object * 10.0);
    if (ntemp < 290 || ntemp > 420)
      continue;
    if (temp > 300 && ntemp < temp)
      continue;

    int diff = ntemp - temp;
    if (diff < 0)
      diff = -diff;

    if (diff < 5)
    {
      temp = (temp + ntemp) / 2;
      break;
    }
    temp = ntemp;
  }
  // mlx90632_i2c_read(MLX90632_REG_CTRL, &reg_ctrl);
  // mlx90632_i2c_write(MLX90632_REG_CTRL, (reg_ctrl & 0xFFF9) + 2);

  if (temp < 290)
  {
    for (int i = 0; i < 10; i++)
    {
      blink(0, 250);
      blink(1, 250);
    }
    turn_off();
  }

  int led = temp_to_led(temp);
  for (int i = 0; i < 20; i++)
    pulse(led, 5);
  turn_off();
}

/*___________________________________________________
 |  _____                       _____ _ _       _    |
 | |  __ \                     |  __ (_) |     | |   |
 | | |__) |__ _ __   __ _ _   _| |__) || | ___ | |_  |
 | |  ___/ _ \ '_ \ / _` | | | |  ___/ | |/ _ \| __| |
 | | |  |  __/ | | | (_| | |_| | |   | | | (_) | |_  |
 | |_|   \___|_| |_|\__, |\__,_|_|   |_|_|\___/ \__| |
 |                   __/ |                           |
 |  GNU/Linux based |___/  Multi-Rotor UAV Autopilot |
 |___________________________________________________|
  
 ODROID U3 IO Shield Code

 Copyright (C) 2014 Jan Roemisch, Integrated Communication Systems Group, TU Ilmenau

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details. */


/*
 * UART stuff
 */

#define BAUD_RATE  115200
#define UBRR_VAL   ((F_CPU + BAUD_RATE * 8) / (BAUD_RATE * 16) - 1)


/*
 * Common
 */
#define PPM_FLAG_FRAME_COMPLETE  0
#define POWER_FLAG_COMPLETE      1

static volatile uint8_t isr_flags;

/* 
 * PWM stuff
 */

#define PWM_SYNC_BYTE 0x55
#define PWM_1MS       127
#define PWM_2MS       250
#define PWM_COUNT_MAX 4

static enum
{
   PWM_READ_SYNC,
   PWM_READ_COUNT,
   PWM_READ_PWM,
   PWM_READ_CS1,
   PWM_READ_CS2
} pwm_state = PWM_READ_SYNC;

static uint8_t pwm_buf[PWM_COUNT_MAX];
static uint8_t pwm_count;
static uint8_t pwm_index;
static uint8_t pwm_input_flag;
static uint16_t pwm_cs;
static uint32_t pwm_noinput_count;

/* 
 * PPM (Sum) input stuff
 */

#define PPM_PREAMBLE      0xAA55
#define PPM_CHANNELS_MAX  8
#define PPM_FAILSAFE_MAX  5

#define PPM_TIME_SYNC     2100
#define PPM_TIME_NOPULSE  500

static volatile enum
{
  PPM_READ_SYNC,
  PPM_READ_PULSE,
  PPM_READ_CHANNELS
} ppm_state = PPM_READ_SYNC;

static volatile uint16_t ppm_ts_last;
static volatile uint8_t ppm_channel_idx;
static struct ppm_sentence_s
{
  uint16_t preamble;
  uint8_t  nchannels;
  volatile uint16_t values[PPM_CHANNELS_MAX];
  uint16_t checksum;
} __attribute__((packed)) ppm_sentence;

/*
 * Voltage and current
 */
#define POWER_PREAMBLE      0xAB56

#define POWER_VOLTAGE_PIN   1
#define POWER_VOLTAGE_SCALE 21370l
#define POWER_VOLTAGE_DIV   10

#define POWER_CURRENT_PIN   0
#define POWER_CURRENT_SCALE 85333333l
#define POWER_CURRENT_DIV   19


static volatile uint16_t power_raw_voltage;
static volatile uint16_t power_raw_current;

static struct power_sentence_s
{
  uint16_t preamble;
  uint32_t voltage;
  uint32_t current;
  uint16_t checksum;
} __attribute__((packed)) power_sentence;


/*
 * Send queue stuff
 */

#define SEND_QUEUE_SIZE      2

static struct send_queue_s
{
  uint8_t *data;
  uint8_t len;
} send_queue[SEND_QUEUE_SIZE];

static uint8_t send_queue_pos;
static uint8_t send_queue_bpos;

ISR(TIMER1_OVF_vect) {}

ISR(ADC_vect)
{
  if(!(isr_flags & _BV(POWER_FLAG_COMPLETE)))
  {
    switch(ADMUX & 0x0F)
    {
      case POWER_VOLTAGE_PIN:
        power_raw_voltage = ADC;
        ADMUX = (ADMUX & 0xF0) | POWER_CURRENT_PIN;
        break;
      case POWER_CURRENT_PIN:
        power_raw_current = ADC;
        ADMUX = (ADMUX & 0xF0) | POWER_VOLTAGE_PIN;
      
        isr_flags |= _BV(POWER_FLAG_COMPLETE);
    }
  }
}

ISR(PCINT2_vect)
{
  uint16_t ts = TCNT1;
  uint16_t dt = (ts - ppm_ts_last) >> 1;

  ppm_ts_last = ts;
 
  if(dt > PPM_TIME_SYNC)
  {
    if(!(isr_flags & _BV(PPM_FLAG_FRAME_COMPLETE)))
    {
      ppm_state = PPM_READ_PULSE;
      ppm_channel_idx = 0;
    }
  }
  else
  {  
    switch(ppm_state)
    {
      case PPM_READ_SYNC:
        break;
      case PPM_READ_PULSE:
        if(dt > PPM_TIME_NOPULSE)
        {
          ppm_state = PPM_READ_SYNC;
        }
        else
        {
          ppm_state = PPM_READ_CHANNELS;
        }     
        
        break;
      case PPM_READ_CHANNELS:
        ppm_sentence.values[ppm_channel_idx++] = dt;
        
        if(ppm_channel_idx == PPM_CHANNELS_MAX)
        {
          ppm_state = PPM_READ_SYNC;
          isr_flags |= _BV(PPM_FLAG_FRAME_COMPLETE);
        }
        else
        {
          ppm_state = PPM_READ_PULSE;
        }
        
        break;
    }
  }
}

static uint16_t checksum(uint8_t *data, uint8_t size)
{
  #define CRC16         0x8005
  
  uint16_t out = 0;
  uint8_t bits_read = 0, bit_flag;
  
  while (size > 0)
  {
    bit_flag = out >> 15;
      
    out <<= 1;
    out |= (*data >> bits_read) & 1;
      
    bits_read++;
    if (bits_read > 7)
    {
      bits_read = 0;
      data++;
      size--;
    }
      
    if (bit_flag)
      out ^= CRC16;
  }
  
  uint16_t i;
  for (i = 0; i < 16; ++i)
  {
    bit_flag = out >> 15;
    out <<= 1;
    if(bit_flag)
      out ^= CRC16;
  }
  
  uint16_t crc = 0;
  i = 0x8000;
  uint16_t j = 0x0001;
  for (; i != 0; i>>= 1, j <<= 1)
  {
    if (i & out) crc |= j;
  }
  
  uint8_t high = crc >> 8;
  uint8_t low = crc & 0xFF;
  if (high == 0xFF)
    high = 0;
  if (low == 0xFF)
    low = 0;
  return (high << 8) | low;
}


static void pwm_set_timers(uint8_t *pwm, uint8_t count)
{
  switch(count - 1)
  {
    case 3: OCR2B = pwm[3];
    case 2: OCR2A = pwm[2];
    case 1: OCR0B = pwm[1];
    case 0: OCR0A = pwm[0];
  }
}


static uint8_t pwm_parse_frame(uint8_t *pwm, uint8_t c)
{
  uint8_t ret = 0;
  
  switch(pwm_state)
  {
    case PWM_READ_SYNC:
      if (c == PWM_SYNC_BYTE)
      {
        pwm_state = PWM_READ_COUNT;
      }
      break;
    case PWM_READ_COUNT:
      if (c > PWM_COUNT_MAX)
      {
        pwm_state = PWM_READ_SYNC;
      }
      else
      {
        pwm_count = c;
        pwm_index = 0;
        pwm_state = PWM_READ_PWM;
      }
      break;
    case PWM_READ_PWM:
      pwm[pwm_index++] = c;
      if (pwm_count == pwm_index)
      {
        pwm_state = PWM_READ_CS1;
      }
      break;
    case PWM_READ_CS1:
      pwm_cs = c << 8;
      pwm_state = PWM_READ_CS2;
      break;
    case PWM_READ_CS2:
      pwm_cs |= c;
      if (checksum(pwm, pwm_count) == pwm_cs)
      {   
        ret = pwm_count;          
      }
      pwm_state = PWM_READ_SYNC;
      break;
  }
  
  return ret;
}

static void send_queue_enqueue(uint8_t *data, uint8_t len)
{
  uint8_t n;

  for(n = 0; n < SEND_QUEUE_SIZE; n++)
  {
    if(!send_queue[n].len)
    {
      send_queue[n].data = data;
      send_queue[n].len = len;
      break;
    }
  }
}

/*
static void uart_write(uint8_t c)
{
  while(!(UCSR0A & _BV(TXC0))) {}
  UDR0 = c;
}
*/

void setup()
{
  uint8_t n;
  
  /* init serial com */
  Serial.begin(115200);
  
  /* we don't want any unnecessary interrupts */
  TIMSK0 = 0;  /* usually enabled in wiring.c */
  TIMSK1 = 0;
  TIMSK2 = 0;

  isr_flags = 0;

  /* 
   * PWM stuff
   */

  /* PWM output pins */
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(11, OUTPUT);
   
  /* setup timer0 and timer2 for PWM operation */
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS01) | _BV(CS00);

  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS22);

  /* 0% duty cycle (off) for timer0 and timer2 */
  OCR0A = 0;
  OCR0B = 0;

  OCR2A = 0;
  OCR2B = 0;

  /* reset pwm flags */
  pwm_input_flag = 0;
  pwm_noinput_count = 0;

  /* 
   * PPM input stuff
   */

  /* PPM input pin */
  pinMode(2, INPUT);
   
  /* setup timer1 (for PPM input) to normal mode with prescaler /8
   * Also enable overflow int for ADC */
  TCCR1A = 0;
  TCCR1B = _BV(CS11);
  TIMSK1 = _BV(TOIE1);

  /* enable pin change interrupt for PPM input on digital pin 2 */
  PCICR  = _BV(PCIE2);
  PCMSK2 = _BV(PCINT18);

  /* setup static parts of PPM output buffer */
  ppm_sentence.preamble  = PPM_PREAMBLE;
  ppm_sentence.nchannels = PPM_CHANNELS_MAX;

  /*
   * Voltage and current
   */
  power_sentence.preamble = POWER_PREAMBLE;

  /* ADC init */
  ADMUX = _BV(REFS0) | POWER_VOLTAGE_PIN;
  ADCSRB = _BV(ADTS2) | _BV(ADTS1);
  DIDR0 = _BV(ADC5D) | _BV(ADC4D) | _BV(ADC3D) | _BV(ADC2D);
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIE) | _BV(ADATE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

  /*
   * Send queue init
   */

  send_queue_pos = 0;
  send_queue_bpos = 0;
  
  for(n = 0; n < SEND_QUEUE_SIZE; n++)
  {
    send_queue[n].data = NULL;
    send_queue[n].len = 0;
  }
}

void loop()
{
  while(Serial.available())
  {
    uint8_t ret;
    uint8_t c = Serial.read();
    
    ret = pwm_parse_frame(pwm_buf, c);
    if(ret)
    {
      pwm_set_timers(pwm_buf, ret);
      pwm_input_flag = 1;
      pwm_noinput_count = 0;
    }
  }

  if(pwm_input_flag)
  {
    if(++pwm_noinput_count > 0x1ffff)
    {
      uint8_t n;
      
      for(n = 0; n < PWM_COUNT_MAX; n++)
      {
        pwm_buf[n] = PWM_1MS;
      }
      pwm_set_timers(pwm_buf, PWM_COUNT_MAX);
    }
  }

  if(isr_flags & _BV(PPM_FLAG_FRAME_COMPLETE))
  {
    ppm_sentence.checksum = checksum(
                              (uint8_t *)(ppm_sentence.values),
                              sizeof(ppm_sentence.values));

    send_queue_enqueue(
      (uint8_t *)&(ppm_sentence),
      sizeof(struct ppm_sentence_s));
  
    isr_flags &= ~(_BV(PPM_FLAG_FRAME_COMPLETE));
  }

  if(isr_flags & _BV(POWER_FLAG_COMPLETE))
  {
    power_sentence.voltage = ((uint32_t)(power_raw_voltage)
                                 * POWER_VOLTAGE_SCALE)
                               >> POWER_VOLTAGE_DIV;

    power_sentence.current = (((uint32_t)(power_raw_current - 512)
                                 * POWER_CURRENT_SCALE)
                               >> POWER_CURRENT_DIV)
                               + 400l;

    power_sentence.checksum = checksum(
                                (uint8_t *)&(power_sentence.voltage),
                                sizeof(struct power_sentence_s) - 4);
  
    send_queue_enqueue(
      (uint8_t *)&(power_sentence),
      sizeof(struct power_sentence_s));

    isr_flags &= ~(_BV(POWER_FLAG_COMPLETE));
  }

  struct send_queue_s *q = &send_queue[send_queue_pos];

  if(q->len)
  {
    uint8_t c = q->data[send_queue_bpos++];
  
    Serial.write(c);
    
    if(send_queue_bpos == q->len)
    {
      q->len = 0;
      send_queue_pos = (send_queue_pos + 1) % SEND_QUEUE_SIZE;
      send_queue_bpos = 0;
    }
  }
  else
  {
    send_queue_pos = (send_queue_pos + 1) % SEND_QUEUE_SIZE;
  }
}


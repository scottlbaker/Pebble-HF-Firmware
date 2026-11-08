
// ============================================================================
// pebble.ino :: Pebble firmware
//
// The original uSDX firmware was written by Guido (PE1NNZ)
// https://github.com/threeme3/usdx
//
// The uSDX 4.00e firmware is derived from the original uSDX
// and includes improvements to SSB performance.
// https://github.com/GW8RDI/uSDXOpen
//
// This Pebble firmware is derived from the uSDX 4.00e firmware
// The code has been simplified and features that are not used by the
// Pebble-HF radio have been removed
//
// This firmware supports 20M and 40M Pebble versions
// but remember, this is a single band radio and you can only select
// the band that your Pebble is built for.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions: The
// above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software. THE SOFTWARE IS PROVIDED
// "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
// HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
// AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
// CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// ============================================================================

#define VERSION "1.00a"            // firmware version
#define RELDATE "11-07-26"         // firmware release date
#define EEPROM_VERSION  2          // increment when eeprom layout changes

// pin definitions
#define LCD_D4      0   // PD0     (pin 30)
#define LCD_D5      1   // PD1     (pin 31)
#define LCD_D6      2   // PD2     (pin 32)
#define LCD_D7      3   // PD3     (pin  1)
#define LCD_EN      4   // PD4     (pin  2)
#define SPARE1      5   // PD5     (pin  9)
#define ROT_A       6   // PD6     (pin 10)
#define ROT_B       7   // PD7     (pin 11)
#define RX          8   // PB0     (pin 12)
#define SIDETONE    9   // PB1     (pin 13)
#define PWM_OUT    10   // PB2     (pin 14)
#define SPARE2     11   // PB3     (pin 15)
#define DAH        12   // PB4     (pin 16)
#define DIT        13   // PB5     (pin 17)
#define AUDIO1     14   // PC0/A0  (pin 23)
#define AUDIO2     15   // PC1/A1  (pin 24)
#define DVM        16   // PC2/A2  (pin 25)
#define BUTTONS    17   // PC3/A3  (pin 26)
#define LCD_RS     18   // PC4     (pin 27)
#define SDA        18   // PC4     (pin 27)
#define SCL        19   // PC5     (pin 28)

#define BUTTON_PRESSED  digitalRead(BUTTONS)
#define DIT_PRESSED    !digitalRead(DIT)
#define DAH_PRESSED    !digitalRead(DAH)

#define NO_SWAP   0
#define SWAP      1

uint8_t rotxxx_swap = NO_SWAP;
uint8_t rotary_swap = NO_SWAP;

#define BAND_20M  0
#define BAND_40M  1

const char *band_label[2] = {"20M", "40M"};
uint8_t bandsel = BAND_20M;

// band configuration macros
#define INIT_FREQ       (bandsel == BAND_40M ? 7095000UL : 14047500UL)
#define FREQ_LIMIT_LOW  (bandsel == BAND_40M ? 7000000UL : 14000000UL)
#define FREQ_LIMIT_HIGH (bandsel == BAND_40M ? 7300000UL : 14350000UL)

#if (ARDUINO < 10810)
#if (ARDUINO != 10607)
#error "Unsupported Arduino IDE version, use Arduino IDE 1.8.10 or later"
#endif
#endif

#if !(defined(ARDUINO_ARCH_AVR))
#error "Unsupported architecture, select Arduino Uno board definition"
#endif

#if (F_CPU != 16000000)
#error "Unsupported clock frequency, must specify 16MHz clock"
#endif

const char *vfosel_label[] = {"A", "B"};
const char *mode_label[5] = {"LSB", "USB", "CW ", "FM ", "AM "};

#define USERNAME_LENGTH  10
#define CALLSIGN_LENGTH  10

#define PARAM_EEPROM     0x010   // 0x010 -> 0x100
#define CWMSG_EEPROM     0x100   // 0x100 -> 0x2FF
#define PRESET_EEPROM    0x300   // 0x300 -> 0x31F
#define USERNAME_ADDRESS 0x3E0   // 0x3E0 -> 0x3EF
#define CALLSIGN_ADDRESS 0x3F0   // 0x3F0 -> 0x3FF

#define N_PARAMS   33
#define I_PARAMS    4
#define ALL_PARAMS (N_PARAMS + I_PARAMS)

enum params_t {
  _NULL,
  VOLUME,        // 1
  TONE_VOL,      // 2
  MODE,          // 3
  FILTER,        // 4
  VFOSEL,        // 5
  RIT,           // 6
  AGC,           // 7
  NR,            // 8
  ATT,           // 9
  ATT2,          // 10
  SMETER,        // 11
  CWDEC,         // 12
  CWTONE,        // 13
  CWINTERVAL,    // 14
  SEMIQSK,       // 15
  KEY_WPM,       // 16
  KEY_MODE,      // 17
  KEY_PIN,       // 18
  KEY_TX,        // 19
  VOX,           // 20
  VOXGAIN,       // 21
  DRIVE,         // 22
  PWM_MIN,       // 23
  PWM_MAX,       // 24
  SIFXTAL,       // 25
  IQ_ADJ,        // 26
  QUAD_ACTIVE,   // 27
  ROTARY_SWAP,   // 28
  BAND,          // 29
  PRESET,        // 30   // push BE to open sub-menu
  CWMSGS,        // 31   // push BE to open sub-menu
  USER_INFO,     // 32   // push BE to open sub-menu
  FW_VER,        // 33

  // Invisible parameters that are always saved/loaded
  FREQA,         // 34
  FREQB,         // 35
  MODEA,         // 36
  MODEB,         // 37

  // Optional parameters for debug and calibration
  CALIB,         // 38
  SR,            // 39
  CPULOAD,       // 40
  PARAM_A,       // 41
  PARAM_B,       // 42
  PARAM_C,       // 43

  // Internal IDs used only while editing a CW message
  CWMSG1 = 100,
  CWMSG2,
  CWMSG3,
  CWMSG4,
  CWMSG5,
  CWMSG6,

  MY_NAME,
  MY_CALL,

  ALL = 0xff
};

enum step_t {
  STEP_10M,
  STEP_1M,
  STEP_500k,
  STEP_100k,
  STEP_10k,
  STEP_1k,
  STEP_500,
  STEP_100,
  STEP_10,
  STEP_1
};

uint32_t stepsizes[10] = {10000000, 1000000, 500000, 100000, 10000,
                          1000,     500,     100,    10,     1};
volatile uint8_t stepsize = STEP_1k;
uint8_t prev_stepsize[] = {STEP_1k, STEP_500}; // stepsize for SSB, CW

// name and callsign can be edited
char my_name[USERNAME_LENGTH] = {"MYNAME"};
char my_call[CALLSIGN_LENGTH] = {"MYCALL"};

// for stored CW messages $N and $C are shorthand for
// name and callsign (expanded when message sent)
#define CW_MESSAGE_LENGTH 48
char cw_msg[6][CW_MESSAGE_LENGTH] = {
    "CQ POTA DE $C",
    "CQ SOTA DE $C",
    "$C",
    "GE TKS 5NN 5NN NAME IS $N HW?",
    "FB RPTR TX 5W ANT EFW 73 CUAGN",
    "73 GL TU EE"
};

// string copy
void cpy(char* dest, char* src) {
  char* ptr = dest;
  while ((*dest++ = *src++) != '\0');
}

// string cat
void cat(char* dest, char* src) {
  while (*dest != '\0') dest++;
  while ((*dest++ = *src++) != '\0');
}

// configuration switches
#define KEY_CLICK 1
#define AF_OUT    1

////////////////////////////////////////////////////////////////////////
// SSB TX experimentation - by 9M2GRC
// Only one of EXPSSB01 / EXPSSB02 / EXPSSB03 should be enabled
////////////////////////////////////////////////////////////////////////

//#define EXPSSB01
//#define EXPSSB02
#define EXPSSB03
#define FIXEDSCALE

#define SI5351_ADDR 0x60 // si5351 I2C address
#define F_XTAL 26000000  // si5351 clock
#undef F_CPU
#define F_CPU 20007000 // CPU actual clock
#undef F_MCU
#define F_MCU 20000000 // CPU reference clock

// si5351 clock controls
#define TX1RX0 0b11111011
#define TX1RX1 0b11111000
#define TX0RX1 0b11111100
#define TX0RX0 0b11111111

// Iambic Morse Code Keyer Sketch, Contribution by Uli, DL2DBG (c) 2009 Steven
// T. Elliott Source: http://openqrp.org  p=343 Trimmed by Bill Bishop -
// wrb[at]wrbishop.com.  This library is free software; you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public License as
// published by the Free Software Foundation; either version 2.1 of the License,
// or any later version. This library is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
// General Public License for more details: Free Software Foundation, Inc., 59
// Temple Place, Suite 330, Boston, MA  02111-1307  U

// keyer control bit definitions
#define DIT_L     0x01   // Dit latch
#define DAH_L     0x02   // Dah latch
#define DIT_PROC  0x04   // Dit is being processed
#define PDLSWAP   0x08   // 0 for normal, 1 for swap
#define IAMBICA   0x00   // Iambic A
#define IAMBICB   0x10   // Iambic B

#define STRAIGHT  0      // Straight  key
#define IAMBIC_A  1      // Iambic A
#define IAMBIC_B  2      // Iambic B
#define ULTIMATIC 3      // Ultimatic mode

uint8_t keyer_speed = 25;
unsigned long ditTime; // milliseconds per dit
uint8_t keyerControl;
uint8_t keyerState;
uint8_t keyer_mode  = STRAIGHT;
uint8_t keyer_swap  = NO_SWAP;
uint8_t last_paddle = 0;  // 0=dit, 1=dah (used by Ultimatic mode)
uint8_t prev_dit_state = HIGH;
uint8_t prev_dah_state = HIGH;

uint32_t ktimer;
uint8_t  key_state;

// State machine states
enum KSTYPE {
  IDLE,
  CHK_DIT,
  CHK_DAH,
  KEYED_PREP,
  KEYED,
  INTER_ELEMENT
};

// Latch dit/dah press; also track most recent paddle press for Ultimatic
void update_PaddleLatch() {
  uint8_t dit_now = digitalRead(DIT);
  uint8_t dah_now = digitalRead(DAH);
  if (dit_now == LOW) {
    keyerControl |= keyer_swap ? DAH_L : DIT_L;
    if (prev_dit_state == HIGH) {
      last_paddle = keyer_swap ? 1 : 0; // new press: update last_paddle
    }
  }
  if (dah_now == LOW) {
    keyerControl |= keyer_swap ? DIT_L : DAH_L;
    if (prev_dah_state == HIGH) {
      last_paddle = keyer_swap ? 0 : 1; // new press: update last_paddle
    }
  }
  prev_dit_state = dit_now;
  prev_dah_state = dah_now;
}

// Calculate new time constants based on wpm value
void loadWPM(uint8_t wpm) {
  ditTime = (1500) / wpm;  // 1500 = (1200 * 20/16)
}

static uint8_t practice = false;
static int8_t prev_mode;
volatile uint8_t vox = 0;

#include <avr/sleep.h>
#include <avr/wdt.h>

// Custom pinMode replacement using direct register access
void fastpinMode(uint8_t pin, uint8_t mode) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg, *out;
  if (port == NOT_A_PIN) return;
  reg = portModeRegister(port);   // Pointer to DDRx
  out = portOutputRegister(port); // Pointer to PORTx
  if (mode == INPUT) {
    *reg &= ~bit; // Clear DDRx bit (Input)
    *out &= ~bit; // Clear PORTx bit (Disable Pull-up)
  } else if (mode == INPUT_PULLUP) {
    *reg &= ~bit; // Clear DDRx bit (Input)
    *out |= bit;  // Set PORTx bit (Enable Pull-up)
  } else {        // OUTPUT
    *reg |= bit;  // Set DDRx bit (Output)
  }
}

// Custom digitalWrite replacement using direct register access
void fastdigitalWrite(uint8_t pin, uint8_t val) {
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;
  if (port == NOT_A_PIN) return;
  out = portOutputRegister(port); // Pointer to PORTx
  if (val == LOW) {
    *out &= ~bit; // Clear PORTx bit (Set Low)
  } else {        // HIGH
    *out |= bit;  // Set PORTx bit (Set High)
  }
}

class LCD : public Print {
public:
#define _dn 0 // PD0 to PD3 connect to D4 to D7 on the display
#define _en 4 // PD4 - MUST have pull-up resistor
#define _rs 4 // PC4 - MUST have pull-up resistor
#define LCD_RS_HI() DDRC &= ~(1 << _rs); asm("nop");
#define LCD_RS_LO() DDRC |= 1 << _rs;
#define LCD_EN_LO() PORTD &= ~(1 << _en);
#define LCD_EN_HI() PORTD |= (1 << _en);
#define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en

  void begin(uint8_t x = 0, uint8_t y = 0) {
    // bool reinit = (x == 0) && (y == 0);
    DDRD |= 0xf << _dn | 1 << _en; // Make data, EN outputs
    DDRC |= 1 << _rs;
    delayMicroseconds(50000);
    LCD_RS_LO();
    LCD_EN_LO();
    cmd(0x33);                // Ensures display is in 8-bit mode
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(150);   // Ensures display is in 8-bit mode
    cmd(0x32);                // Puts display in 4-bit mode
    cmd(0x28);                // Function set: 2-line, 5x8
    cmd(0x0c);                // Display on
    cmd(0x01); // Clear display
    delay(3);  // Allow to execute Clear on display
    cmd(0x06); // Entrymode: left, shift-dec
  }
  void nib(uint8_t b) {
    noInterrupts();
    PORTD = LCD_PREP_NIBBLE(b);
    delayMicroseconds(4);
    LCD_EN_LO();
    interrupts();
    delayMicroseconds(60);
  }
  void cmd(uint8_t b) {
    nib(b >> 4);
    nib(b & 0xf);
  }
  size_t write(uint8_t b) {
    noInterrupts();
    uint8_t nibh = LCD_PREP_NIBBLE(b >> 4);
    PORTD = nibh;
    uint8_t nibl = LCD_PREP_NIBBLE(b & 0xf);
    LCD_RS_HI();
    LCD_EN_LO();
    PORTD = nibl;
    LCD_RS_LO();
    LCD_RS_HI();
    LCD_EN_LO();
    LCD_RS_LO();
    interrupts();
    delayMicroseconds(60); // Execution time  (37+4)*1.25 us
    return 1;
  }
  void setCursor(uint8_t x, uint8_t y) { cmd(0x80 | (x + y * 0x40)); }
  void cursor() { cmd(0x0e); }
  void noCursor() { cmd(0x0c); }
  void noDisplay() { cmd(0x08); }
  void createChar(uint8_t l, uint8_t glyph[]) {
    cmd(0x40 | ((l & 0x7) << 3));
    for (uint8_t i=0; i<8; i++)
      write(glyph[i]);
  }
};

// This class spoofs display contents and cursor state
template <class parent>
class Display : public parent {
  public:
};

Display<LCD> lcd;

volatile int8_t  encoder_val = 0;
volatile uint8_t enc_state = 0;
volatile int8_t  enc_accum = 0;

// Interrupt on rotary encoder turn
ISR(PCINT2_vect) {
  uint8_t curr = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  if (rotary_swap) curr = ((curr & 1) << 1) | (curr >> 1);
  static const int8_t table[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  int8_t delta = table[(enc_state << 2) | curr];
  enc_state = curr;
  if (delta) {
    if ((delta>0 && enc_accum<0)||(delta<0 && enc_accum>0)) enc_accum = 0;
    enc_accum += delta;
    if (enc_accum >= 4) {
      encoder_val++;
      enc_accum = 0;
    } else if (enc_accum <= -4) {
      encoder_val--;
      enc_accum = 0;
    }
  }
}

// initialise the encoder state machine
void initEncoder() {
  enc_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  if (rotary_swap) enc_state = ((enc_state & 1) << 1) | (enc_state >> 1);
  enc_accum = 0;
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);
  PCICR  |= (1 << PCIE2);
  interrupts();
}

class I2C {
public:
#define I2C_DELAY 4
#define I2C_DDR DDRC
#define I2C_PIN PINC
#define I2C_PORT PORTC
#define I2C_SDA (1 << 4) // PC4
#define I2C_SCL (1 << 5) // PC5
#define DELAY(n) for (uint8_t i = 0; i != n; i++) asm("nop");
#define I2C_SDA_GET() I2C_PIN &I2C_SDA
#define I2C_SCL_GET() I2C_PIN &I2C_SCL
#define I2C_SDA_HI() I2C_DDR &= ~I2C_SDA;
#define I2C_SDA_LO() I2C_DDR |= I2C_SDA;
#define I2C_SCL_HI() I2C_DDR &= ~I2C_SCL; DELAY(I2C_DELAY);
#define I2C_SCL_LO() I2C_DDR |= I2C_SCL;  DELAY(I2C_DELAY);

  I2C() {
    I2C_PORT &= ~(I2C_SDA | I2C_SCL);
    I2C_SCL_HI();
    I2C_SDA_HI();
    suspend();
  }
  ~I2C() {
    I2C_PORT &= ~(I2C_SDA | I2C_SCL);
    I2C_DDR &= ~(I2C_SDA | I2C_SCL);
  }
  inline void start() {
    resume(); // prepare for I2C
    I2C_SCL_LO();
    I2C_SDA_HI();
  }
  inline void stop() {
    I2C_SDA_LO();
    I2C_SCL_HI();
    I2C_SDA_HI();
    I2C_DDR &= ~(I2C_SDA | I2C_SCL);
    suspend();
  }
#define SendBit(data, mask)                                                    \
  if (data & mask) {                                                           \
    I2C_SDA_HI();                                                              \
  } else {                                                                     \
    I2C_SDA_LO();                                                              \
  }                                                                            \
  I2C_SCL_HI();                                                                \
  I2C_SCL_LO();

  inline void SendByte(uint8_t data) {
    SendBit(data, 1 << 7);
    SendBit(data, 1 << 6);
    SendBit(data, 1 << 5);
    SendBit(data, 1 << 4);
    SendBit(data, 1 << 3);
    SendBit(data, 1 << 2);
    SendBit(data, 1 << 1);
    SendBit(data, 1 << 0);
    I2C_SDA_HI(); // recv ACK
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SCL_LO();
  }
  inline uint8_t RecvBit(uint8_t mask) {
    I2C_SCL_HI();
    uint16_t i = 60000;
    // wait util slave release SCL to HIGH or timeout at 3ms
    for (; !(I2C_SCL_GET()) && i; i--);
    uint8_t data = I2C_SDA_GET();
    I2C_SCL_LO();
    return (data) ? mask : 0;
  }
  inline uint8_t RecvByte(uint8_t last) {
    uint8_t data = 0;
    data |= RecvBit(1 << 7);
    data |= RecvBit(1 << 6);
    data |= RecvBit(1 << 5);
    data |= RecvBit(1 << 4);
    data |= RecvBit(1 << 3);
    data |= RecvBit(1 << 2);
    data |= RecvBit(1 << 1);
    data |= RecvBit(1 << 0);
    if (last) {
      I2C_SDA_HI(); // NACK
    } else {
      I2C_SDA_LO(); // ACK
    }
    DELAY(I2C_DELAY);
    I2C_SCL_HI();
    I2C_SDA_HI(); // restore SDA for read
    I2C_SCL_LO();
    return data;
  }
  inline void resume() {
  }
  inline void suspend() {
    I2C_SDA_LO(); // pin sharing SDA/LCD_RS
  }

  void begin() {};
  void beginTransmission(uint8_t addr) {
    start();
    SendByte(addr << 1);
  };
  bool write(uint8_t byte) {
    SendByte(byte);
    return 1;
  };
  uint8_t endTransmission() {
    stop();
    return 0;
  };
};

I2C i2c;
class SI5351 {
public:
  volatile int32_t _fout;
  volatile uint8_t _div;
  volatile uint16_t _msa128min512;
  volatile uint32_t _msb128;
  volatile uint8_t pll_regs[8];
  volatile uint32_t fxtal = F_XTAL;

#define BB0(x) ((uint8_t)(x)) // bash byte x of int32_t
#define BB1(x) ((uint8_t)((x) >> 8))
#define BB2(x) ((uint8_t)((x) >> 16))

#define FAST __attribute__((optimize("Ofast")))
#define _MSC 0x10000

  inline void FAST freq_calc_fast(int32_t df) {
    uint32_t msb128 = _msb128 + ((int64_t)(_div * (int32_t)df) * _MSC * 128) / fxtal;
    uint16_t msp1 = _msa128min512 + msb128 / _MSC;
    uint16_t msp2 = msb128;
    pll_regs[4] = BB0(msp1);
    pll_regs[5] = ((_MSC & 0xF0000) >> (16 - 4));
    pll_regs[6] = BB1(msp2);
    pll_regs[7] = BB0(msp2);
  }

  inline void SendPLLRegisterBulk() {
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(26 + 0 * 8 + 4);  // write to PLLA
    i2c.SendByte(pll_regs[4]);
    i2c.SendByte(pll_regs[5]);
    i2c.SendByte(pll_regs[6]);
    i2c.SendByte(pll_regs[7]);
    i2c.stop();
  }

  void SendRegister(uint8_t reg, uint8_t *data, uint8_t n) {
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    while (n--)
      i2c.SendByte(*data++);
    i2c.stop();
  }

  void SendRegister(uint8_t reg, uint8_t val) {
    i2c.start();
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.SendByte(val);
    i2c.stop();
  }

  int16_t iqmsa; // to detect a need for a PLL reset

  enum ms_t {
    PLLA = 0,
    PLLB = 1,
    MSNA = -2,
    MSNB = -1,
    MS0 = 0,
    MS1 = 1,
    MS2 = 2,
    MS3 = 3,
    MS4 = 4,
    MS5 = 5
  };

  void ms(int8_t n, uint32_t div_nom, uint32_t div_denom, uint8_t pll = PLLA,
          uint8_t _int = 0, uint16_t phase = 0, uint8_t rdiv = 0) {
    uint16_t msa;
    uint32_t msb, msc, msp1, msp2, msp3;
    // integer part: msa must be in range 15..90 for
    // PLL, 8+1/1048575..900 for MS
    msa = div_nom / div_denom;
    // To satisfy the MSx_INT=1 requirement of AN619, section 4.1.3
    // which basically says that for MS divider a value of 4 and
    // integer mode must be used
    if (msa == 4) _int = 1;
    msb = (_int) ? 0 : (((uint64_t)(div_nom % div_denom) * _MSC) / div_denom);
    msc = (_int) ? 1 : _MSC;
    msp1 = 128 * msa + 128 * msb / msc - 512;
    msp2 = 128 * msb - 128 * msb / msc * msc;
    msp3 = msc;
    uint8_t ms_reg2 = BB2(msp1) | (rdiv << 4) | ((msa == 4) * 0x0C);
    uint8_t ms_regs[8] = {BB1(msp3), BB0(msp3),
                          ms_reg2,   BB1(msp1),
                          BB0(msp1), BB2(((msp3 & 0x0F0000) << 4) | msp2),
                          BB1(msp2), BB0(msp2)};

    SendRegister(n * 8 + 42, ms_regs, 8); // Write to MSx
    if (n < 0) {
      // MSNx PLLn: 0x40=FBA_INT; 0x80=CLKn_PDN
      SendRegister(n + 16 + 8, 0x80 | (0x40 * _int));
    } else {
      // MSx CLKn: 0x0C=PLLA,0x2C=PLLB local msynth;
      // 3=8mA; 0x40=MSx_INT; 0x80=CLKx_PDN
      SendRegister(n+16, ((pll) * 0x20) | 0x0C | 3 | (0x40 * _int));
      // make sure to configure MS in
      // fractional-mode and perform reset afterwards
      SendRegister(n+165, (!_int) * phase * msa /  90);
    }
  }

  void phase(int8_t n, uint32_t div_nom, uint32_t div_denom, uint16_t phase) {
    SendRegister(n + 165, phase * (div_nom / div_denom) / 90);
  }

  void reset() {
    SendRegister(177, 0xA0);
  }

  void oe(uint8_t mask) {
    SendRegister(3, ~mask);
  }

  // Set a CLK0,1,2 to fout Hz with phase i, q (on PLLA)
  void freq(int32_t fout, uint16_t i, uint16_t q) {
    uint8_t rdiv = 0;     // CLK pin sees fout/(2^rdiv)
    if (fout > 300000000) {
      i /= 3;
      q /= 3;
      fout /= 3;
    } // for higher freqs, use 3rd harmonic
    if (fout < 500000) {
      rdiv = 7;
      fout *= 128;
    } // Divide by 128 for fout 4..500kHz
    uint16_t d;
    if (fout < 30000000)
      d = (16 * fxtal) / fout;
    else
      d = (32 * fxtal) / fout;
    if (fout < 3500000)
      d = (7 * fxtal) / fout;
    if (fout > 140000000)
      d = 4;
    if (d % 2)
      d++; // even numbers preferred for divider (AN619 p.4 and p.6)
    if ((d * (fout - 5000) / fxtal) != (d * (fout + 5000) / fxtal))
      d += 2; // Test if multiplier remains same for freq deviation +/- 5kHz
    uint32_t fvcoa = d * fout; // variable PLLA VCO frequency at integer
    ms(MSNA, fvcoa, fxtal); // PLLA in fractional mode
    ms(MS0, fvcoa, fout, PLLA, 0, i,
       rdiv); // Multisynth stage with integer divider but in frac mode due to
              // phase setting
    ms(MS1, fvcoa, fout, PLLA, 0, q, rdiv);
    ms(MS2, fvcoa, fout, PLLA, 0, 0, rdiv);
    if (iqmsa != (((int8_t)i - (int8_t)q) * ((int16_t)(fvcoa / fout)) / 90)) {
      iqmsa = ((int8_t)i - (int8_t)q) * ((int16_t)(fvcoa / fout)) / 90;
      reset();
    }
    oe(0b00000011); // output enable CLK0, CLK1
    _fout = fout;   // cache
    _div = d;
    _msa128min512 = fvcoa / fxtal * 128 - 512;
    _msb128 = ((uint64_t)(fvcoa % fxtal) * _MSC * 128) / fxtal;
  }

  void freqb(uint32_t fout) { // Set a CLK2 to fout Hz (on PLLB)
    uint16_t d = (16 * fxtal) / fout;
    if (d % 2)
      d++; // even numbers preferred for divider (AN619 p.4 and p.6)
    uint32_t fvcoa = d * fout; // Variable PLLA VCO frequency at integer
                               // multiple of fout at around 27MHz*16 = 432MHz
    ms(MSNB, fvcoa, fxtal);
    ms(MS2, fvcoa, fout, PLLB, 0, 0, 0);
  }

  uint8_t RecvRegister(uint8_t reg) {
    i2c.start(); // Data write to set the register address
    i2c.SendByte(SI5351_ADDR << 1);
    i2c.SendByte(reg);
    i2c.stop();
    i2c.start(); // Data read to retrieve the data from the set address
    i2c.SendByte((SI5351_ADDR << 1) | 1);
    uint8_t data = i2c.RecvByte(true);
    i2c.stop();
    return data;
  }
  void powerDown() {
    SendRegister(3, 0b11111111);  // Disable all CLK outputs
    SendRegister(24, 0b00000000); // LOW state when disabled
    SendRegister(25, 0b00000000); // LOW state when disabled
    for (uint8_t addr = 16; addr != 24; addr++)
      SendRegister(addr, 0b10000000); // Conserve power when output is disabled
    SendRegister(187, 0);             // Disable fanout (power-safe)
    // To initialise things as they should:
    SendRegister(149, 0);          // Disable spread spectrum enable
    SendRegister(183, 0b11010010); // Internal CL = 10 pF (default)
  }
#define SI_CLK_OE 3
};
static SI5351 si5351;

#ifdef DEBUG
static uint32_t sr = 0;
static uint32_t cpu_load = 0;
volatile uint16_t param_a = 0;
volatile int16_t param_b = 0;
volatile int16_t param_c = 0;
#endif

static uint32_t save_event_time = 0;
static uint8_t vox_tx = 0;
static uint8_t vox_sample = 0;
static uint16_t vox_adc = 0;

// PWM value for which PA reaches its minimum:
static uint8_t pwm_min = 0;

// PWM value for which PA reaches its maximum
static uint8_t pwm_max = 160;

#define N_FILT 7
uint8_t prev_filt[] = {0, 4};

const char *offon_label[2] = {"OFF", "ON"};
const char *filt_label[N_FILT + 1] = {"Full", "3000", "2400", "1800",
                                      "500",  "200",  "100",  "50"};

const char *stepsize_label[] = {"10M", "1M",  ".5M", "100k", "10k",
                                "1k",  ".5k", "100", "10",   "1"};
const char *att_label[] = {"0dB",   "-13dB", "-20dB", "-33dB",
                           "-40dB", "-53dB", "-60dB", "-73dB"};
const char *smode_label[] = {"OFF", "dBm", "S", "Sbar", "WPM"};

const char *agc_label[] = {"OFF", "FAST", "SLOW"};
const char *cw_tone_label[] = {"600", "700"};
const char *keyer_mode_label[] = { "STRAIGHT", "IAMBIC A", "IAMBIC B", "ULTIMATIC"};

enum mode_t { LSB, USB, CW, FM, AM };
volatile uint8_t mode = USB;
volatile uint16_t numSamples = 0;

volatile uint8_t tx = 0;
volatile uint8_t filt = 0;

inline void _vox(bool trigger) {
  if (trigger) {
    tx = (tx) ? 254 : 255;
  } else {
    if (tx) tx--;
  }
}

#define F_SAMP_TX 4800
#define _UA 600
#define MAX_DP ((filt == 0) ? _UA : (filt == 3) ? _UA / 4 : _UA / 2)
#define _atan2(z) (_UA / 8 + _UA / 22 - _UA / 22 * z) * z

// simplification .. not accurate at all, but fast
inline int16_t arctan3(int16_t q, int16_t i) {
  int16_t r;
  if (abs(q) > abs(i))
    r = _UA / 4 - _atan2(abs(i) / abs(q));
  else
    r = (i == 0) ? 0 : _atan2(abs(q) / abs(i));
  r = (i < 0) ? _UA / 2 - r : r;
  return (q < 0) ? -r : r;
}

#define magn(i, q) (abs(i) > abs(q) ? abs(i) + (abs(q) / 4) : abs(q) + (abs(i) / 4))

uint8_t lut[256];
volatile uint8_t amp;
volatile uint8_t vox_thresh = (1 << 2);
volatile uint8_t drive = 2;

static uint8_t quad_enabled = false;
volatile uint8_t quad = 0;

inline int16_t ssb(int16_t in) {
  static int16_t dc, z1;
  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for (j = 0; j != 15; j++)
    v[j] = v[j + 1];

////////////////////////////////////////////////////////////////////////
// SSB TX experimentation - EXPSSB01 / EXPSSB02 / EXPSSB03
// 03/02/2026 - 9M2GRC
////////////////////////////////////////////////////////////////////////

#ifdef EXPSSB03
  // Experimental DC-before-compression voice path; 16-bit arithmetic only.
  // dc and z1 retain the existing static int16_t declarations above.
  dc = (in + dc) / 2;              // same HP coefficient as uploaded EXPSSB01
  int16_t mic = in - dc;           // remove offset BEFORE nonlinear processing
  int16_t ac = mic * 2 + z1;       // original gain and delayed-state ordering
  z1 = (mic - 12 * z1) / 13;       // preserve uploaded EXPSSB01 shaping

  if (ac > 250) ac = 250 + (ac - 250) / 2;
  else if (ac < -250) ac = -250 - (-250 - ac) / 2;

  // Protect the existing 16-bit Hilbert weighted sum under overload.
  // |v|<=320 => |sum|<=2*320*(2+8+21+16)=30080 < 32767.
  // This is an overload rail, not an ALC or substitute for correct mic level.
  if (ac > 320) ac = 320;
  else if (ac < -320) ac = -320;
  v[15] = ac;

#else // original experimental/default branches preserved below
#ifdef EXPSSB01
  int16_t ac = in * 2; //   6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;        // lpf
  z1 = (in - (8) * z1) / (8 + 1); // lpf
  //z1 = (in - (2) * z1) / (2 + 1);
  //z1 = (in - (4) * z1) / (4 + 1);
  //z1 = (in - 12 * z1) / 13;
  // Smoothing clipping limiter
  if (ac > 250) {
    ac = 250 + (ac - 250) / 2;
  } else if (ac < -250) {
    ac = -250 - (-250 - ac) / 2;
  }
  //dc = (ac + (2) * dc) / (2 + 1);
  //dc = (ac + (1) * dc) / (1 + 1);
  //dc = (ac + dc) / 2;
  dc = (ac + (2) * dc) / (2 + 1);
  v[15] = (ac - dc);
#else
#ifdef EXPSSB02
  int16_t ac = in * 2;

  // Smooth copy of the microphone input
  z1 = (3 * z1 + in) / 4;

  // Gentle treble emphasis: half the previous boost
  ac += (in - z1) / 2;
  //ac += (in - z1) / 4;
  //ac += (in - z1) / 8;

  // Smoothing clipping limiter
  if (ac > 250) {
    ac = 250 + (ac - 250) / 2;
  } else if (ac < -250) {
    ac = -250 - (-250 - ac) / 2;
  }
  dc = (ac + (2) * dc) / (2 + 1);
  v[15] = ac - dc;
#else // Original SSB process
  int16_t ac = in * 2;             // 6dB gain (justified since lpf/hpf is losing -3dB)
  ac = ac + z1;                    // lpf
  z1 = (in - (2) * z1) / (2 + 1);  // lpf: notch at Fs/2 (alias rejecting)
  dc = (ac + (2) * dc) / (2 + 1);  // hpf: slow average
  v[15] = (ac - dc);               // hpf (dc decoupling)
#endif // EXPSSB02
#endif // EXPSSB01
#endif // EXPSSB03
  i = v[7] *
      2; // 6dB gain for i, q  (to prevent quanitization issues in hilbert
         // transformer and phase calculation, corrected for magnitude calc)
  q = ((v[0] - v[14]) * 2 + (v[2] - v[12]) * 8 + (v[4] - v[10]) * 21 +
       (v[6] - v[8]) * 16) / 64 + (v[6] - v[8]);

  // Hilbert transform, 40dB side-band rejection in
  // 400..1900Hz (@4kSPS) when used in image-rejection
  // scenario; (Hilbert transform require 5 additional bits)

  uint16_t _amp = magn(i / 2, q / 2); // -6dB gain (correction)

  _vox(_amp > vox_thresh);
#ifdef FIXEDSCALE
  if (drive >= 8 || _amp >= (255u >> drive))
    _amp = 255;
  else
    _amp <<= drive;
#else
  _amp = _amp << (drive);
  _amp = ((_amp > 255) || (drive == 8)) ? 255 : _amp;
#endif // FIXEDSCALE
  amp = (tx) ? lut[_amp] : 0;

  static int16_t prev_phase;
  int16_t phase = arctan3(q, i);

  int16_t dp = phase - prev_phase; // phase difference and restriction
  prev_phase = phase;

  if (dp < 0) dp = dp + _UA; // make negative phase shifts positive
  if (dp >= (_UA / 2)) {
    if (quad_enabled) {
      dp = dp - _UA / 2;
      quad = !quad;
    }
  }

#ifdef MAX_DP
  if (dp > MAX_DP) {
    // dp should be less than half unit-angle
    prev_phase = phase - (dp - MAX_DP);
    dp = MAX_DP;
  }
#endif
  // calculate frequency-difference based on phase-difference
  if (mode == USB)
    return dp * (F_SAMP_TX / _UA);
  else
    return dp * (-F_SAMP_TX / _UA);
}

#define MIC_ATTEN  0
volatile int8_t volume = 12;

// This is the ADC ISR, issued with sample-rate via timer1 compb interrupt.
// It performs in real-time the ADC sampling, calculation of SSB
// phase-differences, calculation of SI5351 frequency registers and send the
// registers to SI5351 over I2C.
static int16_t _adc;
void dsp_tx() {
  int16_t adc;
  adc = ADC;
  ADCSRA |= (1 << ADSC);
  // submit frequency registers to SI5351
  si5351.SendPLLRegisterBulk();
  if (quad_enabled) {
    // invert CLK2 in case of a huge phase-change
    si5351.SendRegister(18, (quad) ? 0x1f : 0x0f);
  }
  OCR1BL = amp;    // submit amplitude to PWM register
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  // convert analog input into phase-shifts by periodic frequency shifts
  int16_t df = ssb(_adc >> MIC_ATTEN);
  adc += ADC;
  ADCSRA |= (1 << ADSC);
  si5351.freq_calc_fast(df); // calculate SI5351 registers
  adc += ADC;
  ADCSRA |= (1 << ADSC);
#define AF_BIAS 32
  _adc = (adc / 4 - (512 - AF_BIAS));
  if (tx == 1) {
    OCR1BL = 0;
    si5351.SendRegister(SI_CLK_OE, TX0RX0);
  } // disable carrier
  if (tx == 255) {
    si5351.SendRegister(SI_CLK_OE, TX1RX0);
  } // enable carrier
}

volatile uint16_t acc;
volatile uint32_t cw_offset;
volatile uint8_t tone_vol = 12;
volatile uint8_t cw_tone = 0;
const uint32_t tones[] = {(F_MCU*600ULL)/20000000, (F_MCU*700ULL)/20000000};
volatile int8_t p_sin = 0;       // initialized with A*sin(0) = 0
volatile int8_t n_cos = 448 / 4; // initialized with A*cos(t) = A

// Minsky circle sample [source:
inline void process_minsky() {
  // alpha = f_tone * 2 * pi / fs
  int8_t alpha127 = tones[cw_tone] * 798 / F_SAMP_TX;
  p_sin += alpha127 * n_cos / 127;
  n_cos -= alpha127 * p_sin / 127;
}

// CW Key-click shaping
// raised-cosine(i) = 255 * sq(cos(PI/2 * i/32))
const uint8_t ramp[] PROGMEM = {255, 254, 252, 249, 245, 239, 233, 226,
                                217, 208, 198, 187, 176, 164, 152, 139,
                                127, 115, 102, 90,  78,  67,  56,  46,
                                37,  28,  21,  15,  9,   5,   2};

void dummy() {}

void dsp_tx_cw() {
#ifdef KEY_CLICK
  // ramp up amplitude
  if (OCR1BL < lut[255]) {
    for (uint16_t i = 31; i != 0; i--) { // soft rising slope against key-clicks
      OCR1BL = lut[pgm_read_byte_near(ramp[i])];
      delayMicroseconds(60);
    }
  }
#endif // KEY_CLICK
  OCR1BL = lut[255];
  process_minsky();
  OCR1AL = (p_sin >> (16 - tone_vol)) + 128;
}

void dsp_tx_am() {
  ADCSRA |= (1 << ADSC);
  OCR1BL = amp;
  int16_t adc = ADC - 512;
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive - 4);
#define AM_BASE 32
  in = max(0, min(255, (in + AM_BASE)));
  amp = in; // lut[in];
}

void dsp_tx_fm() {
  ADCSRA |= (1 << ADSC); // start next ADC conversion
  OCR1BL = lut[255];     // submit amplitude to PWM register
  si5351.SendPLLRegisterBulk(); // submit frequency registers to SI5351
  int16_t adc = ADC - 512;      // 10-bit ADC sample
  int16_t in = (adc >> MIC_ATTEN);
  in = in << (drive);
  int16_t df = in;
  si5351.freq_calc_fast(df); // calculate SI5351 registers
}

// exponential averaging [Lyons 13.33.1]
#define EA(y, x, one_over_alpha)               \
  (y) = (y) + ((x) - (y)) / (one_over_alpha);

// multiplierless exponental averaging
#define MLEA(y, x, L, M)                       \
  (y) = (y) + ((((x) - (y)) >> (L)) - (((x) - (y)) >> (M)));

const char m2c[] PROGMEM = "~ "
                           "ETIANMSURWDKGOHVF*L*PJBXCYZQ**54S3***2**+***J16=/"
                           "***H*7*G*8*90************?_****\"**.****@***'**-***"
                           "*****;!*)*****,****:****";

uint8_t delayWithKeySense(uint32_t ms) {
  uint32_t event = millis() + ms;
  for (; millis() < event;) {
    wdt_reset();
    if (DIT_PRESSED || DAH_PRESSED) {
      while (DIT_PRESSED || DAH_PRESSED) wdt_reset(); // wait
      return 1;
    }
  }
  return 0;
}

uint8_t cw_msg_interval = 0; // CW message interval diabled
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0;       // selected message

// 0=not in menu, 1=selects menu item, 2=selects parameter value
#define NOT_IN_MENU    0
#define SELECT_MENU    1
#define SELECT_VALUE   2
#define MENU_NEXT      3

volatile uint8_t menumode = NOT_IN_MENU;

// used for sub-menus
uint8_t preset_sel    = 0;   // slot shown in the preset picker
uint8_t preset_pick   = 0;   // 1 while the preset picker is open
uint8_t cwmsg_sel     = 0;   // slot shown in the cwmsg picker
uint8_t cwmsg_pick    = 0;   // 1 while the cwmsg picker is open
uint8_t userinfo_sel  = 0;   // slot shown in the user info picker
uint8_t userinfo_pick = 0;   // 1 while the user info picker is open

volatile uint8_t cwdec = 1;
static int32_t avg = 256;
static uint8_t sym;
static uint32_t amp32 = 0;
volatile uint32_t _amp32 = 0;
static char out[] = "                ";
volatile uint8_t cw_event = false;

// print spaces
void lcd_blanks(uint8_t x) {
  for (uint8_t i=0; i<x; i++)
    lcd.print(' ');
}

// print a decoded symbol
void printsym(bool submit = true) {
  if (sym < 128) {
    char ch = pgm_read_byte_near(m2c + sym);
    if (ch != '*') {
      for (uint8_t i=0; i<15; i++)
        out[i] = out[i + 1];
      out[15] = ch;
      cw_event = true; // update display
    }
  }
  if (submit) sym = 1;
}

bool realstate = LOW;
bool realstatebefore = LOW;
bool filteredstate = LOW;
bool filteredstatebefore = LOW;
uint8_t nbtime = 16; // 6 // ms noise blanker
uint32_t starttimehigh;
uint32_t highduration;
uint32_t hightimesavg;
uint32_t lowtimesavg;
uint32_t startttimelow;
uint32_t lowduration;
uint32_t laststarttime = 0;
uint8_t wpm = 25;

inline void cw_decode() {
  int32_t in = _amp32;
  EA(avg, in, (1 << 8));
  realstate = (in > (avg * 1 / 2)); // threshold

  // here we clean up the state with a noise blanker
  if (realstate != realstatebefore) {
    laststarttime = millis();
  }
  // #define NB_SCALED_TO_WPM    1
#ifdef NB_SCALED_TO_WPM
  if ((millis() - laststarttime) >
      min(1200 / (20 * 2), max(1200 / (40 * 2), hightimesavg / 6))) {
#else
  if ((millis() - laststarttime) > nbtime) {
#endif
    if (realstate != filteredstate) {
      filteredstate = realstate;
    }
  } else
    avg += avg / 100;
  dec2();
  realstatebefore = realstate;
}

// display stepsize with cursor
void stepsize_showcursor() {
  if (preset_pick || cwmsg_pick || userinfo_pick) {
    lcd.noCursor(); return;
  }
  lcd.setCursor(stepsize+1, 1);
  lcd.cursor();
}

// change the stepsize
void stepsize_change(int8_t val) {
  stepsize += val;
  if (stepsize < STEP_1M)
    stepsize = STEP_10;
  if (stepsize > STEP_10)
    stepsize = STEP_1M;
  if (stepsize == STEP_10k || stepsize == STEP_500k)
    stepsize += val;
  stepsize_showcursor();
}

// Morse decoder
void dec2() {
  if (filteredstate != filteredstatebefore) {
    if (menumode == NOT_IN_MENU) {
      lcd.noCursor();
      lcd.setCursor(15,1);
      lcd.print((filteredstate) ? 'R' : ' ');
      stepsize_showcursor();
    }
    if (filteredstate == HIGH) {
      starttimehigh = millis();
      lowduration = (millis() - startttimelow);
      if ((sym > 1) && lowduration > (hightimesavg * 2))  {
        printsym();
        wpm = (1200 / hightimesavg * 4 / 3);
      }
      if (lowduration >= hightimesavg * (5)) {
        sym = 1;
        printsym();
      }
    }
    if (filteredstate == LOW) {
      startttimelow = millis();
      highduration = (millis() - starttimehigh);
      if (highduration < (2 * hightimesavg) || hightimesavg == 0) {
        hightimesavg = (highduration + hightimesavg + hightimesavg) / 3;
      }
      if (highduration > (5 * hightimesavg)) {
        hightimesavg = highduration / 3; // if speed decrease fast ..
      }
      if (highduration > (hightimesavg / 2)) {
        sym = (sym << 1) | (highduration > (hightimesavg * 2));
      }
    }
  }
  if (((millis() - startttimelow) > hightimesavg * 6) && (sym > 1)) {
    printsym(); // write if no more letters
  }
  filteredstatebefore = filteredstate;
}

#define F_SAMP_PWM (78125)
#define F_SAMP_RX  (62500)
#define F_ADC_CONV (192307 / 2)  // 192307 produced audio clicks

volatile uint8_t agc   = 2;
volatile uint8_t nr    = 2;
volatile uint8_t att   = 0;
volatile uint8_t att2  = 2;
volatile uint8_t _init = 0;

// Old AGC algorithm which only increases gain, but does not decrease it for
// very strong signals. Maximum possible gain is x32 (in practice, x31) so AGC
// range is x1 to x31 = 30dB approx. Decay time is fine (about 1s) but attack
// time is much slower than I like. For weak/medium signals it aims to keep the
// sample value between 1024 and 2048.
static int16_t gain = 1024;
inline int16_t process_agc_fast(int16_t in) {
  int16_t out = (gain >= 1024) ? (gain >> 10) * in : in;
  int16_t accum = (1 - abs(out >> 10));
  if ((INT16_MAX - gain) > accum)
    gain = gain + accum;
  if (gain < 1)
    gain = 1;
  return out;
}

// Contribution by Alan, M0PUB: Experimental new AGC algorithm.
// ASSUMES: Input sample values are constrained to a maximum of +/-4096 to avoid
// integer overflow in earlier calculations.
//
// This algorithm aims to keep signals between a peak sample value of 1024 -
// 1536, with fast attack but slow decay.
//
// The variable centiGain actually represents the applied gain x 128 - i.e. the
// numeric gain applied is centiGain/128
//
// Since the largest valid input sample has a value of +/- 4096, centiGain
// should never be less than 32 (i.e. a 'gain' of 0.25). The maximum value for
// centiGain is 32767, and hence a gain of 255. So the AGC range is 0.25:255, or
// approx. 60dB.
//
// Variable 'slowdown' allows the decay time to be slowed down so that it is not
// directly related to the value of centiCount.

static int16_t centiGain = 128;
#define DECAY_FACTOR 400 // AGC decay occurs <DECAY_FACTOR> slower than attack.
static uint16_t decayCount = DECAY_FACTOR;
#define HI(x) ((x) >> 8)
#define LO(x) ((x) & 0xFF)

inline int16_t process_agc(int16_t in) {
  static bool small = true;
  int16_t out;

  if (centiGain >= 128)
    out = (centiGain >> 5) * in; // net gain >= 1
  else
    out = (centiGain >> 2) * (in >> 3); // net gain < 1
  out >>= 2;

  if (HI(abs(out)) > HI(1536)) {
    centiGain -= (centiGain >> 4); // Fast attack time when big signal
                                   // encountered (relies on CentiGain >= 16)
  } else {
    if (HI(abs(out)) > HI(1024))
      small = false;
    if (--decayCount == 0) { // But slow ramp up of gain when signal disappears
      if (small) { // 400 samples below lower threshold - increase gain
        if (centiGain < (INT16_MAX - (INT16_MAX >> 4)))
          centiGain += (centiGain >> 4);
        else
          centiGain = INT16_MAX;
      }
      decayCount = DECAY_FACTOR;
      small = true;
    }
  }
  return out;
}

inline int16_t process_nr_old(int16_t ac) {
  ac = ac >> (6 - abs(ac)); // non-linear below amp of 6; to reduce noise
                            // (switchoff agc and tune-up volume until noise
                            // dissapears, todo:extra volume control needed)
  ac = ac << 3;
  return ac;
}

inline int16_t process_nr_old2(int16_t ac) {
  static int16_t ea1;
  ea1 = EA(ea1, ac, 64); // alpha=1/64=0.0156
  return ea1;
}

inline int16_t process_nr(int16_t in) {
  static int16_t ea1;
  ea1 = EA(ea1, in, 1 << (nr - 1));
  // static int16_t ea2;
  // ea2 = EA(ea2, ea1, inv_gain);
  return ea1;
}

inline int16_t filt_var(int16_t za0) {
  static int16_t za1, za2;
  static int16_t zb0, zb1, zb2;
  static int16_t zc0, zc1, zc2;

  if (filt < 4) { // for SSB filters
    // 1st Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    static int16_t zz1, zz2;
    zz2 = zz1;
    zz1 = za0;
    za0 = (30 * (za0 - zz2) + 25 * zz1) / 32; // 300-Hz

    // 4th Order (SR=8kHz) IIR in Direct Form I, 8x8:16
    switch (filt) {
    case 1:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (13 * zb1 + 11 * zb2) / 16;
      break;
    case 2:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (2 * zb1 + 8 * zb2) / 16;
      break;
    case 3:
      zb0 = (za0 + 2 * za1 + za2) / 2 - (0 * zb1 + 4 * zb2) / 16;
      break; // 0-1800Hz  elliptic
    }
    switch (filt) {
    case 1:
      zc0 = (zb0 + 2 * zb1 + zb2) / 2 - (18 * zc1 + 11 * zc2) / 16;
      break;
    case 2:
      zc0 = (zb0 + 2 * zb1 + zb2) / 4 - (4 * zc1 + 8 * zc2) / 16;
      break;
    case 3:
      zc0 = (zb0 + 2 * zb1 + zb2) / 4 - (0 * zc1 + 4 * zc2) / 16;
      break; // 0-1800Hz  elliptic
    }

    zc2 = zc1;
    zc1 = zc0;
    zb2 = zb1;
    zb1 = zb0;
    za2 = za1;
    za1 = za0;

    return zc0;
  } else {
    //   2nd Order (SR=4465Hz) IIR in Direct Form for CW filters
    switch (filt) {
      case 4:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (114L * zb1 - 57L * zb2) / 64;
        break; // 600Hz+-250Hz
      case 5:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (113L * zb1 - 60L * zb2) / 64;
        break; // 600Hz+-100Hz
      case 6:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (110L * zb1 - 62L * zb2) / 64;
        break; // 600Hz+-50Hz
      case 7:
        zb0 = (0 * za0 + 1 * za1 + 0 * za2) + (110L * zb1 - 61L * zb2) / 64;
        break; // 600Hz+-18Hz
    }
    switch (filt) {
      case 4:
        zc0 = (zb0 - 2 * zb1 + zb2) / 1 + (95L * zc1 - 52L * zc2) / 64;
        break; // 600Hz+-250Hz
      case 5:
        zc0 = (zb0 - 2 * zb1 + zb2) / 4 + (106L * zc1 - 59L * zc2) / 64;
        break; // 600Hz+-100Hz
      case 6:
        zc0 = (zb0 - 2 * zb1 + zb2) / 16 + (113L * zc1 - 62L * zc2) / 64;
        break; // 600Hz+-50Hz
      case 7:
        zc0 = (zb0 - 2 * zb1 + zb2) / 32 + (112L * zc1 - 62L * zc2) / 64;
        break; // 600Hz+-18Hz
    }
    zc2 = zc1;
    zc1 = zc0;
    zb2 = zb1;
    zb1 = zb0;
    za2 = za1;
    za1 = za0;
    return zc0 / 8;
  }
}

#define __UA 256
#define __atan2(z)  (__UA/8  + __UA/22) * z

inline int16_t _arctan3(int16_t q, int16_t i) {
  int16_t r;
  if (abs(q) > abs(i))
    r = __UA / 4 - __atan2(abs(i) / abs(q));
  else
    r = (i == 0) ? 0 : __atan2(abs(q) / abs(i));
  r = (i < 0) ? __UA / 2 - r : r;
  return (q < 0) ? -r : r;
}

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;
int16_t v[14]; // Process Q (down-sampled) samples
int16_t vi[7];

inline int16_t slow_dsp(int16_t i_ac2, int16_t q_ac2) {
  // Post processing I and Q (down-sampled) results here - G8DI mod
  int16_t ac, qh, acm;

  q_ac2 >>= att2; // digital gain control

  // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0,
  // 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0,
  // -79, 0, -21, 0, -8, 0, -2, 0) / 128;
  if (mode != AM && mode != FM) {
    qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 +
         ((v[4] - v[10]) + (v[6] - v[8])) / 8 +
         ((v[4] - v[10]) * 5 - (v[6] - v[8])) / 128 + (v[6] - v[8]) / 2;
    // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS)
    v[0] = v[1];
    v[1] = v[2];
    v[2] = v[3];
    v[3] = v[4];
    v[4] = v[5];
    v[5] = v[6];
    v[6] = v[7];
    v[7] = v[8];
    v[8] = v[9];
    v[9] = v[10];
    v[10] = v[11];
    v[11] = v[12];
    v[12] = v[13];
    v[13] = q_ac2;
  } // Do not remove, separate block for separate v[]!!!!

  i_ac2 >>= att2; // digital gain control
  i = i_ac2;
  q = q_ac2;
  int16_t id = vi[0];
  vi[0] = vi[1];
  vi[1] = vi[2];
  vi[2] = vi[3];
  vi[3] = vi[4];
  vi[4] = vi[5];
  vi[5] = vi[6];
  vi[6] = i_ac2; // Delay to match Hilbert transform on Q branch

  if (mode == AM) {
    acm = -i - q; // S-Meter
    ac = magn(i, q); // AM = SQRT (I * I + Q * Q)
    static int16_t as_last;
    int16_t as = ac + (int16_t)((float)as_last * 0.9999f);
    ac = as - as_last;
    as_last = as;

  } else if (mode == FM) {
    acm = -i - q; // S-Meter
    static int16_t zi = i;
    ac = ((ac + i) * zi);
    zi = i;
  }
  else {
    // USB, LSB, CW
    // inverting I and Q helps dampening a
    // feedback-loop between PWM out and ADC inputs
    acm = -id - qh;
    ac = acm;
  }
  // Set S-Meter level
  static uint8_t absavg256cnt;
  if (!(absavg256cnt--)) {
    _absavg256 = absavg256;
    absavg256 = 0;
  }
  else
    absavg256 += abs(acm);

  if (agc == 2) {
    ac = process_agc(ac);
    ac = ac >> (16 - volume);
  } else if (agc == 1) {
    ac = process_agc_fast(ac);
    ac = ac >> (16 - volume);
  } else {
    // ac = ac >> (16-volume);
    if (volume <= 13) // if no AGC allow volume control to boost weak signals
      ac = ac >> (13 - volume);
    else
      ac = ac << (volume - 13);
  }
  if (nr) ac = process_nr(ac);
  if (filt) ac = filt_var(ac);
  if (!(absavg256cnt % 64)) {
    _amp32 = amp32;
    amp32 = 0;
  } else
    amp32 += abs(acm);
  ac = min(max(ac, -512), 511);
  return ac;
}

typedef void (*func_t)(void);
volatile func_t func_ptr;
#undef R    // Decimating 2nd Order CIC filter
#define R 4 // Rate change from 62500/2 kSPS to 7812.5SPS, providing 12dB gain

volatile uint8_t admux[3];
volatile int16_t ocomb, qh;
volatile uint8_t rx_state = 0;

#pragma GCC push_options
#pragma GCC optimize("Ofast") // compiler-optimization for speed

// Non-recursive CIC Filter (M=2, R=4) implementation, so two-stages of
// (followed by down-sampling with factor 2): H1(z) = (1 + z^-1)^2 = 1 + 2*z^-1
// + z^-2 = (1 + z^-2) + (2) * z^-1 = FA(z) + FB(z) * z^-1; with down-sampling
// before stage translates into poly-phase components: FA(z) = 1 + z^-1, FB(z) =
// 2 Non-recursive CIC Filter (M=4) implementation (for second-stage only):
// H1(z) = (1 + z^-1)^4 = 1 + 4*z^-1 + 6*z^-2 + 4*z^-3 + z^-4 = 1 + 6*z^-2 +
// z^-4 + (4 + 4*z^-2) * z^-1 = FA(z) + FB(z) * z^-1; with down-sampling before
// stage translates into poly-phase components: FA(z) = 1 + 6*z^-1 + z^-2, FB(z)
// = 4 + 4*z^-1 M=3 FA(z) = 1 + 3*z^-1, FB(z) = 3 + z^-1 source: Lyons
// Understanding Digital Signal Processing 3rd edition 13.24.1

static uint8_t tc = 0;

void process(int16_t i_ac2, int16_t q_ac2) {
  static int16_t ac3;

#ifdef AF_OUT
  static int16_t ozd1, ozd2; // Audio output stage
  if (_init) {
    ac3 = 0;
    ozd1 = 0;
    ozd2 = 0;
    _init = 0;
  }
  int16_t od1 = ac3 - ozd1; // Comb section
  ocomb = od1 - ozd2;
#endif // AF_OUT

#define OUTLET 1
#ifdef OUTLET
  if (tc++ == 0) // prevent recursion : If tc == 0 enable interrupts
#endif

  // hack, since slow_dsp process exceeds rx sample-time, allow
  // subsequent 7 interrupts for further rx sampling while processing
  // prevent nested interrupts with tc
  interrupts();

#ifdef AF_OUT
  ozd2 = od1;
  ozd1 = ac3;
#endif // AF_OUT

  ac3 = slow_dsp(i_ac2, q_ac2);

#ifdef OUTLET
  tc--;
#endif
}

static int16_t i_s0za1, i_s0zb0, i_s0zb1, i_s1za1, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0zb0, q_s0zb1, q_s1za1, q_s1zb0, q_s1zb1, q_ac2;

void sdr_rx_03();
void sdr_rx_07();

#define M_SR 1 // CIC N=3
void sdr_rx_00() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_01;
  int16_t i_s1za0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR;
  i_s0za1 = ac;
  int16_t ac2 = (i_s1za0 + (i_s1za1 + i_s1zb0) * 3 + i_s1zb1);
  i_s1za1 = i_s1za0;
  process(ac2, q_ac2);
}
void sdr_rx_02() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_03;
  i_s0zb1 = i_s0zb0;
  i_s0zb0 = ac;
}
void sdr_rx_04() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_05;
  i_s1zb1 = i_s1zb0;
  i_s1zb0 = (ac + (i_s0za1 + i_s0zb0) * 3 + i_s0zb1) >> M_SR;
  i_s0za1 = ac;
}
void sdr_rx_06() {
  int16_t ac = sdr_rx_common_i();
  func_ptr = sdr_rx_07;
  i_s0zb1 = i_s0zb0;
  i_s0zb0 = ac;
}

void sdr_rx_01() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_02;
  q_s0zb1 = q_s0zb0;
  q_s0zb0 = ac;
}
void sdr_rx_03() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_04;
  q_s1zb1 = q_s1zb0;
  q_s1zb0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR;
  q_s0za1 = ac;
}
void sdr_rx_05() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_06;
  q_s0zb1 = q_s0zb0;
  q_s0zb0 = ac;
}
void sdr_rx_07() {
  int16_t ac = sdr_rx_common_q();
  func_ptr = sdr_rx_00;
  int16_t q_s1za0 = (ac + (q_s0za1 + q_s0zb0) * 3 + q_s0zb1) >> M_SR;
  q_s0za1 = ac;
  q_ac2 = (q_s1za0 + (q_s1za1 + q_s1zb0) * 3 + q_s1zb1);
  q_s1za1 = q_s1za0;
}

static int16_t ozi1, ozi2;

inline int16_t sdr_rx_common_q() {
  ADMUX = admux[0];
  ADCSRA |= (1 << ADSC);
  int16_t ac = ADC - 511;
  return ac;
}

inline int16_t sdr_rx_common_i() // Get RX AC samples
{
  ADMUX = admux[1];
  ADCSRA |= (1 << ADSC);
  int16_t adc = ADC - 511;
  static int16_t prev_adc;
  int16_t ac = (prev_adc + adc) >> 1;
  prev_adc = adc;

#ifdef AF_OUT
  if (_init) {
    ocomb = 0;
    ozi1 = 0;
    ozi2 = 0;
  }
  ozi2 = ozi1 + ozi2; // Integrator section
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max((ozi2 >> 5) + 128, 0), 255);
#endif // AF_OUT
  return ac;
}

// Timer2 COMPA interrupt
ISR(TIMER2_COMPA_vect) {
  func_ptr();
#ifdef DEBUG
  numSamples++;
#endif
}

uint8_t log2(uint16_t x) {
  uint8_t y = 0;
  for (; x >>= 1;) y++;
  return y;
}

#pragma GCC pop_options // end of DSP section

void adc_start(uint8_t adcpin, bool ref1v1, uint32_t fs) {
  DIDR0 |= (1 << adcpin);   // disable digital input
  ADCSRA = 0;               // clear ADCSRA register
  ADCSRB = 0;               // clear ADCSRB register
  ADMUX = 0;                // clear ADMUX register
  ADMUX |= (adcpin & 0x0f); // set analog input pin
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) |
           (1 << REFS0); // If reflvl == true, set AREF=1.1V (Internal ref);
                         // otherwise AREF=AVCC=(5V)
  ADCSRA |=
      ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) &
      0x07; // ADC Prescaler (for normal conversions non-auto-triggered): ADPS =
            // log2(F_CPU / 13 / Fs) - 1; ADSP=0..7 resulting in resp.
            // conversion rate of 1536, 768, 384, 192, 96, 48, 24, 12 kHz
  // ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN); // enable ADC
                         // ADCSRA |= (1 << ADSC);  // start ADC measurements
#ifdef ADC_NR
  //  set_sleep_mode(SLEEP_MODE_ADC);  // ADC NR sleep destroys the timer2
  //  integrity, therefore Idle sleep is better alternative (keeping clkIO as an
  //  active clock domain)
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif
}

void adc_stop() {
  // ADCSRA &= ~(1 << ADATE); // disable auto trigger
  ADCSRA &= ~(1 << ADIE); // disable interrupts when measurement complete
  ADCSRA |=
      (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 prescaler for 9.6kHz
#ifdef ADC_NR
  sleep_disable();
#endif
  ADMUX = (1 << REFS0); // restore reference voltage AREF (5V)
}

void timer1_start(uint32_t fs) { // Timer 1: OC1A and OC1B in PWM mode
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) |
            (1 << WGM11); // Clear OC1A/OC1B on compare match, set OC1A/OC1B at
                          // BOTTOM (non-inverting mode)
  TCCR1B |= (1 << CS10) | (1 << WGM13) |
            (1 << WGM12); // Mode 14 - Fast PWM;  CS10: clkI/O/1 (No prescaling)
  ICR1H = 0x00;
  ICR1L = min(255, F_CPU / fs); // PWM value range (fs>78431):  Fpwm = F_CPU /
                                // [Prescaler * (1 + TOP)]
  // TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM10); // Clear OC1A/OC1B
  // on compare match, set OC1A/OC1B at BOTTOM (non-inverting mode) TCCR1B |= (1
  // << CS10) | (1 << WGM12); // Mode 5 - Fast PWM, 8-bit;  CS10: clkI/O/1 (No
  // prescaling)
  OCR1AH = 0x00;
  OCR1AL = 0x00; // OC1A (SIDETONE) PWM duty-cycle (span defined by ICR).
  OCR1BH = 0x00;
  OCR1BL = 0x00; // OC1B (PWM_OUT) PWM duty-cycle (span defined by ICR).
}

void timer1_stop() {
  OCR1AL = 0x00;
  OCR1BL = 0x00;
}

void timer2_start(uint32_t fs) { // Timer 2: interrupt mode
  ASSR &= ~(1 << AS2); // Timer 2 clocked from CLK I/O (like Timer 0 and 1)
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);  // WGM21: Mode 2 - CTC (Clear Timer on Compare Match)
  TCCR2B |= (1 << CS22);   // Set C22 bits for 64 prescaler
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt TIMER2_COMPA_vect
  uint8_t ocr =
      ((F_CPU / 64) / fs) - 1; // OCRn = (F_CPU / pre-scaler / fs) - 1;
  OCR2A = ocr;
}

void timer2_stop() {        // Stop Timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  delay(1); // wait until potential in-flight interrupts are finished
}

#define N_FONTS 8
const byte fonts[N_FONTS][8] PROGMEM = {
    {0b01000, // 1; logo
     0b00100, 0b01010, 0b00101, 0b01010, 0b00100, 0b01000, 0b00000},
    {0b00000, // 2; s-meter, 0 bars
     0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b10000, // 3; s-meter, 1 bars
     0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000},
    {0b10000, // 4; s-meter, 2 bars
     0b10000, 0b10100, 0b10100, 0b10100, 0b10100, 0b10100, 0b10100},
    {0b10000, // 5; s-meter, 3 bars
     0b10000, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101, 0b10101},
    {0b01100, // 6; vfo-a
     0b10010, 0b11110, 0b10010, 0b10010, 0b00000, 0b00000, 0b00000},
    {0b11100, // 7; vfo-b
     0b10010, 0b11100, 0b10010, 0b11100, 0b00000, 0b00000, 0b00000},
    {0b00000, // 8; TBD
     0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}};

// performs classical analogRead with default Arduino
// sample-rate and analog reference setting; restores previous settings
int analogSafeRead(uint8_t pin, bool ref1v1 = false) {
  noInterrupts();
  // wait until ADC conversion is completed
  for (; !(ADCSRA & (1 << ADIF)););
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  ADCSRA &= ~(1 << ADIE); // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  if (ref1v1) ADMUX &= ~(1 << REFS0); // restore 1V reference voltage
  else ADMUX = (1 << REFS0);          // restore 5V reference voltage
  delay(1);
  int val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}

uint16_t analogSampleMic() {
  uint16_t adc;
  noInterrupts();
  // hack: faster conversion rate necessary for VOX
  ADCSRA = (1 << ADEN) | (((uint8_t)log2((uint8_t)(F_CPU / 13 / (192307 / 1)))) & 0x07);
  if (vox_thresh >= 32)
    fastdigitalWrite(RX, LOW); // disable RF input
  uint8_t oldmux = ADMUX;
  // wait until ADC conversion is completed
  for (; !(ADCSRA & (1 << ADIF)););
  ADMUX = admux[2];      // set MUX for next conversion
  ADCSRA |= (1 << ADSC); // start next ADC conversion
  // wait until ADC conversion is completed
  for (; !(ADCSRA & (1 << ADIF)););
  ADMUX = oldmux;
  if (vox_thresh >= 32)
    fastdigitalWrite(RX, HIGH); // enable RF input
  adc = ADC;
  interrupts();
  return adc;
}

#define VFOA  0
#define VFOB  1

volatile bool change = true;
volatile bool changedMode = 0;
volatile int32_t freq = INIT_FREQ;
static int32_t vfo[] = {INIT_FREQ, INIT_FREQ};
static uint8_t vfomode[] = {CW, CW};
volatile uint8_t vfosel = VFOA;
volatile int32_t rit = 0;
volatile uint8_t semi_qsk = false;

uint8_t smode = 1;
uint32_t max_absavg256 = 0;
int16_t dbm;
static int16_t smeter_cnt = 0;

int16_t smeter(int16_t ref = 0) {
  max_absavg256 = max(_absavg256, max_absavg256); // peak
  if ((smode) && ((++smeter_cnt % 2048) == 0)) { // slowed down display slightly
    float rms = (float)max_absavg256 * (float)(1 << att2);
    rms /= (256.0 * 1024.0 * (float)R * 8.0 * 500.0 * 1.414 / (0.707 * 1.1));
    dbm = 10 * log10((rms * rms) / 50) + 30 - ref; // from rmsV to dBm at 50R
    lcd.noCursor();
    if (smode == 1) { // dBm meter
      lcd.setCursor(9,0);
      lcd.print((int16_t)dbm);
      lcd.print(F("dBm "));
    }
    if (smode == 2) { // S-meter
      // dBm to S (modified to work correctly above S9)
      uint8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10;
      lcd.setCursor(14,0);
      if (s < 10) {
        lcd.print('S');
      }
      lcd.print(s);
    }
    if (smode == 3) { // S-bar
      // dBm to S (modified to work correctly above S9)
      int8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10;
      char tmp[5];
      for (uint8_t i = 0; i != 4; i++) {
        tmp[i] = max(2, min(5, s + 1));
        s = s - 3;
      }
      tmp[4] = 0;
      lcd.setCursor(12,0);
      lcd.print(tmp);
    }
    if (smode == 4) { // wpm-indicator
      lcd.setCursor(14,0);
      if (mode == CW) lcd.print(wpm);
      lcd_blanks(2);
    }
    stepsize_showcursor();
    max_absavg256 /= 2; // Implement peak hold/decay for all meter types
  }
  return dbm;
}

// --- CW-MESSAGE PICKER ---

void cwmsg_draw() {
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("KNOB=sel  ENC=edit"));
  lcd.setCursor(0,1);
  lcd.print(cwmsg_sel + 1);
  lcd.print(F(": "));
  char *p = cw_msg[cwmsg_sel];
  for (uint8_t i = 0; i < 12 && p[i]; i++)
    lcd.print(p[i]);
  lcd_blanks(10);
}

// --- USER INFO PICKER ---

void userinfo_draw() {
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("KNOB=sel  ENC=edit"));
  lcd.setCursor(0,1);
  lcd.print(userinfo_sel + 1);
  lcd.print(F(": "));
  char *p = (userinfo_sel == 0) ? my_name : my_call;
  for (uint8_t i = 0; i < 12 && p[i]; i++) lcd.print(p[i]);
  lcd_blanks(10);
}

// --- FREQ_PRESETS_BEGIN ---

// Frequency presets: 8 slots of 3 bytes for freq and 1 byte for mode
// in their own EEPROM area outside the parameter list.

#define N_PRESET 8
const uint32_t preset_default[N_PRESET] PROGMEM = {
  14030000UL | ((uint32_t)CW << 24), 14060000UL | ((uint32_t)CW << 24), 14100000UL | ((uint32_t)CW << 24),
  14250000UL | ((uint32_t)USB << 24), 14285000UL | ((uint32_t)USB << 24), 0, 0, 0
};

uint32_t preset_get(uint8_t i) {  // 0 = empty slot
  uint32_t v = eeprom_read_dword((const uint32_t *)(PRESET_EEPROM + 4 * i));
  uint32_t f = v & 0xFFFFFFUL;
  if ((f < FREQ_LIMIT_LOW) || (f > FREQ_LIMIT_HIGH) || ((v >> 24) > AM)) v = pgm_read_dword(&preset_default[i]);
  return v;
}

// print the frequency and mode label
void preset_print(uint32_t v) {
  uint32_t f = v & 0xFFFFFFUL;
  lcd.print(f / 1000000UL); lcd.print('.');
  uint16_t r = (f % 1000000UL) / 100;
  if (r < 1000) lcd.print('0'); if (r < 100) lcd.print('0'); if (r < 10) lcd.print('0');
  lcd.print(r); lcd.print(' '); lcd.print(mode_label[v >> 24]);
}

// top line: the keys; bottom line: active VFO
void preset_draw() {
  lcd.noCursor();
  lcd.setCursor(0,0);
  lcd.print(F("KNOB=go SEL=save"));
  lcd.setCursor(0,1);
  lcd.print((vfosel) ? '\x07' : '\x06');
  lcd.print(preset_sel + 1); lcd.print(' ');
  uint32_t v = preset_get(preset_sel);
  uint8_t len = 3 + 11;
  if (v) { preset_print(v); } else { lcd.print(F("empty")); len = 3 + 5; }
  for (; len<15; len++) lcd.print(' ');
}

// store the active VFO's frequency and mode in the shown slot
void preset_store() {
  eeprom_write_dword((uint32_t *)(PRESET_EEPROM + 4 * preset_sel), (uint32_t)freq | ((uint32_t)mode << 24));
  lcd.setCursor(0,1);
  lcd.print(F("Saved to "));
  lcd.print(preset_sel + 1);
  lcd_blanks(5);
  wdt_reset();
  delay(800);
  wdt_reset();
  preset_draw();
}

// --- FREQ_PRESETS_END ---

// Start radio receiver
void start_rx() {
  _init = 1;
  rx_state = 0;
  func_ptr = sdr_rx_00; // enable RX DSP/SDR
  adc_start(2, true, F_ADC_CONV * 4);
  admux[2] = ADMUX; // Note that conversion-rate for TX is factors more
  adc_start(0, !(att == 1), F_ADC_CONV);
  admux[0] = ADMUX;
  adc_start(1, !(att == 1), F_ADC_CONV);
  admux[1] = ADMUX;
  timer1_start(F_SAMP_PWM);
  timer2_start(F_SAMP_RX);
  TCCR1A &= ~(1 << COM1B1);
  fastdigitalWrite(PWM_OUT, LOW); // disable PWM
}

int16_t _centiGain = 0;
uint8_t txdelay = 0;
uint32_t semi_qsk_timeout = 0;

// display frequency
inline void display_vfo(int32_t freq) {
  if (preset_pick) {
    preset_draw();
    return;
  }
  if (cwmsg_pick) {
    cwmsg_draw();
    return;
  }
  if (userinfo_pick) {
    userinfo_draw();
    return;
  }
  lcd.setCursor(0,1);
  lcd.print((rit) ? ' ' : (vfosel ? '\x07' : '\x06'));
  int32_t scale = 10e6;
  if (rit && !tx) {
    freq = rit;     // Show RIT RX offset only
    scale = 1e3; // RIT frequency
    lcd.print(F("RIT"));
    lcd.print(rit < 0 ? '-' : '+');
  } else {
    if (freq / scale == 0) {
      lcd.print(' ');
      scale /= 10;
    } // Initial space instead of zero
  }
  for (; scale != 1; freq %= scale, scale /= 10) {
    lcd.print(abs(freq / scale));
    if (scale == (int32_t)1e3 || scale == (int32_t)1e6)
      lcd.print(','); // Thousands separator
  }
  lcd.print(' ');
  lcd.print(mode_label[mode]);
  lcd.print(' ');
  lcd.setCursor(15,1);
  lcd.print((vox) ? 'V' : 'R');
}

// Set RX or TX mode, with RIT support and CW offset
void switch_rxtx(uint8_t tx_enable) {
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  // delay(1);
  delayMicroseconds(20); // wait until potential RX interrupt is finalized
  noInterrupts();
  tx = tx_enable;
  if (rit) display_vfo(freq);
  if (tx_enable) {
    // TX
    _centiGain = centiGain; // backup AGC setting
    semi_qsk_timeout = 0;
    switch (mode) {
      case CW: func_ptr = dsp_tx_cw; break;
      case AM: func_ptr = dsp_tx_am; break;
      case FM: func_ptr = dsp_tx_fm; break;
      default: func_ptr = dsp_tx; break;
    }
  } else {
    // RX
    if ((mode == CW) && (!(semi_qsk_timeout))) {
      semi_qsk_timeout = millis() + ditTime * 8;
      if (semi_qsk)
        func_ptr = dummy;
      else
        func_ptr = sdr_rx_00;
    } else {
      centiGain = _centiGain; // restore AGC setting
      semi_qsk_timeout = 0;
      func_ptr = sdr_rx_00;
    }
  }
  interrupts();
  if (tx_enable)
    ADMUX = admux[2];
  else
    _init = 1;
  rx_state = 0;
  if ((cwdec) && (mode == CW)) {
    filteredstate = tx_enable;
    dec2();
  }

  if (tx_enable) {
    // TX
    if (practice) {
      fastdigitalWrite(RX, LOW); // TX (disable RX)
      lcd.setCursor(15,1);
      lcd.print('P');
      si5351.SendRegister(SI_CLK_OE, TX0RX0);
    } else {
      fastdigitalWrite(RX, LOW); // TX (disable RX)
      lcd.setCursor(15,1);
      lcd.print('T');
      if (mode == CW) {
        si5351.freq_calc_fast(-cw_offset);
        si5351.SendPLLRegisterBulk();
      } else {
        if (rit)
          si5351.freq_calc_fast(0);
        si5351.SendPLLRegisterBulk();
      }
      si5351.SendRegister(SI_CLK_OE, TX1RX0);
      OCR1AL = 0x80; // make sure SIDETONE is set at 2.5V
      if (mode != CW) TCCR1A &= ~(1 << COM1A1);
      TCCR1A |= (1 << COM1B1); // enable PWM
    }
  } else {
    // RX
#ifdef KEY_CLICK
    // ramp down amplitude
    if (OCR1BL != 0) {
      for (uint16_t i = 0; i != 31; i++) {
        OCR1BL = lut[pgm_read_byte_near(ramp[i])];
        delayMicroseconds(60);
      }
    }
#endif // KEY_CLICK
    TCCR1A |= (1 << COM1A1);
    TCCR1A &= ~(1 << COM1B1);
    fastdigitalWrite(PWM_OUT, LOW);
    OCR1BL = 0;
    if (quad_enabled) {
      si5351.SendRegister(18, 0x0f); // disable invert on CLK2
    }
    si5351.SendRegister(SI_CLK_OE, TX0RX1);
    if ((!semi_qsk_timeout) ||
        (!semi_qsk)) // enable RX when no longer in semi-qsk phase
    {
      fastdigitalWrite(RX,
                       !(att == 2)); // RX (enable RX when attenuator not on)
    }

    si5351.freq_calc_fast(rit);
    si5351.SendPLLRegisterBulk(); // restore original PLL RX frequency
    lcd.setCursor(15,1);
    lcd.print((vox) ? 'V' : 'R');
  }
  OCR2A = ((F_CPU / 64) / ((tx_enable) ? F_SAMP_TX : F_SAMP_RX)) - 1;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt TIMER2_COMPA_vect
}

// Transmit a CW character
int cw_tx(char ch) {
  char sym;
  for (uint8_t j = 0; (sym = pgm_read_byte_near(m2c + j)); j++) {
    if (sym == ch) {
      // transmit a CW character
      wdt_reset();
      lcd.setCursor(3, 1);
      lcd.print(ch);
      uint8_t k = 0x80;
      for (; !(j & k); k >>= 1);
      k >>= 1; // shift start of cw code to MSB
      if (k == 0) delay(ditTime * 4); // space -> add word space
      else {
        for (; k; k >>= 1) {
          switch_rxtx(1);    // key-on  tx
          if (delayWithKeySense(ditTime * ((j & k) ? 3 : 1))) {
            switch_rxtx(0);
            return 1;
          }
          switch_rxtx(0);    // key-off tx
          if (delayWithKeySense(ditTime))
            return 1;        // add symbol space
        }
        if (delayWithKeySense(ditTime * 2))
          return 1;          // add letter space
      }
      break; // next character
    }
  }
  return 0;
}

// Transmit a CW message string
uint8_t cw_tx_str(char *msg) {
  // clear the message area
  lcd.setCursor(3, 1);
  lcd_blanks(12);
  for (uint8_t i=0; msg[i]; i++)
    if (cw_tx(msg[i])) break;
  lcd.setCursor(3, 1);
  lcd.print(msg);
  lcd.setCursor(13, 1);
  lcd_blanks(2);
  return 0;
}

// Transmit an expanded CW message string
uint8_t cw_tx_msg(char *msg) {
  char expanded[96];
  uint8_t out_idx = 0;
  for (uint8_t i = 0; msg[i] != '\0' && out_idx < sizeof(expanded) - 1; i++) {
    if (msg[i] == '$' && msg[i + 1] == 'C') {
      for (uint8_t j = 0; my_call[j] != '\0' && out_idx < sizeof(expanded) - 1; j++) {
        expanded[out_idx++] = my_call[j];
      }
      i++; // skip 'C'
    } else if (msg[i] == '$' && msg[i + 1] == 'N') {
      for (uint8_t j = 0; my_name[j] != '\0' && out_idx < sizeof(expanded) - 1; j++) {
        expanded[out_idx++] = my_name[j];
      }
      i++; // skip 'N'
    } else {
      expanded[out_idx++] = msg[i];
    }
  }
  expanded[out_idx] = '\0';
  return cw_tx_str(expanded);
}

uint8_t rx_ph_q = 90;

void process_encoder_tuning_step(int8_t steps) {
  int32_t stepval = stepsizes[stepsize];
  if (rit) {
    rit += steps * stepval;
    // rit = max(-9999, min(9999, rit));
    rit = max(-99999, min(99999, rit));
  } else {
    freq += steps * stepval;
    freq = max(FREQ_LIMIT_LOW, min(FREQ_LIMIT_HIGH, freq));
  }
  change = true;
}

// Reduces power from 110mA to 70mA
void powerDown() {
  lcd.setCursor(0,1);
  lcd.print(F("Power-off 73 :-)"));
  lcd_blanks(10);
  MCUSR = ~(1 << WDRF); // Must be done before wdt_disable()
  wdt_disable();
  timer2_stop();
  timer1_stop();
  adc_stop();
  si5351.powerDown();
  delay(1500);
  // Disable external interrupts INT0, INT1, Pin Change
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  // Disable internal interrupts
  TIMSK0 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  WDTCSR = 0;
  // Enable BUTTON Pin Change interrupt
  *digitalPinToPCMSK(BUTTONS) |= (1 << digitalPinToPCMSKbit(BUTTONS));
  *digitalPinToPCICR(BUTTONS) |= (1 << digitalPinToPCICRbit(BUTTONS));
  // Power-down sub-systems
  PRR = 0xff;
  lcd.noDisplay();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  interrupts();
  sleep_bod_disable();
  sleep_cpu();
  sleep_disable();
  do {
    wdt_enable(WDTO_15MS);
    for (;;);
  } while (0); // soft reset by trigger watchdog timeout
}

// clear the top line
void show_banner() {
  lcd.setCursor(0,0);
  lcd_blanks(16);
}

volatile uint8_t event;
volatile uint8_t prev_menumode = NOT_IN_MENU;
volatile int8_t menu = 0; // current parameter id selected in menu

#define pgm_cache_item(addr, sz)                                               \
  byte _item[sz];                                                              \
  memcpy_P(_item, addr, sz); // copy array item from PROGMEM to SRAM

#define EEPROM_OFFSET  0x010

uint16_t eeprom_version;
uint16_t eeprom_addr;

// Support functions for parameter and menu handling
enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };

// output menuid in x.y format
void printmenuid(uint8_t menuid) {
  static const char seperator[] = {'.', ' '};
  uint8_t ids[] = {(uint8_t)(menuid >> 4), (uint8_t)(menuid & 0xF)};
  for (uint8_t i=0; i<2; i++) {
    uint8_t id = ids[i];
    if (id >= 10) {
      id -= 10;
      lcd.print('1');
    }
    lcd.print(char('0' + id));
    lcd.print(seperator[i]);
  }
}

void printlabel(uint8_t action, uint8_t menuid, const __FlashStringHelper *label) {
  if (action == UPDATE_MENU) {
    lcd.setCursor(0,0);
    printmenuid(menuid);
    lcd.print(label);
    lcd_blanks(16);
    lcd.setCursor(0,1); // value on next line
    if (menumode >= SELECT_VALUE) lcd.print('>');
  } else { // UPDATE (not in menu)
    lcd.setCursor(0,1);
    lcd.print(label);
    lcd.print(F(": "));
  }
}

void actionCommon(uint8_t action, uint8_t *ptr, uint8_t size) {
  switch (action) {
    case LOAD:
      eeprom_read_block((void *)ptr, (const void *)eeprom_addr, size);
      break;
    case SAVE:
      eeprom_write_block((const void *)ptr, (void *)eeprom_addr, size);
      break;
    default:
      break;
  }
  eeprom_addr += size;
}

// paramAction main
template <typename T>
void paramAction(uint8_t action, volatile T &value, uint8_t menuid,
                 const __FlashStringHelper *label, const char *enumArray[],
                 int32_t _min, int32_t _max, bool continuous) {
  switch (action) {
  case UPDATE:
  case UPDATE_MENU:
    if (((int32_t)value + encoder_val) < _min)
      value = (continuous) ? _max : _min;
    else if (((int32_t)value + encoder_val) > _max)
      value = (continuous) ? _min : _max;
    else
      value = (int32_t)value + encoder_val;
    encoder_val = 0;
    lcd.noCursor();
    printlabel(action, menuid, label); // print label
    if (enumArray == NULL) {           // print value
      if ((_min < 0) && (value >= 0))
        lcd.print('+'); // add + sign for positive values
      lcd.print(value);
    } else {
      lcd.print(enumArray[value]);
    }
    lcd_blanks(15);
    break;
  default:
    actionCommon(action, (uint8_t *)&value, sizeof(value));
    break;
  }
}

// paramAction overload for character strings
void paramAction(uint8_t action, char *value, uint8_t menuid,
                 const __FlashStringHelper *label, uint8_t size) {
  const uint8_t _min = ' ';
  const uint8_t _max = 'Z';
  static uint8_t pos = 0;
  switch (action) {
  case NEXT_CH:
    pos++;
    if (pos >= size) pos = 0;  // cursor wrap-around
    action = UPDATE_MENU;      // fall-through next case
  case UPDATE:
  case UPDATE_MENU:
    if (menumode != MENU_NEXT)
      pos = 0;
    if (pos >= size)
      pos = 0; // safety clamp
    if (menumode == SELECT_VALUE)
      menumode = MENU_NEXT; // enter edit mode for strings
    int16_t newval = (int16_t)value[pos] + encoder_val;
    if (newval < _min)
      value[pos] = _min;
    else if (newval > _max)
      value[pos] = _max;
    else
      value[pos] = (char)newval;
    encoder_val = 0;
    // For short strings (name/callsign), treat space as string terminator
    if (size <= USERNAME_LENGTH && value[pos] == ' ') {
      value[pos] = 0;
      for (uint8_t i = pos + 1; i < size; i++)
        value[i] = 0; // null-terminate the remainder
    }

    printlabel(action, menuid, label);
    for (uint8_t i = 0; i < 13; i++) {
      char ch = value[(pos / 8) * 8 + i];
      if (ch)
        lcd.print(ch);
      else
        break;
    }
    lcd.print('\x01'); // print terminator
    lcd_blanks(10);
    if (menumode >= SELECT_VALUE) {
      lcd.setCursor((pos % 8) + 1, 1);
      lcd.cursor();
    }
    break;
  case SAVE:
    for (uint8_t i = size; i > 0; i--) {
      if ((value[i - 1] == ' ') || (value[i - 1] == 0))
        value[i - 1] = 0; // remove trailing spaces
      else
        break;
    }
    break;
  default:
    actionCommon(action, (uint8_t *)value, size);
    break;
  }
}

#define _N(a) sizeof(a) / sizeof(a[0])

int8_t paramAction(uint8_t action, uint8_t id = ALL) {
  if ((action == SAVE) || (action == LOAD)) {
    if (id == ALL) {
      eeprom_addr = EEPROM_OFFSET;
      for (uint8_t _id = 1; _id <= ALL_PARAMS; _id++) {
        paramAction(action, _id);
      }
      // CW messages live outside the sequential area
      for (uint8_t i = 0; i < 6; i++)
        paramAction(action, (uint8_t)(CWMSG1 + i));
      return ALL;
    } else {
      eeprom_addr = EEPROM_OFFSET;
      for (uint8_t _id = 1; _id < id; _id++) {
        paramAction(SKIP, _id);
      }
    }
  }

  // visible parameters
  switch (id) {
  case VOLUME:
    paramAction(action, volume, 0x11, F("VOLUME"), NULL, 1, 16, false);
    break;
  case TONE_VOL:
    paramAction(action, tone_vol, 0x12, F("TONE VOLUME"), NULL, 1, 16, false);
    break;
  case MODE:
    paramAction(action, mode, 0x13, F("MODE"), mode_label, 0, _N(mode_label) - 1, false);
    break;
  case FILTER:
    paramAction(action, filt, 0x14, F("FILTER BW"), filt_label, 0, _N(filt_label) - 1, false);
    break;
  case VFOSEL:
    paramAction(action, vfosel, 0x15, F("VFO A/B"), vfosel_label, 0, 1, false);
    break;
  case RIT:
    paramAction(action, rit, 0x16, F("RIT"), offon_label, 0, 1, false);
    break;
  case AGC:
    paramAction(action, agc, 0x17, F("AGC"), agc_label, 0, _N(agc_label) - 1, false);
    break;
  case NR:
    paramAction(action, nr, 0x18, F("NR"), NULL, 0, 8, false);
    break;
  case ATT:
    paramAction(action, att, 0x19, F("ATT"), att_label, 0, 7, false);
    break;
  case ATT2:
    paramAction(action, att2, 0x1A, F("ATT2"), NULL, 0, 16, false);
    break;
  case SMETER:
    paramAction(action, smode, 0x1B, F("S-METER"), smode_label, 0, _N(smode_label) - 1, false);
    break;
  case CWDEC:
    paramAction(action, cwdec, 0x21, F("CW DECODER"), offon_label, 0, 1, false);
    break;
  case CWTONE:
    paramAction(action, cw_tone, 0x22, F("CW TONE"), cw_tone_label, 0, 1, false);
    break;
  case CWINTERVAL:
    paramAction(action, cw_msg_interval, 0x23, F("CQ INTERVAL"), NULL, 0, 60, false);
    break;
  case SEMIQSK:
    paramAction(action, semi_qsk, 0x24, F("SEMI QSK"), offon_label, 0, 1, false);
    break;
  case KEY_WPM:
    paramAction(action, keyer_speed, 0x25, F("KEYER SPEED"), NULL, 1, 60, false);
    break;
  case KEY_MODE:
    paramAction(action, keyer_mode, 0x26, F("KEYER MODE"), keyer_mode_label, 0, 3, false);
    break;
  case KEY_PIN:
    paramAction(action, keyer_swap, 0x27, F("KEYER SWAP"), offon_label, 0, 1, false);
    break;
  case KEY_TX:
    paramAction(action, practice, 0x28, F("PRACTICE"), offon_label, 0, 1, false);
    break;
  case VOX:
    paramAction(action, vox, 0x31, F("VOX"), offon_label, 0, 1, false);
    break;
  case VOXGAIN:
    paramAction(action, vox_thresh, 0x32, F("NOISE GATE"), NULL, 0, 255, false);
    break;
  case DRIVE:
    paramAction(action, drive, 0x33, F("TX DRIVE"), NULL, 0, 8, false);
    break;
  case PWM_MIN:
    paramAction(action, pwm_min, 0x34, F("PA BIAS MIN"), NULL, 0, pwm_max - 1, false);
    break;
  case PWM_MAX:
    paramAction(action, pwm_max, 0x35, F("PA BIAS MAX"), NULL, pwm_min, 255, false);
    break;
  case SIFXTAL:
    paramAction(action, si5351.fxtal, 0x36, F("REF FRQ"), NULL, 14000000, 28000000, false);
    break;
  case IQ_ADJ:
    paramAction(action, rx_ph_q, 0x37, F("IQ PHASE"), NULL, 0, 180, false);
    break;
  case QUAD_ACTIVE:
    paramAction(action, quad_enabled, 0x38, F("QUAD"), offon_label, 0, 1, false);
    break;
  case ROTARY_SWAP:
    paramAction(action, rotxxx_swap, 0x39, F("ROTARY SWAP"), offon_label, 0, 1, false);
    break;
  case BAND:
    paramAction(action, bandsel, 0x3A, F("PEBBLE BAND"), band_label, 0, 1, false);
    break;

  // entry to open the frequency preset sub-menu
  case PRESET:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        menumode = NOT_IN_MENU;
        preset_pick = 1; // a click opens the picker
      } else {
        printlabel(action, 0x41, F("FREQ PRESETS"));
        lcd.print(F("click ENC =>"));
        lcd_blanks(10);
      }
    } else {
      // advance EEPROM address; presets themselves live in a separate area
      static uint8_t dummy_preset = 0;
      actionCommon(action, &dummy_preset, sizeof(dummy_preset));
    }
    break;

  // entry to open the CW-message sub-menu
  case CWMSGS:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        menumode = NOT_IN_MENU;
        cwmsg_pick = 1;
        cwmsg_draw();
      } else {
        printlabel(action, 0x42, F("CW MESSAGES"));
        lcd.print(F("click ENC =>"));
        lcd_blanks(10);
      }
    } else {
      static uint8_t dummy_cw = 0;
      actionCommon(action, &dummy_cw, sizeof(dummy_cw));
    }
    break;
  // the six messages (fixed EEPROM addresses)
  case CWMSG1: case CWMSG2: case CWMSG3:
  case CWMSG4: case CWMSG5: case CWMSG6: {
    uint8_t idx = id - CWMSG1;
    uint16_t addr = CWMSG_EEPROM + idx * CW_MESSAGE_LENGTH;
    if (action == LOAD) {
      eeprom_read_block(cw_msg[idx], (void *)addr, CW_MESSAGE_LENGTH);
    } else if (action == SAVE) {
      for (uint8_t i = CW_MESSAGE_LENGTH; i > 0; i--) {
        if (cw_msg[idx][i-1] == ' ' || cw_msg[idx][i-1] == 0)
          cw_msg[idx][i-1] = 0;
        else break;
      }
      eeprom_write_block(cw_msg[idx], (void *)addr, CW_MESSAGE_LENGTH);
    } else if (action == SKIP) {
      // not part of sequential layout
    } else {
      const __FlashStringHelper *lbl =
          (idx==0) ? F("CQ MSG 1") :
          (idx==1) ? F("CW MSG 2") :
          (idx==2) ? F("CW MSG 3") :
          (idx==3) ? F("CW MSG 4") :
          (idx==4) ? F("CW MSG 5") : F("CW MSG 6");
      paramAction(action, cw_msg[idx], 0x42 + idx, lbl, CW_MESSAGE_LENGTH);
    }
    break;
  }

  // entry to open the user name and callsign sub-menu
  case USER_INFO:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        menumode = NOT_IN_MENU;
        userinfo_pick = 1;
        userinfo_draw();
      } else {
        printlabel(action, 0x43, F("USER INFO"));
        lcd.print(F("click ENC =>"));
        lcd_blanks(10);
      }
    } else {
      static uint8_t dummy_user = 0;
      actionCommon(action, &dummy_user, sizeof(dummy_user));
    }
    break;
  case MY_NAME:
    if (action == LOAD) {
      eeprom_read_block(my_name, (void *)USERNAME_ADDRESS, USERNAME_LENGTH);
    } else if (action == SAVE) {
      for (uint8_t i = USERNAME_LENGTH; i > 0; i--) {
        if (my_name[i-1] == ' ' || my_name[i-1] == 0)
          my_name[i-1] = 0;
        else break;
      }
      eeprom_write_block(my_name, (void *)USERNAME_ADDRESS, USERNAME_LENGTH);
    } else if (action == SKIP) {
    } else {
      paramAction(action, my_name, 0x56, F("MY NAME"), USERNAME_LENGTH);
    }
    break;
  case MY_CALL:
    if (action == LOAD) {
      eeprom_read_block(my_call, (void *)CALLSIGN_ADDRESS, CALLSIGN_LENGTH);
    } else if (action == SAVE) {
      for (uint8_t i = CALLSIGN_LENGTH; i > 0; i--) {
        if (my_call[i-1] == ' ' || my_call[i-1] == 0)
          my_call[i-1] = 0;
        else break;
      }
      eeprom_write_block(my_call, (void *)CALLSIGN_ADDRESS, CALLSIGN_LENGTH);
    } else if (action == SKIP) {
    } else {
      paramAction(action, my_call, 0x57, F("MY CALL"), CALLSIGN_LENGTH);
    }
    break;
  case FW_VER:
    if (action == UPDATE_MENU) {
      printlabel(action, 0x44, F("PEBBLE 4.00E"));
      lcd.print(F(VERSION));
      lcd_blanks(1);
      lcd.print(F(RELDATE));  // release date
      lcd_blanks(10);
    } else {
      paramAction(action, eeprom_version, 0, NULL, NULL, 0, 0, false);
    }
    break;

#ifdef CAL_IQ
  case CALIB:
#endif
#ifdef DEBUG
  case SR:
    paramAction(action, sr, 0x91, F("SAMPLE RATE"), NULL, INT32_MIN, INT32_MAX, false);
    break;
  case CPULOAD:
    paramAction(action, cpu_load, 0x92, F("CPU LOAD %"), NULL, INT32_MIN,
                INT32_MAX, false);
    break;
  case PARAM_A:
    paramAction(action, param_a, 0x93, F("PARAM A"), NULL, 0, UINT16_MAX, false);
    break;
  case PARAM_B:
    paramAction(action, param_b, 0x94, F("PARAM B"), NULL, INT16_MIN, INT16_MAX, false);
    break;
  case PARAM_C:
    paramAction(action, param_c, 0x95, F("PARAM C"), NULL, INT16_MIN, INT16_MAX,  false);
    break;
#endif

  // Invisible parameters
  case FREQA:
    paramAction(action, vfo[VFOA], 0, NULL, NULL, 0, 0, false);
    break;
  case FREQB:
    paramAction(action, vfo[VFOB], 0, NULL, NULL, 0, 0, false);
    break;
  case MODEA:
    paramAction(action, vfomode[VFOA], 0, NULL, NULL, 0, 0, false);
    break;
  case MODEB:
    paramAction(action, vfomode[VFOB], 0, NULL, NULL, 0, 0, false);
    break;

  // Non-parameters
  case _NULL:
    menumode = NOT_IN_MENU;
    show_banner();
    change = true;
    break;

  default:
    if ((action == NEXT_MENU) && (id != N_PARAMS))
      id = paramAction(action, max(1, min(N_PARAMS, id + ((encoder_val > 0) ? 1 : -1))));
    break; // keep iterating util menu item found
  }
  return id;
}

void initPins() {
  // initialize
  fastdigitalWrite(RX, HIGH);
  fastdigitalWrite(PWM_OUT, LOW);
  fastdigitalWrite(SIDETONE, LOW);
  fastdigitalWrite(AUDIO1, LOW);
  fastdigitalWrite(AUDIO2, LOW);
  // pins
  fastpinMode(SIDETONE, OUTPUT);
  fastpinMode(RX, OUTPUT);
  fastpinMode(PWM_OUT, OUTPUT);
  fastpinMode(ROT_A, INPUT_PULLUP);
  fastpinMode(ROT_B, INPUT_PULLUP);
  fastpinMode(BUTTONS, INPUT);
  fastpinMode(DIT, INPUT_PULLUP);
  fastpinMode(DAH, INPUT);
  fastpinMode(AUDIO1, INPUT);
  fastpinMode(AUDIO2, INPUT);
  fastpinMode(PD3, OUTPUT);
}

void fatal(const __FlashStringHelper *msg, int value = 0, char unit = '\0') {
  lcd.setCursor(0,1);
  lcd.print('!');
  lcd.print('!');
  lcd.print(msg);
  if (unit != '\0') {
    lcd.print('=');
    lcd.print(value);
    lcd.print(unit);
  }
  lcd_blanks(10);
  delay(1500);
  wdt_reset();
}

// refresh LUT based on pwm_min, pwm_max
void build_lut() {
  for (uint16_t i = 0; i != 256; i++) // refresh LUT based on pwm_min, pwm_max
    lut[i] = (i * (pwm_max - pwm_min)) / 255 + pwm_min;
}

// check that freq is within the band limits
void band_check() {
  if (freq < FREQ_LIMIT_LOW) {
    freq = FREQ_LIMIT_LOW;
    vfo[vfosel] = freq;
  }
  if (freq > FREQ_LIMIT_HIGH) {
    freq = FREQ_LIMIT_HIGH;
    vfo[vfosel] = freq;
  }
}

void setup() {
  fastdigitalWrite(PWM_OUT, LOW);
  si5351.powerDown();
  MCUSR = 0;
  wdt_enable(WDTO_4S); // Enable watchdog
  uint32_t t0, t1;
#ifdef DEBUG
  // Benchmark dsp_tx() ISR
  func_ptr = dsp_tx;
  t0 = micros();
  TIMER2_COMPA_vect();
  t1 = micros();
  uint16_t load_tx = (float)(t1 - t0) * (float)F_SAMP_TX * 100.0 / 1000000.0 *
                 16000000.0 / (float)F_CPU;
  func_ptr = sdr_rx_00;
  rx_state = 0;
  uint16_t load_rx[8];
  uint16_t load_rx_avg = 0;
  uint16_t i;
  for (i = 0; i != 8; i++) {
    rx_state = i;
    t0 = micros();
    TIMER2_COMPA_vect();
    // func_ptr();
    t1 = micros();
    load_rx[i] = (float)(t1 - t0) * (float)F_SAMP_RX * 100.0 / 1000000.0 *
                 16000000.0 / (float)F_CPU;
    load_rx_avg += load_rx[i];
  }
  load_rx_avg /= 8;
#endif  // DEBUG

  ADMUX = (1 << REFS0); // restore reference voltage AREF (5V)
  // disable external interrupts
  PCICR = 0;
  PCMSK0 = 0;
  PCMSK1 = 0;
  PCMSK2 = 0;
  initPins();
  initEncoder();
  delay(100);
  lcd.begin(16, 4);
  for (i = 0; i != N_FONTS; i++) { // Init fonts
    pgm_cache_item(fonts[i], 8);
    lcd.createChar(0x01 + i, _item);
  }

#ifdef DEBUG
  // Measure CPU loads
  if (!(load_tx <= 100)) {
    fatal(F("CPU_tx"), load_tx, '%');
  }
  if (!(load_rx_avg <= 100)) {
    fatal(F("CPU_rx"), load_rx_avg, '%');
  }
#endif

#ifdef DIAG
  // Measure VDD (+5V); should be ~5V
  si5351.SendRegister(SI_CLK_OE, TX0RX0); // Mute QSD
  fastdigitalWrite(PWM_OUT, LOW);
  fastdigitalWrite(RX, LOW); // mute RX
  delay(100);                // settle
  float vdd = 2.0 * (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  fastdigitalWrite(RX, HIGH);
  if (!(vdd > 4.8 && vdd < 5.2)) {
    fatal(F("V5.0"), vdd, 'V');
  }
  // Measure VEE (+3.3V); should be ~3.3V
  float vee = (float)analogRead(SCL) * 5.0 / 1024.0;
  if (!(vee > 3.2 && vee < 3.8)) {
    fatal(F("V3.3"), vee, 'V');
  }
  // Measure AVCC via AREF and using internal 1.1V reference fed to ADC; should
  // be ~5V
  analogRead(6);    // setup almost proper ADC readout
  bitSet(ADMUX, 3); // Switch to channel 14 (Vbg=1.1V)
  delay(1);         // delay improves accuracy
  bitSet(ADCSRA, ADSC);
  for (; bit_is_set(ADCSRA, ADSC);)
    ;
  float avcc = 1.1 * 1023.0 / ADC;
  if (!(avcc > 4.6 && avcc < 5.2)) {
    fatal(F("Vavcc"), avcc, 'V');
  }

  // Measure DVM bias; should be ~VAREF/2
  PORTD |= 1 << 1;
  DDRD |= 1 << 1; // keep PD1 HIGH so that in case diode is installed to PC2 it
                  // is kept blocked (otherwise ADC2 input is pulled down!)
  delay(10);
  float dvm = (float)analogRead(DVM) * 5.0 / 1024.0;
  if (!(dvm > 1.8 && dvm < 3.2)) {
    fatal(F("Vadc2"), dvm, 'V');
  }

  // Measure AUDIO1, AUDIO2 bias; should be ~VAREF/2
  float audio1 = (float)analogRead(AUDIO1) * 5.0 / 1024.0;
  if (!(audio1 > 1.8 && audio1 < 3.2)) {
    fatal(F("Vadc0"), audio1, 'V');
  }
  float audio2 = (float)analogRead(AUDIO2) * 5.0 / 1024.0;
  if (!(audio2 > 1.8 && audio2 < 3.2)) {
    fatal(F("Vadc1"), audio2, 'V');
  }
  // Measure I2C Bus speed for Bulk Transfers
  // si5351.freq(freq, 0, 90);
  wdt_reset();
  t0 = micros();
  for (i = 0; i != 1000; i++)
    si5351.SendPLLRegisterBulk();
  t1 = micros();
  uint32_t speed = (1000000 * 8 * 7) / (t1 - t0); // speed in kbit/s
  if (false) {
    fatal(F("i2cspeed"), speed, 'k');
  }
  // Measure I2C Bit-Error Rate (BER); should be error free for a thousand
  // random bulk PLLB writes
  si5351.freq(freq, 0, 90);
  wdt_reset();
  uint16_t i2c_error = 0; // number of I2C byte transfer errors
  for (i = 0; i != 1000; i++) {
    si5351.freq_calc_fast(i);
    si5351.SendPLLRegisterBulk();
    #define SI_SYNTH_PLL_A 26
    for (uint8_t j=4; j<8; j++)
      if (si5351.RecvRegister(SI_SYNTH_PLL_A + j) != si5351.pll_regs[j])
        i2c_error++;
  }
  wdt_reset();
  if (i2c_error) {
    fatal(F("BER_i2c"), i2c_error, ' ');
  }
#endif // DIAG

  drive = 4; // Init settings
  cw_offset = tones[cw_tone];

  // Load parameters from EEPROM, reset to factory defaults
  // when stored values are from a different version
  paramAction(LOAD, FW_VER);

  if ((eeprom_version != EEPROM_VERSION) || BUTTON_PRESSED) {
    eeprom_version = EEPROM_VERSION;
    paramAction(SAVE); // save default parameter values
    lcd.setCursor(0,1);
    lcd.print(F("Reset settings.."));
    delay(500);
    wdt_reset();
  } else {
    paramAction(LOAD);    // load all parameters
  }
  show_banner();
  si5351.iqmsa = 0; // enforce PLL reset
  change = true;
  vox = false; // disable VOX
  rit = false; // disable RIT
  freq = vfo[vfosel];
  band_check();
  mode = vfomode[vfosel];
  build_lut();
  delay(800);
  start_rx();
  keyerState = IDLE;
  switch (keyer_mode) {
    case IAMBIC_A: keyerControl = IAMBICA; break;
    case IAMBIC_B: keyerControl = IAMBICB; break;
    default:       keyerControl = STRAIGHT; break;
  }
  loadWPM(keyer_speed);
  while (DIT_PRESSED || ((mode == CW && keyer_mode) && DAH_PRESSED)) {
    fatal(F("Check PTT/key"));
  }
}

void loop() {
  if ((vox) && ((mode == LSB) || (mode == USB))) {
    if (!vox_tx) {
      if (vox_sample++ == 16) {
        ssb(((int16_t)(vox_adc / 16) - (512 - AF_BIAS)) >> MIC_ATTEN);
        vox_sample = 0;
        vox_adc = 0;
      } else {
        vox_adc += analogSampleMic();
      }
      if (tx) {
        vox_tx = 1;
        switch_rxtx(255);
      }
    } else if (!tx) { // VOX activated, no audio detected -> RX
      switch_rxtx(0);
      vox_tx = 0;
      delay(32); // delay(10);
    }
  }

  if ((mode == CW) && cwdec && (!tx && !semi_qsk_timeout))
    cw_decode(); // CW decoder only active during RX
  if ((menumode == NOT_IN_MENU) && !preset_pick && !cwmsg_pick && !userinfo_pick) {
    if (cw_event) {
      const char offv[] = {0, 7, 3, 5, 3, 7, 8};
      uint8_t offset = offv[smode];
      lcd.noCursor();
      cw_event = false;
      lcd.setCursor(0,0);
      lcd.print(out + offset);
      stepsize_showcursor();
    } else if ((!semi_qsk_timeout) && (!vox_tx))
      smeter();
  }

  if (mode == CW && keyer_mode != STRAIGHT) {
    switch (keyerState) {
    case IDLE: // Wait for direct or latched paddle press
      if ((digitalRead(DAH) == LOW) || (digitalRead(DIT) == LOW) ||
          (keyerControl & 0x03)) {
        cw_msg_event = 0; // clear cw message event
        update_PaddleLatch();
        // Ultimatic: prefer the most recently pressed paddle
        if (keyer_mode == ULTIMATIC && last_paddle == 1) {
          keyerState = CHK_DAH;
        } else {
          keyerState = CHK_DIT;
        }
      }
      break;
    case CHK_DIT: // See if the dit paddle was pressed
      if (keyerControl & DIT_L) {
        keyerControl |= DIT_PROC;
        ktimer = ditTime;
        keyerState = KEYED_PREP;
      } else {
        keyerState = CHK_DAH;
      }
      break;
    case CHK_DAH: // See if dah paddle was pressed
      if (keyerControl & DAH_L) {
        ktimer = ditTime * 3;
        keyerState = KEYED_PREP;
      } else {
        keyerState = IDLE;
      }
      break;
    case KEYED_PREP: // Assert key down, start timing
      key_state = HIGH;
      switch_rxtx(key_state);
      ktimer += millis();               // set ktimer to interval end time
      keyerControl &= ~(DIT_L + DAH_L); // clear both paddle latch bits
      keyerState = KEYED;               // next state
      break;
    case KEYED:                // Wait for timer to expire
      if (millis() > ktimer) { // are we at end of key down ?
        key_state = LOW;
        switch_rxtx(key_state);
        ktimer = millis() + ditTime; // inter-element time
        keyerState = INTER_ELEMENT;  // next state
      } else if (keyerControl & IAMBICB) {
        update_PaddleLatch(); // early paddle latch in Iambic B mode
      }
      break;
    case INTER_ELEMENT:
      // Insert time between dits/dahs
      update_PaddleLatch();                    // latch paddle state
      if (millis() > ktimer) {                 // are we at end of inter-space ?
        if (keyer_mode == ULTIMATIC) {
          // Ultimatic: return to IDLE so the same element can repeat if still held
          keyerControl &= ~(DIT_L + DAH_L + DIT_PROC);
          keyerState = IDLE;
        } else if (keyerControl & DIT_PROC) {  // was it a dit or dah ?
          keyerControl &= ~(DIT_L + DIT_PROC); // clear two bits
          keyerState = CHK_DAH;                // dit done, check for dah
        } else {
          keyerControl &= ~(DAH_L); // clear dah latch
          keyerState = IDLE;        // go idle
        }
      }
      break;
    default:
      break;
    }
  } else {
    uint8_t pin = ((mode == CW) && (keyer_swap)) ? DAH : DIT;
    if (!vox_tx) //  ONLY if VOX not active, then check DIT/DAH (fix for VOX to
                 //  prevent RFI feedback through EMI on DIT or DAH line)
      if (!digitalRead(pin)) { // PTT/DIT keys transmitter
        cw_msg_event = 0;       // clear cw message event
        switch_rxtx(1);         // Switch to TX mode
        do {
          wdt_reset();
          delay((mode == CW)
                    ? 10
                    : 100); // keep the tx keyed for a while before sensing
                            // (helps against RFI issues on DAH/DAH line)
          if (BUTTON_PRESSED)
            break; // break if button is pressed (to prevent potential lock-up)
        } while (!digitalRead(pin)); // until released
        switch_rxtx(0);
      }
  }

  if ((semi_qsk_timeout) && (millis() > semi_qsk_timeout)) {
    switch_rxtx(0);
  } // delayed QSK RX

  // button-left, button-right, button-encoder
  // single-click, double-click, push-long, push-and-turn
  enum event_t {
    BL  = 0x10,
    BR  = 0x20,
    BE  = 0x30,
    SC  = 0x01,
    DC  = 0x02,
    PL  = 0x04,
    PLC = 0x05,
    PT  = 0x0C
  };

  if (BUTTON_PRESSED) {
    // Left-/Right-/Rotary-button (while not already pressed)
    if (!((event & PL) ||
          (event &
           PLC))) { // hack: if there was long-push before, then fast forward
      uint16_t adcval = analogSafeRead(BUTTONS);
      event = SC;
      int32_t t0 = millis();
      while (BUTTON_PRESSED) { // until released or long-press
        if ((millis() - t0) > 300) {
          event = PL;
          break;
        }
        wdt_reset();
      }
      delay(10); // debounce
      for (; (event != PL) && ((millis() - t0) < 500);) {
        // until 2nd press or timeout
        if (BUTTON_PRESSED) {
          event = DC;
          break;
        }
        wdt_reset();
      }
      while (BUTTON_PRESSED) {  // until released, or encoder
                                // is turned while longpress
        if (encoder_val && event == PL) {
          event = PT;
          break;
        }
        wdt_reset();
      }
      // determine which button is pressed
      #define BE_THRESHOLD 999   // 4.88V
      #define BR_THRESHOLD 890   // 4.34V
      if (adcval      > BE_THRESHOLD) event |= BE;
      else if (adcval > BR_THRESHOLD) event |= BR;
      else event |= BL;
    } else {        // hack: fast forward handling
      event = (event & 0xf0) | ((encoder_val) ? PT : PLC);
    }
    if (preset_pick) {
      // press Encoder=go  press Select=store  press Menu=cancel
      if ((event == (BE|SC)) && preset_get(preset_sel)) {
        uint32_t v = preset_get(preset_sel);
        freq = v & 0xFFFFFFUL; mode = v >> 24; rit = 0;
        vfomode[vfosel%2] = mode;
        paramAction(SAVE, (vfosel%2) ? MODEB : MODEA);
        if (mode != CW) stepsize = STEP_1k; else stepsize = STEP_500;
        if (mode == CW) {
          filt = 4;
          nr = 0;
        } else filt = 0;
        si5351.iqmsa = 0;  // enforce PLL reset
        preset_pick = 0;
        show_banner();
        change = true;
      }
      if (event == (BL|SC)) {
        preset_pick = 0;
        show_banner();
        change = true;
      }
      if (event == (BR|SC)) preset_store();
      event = 0;
    }
    if (cwmsg_pick) {
      // press Encoder=edit  press Select=send  press Menu=cancel
      if (event == (BE|SC)) {
        cwmsg_pick = 0;
        menu = CWMSG1 + cwmsg_sel;
        menumode = SELECT_VALUE;
        paramAction(UPDATE_MENU, menu);
      }
      if (event == (BL|SC)) {
        // cancel any scheduled repeating beacon
        cw_msg_event = 0;
        switch_rxtx(0);
        cwmsg_pick = 0;
        show_banner();
        change = true;
      }
      if (event == (BR|SC)) {
        // transmit the message
        cw_msg_id = cwmsg_sel;
        cw_tx_msg(cw_msg[cwmsg_sel]);
        // for non-zero interval setup the next event
        if (cw_msg_interval) {
          cw_msg_event = millis() + ((uint32_t)cw_msg_interval * 1000UL);
        }
      }
      event = 0;
    }
    if (userinfo_pick) {
      // press Encoder=edit  press Select=back/cancel  press Menu=cancel
      if (event == (BE|SC)) {
        userinfo_pick = 0;
        menu = (userinfo_sel == 0) ? MY_NAME : MY_CALL;
        menumode = SELECT_VALUE;
        paramAction(UPDATE_MENU, menu);
      }
      if (event == (BL|SC)) {
        userinfo_pick = 0;
        show_banner();
        change = true;
      }
      event = 0;
    }
    switch (event) {
    case BL | PL:  // Called when menu button pressed
    case BL | PLC: // or kept pressed
      menumode = SELECT_VALUE;
      break;
    case BL | PT:
      menumode = SELECT_MENU;
      break;
    case BL | SC:
      if ((menumode == SELECT_MENU) && (menu >= CWMSG1 && menu <= CWMSG6)) {
        cw_msg_event = millis();
        cw_msg_id = menu - CWMSG1;
        menumode = NOT_IN_MENU;
        break;
      }
      int8_t _menumode;
      if (menumode == NOT_IN_MENU) {
        _menumode = SELECT_MENU;
        if (menu == 0)
          menu = 1;
      }
      else if (menumode == SELECT_MENU) {
        if (menu == FW_VER || menu == PRESET ||
            menu == CWMSGS || menu == USER_INFO) {
          _menumode = SELECT_MENU;
        } else {
          _menumode = SELECT_VALUE;
        }
      }
      else if (menumode >= SELECT_VALUE) {
        paramAction(SAVE, menu);
        _menumode = NOT_IN_MENU;
      }
      menumode = _menumode;
      break;
    case BL | DC:
      break;
    case BR | SC: // Mode change - Button Right, Single Click
      if (!(menumode >= SELECT_VALUE && (menu >= CWMSG1 && menu <= CWMSG6
        || menu == MY_NAME || menu == MY_CALL))) {
        changedMode = true;
      }
      break;
    case BR | DC:
      filt++;
      _init = true;
      if (mode == CW && filt > N_FILT)
        filt = 4;
      if (mode == CW && filt == 4)
        stepsize = STEP_500; // reset stepsize for 500Hz filter
      if (mode == CW && (filt == 5 || filt == 6) && stepsize < STEP_100)
        stepsize = STEP_100; // for CW BW 200, 100      -> step = 100 Hz
      if (mode == CW && filt == 7 && stepsize < STEP_10)
        stepsize = STEP_10; // for CW BW 50 -> step = 10 Hz
      if (mode != CW && filt > 3)
        filt = 0;
      encoder_val = 0;
      paramAction(UPDATE, FILTER);
      paramAction(SAVE, FILTER);
      wdt_reset();
      delay(1500);
      wdt_reset();
      change = true; // refresh display
      break;
    case BR | PL:
      rit = !rit;
      stepsize = (rit) ? STEP_10 : prev_stepsize[mode == CW];
      if (!rit) { // after RIT comes VFO A/B swap
        vfosel = !vfosel;
        freq = vfo[vfosel];
        mode = vfomode[vfosel];
        // make more generic:
        if (mode != CW)
          stepsize = STEP_1k;
        else
          stepsize = STEP_500;
        if (mode == CW) {
          filt = 4;
          nr = 0;
        } else
          filt = 0;
      }
      change = true;
      break;
    case BE | SC:
      if (menumode == NOT_IN_MENU) {
        stepsize_change(+1);
      } else {
        int8_t _menumode;
        if (menumode == SELECT_MENU) {
          _menumode = SELECT_VALUE;
        }
        if (menumode == SELECT_VALUE) {
          _menumode = SELECT_MENU;
          change = true;
          paramAction(SAVE, menu);
        }
        if (menumode == MENU_NEXT) {
          _menumode = MENU_NEXT;
          paramAction(NEXT_CH, menu);
        }
        menumode = _menumode;
      }
      break;
    case BE | PL:
      stepsize_change(-1);
      break;
    case BE | PT:
      while (BUTTON_PRESSED) {
        wdt_reset();
        if (encoder_val) {
          paramAction(UPDATE, VOLUME);
          paramAction(SAVE, VOLUME);
        }
      }
      change = true; // refresh display
      break;
    default:
      break;
    }
  } else event = 0; // no button pressed

  if (changedMode) {
    changedMode = false;
    if (menumode == NOT_IN_MENU) {
      prev_mode = mode;
      mode++; // Change to next mode += 1
      if (mode > AM)  mode = LSB;
      if (mode == CW) nr = 0;
      prev_stepsize[prev_mode == CW] = stepsize;
      stepsize = prev_stepsize[mode == CW];
      prev_filt[prev_mode == CW] = filt;
      filt = prev_filt[mode == CW];
      // paramAction(UPDATE, MODE);
      vfomode[vfosel] = mode;
      paramAction(SAVE, (vfosel) ? MODEB : MODEA); // save vfoa/b changes
      paramAction(SAVE, MODE);
      paramAction(SAVE, FILTER);
      si5351.iqmsa = 0; // enforce PLL reset
      if (mode < CW) show_banner();
      change = true;
    } else {
      if (menumode == SELECT_MENU) {
        menumode = NOT_IN_MENU;
      } // short right-click while in menu: enter value selection screen
      if (menumode >= SELECT_VALUE) {
        menumode = SELECT_MENU;
        change = true;
        paramAction(SAVE, menu);
      } // short right-click while in value selection screen
        // save and return to menu screen
    }
  }

  if ((menumode) || (prev_menumode != menumode)) { // Show parameter and value
    int8_t encoder_change = encoder_val;
    if ((menumode == SELECT_MENU) && encoder_change) {
      menu += encoder_val; // Navigate through menu
      if (menu > N_PARAMS)
        menu = 1;
      else if (menu < 1)
        menu = N_PARAMS;
      menu = paramAction(NEXT_MENU, menu); // auto probe next menu item
      encoder_val = 0;
    }
    if (encoder_change || (prev_menumode != menumode))
      paramAction(UPDATE_MENU, (menumode) ? menu : 0);
    // force display refresh when exiting menu (required for PRESET picker)
    if ((prev_menumode != NOT_IN_MENU) && (menumode == NOT_IN_MENU)) {
      if (mode != CW) show_banner();
      display_vfo(freq);
      stepsize_showcursor();
    }
    prev_menumode = menumode;
    if (menumode == SELECT_VALUE) {
      if (encoder_change) {
        lcd.setCursor(0,1);
        lcd.cursor();
        if (menu == MODE) {
          vfomode[vfosel] = mode;
          paramAction(SAVE, (vfosel) ? MODEB : MODEA);
          change = true;
          si5351.iqmsa = 0; // enforce PLL reset
          // make more generic:
          if (mode != CW)
            stepsize = STEP_1k;
          else
            stepsize = STEP_500;
          if (mode == CW) {
            filt = 4;
            nr = 0;
          } else
            filt = 0;
        }

        // if(menu == NR){ if(mode == CW) nr = false; }
        if (menu == VFOSEL) {
          freq = vfo[vfosel];
          mode = vfomode[vfosel];
          // make more generic:
          if (mode != CW)
            stepsize = STEP_1k;
          else
            stepsize = STEP_500;
          if (mode == CW) {
            filt = 4;
            nr = 0;
          } else
            filt = 0;
          change = true;
        }
        if (menu == RIT) {
          stepsize = (rit) ? STEP_10 : STEP_500;
          change = true;
        }
        if (menu == BAND) {
          band_check();
        }
        // if(menu == VOX){ if(vox){ vox_thresh-=1; } else { vox_thresh+=1; }; }
        if (menu == ATT) { // post-handling ATT parameter
          noInterrupts();
          adc_start(0, !(att & 0x01), F_ADC_CONV);
          admux[0] = ADMUX;
          adc_start(1, !(att & 0x01), F_ADC_CONV);
          admux[1] = ADMUX;
          interrupts();
          fastdigitalWrite(RX, !(att & 0x02));
          // att bit 1 ON: attenuate -20dB by disabling
          // RX line, switching Q5 (antenna input
          // switch) into 100k resistence
          fastpinMode(AUDIO1, (att & 0x04) ? OUTPUT : INPUT);
          // att bit 2 ON: attenuate -40dB by
          // terminating ADC inputs with 10R
          fastpinMode(AUDIO2, (att & 0x04) ? OUTPUT : INPUT);
        }
        if (menu == SIFXTAL) {
          change = true;
        }
        if ((menu == PWM_MIN) || (menu == PWM_MAX)) {
          build_lut();
        }
        if (menu == CWTONE) {
          cw_offset = tones[cw_tone];
        }
        if (menu == IQ_ADJ) {
          change = true;
        }
#ifdef CAL_IQ
        if (menu == CALIB) {
          menu = 0;
        }
#endif
        if (menu == KEY_WPM) {
          loadWPM(keyer_speed);
        }
        if (menu == KEY_MODE) {
          switch (keyer_mode) {
            case IAMBIC_A: keyerControl = IAMBICA; break;
            case IAMBIC_B: keyerControl = IAMBICB; break;
            default:       keyerControl = STRAIGHT; break;
          }
        }
      }
#ifdef DEBUG
      if (menu == SR) { // measure sample-rate
        numSamples = 0;
        delay(F_MCU * 500UL / 16000000);
        sr = numSamples * 2;            // samples per second
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if (menu == CPULOAD) { // measure CPU-load
        uint32_t i = 0;
        uint32_t prev_time = millis();
        for (i = 0; i != 300000; i++)
          wdt_reset();
        cpu_load = 100 - 132 * 100 / (millis() - prev_time);
        paramAction(UPDATE_MENU, menu); // refresh
      }
      if ((menu == PARAM_A) || (menu == PARAM_B) || (menu == PARAM_C)) {
        delay(300);
        paramAction(UPDATE_MENU, menu); // refresh
      }
#endif
    }
  }
  if (menumode == NOT_IN_MENU) {
    if (preset_pick && encoder_val) {
      // scroll the preset picker
      preset_sel = (((int16_t)preset_sel + encoder_val) % N_PRESET + N_PRESET) % N_PRESET;
      encoder_val = 0;
      preset_draw();
    }
    else if (cwmsg_pick && encoder_val) {
      // scroll the CW message picker
      cwmsg_sel = (((int16_t)cwmsg_sel + encoder_val) % 6 + 6) % 6;
      encoder_val = 0;
      cwmsg_draw();
    }
    else if (userinfo_pick && encoder_val) {
      // scroll the user info picker (2 items: Name and Callsign)
      userinfo_sel = (((int16_t)userinfo_sel + encoder_val) % 2 + 2) % 2;
      encoder_val = 0;
      userinfo_draw();
    }
    // hack for reversing rotary-encoder direction
    if (rotary_swap != rotxxx_swap) rotary_swap = rotxxx_swap;
    if (encoder_val) {
      process_encoder_tuning_step(encoder_val);
      encoder_val = 0;
    }
  }
  if (change && !tx && !vox_tx) {
    change = false;
    vfo[vfosel] = freq;
    // save_event_time = millis() + 1000;
    save_event_time = millis() + 2000;
    if (menumode == NOT_IN_MENU) {
      display_vfo(freq);
      stepsize_showcursor();
    }
    if (mode == CW) {
      si5351.freq(freq + cw_offset, rx_ph_q, 0);
    } else if (mode == LSB)
      si5351.freq(freq, rx_ph_q, 0); // RX in LSB
    else
      si5351.freq(freq, 0, rx_ph_q); // RX in USB
    if (rit) {
      si5351.freq_calc_fast(rit);
      si5351.SendPLLRegisterBulk();
    }
  }
  if (save_event_time && (millis() > save_event_time)) {
    paramAction(SAVE, (vfosel) ? FREQB : FREQA); // save vfoa/b changes
    save_event_time = 0;
  }
  // Check for start of next CW repeating beacon
  if ((mode == CW) && (cw_msg_event) && (millis() > cw_msg_event)) {
    cw_tx_msg(cw_msg[cw_msg_id]);
    if (cw_msg_interval)
      cw_msg_event = millis() + (1000 * cw_msg_interval);
    else
      cw_msg_event = 0;
  }
  wdt_reset();
}


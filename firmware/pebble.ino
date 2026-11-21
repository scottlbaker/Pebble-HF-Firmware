
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

#define VERSION "1.00d"            // firmware version
#define TIMESTAMP "11210931"       // firmware timestamp
#define EEPROM_VERSION   2         // track eeprom layout changes

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
#define KEY_PRESSED    (DIT_PRESSED || DAH_PRESSED)

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

#include <avr/sleep.h>
#include <avr/wdt.h>

// function prototypes
void fastpinMode(uint8_t, uint8_t);
void fastdigitalWrite(uint8_t, uint8_t);
int16_t arctan3(int16_t, int16_t);
int16_t ssb(int16_t);
void dsp_tx();
void process_minsky();
void dsp_tx_cw();
void wait_us(uint16_t);
bool delayWithKeySense(uint16_t);
void lcd_blanks(uint8_t);
void printsym(uint8_t);
void smart_cq_stop();
void smart_cq_start_listen();
void smart_cq_check();
void cw_decode();
void stepsize_showcursor();
void stepsize_change(int8_t);
void dec2();
int16_t process_agc_fast(int16_t);
int16_t process_agc(int16_t);
int16_t process_nr(int16_t);
int16_t auto_nr(int16_t);
int16_t filt_var(int16_t);
int16_t slow_dsp(int16_t, int16_t);
void process(int16_t, int16_t);
void sdr_rx_00();
void sdr_rx_01();
void sdr_rx_02();
void sdr_rx_03();
void sdr_rx_04();
void sdr_rx_05();
void sdr_rx_06();
void sdr_rx_07();
int16_t sdr_rx_common_q();
int16_t sdr_rx_common_i();
uint8_t log2(uint16_t);
void adc_start(uint8_t, bool, uint32_t);
void adc_stop();
uint16_t analogSafeRead(uint8_t, uint8_t);
uint16_t analogSampleMic();
void timer1_start(uint32_t);
void timer1_stop();
void timer2_start(uint32_t);
void timer2_stop();
int16_t smeter(int16_t);
void autotune_start();
void autotune_print(char *);
void autotune_sample(int16_t);
void autotune_finish();
uint32_t preset_get(uint8_t);
void preset_print(uint32_t);
static void submenu_preset();
static void submenu_cwmsg();
void submenu_draw();
void submenu_enter(uint8_t);
void submenu_exit();
static void return_to_picker(bool);
static void preset_store();
static void picker_apply_preset();
bool picker_handle_event(uint8_t);
void picker_encoder(int8_t);
void start_rx();
void display_vfo(int32_t);
uint8_t cw_tx(char);
uint8_t cw_tx_str(char *);
void process_encoder_tuning_step(int8_t);
void printmenuid(uint8_t);
void initPins();
void initEncoder();
void build_lut();
void band_check();
void update_PaddleLatch();
void loadWPM(uint8_t);
void iambic_keyer();

const char *vfosel_label[] = {"A", "B"};
const char *mode_label[3] = {"LSB", "USB", "CW "};

#define ONE_SECOND     1000
#define TWO_SECONDS    2000

#define USERNAME_LENGTH  10
#define CALLSIGN_LENGTH  10

#define PARAM_EEPROM     0x010   // 0x010 -> 0x100
#define CWMSG_EEPROM     0x100   // 0x100 -> 0x2FF
#define PRESET_EEPROM    0x300   // 0x300 -> 0x31F
#define USERNAME_ADDRESS 0x3E0   // 0x3E0 -> 0x3EF
#define CALLSIGN_ADDRESS 0x3F0   // 0x3F0 -> 0x3FF

// Menu navigation boundaries
#define MAIN_MENU_START     VOLUME
#define MAIN_MENU_END       SETTINGS
#define SETTINGS_MENU_START TONE_VOL
#define SETTINGS_MENU_END   FW_VER

#define N_PARAMS   FW_VER
#define I_PARAMS    4
#define ALL_PARAMS (N_PARAMS + I_PARAMS)

enum params_t {
  _NULL = 0,

  // Main Menu (Operating)
  VOLUME,        // 1
  MODE,          // 2
  FILTER,        // 3
  RIT,           // 4
  KEY_WPM,       // 5
  KEY_MODE,      // 6
  VOX,           // 7
  PRESET,        // 8   -- submenu
  CWMSGS,        // 9   -- submenu
  SETTINGS,      // 10  -- submenu

  // Submenu (Settings)
  TONE_VOL,      // 11
  CWDEC,         // 12
  CWTONE,        // 13
  CWINTERVAL,    // 14
  SEMIQSK,       // 15
  KEY_SWAP,      // 16
  SMART_CQ,      // 17
  VFOSEL,        // 18
  SMETER,        // 19
  AGC,           // 20
  NR,            // 21
  ATT,           // 22
  ATT2,          // 23
  NOISE_GATE,    // 24
  TX_DRIVE,      // 25
  PWM_MIN,       // 26
  PWM_MAX,       // 27
  VFO_XTAL,      // 28
  IQ_ADJ,        // 29
  QUAD_ACTIVE,   // 30
  BAND,          // 31
  ROTARY_SWAP,   // 32
  MY_NAME,       // 33
  MY_CALL,       // 34
  PRACTICE,      // 35
  FW_VER,        // 36

  // Invisible parameters that are always saved/loaded
  FREQA,         // 37
  FREQB,         // 38
  MODEA,         // 39
  MODEB,         // 40

  MODE_FILT,     // 41
  MODE_NR,       // 42
  MODE_STEP,     // 43

  // Optional parameters for debug and calibration
  CALIB,         // 44
  SR,            // 45
  CPULOAD,       // 46
  PARAM_A,       // 47
  PARAM_B,       // 48
  PARAM_C,       // 49

  // eeprom addresses for the sub-menu vars
  PRESET1 = 100,
  PRESET2,
  PRESET3,
  PRESET4,
  PRESET5,
  PRESET6,
  PRESET7,
  PRESET8,

  CWMSG1,
  CWMSG2,
  CWMSG3,
  CWMSG4,
  CWMSG5,
  CWMSG6,

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

// configuration switches
#define KEY_CLICK 1
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

static uint8_t rx_ph_q = 90;
static uint8_t practice = false;
volatile int8_t  volume = 12;
volatile uint8_t vox = 0;
volatile uint8_t sending_cw_msg = false;

class LCD : public Print {
public:
#define _dn 0 // PD0 to PD3 connect to D4 to D7
#define _en 4 // PD4 - MUST have pull-up resistor
#define _rs 4 // PC4 - MUST have pull-up resistor
#define LCD_RS_HI() DDRC &= ~(1 << _rs); asm("nop");
#define LCD_RS_LO() DDRC |= 1 << _rs;
#define LCD_EN_LO() PORTD &= ~(1 << _en);
#define LCD_EN_HI() PORTD |= (1 << _en);
#define LCD_PREP_NIBBLE(b) (PORTD & ~(0xf << _dn)) | (b) << _dn | 1 << _en

  void begin(uint8_t x = 0, uint8_t y = 0) {
    DDRD |= 0xf << _dn | 1 << _en; // Make data, EN outputs
    DDRC |= 1 << _rs;
    delayMicroseconds(50000);
    LCD_RS_LO();
    LCD_EN_LO();
    cmd(0x33);                // set to 8-bit mode
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(4500);
    cmd(0x33);
    delayMicroseconds(150);   // set to 8-bit mode
    cmd(0x32);                // set to 4-bit mode
    cmd(0x28);                // config: 2-line, 5x8
    cmd(0x0c);                // display on
    cmd(0x01);                // clear display
    delay(3);                 // wait for clear to complete
    cmd(0x06);                // entrymode: left, shift-dec
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
    for (uint8_t i=0; i<8; i++) write(glyph[i]);
  }
};

LCD lcd;

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

uint8_t mode_filt[] = {0, 4}; // filter for SSB, CW
uint8_t mode_step[] = {STEP_1k, STEP_500}; // stepsize for SSB, CW
uint8_t mode_nr[] = {2, 0};   // noise reduction for SSB, CW

#define N_FILT 7

const char *offon_label[2] = {"OFF", "ON"};
const char *filt_label[N_FILT + 1] = {"FULL", "3000", "2400", "1800",
                                      "500",  "200",  "100",  "50"};

const char *stepsize_label[] = {"10M", "1M",  ".5M", "100K", "10K",
                                "1K",  ".5K", "100", "10",   "1"};

const char *att_label[] = {"0dB",   "-13dB", "-20dB", "-33dB",
                           "-40dB", "-53dB", "-60dB", "-73dB"};

const char *smeter_label[] = {"OFF", "DBM", "S", "SBAR", "WPM"};
const char *agc_label[] = {"OFF", "FAST", "SLOW"};
const char *cw_tone_label[] = {"600", "700"};
const char *keyer_label[] = { "STRAIGHT", "IAMBIC A", "IAMBIC B", "ULTIMATIC"};

enum mode_t { LSB, USB, CW };
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
volatile uint8_t txdrive = 2;

static uint8_t quad_enabled = false;
volatile uint8_t quad = 0;

inline int16_t ssb(int16_t in) {
  static int16_t dc, z1;
  int16_t i, q;
  uint8_t j;
  static int16_t v[16];
  for (j = 0; j != 15; j++)
    v[j] = v[j + 1];

  // experimental DC-before-compression voice path by 9M2GRC
  dc = (in + dc) / 2;
  int16_t mic = in - dc;  // remove DC offset
  int16_t ac = mic * 2 + z1;
  z1 = (mic - 12 * z1) / 13;

  if (ac > 250) ac = 250 + (ac - 250) / 2;
  else if (ac < -250) ac = -250 - (-250 - ac) / 2;

  // Protect the existing 16-bit Hilbert weighted sum under overload.
  // |v|<=320 => |sum|<=2*320*(2+8+21+16)=30080 < 32767
  // This is an overload rail, not an ALC or substitute for correct mic level.
  if (ac > 320) ac = 320;
  else if (ac < -320) ac = -320;
  v[15] = ac;

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
  if (txdrive >= 8 || _amp >= (255u >> txdrive))
    _amp = 255;
  else
    _amp <<= txdrive;
#else
  _amp = _amp << (txdrive);
  _amp = ((_amp > 255) || (txdrive == 8)) ? 255 : _amp;
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

volatile uint32_t cw_offset;
volatile uint8_t tone_vol = 12;
volatile uint8_t cw_tone = 0;
const uint32_t tones[] = {(F_MCU*600ULL)/20000000, (F_MCU*700ULL)/20000000};
volatile int8_t p_sin = 0;       // initialized with A*sin(0) = 0
volatile int8_t n_cos = 448 / 4; // initialized with A*cos(t) = A

// Minsky circle sample
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

// microsecond delay
void wait_us(uint16_t x) {
  uint16_t t = ((x * 3) + (x>>1));
  for (uint16_t i=0; i<t; i++) {
    asm("");
  }
}

bool delayWithKeySense(uint16_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {
    wdt_reset();
    if (KEY_PRESSED) {
      while (KEY_PRESSED) wdt_reset(); // wait
      return true;
    }
  }
  return false;
}

uint8_t cw_msg_interval = 5; // CW message repeat interval
uint32_t cw_msg_event = 0;
uint8_t cw_msg_id = 0;       // selected message

// 0=not in menu, 1=selects menu item, 2=selects parameter value
#define NOT_IN_MENU    0
#define SELECT_MENU    1
#define SELECT_VALUE   2
#define MENU_NEXT      3

volatile uint8_t menumode = NOT_IN_MENU;
bool in_settings = false;

// button event codes
enum event_t {
  BL  = 0x10,  // button left (menu)
  BR  = 0x20,  // button right (select)
  BE  = 0x30,  // button encoder
  SC  = 0x01,  // single click
  DC  = 0x02,  // double click
  PL  = 0x04,  // push long
  PLC = 0x05,  // push long continuous
  PT  = 0x0C   // push and turn
};

// unified sub-menu picker (presets / CW messages)
enum picker_t {
  NOT_SUBMENU = 0,
  SUBMENU_PRESET,
  SUBMENU_CWMSG
};
uint8_t submenu_type = NOT_SUBMENU;
uint8_t picker_sel  = 0;   // current slot within the submenu

// menu / parameter action codes (needed by picker before paramAction is defined)
enum action_t { UPDATE, UPDATE_MENU, NEXT_MENU, LOAD, SAVE, SKIP, NEXT_CH };
volatile int8_t menu = 0; // current parameter id selected in menu

// forward declarations used by the picker
void show_banner();
void cw_tx_msg(char *msg);
int8_t paramAction(uint8_t action, uint8_t id = ALL);
void switch_rxtx(uint8_t tx_enable);

volatile uint8_t cwdec = 1;
static int32_t avg = 256;
static uint8_t sym;
static uint32_t amp32 = 0;
volatile uint32_t _amp32 = 0;
static char cwbuf[] = "                ";
volatile uint8_t cw_event = false;

// print spaces
void lcd_blanks(uint8_t x) {
  while (x--) lcd.print(' ');
}

// print a decoded symbol
void printsym(uint8_t submit = true) {
  if (sym < 128) {
    char ch = pgm_read_byte_near(m2c + sym);
    if (ch != '*') {
      for (uint8_t i=0; i<15; i++)
        cwbuf[i] = cwbuf[i + 1];
      cwbuf[15] = ch;
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

// While a repeating CQ is between transmissions
// listen for a CW reply and cancel the repeat if one is heard.
// Smart CQ uses the decoder's already-filtered CW state
// It does not require successful character decoding,
// only several plausible key-down pulses.

#define SMART_CQ_SETTLE_MS       150UL
#define SMART_CQ_WINDOW_MS      5000UL
#define SMART_CQ_MIN_EDGES         6U
#define SMART_CQ_MIN_ON_MS        20U
#define SMART_CQ_MAX_ON_MS       500U

uint8_t  smart_cq_enabled   = true;
uint8_t  smart_cq_listening = false;
uint8_t  smart_cq_edges = 0;
uint8_t  smart_cq_prev_state = LOW;
uint32_t smart_cq_start = 0;
uint32_t smart_cq_edge_time = 0;

void smart_cq_stop() {
  smart_cq_listening = false;
  smart_cq_edges = 0;
}

void smart_cq_start_listen() {
  smart_cq_listening = true;
  smart_cq_edges = 0;
  smart_cq_prev_state = filteredstate;
  smart_cq_start = millis();
  smart_cq_edge_time = smart_cq_start;
}

void smart_cq_check() {
  if (!smart_cq_listening) return;
  if (tx || sending_cw_msg || mode != CW || !cw_msg_event) {
    smart_cq_stop();
    return;
  }

  uint32_t now = millis();
  if ((now - smart_cq_start) >= SMART_CQ_WINDOW_MS) {
    smart_cq_stop();
    return;
  }
  if ((now - smart_cq_start) < SMART_CQ_SETTLE_MS) {
    smart_cq_prev_state = filteredstate;
    return;
  }

  if (filteredstate != smart_cq_prev_state) {
    uint32_t duration = now - smart_cq_edge_time;
    smart_cq_edge_time = now;

    // Count completed key-down pulses only, and reject very short noise
    // spikes or implausibly long carriers.
    if (smart_cq_prev_state == HIGH &&
        duration >= SMART_CQ_MIN_ON_MS &&
        duration <= SMART_CQ_MAX_ON_MS) {
      if (smart_cq_edges < 255) smart_cq_edges++;
      if (smart_cq_edges >= SMART_CQ_MIN_EDGES) {
        cw_msg_event = 0;
        smart_cq_stop();
        lcd.noCursor();
        lcd.setCursor(0, 0);
        lcd.print(F("CW HEARD        "));
        return;
      }
    }
    smart_cq_prev_state = filteredstate;
  }
}

// CW decoder
inline void cw_decode() {
  int32_t in = _amp32;
  EA(avg, in, (1 << 8));
  realstate = (in > (avg * 1 / 2)); // threshold
  // clean up the state with a noise blanker
  if (realstate != realstatebefore) {
    laststarttime = millis();
  }
  if ((millis() - laststarttime) >= nbtime) {
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
  if (submenu_type != NOT_SUBMENU) {
    lcd.noCursor();
    return;
  }
  lcd.setCursor(stepsize+1, 1);
  lcd.cursor();
}

// change the stepsize
void stepsize_change(int8_t val) {
  stepsize += val;
  if (stepsize < STEP_1M) stepsize = STEP_10;
  if (stepsize > STEP_10) stepsize = STEP_1M;
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
  if (((millis() - startttimelow) >= hightimesavg * 6) && (sym > 1)) {
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

inline int16_t process_nr(int16_t in) {
  static int16_t ea1;
  ea1 = EA(ea1, in, 1 << (nr - 1));
  return ea1;
}

inline int16_t auto_nr(int16_t in) {
  static int16_t ea2;
  ea2 = EA(ea2, in, 2);
  return ea2;
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

static uint32_t absavg256 = 0;
volatile uint32_t _absavg256 = 0;
volatile int16_t i, q;
int16_t v[14]; // Process Q (down-sampled) samples
int16_t vi[7];

uint8_t autotune_active = false;
uint8_t autotune_done = false;
uint8_t autotune_restart = false;

inline int16_t slow_dsp(int16_t i_ac2, int16_t q_ac2) {
  // Post processing I and Q (down-sampled) results here - G8DI mod
  int16_t ac, qh, acm;

  q_ac2 >>= att2; // digital gain control

  // Hilbert transform, BasicDSP model:  outi= fir(inl,  0, 0, 0, 0, 0,  0,  0,
  // 1,   0, 0,   0, 0,  0, 0, 0, 0); outq = fir(inr, 2, 0, 8, 0, 21, 0, 79, 0,
  // -79, 0, -21, 0, -8, 0, -2, 0) / 128;
  // Hilbert transform, 43dB side-band rejection in 650..3400Hz (@8kSPS)
  qh = ((v[0] - q_ac2) + (v[2] - v[12]) * 4) / 64 +
       ((v[4] - v[10]) + (v[6] - v[8])) / 8 +
       ((v[4] - v[10]) * 5 - (v[6] - v[8])) / 128 + (v[6] - v[8]) / 2;
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

  // inverting I and Q helps dampening a feedback
  // between PWM out and ADC inputs
  acm = -id - qh;
  ac = acm;
  // zero-beat auto-tune sampling
  if (autotune_active) autotune_sample(auto_nr(ac));
  // set S-Meter level
  static uint8_t absavg256cnt;
  if (!(absavg256cnt--)) {
    _absavg256 = absavg256;
    absavg256 = 0;
  }
  else absavg256 += abs(acm);
  // AGC
  #define FAST_AGC 1
  #define SLOW_AGC 2
  switch (agc) {
    case FAST_AGC: ac = process_agc_fast(ac); break;
    case SLOW_AGC: ac = process_agc(ac); break;
    default: break;
  }
  // volume control
  switch (volume) {
    case 16: break;
    case 15: ac = (ac >> 1) + (ac >> 2); break;
    case 14: ac = (ac >> 1); break;
    case 13: ac = (ac >> 2) + (ac >> 3); break;
    case 12: ac = (ac >> 2); break;
    case 11: ac = (ac >> 3) + (ac >> 4); break;
    case 10: ac = (ac >> 3); break;
    case  9: ac = (ac >> 4) + (ac >> 5); break;
    case  8: ac = (ac >> 4); break;
    case  7: ac = (ac >> 5) + (ac >> 6); break;
    case  6: ac = (ac >> 5); break;
    case  5: ac = (ac >> 6) + (ac >> 7); break;
    case  4: ac = (ac >> 6); break;
    case  3: ac = (ac >> 7) + (ac >> 8); break;
    case  2: ac = (ac >> 7); break;
    default: ac = 0; break;
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

static uint8_t tc = 0;

void process(int16_t i_ac2, int16_t q_ac2) {
  static int16_t ac3;

  static int16_t ozd1, ozd2; // Audio output stage
  if (_init) {
    ac3 = 0;
    ozd1 = 0;
    ozd2 = 0;
    _init = 0;
  }
  int16_t od1 = ac3 - ozd1; // Comb section
  ocomb = od1 - ozd2;

#define OUTLET 1
#ifdef OUTLET
  if (tc++ == 0) // prevent recursion : If tc == 0 enable interrupts
#endif

  interrupts();

  ozd2 = od1;
  ozd1 = ac3;

  ac3 = slow_dsp(i_ac2, q_ac2);

#ifdef OUTLET
  tc--;
#endif
}

static int16_t i_s0za1, i_s0zb0, i_s0zb1, i_s1za1, i_s1zb0, i_s1zb1;
static int16_t q_s0za1, q_s0zb0, q_s0zb1, q_s1za1, q_s1zb0, q_s1zb1, q_ac2;

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

// Get RX AC samples
inline int16_t sdr_rx_common_i() {
  ADMUX = admux[1];
  ADCSRA |= (1 << ADSC);
  int16_t adc = ADC - 511;
  static int16_t prev_adc;
  int16_t ac = (prev_adc + adc) >> 1;
  prev_adc = adc;
  if (_init) {
    ocomb = 0;
    ozi1 = 0;
    ozi2 = 0;
  }
  ozi2 = ozi1 + ozi2; // Integrator section
  ozi1 = ocomb + ozi1;
  OCR1AL = min(max((ozi2 >> 5) + 128, 0), 255);
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
  ADMUX |= ((ref1v1) ? (1 << REFS1) : 0) | (1 << REFS0);
  ADCSRA |= ((uint8_t)log2((uint8_t)(F_CPU / 13 / fs))) & 0x07;
  ADCSRA |= (1 << ADEN); // enable ADC
#ifdef ADC_NR
  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
#endif
}

void adc_stop() {
  ADCSRA &= ~(1 << ADIE);
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
#ifdef ADC_NR
  sleep_disable();
#endif
  ADMUX = (1 << REFS0); // restore reference voltage AREF (5V)
}

// performs classical analogRead with default Arduino
// sample-rate and analog reference setting; restores previous settings
uint16_t analogSafeRead(uint8_t pin, uint8_t ref1v1=false) {
  noInterrupts();
  // wait until ADC conversion is complete
  for (; !(ADCSRA & (1 << ADIF)););
  uint8_t adcsra = ADCSRA;
  uint8_t admux = ADMUX;
  ADCSRA &= ~(1 << ADIE); // disable interrupts when measurement complete
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
  if (ref1v1) ADMUX &= ~(1 << REFS0); // restore 1V reference voltage
  else ADMUX = (1 << REFS0);          // restore 5V reference voltage
  wait_us(1000);
  uint16_t val = analogRead(pin);
  ADCSRA = adcsra;
  ADMUX = admux;
  interrupts();
  return val;
}

uint16_t analogSampleMic() {
  uint16_t adc;
  noInterrupts();
  // hack: faster conversion rate necessary for VOX
  ADCSRA = (1<<ADEN) | (((uint8_t)log2((uint8_t)(F_CPU/13/(192307/1)))) & 0x07);
  if (vox_thresh >= 32)
    fastdigitalWrite(RX, LOW); // disable RF input
  uint8_t oldmux = ADMUX;
  // wait until ADC conversion is complete
  for (; !(ADCSRA & (1 << ADIF)););
  ADMUX = admux[2];      // set MUX for next conversion
  ADCSRA |= (1 << ADSC); // start next ADC conversion
  // wait until ADC conversion is complete
  for (; !(ADCSRA & (1 << ADIF)););
  ADMUX = oldmux;
  if (vox_thresh >= 32)
    fastdigitalWrite(RX, HIGH); // enable RF input
  adc = ADC;
  interrupts();
  return adc;
}

// Timer 1: OC1A and OC1B in PWM mode
void timer1_start(uint32_t fs) {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) |
            (1 << WGM11);
  TCCR1B |= (1 << CS10) | (1 << WGM13) |
            (1 << WGM12);
  ICR1H = 0x00;
  ICR1L = min(255, F_CPU / fs);
  OCR1AH = 0x00;
  OCR1AL = 0x00; // OC1A (SIDETONE)
  OCR1BH = 0x00;
  OCR1BL = 0x00; // OC1B (PWM_OUT)
}

void timer1_stop() {
  OCR1AL = 0x00;
  OCR1BL = 0x00;
}

// start timer 2
void timer2_start(uint32_t fs) {
  ASSR &= ~(1 << AS2);
  TCCR2A = 0;
  TCCR2B = 0;
  TCNT2 = 0;
  TCCR2A |= (1 << WGM21);  // WGM21: Mode 2
  TCCR2B |= (1 << CS22);   // 64 prescaler
  TIMSK2 |= (1 << OCIE2A); // enable timer compare
  OCR2A   = ((F_CPU / 64) / fs) - 1;
}

void timer2_stop() {        // Stop Timer 2 interrupt
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  delay(1);                 // wait for any interrupts
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

#define VFOA  0
#define VFOB  1

volatile bool change = true;
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
      uint8_t s = (dbm < -63) ? ((dbm - -127) / 6) : (((uint8_t)(dbm - -73)) / 10) * 10;
      lcd.setCursor(14,0);
      if (s < 10) {
        lcd.print('S');
      }
      lcd.print(s);
    }
    if (smode == 3) { // S-bar
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

// =======================================
// Zero-Beat Auto-Tune
// =======================================

#define AT_IDLE    0      // idle
#define AT_WAIT    1      // wait
#define AT_NHC     3      // negative half cycle
#define AT_PHC     4      // positive half cycle
#define AT_DONE    5      // measurement complete

#define AT_THRESH  120    // signal strength
#define AT_NSAMPL  256    // number of samples

uint8_t autotune_state = AT_IDLE;
uint8_t zxx_count = 0;    // zero-crossings
uint8_t first_zxx = 0;
uint8_t target_zxx = (cw_tone == 0) ? 39 : 46;
uint8_t autotune_pass = 0;

void autotune_print(char *msg) {
  lcd.noCursor();
  lcd.setCursor(11, 1);
  lcd.print(msg);
}

void autotune_start() {
  autotune_restart = false;
  autotune_pass = 1;
  first_zxx = 0;
  target_zxx = (cw_tone == 0) ? 39 : 46;
  autotune_state  = AT_IDLE;
  autotune_active = true;
  autotune_print("ZB");
}

// auto-tune sampling and zero-crossing detection
void autotune_sample(int16_t sample) {

  static uint16_t loopcount = 0;  // sample loop count
  static uint8_t  valid = 0;      // valid sample
  static uint8_t  nhc_count = 0;  // negative sample count
  static uint8_t  phc_count = 0;  // positive sample count

  if (abs(sample) > AT_THRESH) valid = 1;
  else valid = 0;

  switch (autotune_state) {
    case AT_IDLE:
      loopcount = 0;
      nhc_count = 0;
      phc_count = 0;
      zxx_count = 0;
      if (valid && (sample > 0)) {
        // found positive peak
        autotune_state = AT_WAIT;
      }
      break;
    case AT_WAIT:
      loopcount = 0;
      // wait for zero-cross to negative
      if (sample < 0) {
        nhc_count = 1;
        autotune_state = AT_NHC;
      }
      break;
    case AT_NHC:
      // was negative .. crosses to positive
      if ((sample > 0) && nhc_count>1) {
        zxx_count++;  // zero-crossing
        nhc_count = 0;
        autotune_state = AT_PHC;
      }
      if (sample < 0) nhc_count++;
      break;
    case AT_PHC:
      // was positive .. crosses to negative
      if ((sample < 0) && phc_count>1) {
        zxx_count++;  // zero-crossing
        phc_count = 0;
        autotune_state = AT_NHC;
      }
      if (sample > 0) phc_count++;
      break;
    default:
      break;
  }
  loopcount++;
  if (loopcount == AT_NSAMPL) {
    autotune_state = AT_DONE;
    autotune_active = false;
  }
}

void autotune_finish() {
  autotune_state = AT_IDLE;
  if (autotune_pass == 1) {
    // first pass complete
    first_zxx = zxx_count;
    autotune_pass = 2;
    autotune_active = true;
    return;
  }
  // second pass complete
  autotune_pass = 0;
  // check for two consistent measurements
  int16_t delta_zxx = (int16_t)zxx_count - (int16_t)first_zxx;
  uint8_t diff = (uint8_t)abs(delta_zxx);
  if (diff <= 1) {
    // calculate the average zero-crossing count
    uint8_t avg_zxx = (first_zxx + zxx_count) / 2;
    // each count represents ~15.26 Hz offset
    int16_t freq_error = (target_zxx - avg_zxx) * 15;
    // adjust VFO frequency toward zero-beat
    freq += freq_error;
    change = true;
    band_check();
    // update VFO frequency
    si5351.freq(freq + cw_offset, rx_ph_q, 0);
    display_vfo(freq);
    // check for large freq adjustment -- 15Hz per bin
    if (abs(target_zxx - avg_zxx) > 7) {
      // autotune_print("oo");
      // wdt_reset();
      // delay(ONE_SECOND);
      autotune_restart = true;
      return;
    }
    // display the tuning result
    if (freq_error > 0) autotune_print("++");
    else if (freq_error < 0) autotune_print("--");
    else autotune_print("OK");
    wdt_reset();
    delay(ONE_SECOND);
  } else {
    // autotune_print("xx");
    // wdt_reset();
    // delay(ONE_SECOND);
    autotune_restart = true;
    return;
  }
  // let the user read the results
  wdt_reset();
  show_banner();
}

// for sub-menus
#define N_PRESET   8
#define N_CWMSG    6

// for preset sub-menu
uint32_t preset_get(uint8_t i) {
  uint32_t eeval = eeprom_read_dword((const uint32_t *)(PRESET_EEPROM + 4 * i));
  uint32_t emode = eeval >> 24;
  uint32_t efreq = eeval & 0xFFFFFFUL;
  uint8_t  oob = 0;
  if (emode > CW)              {oob=1; emode = CW;}
  if (efreq < FREQ_LIMIT_LOW)  {oob=1; efreq = FREQ_LIMIT_LOW;}
  if (efreq > FREQ_LIMIT_HIGH) {oob=1; efreq = FREQ_LIMIT_HIGH;}
  if (oob) eeval = (emode << 24) | (efreq & 0xFFFFFFUL);
  return eeval;
}

// for preset sub-menu
void preset_print(uint32_t v) {
  uint32_t f = v & 0xFFFFFFUL;
  lcd.print(f / 1000000UL); lcd.print('.');
  uint16_t r = (f % 1000000UL) / 100;
  if (r < 1000) lcd.print('0');
  if (r < 100)  lcd.print('0');
  if (r < 10)   lcd.print('0');
  lcd.print(r); lcd.print(' ');
  lcd.print(mode_label[v >> 24]);
}

// for preset sub-menu
static void submenu_preset() {
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd.print(F("KNOB=go SEL=save"));
  lcd.setCursor(0, 1);
  lcd.print((vfosel) ? '\x07' : '\x06');
  lcd.print(picker_sel + 1);
  lcd.print(' ');
  uint32_t v = preset_get(picker_sel);
  uint8_t len = 3 + 11;
  if (v) {
    preset_print(v);
  } else {
    lcd.print(F("empty"));
    len = 3 + 5;
  }
  for (; len < 15; len++) lcd.print(' ');
}

// for cwmsg sub-menu
static void submenu_cwmsg() {
  lcd.noCursor();
  lcd.setCursor(0, 0);
  lcd_blanks(16);
  lcd.setCursor(0, 1);
  lcd.print(picker_sel + 1);
  lcd.print(F(": "));
  char *p = cw_msg[picker_sel];
  for (uint8_t i = 0; i < 12 && p[i]; i++)
    lcd.print(p[i]);
  lcd_blanks(10);
}

// common for all sub-menus
void submenu_draw() {
  switch (submenu_type) {
    case SUBMENU_PRESET:   submenu_preset();   break;
    case SUBMENU_CWMSG:    submenu_cwmsg();    break;
    default: break;
  }
}

// common for all sub-menus
void submenu_enter(uint8_t type) {
  submenu_type = type;
  picker_sel = 0;
  menumode = NOT_IN_MENU;
  submenu_draw();
}

// common for all sub-menus
void submenu_exit() {
  submenu_type = NOT_SUBMENU;
  show_banner();
  change = true;
}

// return from CW-message string edit
// if save==true writes EEPROM; if save==false the edit is discarded
static void return_to_picker(bool save) {
  if (menu >= CWMSG1 && menu <= CWMSG6) {
    uint8_t idx = menu - CWMSG1;
    if (save) {
      paramAction(SAVE, menu);
    } else {
      // discard edit and reload from EEPROM
      eeprom_read_block(cw_msg[idx],
      (void *)(CWMSG_EEPROM + idx * CW_MESSAGE_LENGTH), CW_MESSAGE_LENGTH);
    }
    picker_sel = idx;
    menu = CWMSGS;                 // so a later Menu press opens 1.9, not the message
    menumode = NOT_IN_MENU;
    submenu_type = SUBMENU_CWMSG;
    submenu_draw();
  }
}

static void preset_store() {
  eeprom_write_dword((uint32_t *)(PRESET_EEPROM + 4 * picker_sel),
                     (uint32_t)freq | ((uint32_t)mode << 24));
  lcd.setCursor(0, 1);
  lcd.print(F("Saved to "));
  lcd.print(picker_sel + 1);
  lcd_blanks(5);
  wdt_reset();
  delay(800);
  wdt_reset();
  submenu_draw();
}

static void picker_apply_preset() {
  uint32_t v = preset_get(picker_sel);
  if (!v) return;
  freq = v & 0xFFFFFFUL;
  mode = v >> 24;
  rit = 0;
  vfomode[vfosel % 2] = mode;
  paramAction(SAVE, (vfosel % 2) ? MODEB : MODEA);
  if (mode != CW) stepsize = STEP_1k;
  else stepsize = STEP_500;
  if (mode == CW) {
    filt = 4;
    nr = 0;
  } else {
    filt = 0;
  }
  si5351.iqmsa = 0; // enforce PLL reset
  submenu_exit();
}

// handle button events while a picker is open
// returns true if the event was consumed
bool picker_handle_event(uint8_t event) {
  if (submenu_type == NOT_SUBMENU) return false;
  // Menu (left) single-click always cancels
  if (event == (BL | SC)) {
    if (submenu_type == SUBMENU_CWMSG) {
      cw_msg_event = 0;   // cancel any scheduled repeating beacon
    }
    submenu_exit();
    return true;
  }

  switch (submenu_type) {
  case SUBMENU_PRESET:
    // Encoder = go (load), Select = save
    if (event == (BE | SC)) {
      picker_apply_preset();
    } else if (event == (BR | SC)) {
      preset_store();
    }
    break;

  case SUBMENU_CWMSG:
    // Encoder = edit, Select = send
    if (event == (BE | SC)) {
      submenu_type = NOT_SUBMENU;
      menu = CWMSG1 + picker_sel;
      menumode = SELECT_VALUE;
      paramAction(UPDATE_MENU, menu);
    } else if (event == (BR | SC)) {
      cw_msg_id = picker_sel;
      cw_tx_msg(cw_msg[picker_sel]);
    }
    break;

  default:
    break;
  }
  return true;
}

void picker_encoder(int8_t delta) {
  if (submenu_type == NOT_SUBMENU || delta == 0) return;
  uint8_t count = 0;
  switch (submenu_type) {
    case SUBMENU_PRESET:   count = N_PRESET;   break;
    case SUBMENU_CWMSG:    count = N_CWMSG;    break;
    default: return;
  }
  picker_sel = (((int16_t)picker_sel + delta) % count + count) % count;
  submenu_draw();
}

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
uint32_t semi_qsk_timeout = 0;

// display frequency
inline void display_vfo(int32_t freq) {
  if (submenu_type != NOT_SUBMENU) {
    submenu_draw();
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
  if (autotune_active) lcd.print(F("ZB"));
  else lcd.print(mode_label[mode]);
  lcd.print(' ');
  lcd.setCursor(15,1);
  lcd.print((vox) ? 'V' : 'R');
}

#define STRAIGHT  0      // Straight  key
#define IAMBIC_A  1      // Iambic A
#define IAMBIC_B  2      // Iambic B
#define ULTIMATIC 3      // Ultimatic mode

uint16_t ditTime;                // milliseconds per dit
uint8_t  keyer_speed = 25;       // WPM
uint8_t  keyer_mode  = STRAIGHT;
uint8_t  keyer_swap  = NO_SWAP;

// Set RX or TX mode, with RIT support and CW offset
void switch_rxtx(uint8_t tx_enable) {
  TIMSK2 &= ~(1 << OCIE2A); // disable timer compare interrupt
  delayMicroseconds(20); // wait until potential RX interrupt is finalized
  noInterrupts();
  tx = tx_enable;
  if (rit) display_vfo(freq);
  if (tx_enable) {
    // TX
    _centiGain = centiGain; // backup AGC setting
    semi_qsk_timeout = 0;
    switch (mode) {
      case CW: func_ptr = dsp_tx_cw; break;  // CW
      default: func_ptr = dsp_tx; break;     // SSB
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
  if ((cwdec) && (mode == CW) && !sending_cw_msg) {
    filteredstate = tx_enable;
    dec2();
  }

  if (tx_enable) {
    // TX
    if (practice) {
      fastdigitalWrite(RX, LOW); // TX (disable RX)
      if (!sending_cw_msg) {
        lcd.setCursor(15,1);
        lcd.print('P');
      }
      si5351.SendRegister(SI_CLK_OE, TX0RX0);
    } else {
      fastdigitalWrite(RX, LOW); // TX (disable RX)
      if (!sending_cw_msg) {
        lcd.setCursor(15,1);
        lcd.print('T');
      }
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
    // enable RX when not in semi-qsk phase and attenuator off
    if ((!semi_qsk_timeout) || (!semi_qsk)) {
      fastdigitalWrite(RX, !(att == 2));
    }

    delay(1);                   // decoder blanking fix
    sym = 1;                    // reset partial decoder symbol state
    filteredstate = LOW;        // force clean baseline state
    filteredstatebefore = LOW;

    si5351.freq_calc_fast(rit);
    si5351.SendPLLRegisterBulk(); // restore original PLL RX frequency
    if (!sending_cw_msg) {
      lcd.setCursor(15,1);
      lcd.print((vox) ? 'V' : 'R');
    }
  }
  OCR2A = ((F_CPU / 64) / ((tx_enable) ? F_SAMP_TX : F_SAMP_RX)) - 1;
  TIMSK2 |= (1 << OCIE2A); // enable timer compare interrupt TIMER2_COMPA_vect
}

static char cw_scroll_buf[14] = "            ";

// Transmit a CW character
uint8_t cw_tx(char ch) {
  char mch;
  for (uint8_t j = 0; (mch = pgm_read_byte_near(m2c + j)); j++) {
    if (mch == ch) {
      wdt_reset();
      // shift characters left in the scroll buffer
      for (uint8_t i = 0; i < 11; i++) {
        cw_scroll_buf[i] = cw_scroll_buf[i + 1];
      }
      cw_scroll_buf[11] = ch;
      cw_scroll_buf[12] = '\0';
      lcd.setCursor(3, 1);
      lcd.print(cw_scroll_buf);
      uint8_t k = 0x80;
      for (; !(j & k); k >>= 1);
      k >>= 1; // shift start of cw code to MSB
      if (k == 0) {
        delay(ditTime * 4); // space -> add word space
      } else {
        for (; k; k >>= 1) {
          switch_rxtx(1);    // key-on tx
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
      break;
    }
  }
  return 0;
}

// transmit a CW message string
uint8_t cw_tx_str(char *msg) {
  sending_cw_msg = true;
  // clear the scroll buffer
  for (uint8_t i=0; i<12; i++) cw_scroll_buf[i] = ' ';
  cw_scroll_buf[12] = '\0';
  lcd.setCursor(3, 1);
  lcd_blanks(12);
  uint8_t interrupted = 0;
  for (uint8_t i = 0; msg[i]; i++) {
    if (cw_tx(msg[i])) {
      interrupted = 1;
      break;
    }
  }
  sending_cw_msg = false;
  wdt_reset();
  delay(ONE_SECOND);
  if (submenu_type == SUBMENU_CWMSG) {
    submenu_draw();
  }
  return interrupted;
}

// Transmit an expanded CW message string
void cw_tx_msg(char *msg) {
  char expanded[CW_MESSAGE_LENGTH];
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
  // clear the repeating event timer
  cw_msg_event = 0;
  uint8_t interrupted = cw_tx_str(expanded);
  // clear the decoder output buffer
  for (uint8_t i = 0; i < 16; i++) cwbuf[i] = ' ';
  // only reschedule if CQ message and not interrupted
  if (!interrupted && cw_msg_interval && (cw_msg[cw_msg_id][0] == 'C') &&
     (cw_msg[cw_msg_id][1] == 'Q')) {
    cw_msg_event = millis() + (1000UL * cw_msg_interval);
    if (smart_cq_enabled) {
      smart_cq_start_listen();
    } else {
      smart_cq_stop();
    }
  } else {
    smart_cq_stop();
  }
}

void process_encoder_tuning_step(int8_t steps) {
  int32_t stepval = stepsizes[stepsize];
  if (rit) {
    rit += steps * stepval;
    rit = max(-99999, min(99999, rit));
  } else {
    freq += steps * stepval;
    freq = max(FREQ_LIMIT_LOW, min(FREQ_LIMIT_HIGH, freq));
  }
  change = true;
}

// clear the top line
void show_banner() {
  lcd.setCursor(0,0);
  lcd_blanks(16);
}

#define pgm_cache_item(addr, sz)                                               \
  byte _item[sz];                                                              \
  memcpy_P(_item, addr, sz); // copy array item from PROGMEM to SRAM

#define EEPROM_OFFSET  0x010

uint16_t eeprom_version;
uint16_t eeprom_addr;

// Support functions for parameter and menu handling

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
    if (menumode != MENU_NEXT) pos = 0;
    if (pos >= size) pos = 0;

    if (menumode == SELECT_VALUE)
      menumode = MENU_NEXT; // enter edit mode for strings
    // handle editing when current position is null-terminated
    if (value[pos] == 0) {
      if (encoder_val > 0) {
        value[pos] = _min;  // start from space (' ')
        encoder_val = 0;
      } else if (encoder_val < 0) {
        value[pos] = _max;  // wrap to 'Z'
        encoder_val = 0;
      }
    } else {
      int16_t newval = (int16_t)value[pos] + encoder_val;
      if (newval < _min)
        value[pos] = _min;
      else if (newval > _max)
        value[pos] = _max;
      else
        value[pos] = (char)newval;
      encoder_val = 0;
    }
    // for short strings (name/callsign), treat space as string terminator
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

int8_t paramAction(uint8_t action, uint8_t id) {
  if ((action == SAVE) || (action == LOAD)) {
    if (id == ALL) {
      eeprom_addr = EEPROM_OFFSET;
      for (uint8_t _id = 1; _id <= ALL_PARAMS; _id++) {
        paramAction(action, _id);
      }
      // include frequency presets
      for (uint8_t i = 0; i < 8; i++)
        paramAction(action, (uint8_t)(PRESET1 + i));
      // include CW messages
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
  // =========================================================================
  // MAIN (OPERATING) MENU (1.1 - 1.10)
  // =========================================================================
  case VOLUME:
    paramAction(action, volume, 0x11, F("VOLUME"), NULL, 1, 16, false);
    break;
  case MODE:
    paramAction(action, mode, 0x12, F("MODE"), mode_label, 0, _N(mode_label) - 1, false);
    break;
  case FILTER:
    paramAction(action, filt, 0x13, F("FILTER BW"), filt_label, 0, _N(filt_label) - 1, false);
    break;
  case RIT:
    paramAction(action, rit, 0x14, F("RIT"), offon_label, 0, 1, false);
    break;
  case KEY_WPM:
    paramAction(action, keyer_speed, 0x15, F("KEYER SPEED"), NULL, 1, 60, false);
    break;
  case KEY_MODE:
    paramAction(action, keyer_mode, 0x16, F("KEYER MODE"), keyer_label, 0, 3, false);
    break;
  case VOX:
    paramAction(action, vox, 0x17, F("VOX"), offon_label, 0, 1, false);
    break;
  case PRESET:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        submenu_enter(SUBMENU_PRESET);
      } else {
        printlabel(action, 0x18, F("FREQ PRESETS"));
        lcd_blanks(16);
      }
    } else {
      static uint8_t dummy_preset = 0;
      actionCommon(action, &dummy_preset, sizeof(dummy_preset));
    }
    break;
  case CWMSGS:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        submenu_enter(SUBMENU_CWMSG);
      } else {
        printlabel(action, 0x19, F("CW MESSAGES"));
        lcd_blanks(16);
      }
    } else {
      static uint8_t dummy_cw = 0;
      actionCommon(action, &dummy_cw, sizeof(dummy_cw));
    }
    break;
  case SETTINGS:
    if (action == UPDATE_MENU) {
      if (menumode >= SELECT_VALUE) {
        in_settings = true;
        menu = SETTINGS_MENU_START;
        menumode = SELECT_MENU;
        paramAction(UPDATE_MENU, menu);
      } else {
        printlabel(action, 0x1A, F("SETTINGS"));
        lcd_blanks(16);
      }
    } else {
      static uint8_t dummy_settings = 0;
      actionCommon(action, &dummy_settings, sizeof(dummy_settings));
    }
    break;

  // =========================================================================
  // SETTINGS SUBMENU (2.1 - 5.6)
  // =========================================================================
  case TONE_VOL:
    paramAction(action, tone_vol, 0x21, F("TONE VOLUME"), NULL, 1, 16, false);
    break;
  case CWDEC:
    paramAction(action, cwdec, 0x22, F("CW DECODER"), offon_label, 0, 1, false);
    break;
  case CWTONE:
    paramAction(action, cw_tone, 0x23, F("CW TONE"), cw_tone_label, 0, 1, false);
    break;
  case CWINTERVAL:
    paramAction(action, cw_msg_interval, 0x24, F("CW INTERVAL"), NULL, 0, 60, false);
    break;
  case SEMIQSK:
    paramAction(action, semi_qsk, 0x25, F("SEMI QSK"), offon_label, 0, 1, false);
    break;
  case KEY_SWAP:
    paramAction(action, keyer_swap, 0x26, F("KEYER SWAP"), offon_label, 0, 1, false);
    break;
  case SMART_CQ:
    paramAction(action, smart_cq_enabled, 0x27, F("SMART CQ"), offon_label, 0, 1, false);
    break;
  case VFOSEL:
    paramAction(action, vfosel, 0x31, F("VFO SELECT"), vfosel_label, 0, 1, false);
    break;
  case SMETER:
    paramAction(action, smode, 0x32, F("S-METER"), smeter_label, 0, _N(smeter_label) - 1, false);
    break;
  case AGC:
    paramAction(action, agc, 0x33, F("AGC"), agc_label, 0, _N(agc_label) - 1, false);
    break;
  case NR:
    paramAction(action, nr, 0x34, F("NR"), NULL, 0, 8, false);
    break;
  case ATT:
    paramAction(action, att, 0x35, F("ATT"), att_label, 0, 7, false);
    break;
  case ATT2:
    paramAction(action, att2, 0x36, F("ATT2"), NULL, 0, 16, false);
    break;
  case NOISE_GATE:
    paramAction(action, vox_thresh, 0x37, F("NOISE GATE"), NULL, 0, 255, false);
    break;
  case TX_DRIVE:
    paramAction(action, txdrive, 0x41, F("TX DRIVE"), NULL, 0, 8, false);
    break;
  case PWM_MIN:
    paramAction(action, pwm_min, 0x42, F("PA BIAS MIN"), NULL, 0, pwm_max - 1, false);
    break;
  case PWM_MAX:
    paramAction(action, pwm_max, 0x43, F("PA BIAS MAX"), NULL, pwm_min, 255, false);
    break;
  case VFO_XTAL:
    paramAction(action, si5351.fxtal, 0x44, F("REF FREQ"), NULL, 20000000, 28000000, false);
    break;
  case IQ_ADJ:
    paramAction(action, rx_ph_q, 0x45, F("IQ PHASE"), NULL, 0, 180, false);
    break;
  case QUAD_ACTIVE:
    paramAction(action, quad_enabled, 0x46, F("QUAD"), offon_label, 0, 1, false);
    break;
  case BAND:
    paramAction(action, bandsel, 0x51, F("PEBBLE BAND"), band_label, 0, 1, false);
    break;
  case ROTARY_SWAP:
    paramAction(action, rotxxx_swap, 0x52, F("ROTARY SWAP"), offon_label, 0, 1, false);
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
      paramAction(action, my_name, 0x53, F("USER NAME"), USERNAME_LENGTH);
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
      paramAction(action, my_call, 0x54, F("USER CALLSIGN"), CALLSIGN_LENGTH);
    }
    break;
  case PRACTICE:
    paramAction(action, practice, 0x55, F("PRACTICE"), offon_label, 0, 1, false);
    break;
  case FW_VER:
    if (action == UPDATE_MENU) {
      printlabel(action, 0x56, F("FIRMWARE VER"));
      lcd.print(F(VERSION));
      lcd_blanks(1);
      lcd.print(F(TIMESTAMP));
      lcd_blanks(10);
    } else {
      paramAction(action, eeprom_version, 0, NULL, NULL, 0, 0, false);
    }
    break;

  // =========================================================================
  //  EEPROM SUB-ITEMS (Presets & CW Messages)
  // =========================================================================
  case PRESET1: case PRESET2: case PRESET3: case PRESET4:
  case PRESET5: case PRESET6: case PRESET7: case PRESET8:
    if (action == SAVE) {
      uint8_t  idx  = id - PRESET1;
      uint16_t addr = PRESET_EEPROM + 4 * idx;
      uint32_t v = preset_get(idx);
      eeprom_write_dword((uint32_t *)addr, v);
    }
    break;

  case CWMSG1: case CWMSG2: case CWMSG3:
  case CWMSG4: case CWMSG5: case CWMSG6: {
    uint8_t idx = id - CWMSG1;
    uint16_t addr = CWMSG_EEPROM + idx * CW_MESSAGE_LENGTH;
    if (action == LOAD) {
      eeprom_read_block(cw_msg[idx], (void *)addr, CW_MESSAGE_LENGTH);
    } else if (action == SAVE) {
      for (uint8_t i = CW_MESSAGE_LENGTH; i > 0; i--) {
        if (cw_msg[idx][i-1] == ' ') cw_msg[idx][i-1] = 0;
        else break;
      }
      eeprom_write_block(cw_msg[idx], (void *)addr, CW_MESSAGE_LENGTH);
    } else if (action == SKIP) {
    } else {
      const __FlashStringHelper *lbl = F("EDIT CW MSG");
      paramAction(action, cw_msg[id-CWMSG1], 0x19, lbl, CW_MESSAGE_LENGTH);
    }
    break;
  }

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

  case MODE_FILT:
    actionCommon(action, mode_filt, sizeof(mode_filt));
    break;
  case MODE_NR:
    actionCommon(action, mode_nr, sizeof(mode_nr));
    break;
  case MODE_STEP:
    actionCommon(action, mode_step, sizeof(mode_step));
    break;

  // Non-parameters
  case _NULL:
    menumode = NOT_IN_MENU;
    in_settings = false;
    show_banner();
    change = true;
    break;

  default:
    if (action == NEXT_MENU) {
      uint8_t min_id = in_settings ? SETTINGS_MENU_START : MAIN_MENU_START;
      uint8_t max_id = in_settings ? SETTINGS_MENU_END   : MAIN_MENU_END;
      if (id > max_id) id = min_id;
      else if (id < min_id) id = max_id;
    }
    break;
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

// initialise the encoder state machine
void initEncoder() {
  enc_state = (digitalRead(ROT_B) << 1) | digitalRead(ROT_A);
  if (rotary_swap) enc_state = ((enc_state & 1) << 1) | (enc_state >> 1);
  enc_accum = 0;
  PCMSK2 |= (1 << PCINT22) | (1 << PCINT23);
  PCICR  |= (1 << PCIE2);
  interrupts();
}

#if defined(DIAG) || defined(DEBUG)
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
#endif

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

uint8_t keyerControl;
uint8_t keyerState;
uint8_t last_paddle = 0;  // 0=dit, 1=dah (used by Ultimatic mode)
uint8_t prev_dit_state = HIGH;
uint8_t prev_dah_state = HIGH;

uint32_t ktimer;
uint32_t key_start;
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

// Iambic keyer state machine
void iambic_keyer() {
  switch (keyerState) {
  case IDLE: // Wait for direct or latched paddle press
    if ((digitalRead(DAH) == LOW) || (digitalRead(DIT) == LOW) ||
        (keyerControl & 0x03)) {
      cw_msg_event = 0; // clear cw message event
      smart_cq_stop();
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
    key_start = millis();             // start of key-down interval
    keyerControl &= ~(DIT_L + DAH_L); // clear both paddle latch bits
    keyerState = KEYED;               // next state
    break;
  case KEYED: // Wait for timer to expire
    if ((millis() - key_start) >= ktimer) { // are we at end of key down ?
      key_state = LOW;
      switch_rxtx(key_state);
      ktimer = ditTime;            // inter-element duration
      key_start = millis();        // start of inter-element interval
      keyerState = INTER_ELEMENT;  // next state
    } else if (keyerControl & IAMBICB) {
      update_PaddleLatch(); // early paddle latch in Iambic B mode
    }
    break;
  case INTER_ELEMENT:
    // Insert time between dits/dahs
    update_PaddleLatch();                       // latch paddle state
    if ((millis() - key_start) >= ktimer) {     // are we at end of inter-space ?
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
  for (uint8_t i = 0; i != N_FONTS; i++) { // Init fonts
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
  wdt_reset();
  t0 = micros();
  for (uint16_t i = 0; i != 1000; i++)
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
  for (uint16_t i = 0; i != 1000; i++) {
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

  txdrive = 4;
  cw_offset = tones[cw_tone];

  // Load parameters from EEPROM, reset to factory defaults
  // when stored values are from a different version
  paramAction(LOAD, FW_VER);

  if ((eeprom_version != EEPROM_VERSION) || BUTTON_PRESSED) {
    eeprom_version = EEPROM_VERSION;
    paramAction(SAVE); // save default parameter values
    lcd.setCursor(0,1);
    lcd.print(F("FACTORY RESET"));
    lcd_blanks(5);
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
  if (vfomode[VFOA] > CW) vfomode[VFOA] = CW;
  if (vfomode[VFOB] > CW) vfomode[VFOB] = CW;
  mode = vfomode[vfosel];
  // init the mode-specific parameters
  filt = mode_filt[mode == CW];
  nr = mode_nr[mode == CW];
  stepsize = mode_step[mode == CW];
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
    lcd.setCursor(0,1);
    lcd.print(F("CHECK PTT !!"));
  }
}

#define LONG_PRESS 500

void loop() {
  static uint8_t event = 0;
  static uint8_t prev_menumode = NOT_IN_MENU;
  uint8_t changedMode = 0;
  int8_t _menumode = 0;
  int8_t encoder_change = 0;
  uint8_t offset = 0;
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
      delay(32);
    }
  }
  if (((mode == CW) && cwdec && (!tx && !semi_qsk_timeout)) ||
      ((submenu_type == SUBMENU_CWMSG) && cwdec && (!tx)))
    cw_decode();
  if ((menumode == NOT_IN_MENU) && (submenu_type == NOT_SUBMENU)) {
    if (cw_event) {
      const char offv[] = {0, 7, 3, 5, 3, 7, 8};
      offset = offv[smode];
      lcd.noCursor();
      cw_event = false;
      lcd.setCursor(0,0);
      lcd.print(cwbuf + offset);
      stepsize_showcursor();
    } else if ((!semi_qsk_timeout) && (!vox_tx))
      smeter();
  }
  if (mode == CW && keyer_mode) {
    iambic_keyer();  // call the iambic keyer
  } else {
    uint8_t pin = ((mode == CW) && keyer_swap) ? DAH : DIT;
    if (!vox_tx)
      // if VOX not active, then check DIT/DAH
      if (!digitalRead(pin)) {
        cw_msg_event = 0;        // clear cw message event
        smart_cq_stop();
        switch_rxtx(1);
        do {
          wdt_reset();
          delay((mode == CW) ? 10 : 100);
          if (BUTTON_PRESSED) break;
        } while (!digitalRead(pin)); // until released
        switch_rxtx(0);
      }
  }
  smart_cq_check();
  if ((semi_qsk_timeout) && ((int32_t)(millis() - semi_qsk_timeout) >= 0)) {
    switch_rxtx(0); // delayed QSK RX
  }
  // button-left, button-right, button-encoder
  // single-click, double-click, push-long, push-and-turn
  if (BUTTON_PRESSED) {
    // Left-/Right-/Rotary-button (while not already pressed)
    if (!((event & PL) ||
          (event &
           PLC))) { // hack: if there was long-push before, then fast forward
      uint16_t adcval = analogSafeRead(BUTTONS);
      event = SC;
      uint32_t t0 = millis();
      while (BUTTON_PRESSED) { // until released or long-press
        if ((millis() - t0) > LONG_PRESS) {
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
    if (picker_handle_event(event)) {
      event = 0; // picker consumed the event
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
      // While editing a CW message string:
      // Menu = cancel (discard) and return to the picker on the same slot.
      if (menumode >= SELECT_VALUE && (menu >= CWMSG1 && menu <= CWMSG6)) {
        return_to_picker(false);   // discard changes
        break;
      }
      if (menumode == NOT_IN_MENU) {
        _menumode = SELECT_MENU;
        in_settings = false;
        if (menu < MAIN_MENU_START || menu > MAIN_MENU_END)
          menu = MAIN_MENU_START;
      }
      else if (menumode == SELECT_MENU) {
        if (menu == FW_VER) {
          _menumode = SELECT_MENU;
        } else {
          _menumode = SELECT_VALUE;
        }
      }
      else if (menumode >= SELECT_VALUE) {
        paramAction(SAVE, menu);
        if (in_settings) {
          // If we were editing a setting value
          // pressing Menu saves and goes back to browsing Settings list
          _menumode = SELECT_MENU;
        } else {
          _menumode = NOT_IN_MENU;
        }
      }
      menumode = _menumode;
      break;
    case BL | DC:
      // does nothing
      break;
    case BR | SC: // Mode change - Button Right, Single Click
      if (menumode >= SELECT_VALUE && (menu >= CWMSG1 && menu <= CWMSG6)) {
        return_to_picker(true);    // save changes
        break;
      }
      // if we are browsing the Settings submenu
      // pressing Select (BR) exits back to 1.10 SETTINGS
      if (menumode == SELECT_MENU && in_settings) {
        in_settings = false;
        menu = SETTINGS;
        menumode = SELECT_MENU;
        paramAction(UPDATE_MENU, menu);
        break;
      }
      changedMode = true;
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
      stepsize = (rit) ? STEP_10 : mode_step[mode == CW];
      if (!rit) { // after RIT comes VFO A/B swap
        vfosel = !vfosel;
        freq = vfo[vfosel];
        mode = vfomode[vfosel];
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
      switch (menumode) {
        case NOT_IN_MENU: stepsize_change(+1);
          break;
        case SELECT_MENU:
          // Pressing encoder while browsing menu enters the selected item
          if (menu != FW_VER) {
            menumode = SELECT_VALUE;
          }
          break;
        case MENU_NEXT:
          _menumode = MENU_NEXT;
          paramAction(NEXT_CH, menu);
          menumode = _menumode;
          break;
        default: break;
      }
      break;
    case BE | DC:
      // zero-beat auto-tune on double-click
      if ((menumode == NOT_IN_MENU) && mode == CW && !(tx || vox_tx || rit))
        autotune_start();
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
      uint8_t prev_mode = mode;
      mode++;
      if (mode > CW)  mode = LSB;
      if (mode == CW) nr = 0;
      mode_step[prev_mode == CW] = stepsize;
      stepsize = mode_step[mode == CW];
      mode_filt[prev_mode == CW] = filt;
      filt = mode_filt[mode == CW];
      vfomode[vfosel] = mode;
      paramAction(SAVE, (vfosel) ? MODEB : MODEA); // save vfoa/b changes
      paramAction(SAVE, MODE);
      paramAction(SAVE, FILTER);
      si5351.iqmsa = 0; // enforce PLL reset
      if (mode < CW) show_banner();
      change = true;
    } else {
      if (menumode == SELECT_MENU) {
        if (in_settings) {
          // Right-click while browsing Settings
          // returns to Main menu at 1.10 Settings
          in_settings = false;
          menu = SETTINGS;
        } else {
          menumode = NOT_IN_MENU;
        }
      } else if (menumode >= SELECT_VALUE) {
        // Right-click while editing a value saves
        // and returns to the current menu list
        menumode = SELECT_MENU;
        change = true;
        paramAction(SAVE, menu);
      }
    }
  }
  // Show parameter and value
  if ((menumode) || (prev_menumode != menumode)) {
    encoder_change = encoder_val;
    if ((menumode == SELECT_MENU) && encoder_change) {
      uint8_t min_id = in_settings ? SETTINGS_MENU_START : MAIN_MENU_START;
      uint8_t max_id = in_settings ? SETTINGS_MENU_END   : MAIN_MENU_END;
      menu += encoder_val; // Navigate through active menu level
      if (menu > max_id)
        menu = min_id;
      else if (menu < min_id)
        menu = max_id;
      menu = paramAction(NEXT_MENU, menu);
      encoder_val = 0;
    }
    if (encoder_change || (prev_menumode != menumode))
      paramAction(UPDATE_MENU, (menumode) ? menu : 0);
    // force display refresh when exiting menu (required for PRESET picker)
    if ((prev_menumode != NOT_IN_MENU) && (menumode == NOT_IN_MENU)) {
      in_settings = false;
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

        if (menu == VFOSEL) {
          freq = vfo[vfosel];
          mode = vfomode[vfosel];
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
        if (menu == ATT) { // post-handling ATT parameter
          noInterrupts();
          adc_start(0, !(att & 0x01), F_ADC_CONV);
          admux[0] = ADMUX;
          adc_start(1, !(att & 0x01), F_ADC_CONV);
          admux[1] = ADMUX;
          interrupts();
          fastdigitalWrite(RX, !(att & 0x02));
          fastpinMode(AUDIO1, (att & 0x04) ? OUTPUT : INPUT);
          fastpinMode(AUDIO2, (att & 0x04) ? OUTPUT : INPUT);
        }
        if (menu == VFO_XTAL) {
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
    if (submenu_type != NOT_SUBMENU) {
      if (encoder_val) {
        picker_encoder(encoder_val);
        encoder_val = 0;
      }
      if (cw_event && submenu_type == SUBMENU_CWMSG) {
        cw_event = false;
        lcd.noCursor();
        lcd.setCursor(0, 0);
        lcd.print(cwbuf); // print the decoder buffer
      }
    } else {
      // hack for reversing rotary-encoder direction
      if (rotary_swap != rotxxx_swap) rotary_swap = rotxxx_swap;
      if (encoder_val) {
        process_encoder_tuning_step(encoder_val);
        encoder_val = 0;
      }
    }
  }
  if (change && !tx && !vox_tx) {
    change = false;
    vfo[vfosel] = freq;
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
  if (save_event_time && ((int32_t)(millis() - save_event_time) >= 0)) {
    paramAction(SAVE, (vfosel) ? FREQB : FREQA); // save vfoa/b changes
    save_event_time = 0;
  }
  // check for start of next CW repeating beacon
  if ((mode == CW) && (cw_msg_event) && ((int32_t)(millis() - cw_msg_event) >= 0)) {
    smart_cq_stop();
    cw_tx_msg(cw_msg[cw_msg_id]);
  }
  // check for zero-beat auto-tune
  if (autotune_state == AT_DONE) autotune_finish();
  // check for restart of zero-beat auto-tune
  if (autotune_restart) autotune_start();

  wdt_reset();
}


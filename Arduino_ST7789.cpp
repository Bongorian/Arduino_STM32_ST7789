/***************************************************
  This is a library for the ST7789 IPS SPI display.

  Originally written by Limor Fried/Ladyada for 
  Adafruit Industries.

  Modified by Ananev Ilia
 ****************************************************/

#include "Arduino_ST7789.h"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

static const uint8_t PROGMEM init_240x240[] = {
    9,                               // 9 commands in list:
    ST7789_SWRESET, ST_CMD_DELAY,    // 1: Software reset, no args, w/delay
    150,                             // 150 ms delay
    ST7789_SLPOUT, ST_CMD_DELAY,     // 2: Out of sleep mode, no args, w/delay
    255,                             // 255 = 500 ms delay
    ST7789_COLMOD, 1 + ST_CMD_DELAY, // 3: Set color mode, 1 arg + delay:
    0x55,                            // 16-bit color
    10,                              // 10 ms delay
    ST7789_MADCTL, 1,                // 4: Memory access ctrl (directions), 1 arg:
    0x00,                            // Row addr/col addr, bottom to top refresh
    ST7789_CASET, 4,                 // 5: Column addr set, 4 args, no delay:
    0x00, 0,                         // XSTART = 0
    0,
    240 & 0xFF,      // XEND = 240
    ST7789_RASET, 4, // 6: Row addr set, 4 args, no delay:
    0x00, 0,         // YSTART = 0
    0,
    240 & 0xFF,                 // YEND = 240
    ST7789_INVON, ST_CMD_DELAY, // 7: Inversion ON
    10,
    ST7789_NORON, ST_CMD_DELAY,  // 8: Normal display on, no args, w/delay
    10,                          // 10 ms delay
    ST7789_DISPON, ST_CMD_DELAY, // 9: Main screen turn on, no args, w/delay
    20};                         // 255 = 500 ms delay

static const uint8_t PROGMEM init_160x144[] = {
    9,                               // 9 commands in list:
    ST7789_SWRESET, ST_CMD_DELAY,    // 1: Software reset, no args, w/delay
    150,                             // 150 ms delay
    ST7789_SLPOUT, ST_CMD_DELAY,     // 2: Out of sleep mode, no args, w/delay
    255,                             // 255 = 500 ms delay
    ST7789_COLMOD, 1 + ST_CMD_DELAY, // 3: Set color mode, 1 arg + delay:
    0x55,                            // 16-bit color
    10,                              // 10 ms delay
    ST7789_MADCTL, 1,                // 4: Memory access ctrl (directions), 1 arg:
    0x00,                            // Row addr/col addr, bottom to top refresh
    ST7789_CASET, 4,                 // 5: Column addr set, 4 args, no delay:
    0x00, ST7789_240x240_XSTART,     // XSTART = 0
    (ST7789_TFTWIDTH + ST7789_240x240_XSTART) >> 8,
    (ST7789_TFTWIDTH + ST7789_240x240_XSTART) & 0xFF, // XEND = 240
    ST7789_RASET, 4,                                  // 6: Row addr set, 4 args, no delay:
    0x00, ST7789_240x240_YSTART,                      // YSTART = 0
    (ST7789_TFTHEIGHT + ST7789_240x240_YSTART) >> 8,
    (ST7789_TFTHEIGHT + ST7789_240x240_YSTART) & 0xFF, // YEND = 240
    ST7789_INVON, ST_CMD_DELAY,                        // 7: Inversion ON
    10,
    ST7789_NORON, ST_CMD_DELAY,  // 8: Normal display on, no args, w/delay
    10,                          // 10 ms delay
    ST7789_DISPON, ST_CMD_DELAY, // 9: Main screen turn on, no args, w/delay
    20};                         // 255 = 500 ms delay

inline uint16_t swapcolor(uint16_t x)
{
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

static SPISettings mySPISettings;

#define SPI_BEGIN_TRANSACTION() SPI.beginTransaction(mySPISettings)
#define SPI_END_TRANSACTION() SPI.endTransaction()

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
Arduino_ST7789::Arduino_ST7789(int8_t dc, int8_t rst, int8_t cs)
    : Adafruit_GFX(ST7789_TFTWIDTH, ST7789_TFTHEIGHT)
{
  _cs = cs;
  _dc = dc;
  _rst = rst;
  _hwSPI = true;
  _SPI9bit = false;
  _sid = _sclk = -1;
}

inline void Arduino_ST7789::spiwrite(uint8_t c)
{
  SPI.transfer(c);
}

inline void Arduino_ST7789::spiwrite16(uint16_t c)
{
  SPI.transfer16(c);
}

void Arduino_ST7789::writecommand(uint8_t c)
{
  DC_LOW();
  spiwrite(c);
}

void Arduino_ST7789::writedata(uint8_t c)
{
  DC_HIGH();
  spiwrite(c);
}

void Arduino_ST7789::writedata16(uint16_t c)
{
  DC_HIGH();
  spiwrite16(c);
}

// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void Arduino_ST7789::displayInit(const uint8_t *addr)
{

  uint8_t numCommands, numArgs;
  uint16_t ms;
  //<-----------------------------------------------------------------------------------------
  DC_HIGH();
  digitalWrite(_sclk, HIGH);
  //<-----------------------------------------------------------------------------------------

  numCommands = pgm_read_byte(addr++); // Number of commands to follow
  while (numCommands--)
  {                                      // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs = pgm_read_byte(addr++);     //   Number of args to follow
    ms = numArgs & ST_CMD_DELAY;         //   If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            //   Mask out delay bit
    while (numArgs--)
    {                                   //   For each argument...
      writedata(pgm_read_byte(addr++)); //     Read, issue argument
    }

    if (ms)
    {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      delay(ms);
    }
  }
}

// Initialization code common to all ST7789 displays
void Arduino_ST7789::commonInit(const uint8_t *cmdList)
{
  _ystart = _xstart = 0;
  _colstart = _rowstart = 0; // May be overridden in init func

  pinMode(_dc, OUTPUT);

  SPI.begin();
  mySPISettings = SPISettings(42000000, MSBFIRST, SPI_MODE2);
  SPI.beginTransaction(mySPISettings);

  if (_rst != -1)
  {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(50);
    digitalWrite(_rst, LOW);
    delay(50);
    digitalWrite(_rst, HIGH);
    delay(50);
  }

  if (cmdList)
    displayInit(cmdList);
}

void Arduino_ST7789::setRotation(uint8_t m)
{

  writecommand(ST7789_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation)
  {
  case 0:
    writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MY | ST7789_MADCTL_RGB);

    _xstart = _colstart;
    _ystart = _rowstart;
    break;
  case 1:
    writedata(ST7789_MADCTL_MY | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  case 2:
    writedata(ST7789_MADCTL_RGB);

    _xstart = _colstart;
    _ystart = _rowstart;
    break;

  case 3:
    writedata(ST7789_MADCTL_MX | ST7789_MADCTL_MV | ST7789_MADCTL_RGB);

    _ystart = _colstart;
    _xstart = _rowstart;
    break;
  }
}

inline void Arduino_ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{

  uint16_t x_start = x0 + _xstart, x_end = x1 + _xstart;
  uint16_t y_start = y0 + _ystart, y_end = y1 + _ystart;

  writecommand(ST7789_CASET); // Column addr set
  writedata16(x_start);       // XSTART
  writedata16(x_end);         // XEND

  writecommand(ST7789_RASET); // Row addr set
  writedata16(y_start);       // YSTART
  writedata16(y_end);         // YEND

  writecommand(ST7789_RAMWR); // write to RAM
}

void Arduino_ST7789::pushColor(uint16_t color)
{
  DC_HIGH();
  spiwrite16(color);
}

void Arduino_ST7789::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height))
    return;
  setAddrWindow(x, y, x + 1, y + 1);
  DC_HIGH();
  spiwrite16(color);
}

void Arduino_ST7789::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{

  // Rudimentary clipping
  if ((x >= _width) || (y >= _height))
    return;
  if ((y + h - 1) >= _height)
    h = _height - y;
  setAddrWindow(x, y, x, y + h - 1);

  DC_HIGH();

  while (h--)
  {
    spiwrite16(color);
  }
}

void Arduino_ST7789::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{

  // Rudimentary clipping
  if ((x >= _width) || (y >= _height))
    return;
  if ((x + w - 1) >= _width)
    w = _width - x;
  setAddrWindow(x, y, x + w - 1, y);

  DC_HIGH();

  while (w--)
  {
    spiwrite16(color);
  }
}

void Arduino_ST7789::fillScreen(uint16_t color)
{
  fillRect(0, 0, _width, _height, color);
}

// fill a rectangle
void Arduino_ST7789::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height))
    return;
  if ((x + w - 1) >= _width)
    w = _width - x;
  if ((y + h - 1) >= _height)
    h = _height - y;

  setAddrWindow(x, y, x + w - 1, y + h - 1);

  DC_HIGH();
  for (y = h; y > 0; y--)
  {
    for (x = w; x > 0; x--)
    {
      spiwrite16(color);
    }
  }
}

// ----------------------------------------------------------
// draws GB from RAM
void Arduino_ST7789::drawGB(int16_t x, int16_t y, int16_t w, int16_t h, unsigned char *img16)
{
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  DC_HIGH();
  for (int i = 0; i < 2880; i++)
  {
    if (img16[i] == 0xFF)
    {
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
      spiwrite16(Color565(0xFF, 0xFF, 0xFF));
    }
    else
    {
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
      spiwrite16(Color565(0, 0xFF, 0));
    }
  }
}

// ----------------------------------------------------------
// draws image from RAM
void Arduino_ST7789::drawImage(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *img16)
{
  if (x >= _width || y >= _height || w <= 0 || h <= 0)
    return;
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  DC_HIGH();

  uint32_t num = (uint32_t)w * h;
  uint16_t num16 = num >> 3;
  uint8_t *img = (uint8_t *)img16;
  while (num16--)
  {
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
  }
  uint8_t num8 = num & 0x7;
  while (num8--)
  {
    spiwrite(*(img + 1));
    spiwrite(*(img + 0));
    img += 2;
  }
}

// ----------------------------------------------------------
// draws image from flash (PROGMEM)
void Arduino_ST7789::drawImageF(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *img16)
{
  if (x >= _width || y >= _height || w <= 0 || h <= 0)
    return;
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  DC_HIGH();

  uint32_t num = (uint32_t)w * h;
  uint16_t num16 = num >> 3;
  uint8_t *img = (uint8_t *)img16;
  while (num16--)
  {
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
  }
  uint8_t num8 = num & 0x7;
  while (num8--)
  {
    spiwrite(pgm_read_byte(img + 1));
    spiwrite(pgm_read_byte(img + 0));
    img += 2;
  }
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t Arduino_ST7789::Color565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

void Arduino_ST7789::invertDisplay(boolean i)
{
  writecommand(i ? ST7789_INVON : ST7789_INVOFF);
}

/******** low level bit twiddling **********/

inline void Arduino_ST7789::CS_HIGH(void)
{
}

inline void Arduino_ST7789::CS_LOW(void)
{
}

inline void Arduino_ST7789::DC_HIGH(void)
{
  _DCbit = true;
  digitalWrite(_dc, HIGH);
}

inline void Arduino_ST7789::DC_LOW(void)
{
  _DCbit = false;
  digitalWrite(_dc, LOW);
}

void Arduino_ST7789::init(uint16_t width, uint16_t height)
{
  commonInit(NULL);
  _colstart = ST7789_240x240_XSTART;
  _rowstart = ST7789_240x240_YSTART;
  _height = 240;
  _width = 340;
  displayInit(init_240x240);
  fillScreen(BLACK);

  _colstart = ST7789_160x144_XSTART;
  _rowstart = ST7789_160x144_YSTART;
  _height = 144;
  _width = 160;

  displayInit(init_160x144);

  setRotation(2);
}

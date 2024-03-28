#include "Adafruit_RA8875.h"

Adafruit_RA8875::Adafruit_RA8875() {}

/**************************************************************************/
/*!
            Constructor for a new RA8875 instance

            @param CS    Location of the SPI chip select pin
            @param RST Location of the reset pin
*/
/**************************************************************************/
Adafruit_RA8875::Adafruit_RA8875(uint8_t RST, SPI_HandleTypeDef DIS_HSPI, DataManager *dm) {
  _DIS_HSPI = DIS_HSPI;
  _rst = RST;
  _dm = dm;
}

void Adafruit_RA8875::drawMainScreen() {
    fillScreen(RA8875_WHITE);

    // determining status color
    const int MAX_PRESSURE = 2000;
    const int MAX_HEIGHT = 350;
    const int HEIGHT_BLOCK = MAX_HEIGHT / 10;
    const int TANK_X = 50;
    const int TANK_Y = 100;

    uint16_t pressure = _dm->getPressure_PSI();
    uint16_t color;
    uint16_t height = (pressure/200) * HEIGHT_BLOCK;

    printf("height = %d\n", height);

    if(pressure >= 1600) {
        color = COLOR_GREEN;
    }
    else if(pressure >= 1200) {
        color = COLOR_YELLOW_GREEN;
    }
    else if(pressure >= 800) {
        color = COLOR_YELLOW;
    }
    else if(pressure >= 400) {
        color = COLOR_ORANGE;
    }
    else {
        color = RA8875_RED;
    }

    const int RECT_WIDTH = 100;

    if (height) {
        fillRoundRect(TANK_X,TANK_Y+MAX_HEIGHT-height,RECT_WIDTH,height,10,color);
    }
    drawRoundRect(TANK_X,TANK_Y,RECT_WIDTH,MAX_HEIGHT,10,RA8875_BLACK);

    // Buttons on right
    const int BUTTON_X = 550;
    const int BUTTON_WIDTH = 225;
    const int BUTTON_HEGIHT = 90;
    const int BUTTON1_Y = 25;
    const int BUTTON2_Y = 141;
    const int BUTTON3_Y = 257;
    const int BUTTON4_Y = 373;
    const int TEXT_X_OFFSET = 10;
    const int TEXT_Y_OFFSET = 15;

    fillRoundRect(BUTTON_X,BUTTON1_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,COLOR_CYAN);
    fillRoundRect(BUTTON_X,BUTTON2_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,COLOR_CYAN);
    fillRoundRect(BUTTON_X,BUTTON3_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,COLOR_CYAN);
    fillRoundRect(BUTTON_X,BUTTON4_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,COLOR_CYAN);

    drawRoundRect(BUTTON_X,BUTTON1_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10 ,RA8875_BLACK);
    drawRoundRect(BUTTON_X,BUTTON2_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,RA8875_BLACK);
    drawRoundRect(BUTTON_X,BUTTON3_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,RA8875_BLACK);
    drawRoundRect(BUTTON_X,BUTTON4_Y,BUTTON_WIDTH,BUTTON_HEGIHT,10,RA8875_BLACK);

    // drawing top menu bar
    fillRoundRect(-10,-10,525,90,10,COLOR_NOAH);
    drawRoundRect(-10,-10,525,90,10,RA8875_BLACK);




    textColor(RA8875_BLACK,RA8875_RED);

    /* Switch to text mode */
    textMode();
    textTransparent(RA8875_BLACK);
    textEnlarge(10);
    // cursorBlink(32);

    /* Set a solid for + bg color ... */

    /* ... or a fore color plus a transparent background */


    /* Set the cursor location (in pixels) */
    const int CURSOR_X = 200;

    /* Render some text! */
    char bla1[32];   // Use an array which is large enough
    char bla2[32];   // Use an array which is large enough
    char bla3[32];   // Use an array which is large enough
    snprintf(bla3, sizeof(bla3), "%d Minutes", _dm->getTimeRemaining_Minutes());
    snprintf(bla1, sizeof(bla1), "%d PSI", _dm->getPressure_PSI());
    snprintf(bla2, sizeof(bla2), "%d LPM", _dm->getFlow_LPM());

    textSetCursor(CURSOR_X, TANK_Y+25);
    textWrite(bla3);
    textSetCursor(CURSOR_X, TANK_Y+125);
    textWrite(bla1);
    textSetCursor(CURSOR_X, TANK_Y+225);
    textWrite(bla2);

    char btn1Text[15] = "Settings";
    textEnlarge(2);
    textSetCursor(BUTTON_X+TEXT_X_OFFSET, BUTTON1_Y+TEXT_Y_OFFSET);
    textWrite(btn1Text);

    char btn2Text[15] = "Charting";
    textEnlarge(2);
    textSetCursor(BUTTON_X+TEXT_X_OFFSET, BUTTON2_Y+TEXT_Y_OFFSET);
    textWrite(btn2Text);

    char btn3Text[15] = "Alarm";
    textEnlarge(2);
    textSetCursor(BUTTON_X+TEXT_X_OFFSET, BUTTON3_Y+TEXT_Y_OFFSET);
    textWrite(btn3Text);

    char btn4Text[15] = "Threshold";
    textEnlarge(2);
    textSetCursor(BUTTON_X+TEXT_X_OFFSET, BUTTON4_Y+TEXT_Y_OFFSET);
    textWrite(btn4Text);

    char btn5Text[30] = "B? | DT | S | B1 B2";
    textEnlarge(2);
    textSetCursor(10,10);
    textWrite(btn5Text);

    graphicsMode();
}

/**************************************************************************/
/*!
            Initialises the LCD driver and any HW required by the display

            @param s The display size, which can be either:
                                    'RA8875_480x80'    (3.8" displays) or
                                    'RA8875_480x128' (3.9" displays) or
                                    'RA8875_480x272' (4.3" displays) or
                                    'RA8875_800x480' (5" and 7" displays)

            @return True if we reached the end
//*/
///**************************************************************************/
bool Adafruit_RA8875::begin() {
    _size = RA8875_800x480;
    _width = 800;
    _height = 480;
    _rotation = 0;

    // TODO: init pins manually
      HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);

      // TODO: figure out if needed
//    pinMode(_rst, OUTPUT);
//    digitalWrite(_rst, LOW);
//    delay(100);
//    digitalWrite(_rst, HIGH);
//    delay(100);

//
//    SPI.begin();

    uint8_t x = Adafruit_RA8875::readReg(0);
    printf("x = %d\r\n", x);
    if (x != 0x75) {
		printf("DISPLAY READ REGISTER FAILED!!!! \r\n");
        return false;
    }

    Adafruit_RA8875::initialize();
    return true;
}

/************************* Initialization *********************************/

/**************************************************************************/
/*!
            Performs a SW-based reset of the RA8875
*/
/**************************************************************************/
void Adafruit_RA8875::softReset(void) {
    Adafruit_RA8875::writeCommand(RA8875_PWRR);
    Adafruit_RA8875::writeData(RA8875_PWRR_SOFTRESET);
    Adafruit_RA8875::writeData(RA8875_PWRR_NORMAL);
    HAL_Delay(1);
}

/**************************************************************************/
/*!
            Initialise the PLL
*/
/**************************************************************************/
void Adafruit_RA8875::PLLinit(void) {
    // _size == RA8875_800x480
    Adafruit_RA8875::writeReg(RA8875_PLLC1, RA8875_PLLC1_PLLDIV1 + 11);
    HAL_Delay(1);
    Adafruit_RA8875::writeReg(RA8875_PLLC2, RA8875_PLLC2_DIV4);
    HAL_Delay(1);
}

/**************************************************************************/
/*!
            Initialises the driver IC (clock setup, etc.)
*/
/**************************************************************************/
void Adafruit_RA8875::initialize(void) {
    Adafruit_RA8875::PLLinit();
    Adafruit_RA8875::writeReg(RA8875_SYSR, RA8875_SYSR_16BPP | RA8875_SYSR_MCU8);

    /* Timing values */
    uint8_t pixclk;
    uint8_t hsync_start;
    uint8_t hsync_pw;
    uint8_t hsync_finetune;
    uint8_t hsync_nondisp;
    uint8_t vsync_pw;
    uint16_t vsync_nondisp;
    uint16_t vsync_start;

    // (_size == RA8875_800x480)
    pixclk = RA8875_PCSR_PDATL | RA8875_PCSR_2CLK;
    hsync_nondisp = 26;
    hsync_start = 32;
    hsync_pw = 96;
    hsync_finetune = 0;
    vsync_nondisp = 32;
    vsync_start = 23;
    vsync_pw = 2;
    _voffset = 0;

    Adafruit_RA8875::writeReg(RA8875_PCSR, pixclk);
    HAL_Delay(1);

    /* Horizontal settings registers */
    Adafruit_RA8875::writeReg(RA8875_HDWR, (_width / 8) - 1); // H width: (HDWR + 1) * 8 = 480
    Adafruit_RA8875::writeReg(RA8875_HNDFTR, RA8875_HNDFTR_DE_HIGH + hsync_finetune);
    Adafruit_RA8875::writeReg(RA8875_HNDR, (hsync_nondisp - hsync_finetune - 2) /
                                                        8); // H non-display: HNDR * 8 + HNDFTR + 2 = 10
    Adafruit_RA8875::writeReg(RA8875_HSTR, hsync_start / 8 - 1); // Hsync start: (HSTR + 1)*8
    Adafruit_RA8875::writeReg(RA8875_HPWR,
                     RA8875_HPWR_LOW +
                             (hsync_pw / 8 - 1)); // HSync pulse width = (HPWR+1) * 8

    /* Vertical settings registers */
    Adafruit_RA8875::writeReg(RA8875_VDHR0, (uint16_t)(_height - 1 + _voffset) & 0xFF);
    Adafruit_RA8875::writeReg(RA8875_VDHR1, (uint16_t)(_height - 1 + _voffset) >> 8);
    Adafruit_RA8875::writeReg(RA8875_VNDR0, vsync_nondisp - 1); // V non-display period = VNDR + 1
    Adafruit_RA8875::writeReg(RA8875_VNDR1, vsync_nondisp >> 8);
    Adafruit_RA8875::writeReg(RA8875_VSTR0, vsync_start - 1); // Vsync start position = VSTR + 1
    Adafruit_RA8875::writeReg(RA8875_VSTR1, vsync_start >> 8);
    Adafruit_RA8875::writeReg(RA8875_VPWR,
                     RA8875_VPWR_LOW + vsync_pw - 1); // Vsync pulse width = VPWR + 1

    /* Set active window X */
    Adafruit_RA8875::writeReg(RA8875_HSAW0, 0); // horizontal start point
    Adafruit_RA8875::writeReg(RA8875_HSAW1, 0);
    Adafruit_RA8875::writeReg(RA8875_HEAW0, (uint16_t)(_width - 1) & 0xFF); // horizontal end point
    Adafruit_RA8875::writeReg(RA8875_HEAW1, (uint16_t)(_width - 1) >> 8);

    /* Set active window Y */
    Adafruit_RA8875::writeReg(RA8875_VSAW0, 0 + _voffset); // vertical start point
    Adafruit_RA8875::writeReg(RA8875_VSAW1, 0 + _voffset);
    Adafruit_RA8875::writeReg(RA8875_VEAW0,
                     (uint16_t)(_height - 1 + _voffset) & 0xFF); // vertical end point
    Adafruit_RA8875::writeReg(RA8875_VEAW1, (uint16_t)(_height - 1 + _voffset) >> 8);

    /* Clear the entire window */
    Adafruit_RA8875::writeReg(RA8875_MCLR, RA8875_MCLR_START | RA8875_MCLR_FULL);
    HAL_Delay(500);
}

/**************************************************************************/
/*!
            Returns the display width in pixels

            @return    The 1-based display width in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875::width(void) { return _width; }

/**************************************************************************/
/*!
            Returns the display height in pixels

            @return    The 1-based display height in pixels
*/
/**************************************************************************/
uint16_t Adafruit_RA8875::height(void) { return _height; }

/**************************************************************************/
/*!
 Returns the current rotation (0-3)

 @return    The Rotation Setting
 */
/**************************************************************************/
int8_t Adafruit_RA8875::getRotation(void) { return _rotation; }

/**************************************************************************/
/*!
 Sets the current rotation (0-3)

 @param rotation The Rotation Setting
 */
/**************************************************************************/
void Adafruit_RA8875::setRotation(int8_t rotation) {
    switch (rotation) {
    case 2:
        _rotation = rotation;
        break;
    default:
        _rotation = 0;
        break;
    }
}

/************************* Text Mode ***********************************/

/**************************************************************************/
/*!
            Sets the display in text mode (as opposed to graphics mode)
*/
/**************************************************************************/
void Adafruit_RA8875::textMode(void) {
    /* Set text mode */
    Adafruit_RA8875::writeCommand(RA8875_MWCR0);
    uint8_t temp = readData();
    temp |= RA8875_MWCR0_TXTMODE; // Set bit 7
    Adafruit_RA8875::writeData(temp);

    /* Select the internal (ROM) font */
    Adafruit_RA8875::writeCommand(0x21);
    temp = Adafruit_RA8875::readData();
    temp &= ~((1 << 7) | (1 << 5)); // Clear bits 7 and 5
    Adafruit_RA8875::writeData(temp);
}

/**************************************************************************/
/*!
            Sets the display in text mode (as opposed to graphics mode)

            @param x The x position of the cursor (in pixels, 0..1023)
            @param y The y position of the cursor (in pixels, 0..511)
*/
/**************************************************************************/
void Adafruit_RA8875::textSetCursor(uint16_t x, uint16_t y) {
    x = Adafruit_RA8875::applyRotationX(x);
    y = Adafruit_RA8875::applyRotationY(y);

    /* Set cursor location */
    Adafruit_RA8875::writeCommand(0x2A);
    Adafruit_RA8875::writeData(x & 0xFF);
    Adafruit_RA8875::writeCommand(0x2B);
    Adafruit_RA8875::writeData(x >> 8);
    Adafruit_RA8875::writeCommand(0x2C);
    Adafruit_RA8875::writeData(y & 0xFF);
    Adafruit_RA8875::writeCommand(0x2D);
    Adafruit_RA8875::writeData(y >> 8);
}

/**************************************************************************/
/*!
            Sets the fore and background color when rendering text

            @param foreColor The RGB565 color to use when rendering the text
            @param bgColor     The RGB565 colot to use for the background
*/
/**************************************************************************/
void Adafruit_RA8875::textColor(uint16_t foreColor, uint16_t bgColor) {
    /* Set Fore Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((foreColor & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((foreColor & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((foreColor & 0x001f));

    /* Set Background Color */
    Adafruit_RA8875::writeCommand(0x60);
    Adafruit_RA8875::writeData((bgColor & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x61);
    Adafruit_RA8875::writeData((bgColor & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x62);
    Adafruit_RA8875::writeData((bgColor & 0x001f));

    /* Clear transparency flag */
    Adafruit_RA8875::writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875::readData();
    temp &= ~(1 << 6); // Clear bit 6
    Adafruit_RA8875::writeData(temp);
}

/**************************************************************************/
/*!
            Sets the fore color when rendering text with a transparent bg

            @param foreColor The RGB565 color to use when rendering the text
*/
/**************************************************************************/
void Adafruit_RA8875::textTransparent(uint16_t foreColor) {
    /* Set Fore Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((foreColor & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((foreColor & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((foreColor & 0x001f));

    /* Set transparency flag */
    Adafruit_RA8875::writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875::readData();
    temp |= (1 << 6); // Set bit 6
    Adafruit_RA8875::writeData(temp);
}

/**************************************************************************/
/*!
            Sets the text enlarge settings, using one of the following values:

            0 = 1x zoom
            1 = 2x zoom
            2 = 3x zoom
            3 = 4x zoom

            @param scale     The zoom factor (0..3 for 1-4x zoom)
*/
/**************************************************************************/
void Adafruit_RA8875::textEnlarge(uint8_t scale) {
    if (scale > 3)
        scale = 3; // highest setting is 3

    /* Set font size flags */
    Adafruit_RA8875::writeCommand(0x22);
    uint8_t temp = Adafruit_RA8875::readData();
    temp &= ~(0xF); // Clears bits 0..3
    temp |= scale << 2;
    temp |= scale;

    Adafruit_RA8875::writeData(temp);

    _textScale = scale;
}

/**************************************************************************/
/*!
         Enable Cursor Visibility and Blink
         Here we set bits 6 and 5 in 40h
         As well as the set the blink rate in 44h
         The rate is 0 through max 255
         the lower the number the faster it blinks (00h is 1 frame time,
         FFh is 256 Frames time.
         Blink Time (sec) = BTCR[44h]x(1/Frame_rate)

         @param rate The frame rate to blink
 */
/**************************************************************************/

void Adafruit_RA8875::cursorBlink(uint8_t rate) {

    Adafruit_RA8875::writeCommand(RA8875_MWCR0);
    uint8_t temp = Adafruit_RA8875::readData();
    temp |= RA8875_MWCR0_CURSOR;
    Adafruit_RA8875::writeData(temp);

    Adafruit_RA8875::writeCommand(RA8875_MWCR0);
    temp = Adafruit_RA8875::readData();
    temp |= RA8875_MWCR0_BLINK;
    Adafruit_RA8875::writeData(temp);

    if (rate > 255)
        rate = 255;
    Adafruit_RA8875::writeCommand(RA8875_BTCR);
    Adafruit_RA8875::writeData(rate);
}

/**************************************************************************/
/*!
            Renders some text on the screen when in text mode

            @param buffer        The buffer containing the characters to render
            @param len             The size of the buffer in bytes
*/
/**************************************************************************/
void Adafruit_RA8875::textWrite(const char *buffer, uint16_t len) {
    if (len == 0)
        len = strlen(buffer);
    Adafruit_RA8875::writeCommand(RA8875_MRWC);
    for (uint16_t i = 0; i < len; i++) {
        Adafruit_RA8875::writeData(buffer[i]);
/// @cond DISABLE
#if defined(__arm__)
        /// @endcond
        // This delay is needed with textEnlarge(1) because
        // Teensy 3.X is much faster than Arduino Uno
        if (_textScale > 0)
            HAL_Delay(1);
/// @cond DISABLE
#else
        /// @endcond
        // For others, delay starting with textEnlarge(2)
        if (_textScale > 1)
            HAL_Delay(1);
/// @cond DISABLE
#endif
        /// @endcond
    }
}

/************************* Graphics ***********************************/

/**************************************************************************/
/*!
            Sets the display in graphics mode (as opposed to text mode)
*/
/**************************************************************************/
void Adafruit_RA8875::graphicsMode(void) {
    Adafruit_RA8875::writeCommand(RA8875_MWCR0);
    uint8_t temp = Adafruit_RA8875::readData();
    temp &= ~RA8875_MWCR0_TXTMODE; // bit #7
    Adafruit_RA8875::writeData(temp);
}

/**************************************************************************/
/*!
            Waits for screen to finish by polling the status!

            @param regname The register name to check
            @param waitflag The value to wait for the status register to match

            @return True if the expected status has been reached
*/
/**************************************************************************/
bool Adafruit_RA8875::waitPoll(uint8_t regname, uint8_t waitflag) {
    /* Wait for the command to finish */
    while (1) {
        uint8_t temp = Adafruit_RA8875::readReg(regname);
        if (!(temp & waitflag))
            return true;
    }
    return false; // MEMEFIX: yeah i know, unreached! - add timeout?
}

/**************************************************************************/
/*!
            Sets the current X/Y position on the display before drawing

            @param x The 0-based x location
            @param y The 0-base y location
*/
/**************************************************************************/
void Adafruit_RA8875::setXY(uint16_t x, uint16_t y) {
    Adafruit_RA8875::writeReg(RA8875_CURH0, x);
    Adafruit_RA8875::writeReg(RA8875_CURH1, x >> 8);
    Adafruit_RA8875::writeReg(RA8875_CURV0, y);
    Adafruit_RA8875::writeReg(RA8875_CURV1, y >> 8);
}

/**************************************************************************/
/*!
            HW accelerated function to push a chunk of raw pixel data

            @param num The number of pixels to push
            @param p     The pixel color to use
*/
/**************************************************************************/
void Adafruit_RA8875::pushPixels(uint32_t num, uint16_t p) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);

    while (num--) {
//        SPI.transfer(p >> 8);
    	uint8_t pHigh = p >> 8;
    	HAL_SPI_Transmit(&_DIS_HSPI, &pHigh, 1, HAL_MAX_DELAY);
//    	SPI.transfer(p);
    	uint8_t pLow = p&0xFF;
    	HAL_SPI_Transmit(&_DIS_HSPI, &pLow, 1, HAL_MAX_DELAY);
    }
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
        Fill the screen with the current color
*/
/**************************************************************************/

//NOTE: no function overloading in C
//void Adafruit_RA8875::fillRect(void) {
//    Adafruit_RA8875::writeCommand(RA8875_DCR);
//    Adafruit_RA8875::writeData(RA8875_DCR_LINESQUTRI_STOP | RA8875_DCR_DRAWSQUARE);
//    Adafruit_RA8875::writeData(RA8875_DCR_LINESQUTRI_START | RA8875_DCR_FILL |
//                        RA8875_DCR_DRAWSQUARE);
//}

/**************************************************************************/
/*!
        Apply current rotation in the X direction

        @return the X value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875::applyRotationX(int16_t x) {
    switch (_rotation) {
    case 2:
        x = _width - 1 - x;
        break;
    }

    return x;
}

/**************************************************************************/
/*!
        Apply current rotation in the Y direction

        @return the Y value with current rotation applied
 */
/**************************************************************************/
int16_t Adafruit_RA8875::applyRotationY(int16_t y) {
    switch (_rotation) {
    case 2:
        y = _height - 1 - y;
        break;
    }

    return y + _voffset;
}

/**************************************************************************/
/*!
            Draws a single pixel at the specified location

            @param x         The 0-based x location
            @param y         The 0-base y location
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawPixel(int16_t x, int16_t y, uint16_t color) {
    x = Adafruit_RA8875::applyRotationX(x);
    y = Adafruit_RA8875::applyRotationY(y);

    Adafruit_RA8875::writeReg(RA8875_CURH0, x);
    Adafruit_RA8875::writeReg(RA8875_CURH1, x >> 8);
    Adafruit_RA8875::writeReg(RA8875_CURV0, y);
    Adafruit_RA8875::writeReg(RA8875_CURV1, y >> 8);
    Adafruit_RA8875::writeCommand(RA8875_MRWC);

    // TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);
//    SPI.transfer(color >> 8);
	uint8_t pHigh = color >> 8;
	HAL_SPI_Transmit(&_DIS_HSPI, &pHigh, 1, HAL_MAX_DELAY);
//    SPI.transfer(color);
	uint8_t pLow = color&0xFF;
	HAL_SPI_Transmit(&_DIS_HSPI, &pLow, 1, HAL_MAX_DELAY);
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
 Draws a series of pixels at the specified location without the overhead

 @param p         An array of RGB565 color pixels
 @param num     The number of the pixels to draw
 @param x         The 0-based x location
 @param y         The 0-base y location
 */
/**************************************************************************/
void Adafruit_RA8875::drawPixels(uint16_t *p, uint32_t num, int16_t x,
                                                                 int16_t y) {
	Adafruit_RA8875::writeReg(RA8875_CURH0, x);
	Adafruit_RA8875::writeReg(RA8875_CURH1, x >> 8);
	Adafruit_RA8875::writeReg(RA8875_CURV0, y);
	Adafruit_RA8875::writeReg(RA8875_CURV1, y >> 8);
	Adafruit_RA8875::writeCommand(RA8875_MRWC);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);
	    while (num--) {
	    	uint8_t p1 = *p >> 8;
	    	uint8_t p2 = *p & 0xFF;
	    	HAL_SPI_Transmit(&_DIS_HSPI, &p1, 1, HAL_MAX_DELAY);
//	        SPI.transfer(*p >> 8);
//	        SPI.transfer(*p & 0xFF);
	        HAL_SPI_Transmit(&_DIS_HSPI, &p2, 1, HAL_MAX_DELAY);
	        p++;
	    }
	    HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
            Draws a HW accelerated line on the display

            @param x0        The 0-based starting x location
            @param y0        The 0-base starting y location
            @param x1        The 0-based ending x location
            @param y1        The 0-base ending y location
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                                                             uint16_t color) {
    x0 = Adafruit_RA8875::applyRotationX(x0);
    y0 = Adafruit_RA8875::applyRotationY(y0);
    x1 = Adafruit_RA8875::applyRotationX(x1);
    y1 = Adafruit_RA8875::applyRotationY(y1);

    /* Set X */
    Adafruit_RA8875::writeCommand(0x91);
    Adafruit_RA8875::writeData(x0);
    Adafruit_RA8875::writeCommand(0x92);
    Adafruit_RA8875::writeData(x0 >> 8);

    /* Set Y */
    Adafruit_RA8875::writeCommand(0x93);
    Adafruit_RA8875::writeData(y0);
    Adafruit_RA8875::writeCommand(0x94);
    Adafruit_RA8875::writeData(y0 >> 8);

    /* Set X1 */
    Adafruit_RA8875::writeCommand(0x95);
    Adafruit_RA8875::writeData(x1);
    Adafruit_RA8875::writeCommand(0x96);
    Adafruit_RA8875::writeData((x1) >> 8);

    /* Set Y1 */
    Adafruit_RA8875::writeCommand(0x97);
    Adafruit_RA8875::writeData(y1);
    Adafruit_RA8875::writeCommand(0x98);
    Adafruit_RA8875::writeData((y1) >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(RA8875_DCR);
    Adafruit_RA8875::writeData(0x80);

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
        Draw a vertical line

        @param x The X position
        @param y The Y position
        @param h Height
        @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                                                        uint16_t color) {
    Adafruit_RA8875::drawLine(x, y, x, y + h, color);
}

/**************************************************************************/
/*!
         Draw a horizontal line

         @param x The X position
         @param y The Y position
         @param w Width
         @param color The color
*/
/**************************************************************************/
void Adafruit_RA8875::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                                                        uint16_t color) {
    Adafruit_RA8875::drawLine(x, y, x + w, y, color);
}

/**************************************************************************/
/*!
            Draws a HW accelerated rectangle on the display

            @param x         The 0-based x location of the top-right corner
            @param y         The 0-based y location of the top-right corner
            @param w         The rectangle width
            @param h         The rectangle height
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                             uint16_t color) {
    Adafruit_RA8875::rectHelper(x, y, x + w - 1, y + h - 1, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled rectangle on the display

            @param x         The 0-based x location of the top-right corner
            @param y         The 0-based y location of the top-right corner
            @param w         The rectangle width
            @param h         The rectangle height
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                             uint16_t color) {
    Adafruit_RA8875::rectHelper(x, y, x + w - 1, y + h - 1, color, true);
}

/**************************************************************************/
/*!
            Fills the screen with the spefied RGB565 color

            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillScreen(uint16_t color) {
    Adafruit_RA8875::rectHelper(0, 0, _width - 1, _height - 1, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated circle on the display

            @param x         The 0-based x location of the center of the circle
            @param y         The 0-based y location of the center of the circle
            @param r         The circle's radius
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawCircle(int16_t x, int16_t y, int16_t r,
                                                                 uint16_t color) {
    Adafruit_RA8875::circleHelper(x, y, r, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled circle on the display

            @param x         The 0-based x location of the center of the circle
            @param y         The 0-based y location of the center of the circle
            @param r         The circle's radius
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillCircle(int16_t x, int16_t y, int16_t r,
                                                                 uint16_t color) {
    Adafruit_RA8875::circleHelper(x, y, r, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated triangle on the display

            @param x0        The 0-based x location of point 0 on the triangle
            @param y0        The 0-based y location of point 0 on the triangle
            @param x1        The 0-based x location of point 1 on the triangle
            @param y1        The 0-based y location of point 1 on the triangle
            @param x2        The 0-based x location of point 2 on the triangle
            @param y2        The 0-based y location of point 2 on the triangle
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawTriangle(int16_t x0, int16_t y0, int16_t x1,
                                                                     int16_t y1, int16_t x2, int16_t y2,
                                                                     uint16_t color) {
    Adafruit_RA8875::triangleHelper(x0, y0, x1, y1, x2, y2, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled triangle on the display

            @param x0        The 0-based x location of point 0 on the triangle
            @param y0        The 0-based y location of point 0 on the triangle
            @param x1        The 0-based x location of point 1 on the triangle
            @param y1        The 0-based y location of point 1 on the triangle
            @param x2        The 0-based x location of point 2 on the triangle
            @param y2        The 0-based y location of point 2 on the triangle
            @param color The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillTriangle(int16_t x0, int16_t y0, int16_t x1,
                                                                     int16_t y1, int16_t x2, int16_t y2,
                                                                     uint16_t color) {
    Adafruit_RA8875::triangleHelper(x0, y0, x1, y1, x2, y2, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated ellipse on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawEllipse(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint16_t color) {
    Adafruit_RA8875::ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled ellipse on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillEllipse(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint16_t color) {
    Adafruit_RA8875::ellipseHelper(xCenter, yCenter, longAxis, shortAxis, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated curve on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param curvePart The corner to draw, where in clock-wise motion:
                                                        0 = 180-270°
                                                        1 = 270-0°
                                                        2 = 0-90°
                                                        3 = 90-180°
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::drawCurve(int16_t xCenter, int16_t yCenter,
                                                                int16_t longAxis, int16_t shortAxis,
                                                                uint8_t curvePart, uint16_t color) {
    Adafruit_RA8875::curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled curve on the display

            @param xCenter     The 0-based x location of the ellipse's center
            @param yCenter     The 0-based y location of the ellipse's center
            @param longAxis    The size in pixels of the ellipse's long axis
            @param shortAxis The size in pixels of the ellipse's short axis
            @param curvePart The corner to draw, where in clock-wise motion:
                                                        0 = 180-270°
                                                        1 = 270-0°
                                                        2 = 0-90°
                                                        3 = 90-180°
            @param color         The RGB565 color to use when drawing the pixel
*/
/**************************************************************************/
void Adafruit_RA8875::fillCurve(int16_t xCenter, int16_t yCenter,
                                                                int16_t longAxis, int16_t shortAxis,
                                                                uint8_t curvePart, uint16_t color) {
    Adafruit_RA8875::curveHelper(xCenter, yCenter, longAxis, shortAxis, curvePart, color, true);
}

/**************************************************************************/
/*!
            Draws a HW accelerated rounded rectangle on the display

            @param x     The 0-based x location of the rectangle's upper left corner
            @param y     The 0-based y location of the rectangle's upper left corner
            @param w     The size in pixels of the rectangle's width
            @param h     The size in pixels of the rectangle's height
            @param r     The radius of the curves in the corners of the rectangle
            @param color    The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                        int16_t r, uint16_t color) {
    Adafruit_RA8875::roundRectHelper(x, y, x + w, y + h, r, color, false);
}

/**************************************************************************/
/*!
            Draws a HW accelerated filled rounded rectangle on the display

            @param x     The 0-based x location of the rectangle's upper left corner
            @param y     The 0-based y location of the rectangle's upper left corner
            @param w     The size in pixels of the rectangle's width
            @param h     The size in pixels of the rectangle's height
            @param r     The radius of the curves in the corners of the rectangle
            @param color    The RGB565 color to use when drawing the pixel
 */
/**************************************************************************/
void Adafruit_RA8875::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                        int16_t r, uint16_t color) {
    Adafruit_RA8875::roundRectHelper(x, y, x + w, y + h, r, color, true);
}

/**************************************************************************/
/*!
            Helper function for higher level circle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::circleHelper(int16_t x, int16_t y, int16_t r,
                                                                     uint16_t color, bool filled) {
    x = Adafruit_RA8875::applyRotationX(x);
    y = Adafruit_RA8875::applyRotationY(y);

    /* Set X */
    Adafruit_RA8875::writeCommand(0x99);
    Adafruit_RA8875::writeData(x);
    Adafruit_RA8875::writeCommand(0x9a);
    Adafruit_RA8875::writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875::writeCommand(0x9b);
    Adafruit_RA8875::writeData(y);
    Adafruit_RA8875::writeCommand(0x9c);
    Adafruit_RA8875::writeData(y >> 8);

    /* Set Radius */
    Adafruit_RA8875::writeCommand(0x9d);
    Adafruit_RA8875::writeData(r);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875::writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_FILL);
    } else {
        Adafruit_RA8875::writeData(RA8875_DCR_CIRCLE_START | RA8875_DCR_NOFILL);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_DCR, RA8875_DCR_CIRCLE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level rectangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::rectHelper(int16_t x, int16_t y, int16_t w, int16_t h,
                                                                 uint16_t color, bool filled) {
    x = Adafruit_RA8875::applyRotationX(x);
    y = Adafruit_RA8875::applyRotationY(y);
    w = Adafruit_RA8875::applyRotationX(w);
    h = Adafruit_RA8875::applyRotationY(h);

    /* Set X */
    Adafruit_RA8875::writeCommand(0x91);
    Adafruit_RA8875::writeData(x);
    Adafruit_RA8875::writeCommand(0x92);
    Adafruit_RA8875::writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875::writeCommand(0x93);
    Adafruit_RA8875::writeData(y);
    Adafruit_RA8875::writeCommand(0x94);
    Adafruit_RA8875::writeData(y >> 8);

    /* Set X1 */
    Adafruit_RA8875::writeCommand(0x95);
    Adafruit_RA8875::writeData(w);
    Adafruit_RA8875::writeCommand(0x96);
    Adafruit_RA8875::writeData((w) >> 8);

    /* Set Y1 */
    Adafruit_RA8875::writeCommand(0x97);
    Adafruit_RA8875::writeData(h);
    Adafruit_RA8875::writeCommand(0x98);
    Adafruit_RA8875::writeData((h) >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875::writeData(0xB0);
    } else {
        Adafruit_RA8875::writeData(0x90);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level triangle drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::triangleHelper(int16_t x0, int16_t y0, int16_t x1,
                                                                         int16_t y1, int16_t x2, int16_t y2,
                                                                         uint16_t color, bool filled) {
    x0 = Adafruit_RA8875::applyRotationX(x0);
    y0 = Adafruit_RA8875::applyRotationY(y0);
    x1 = Adafruit_RA8875::applyRotationX(x1);
    y1 = Adafruit_RA8875::applyRotationY(y1);
    x2 = Adafruit_RA8875::applyRotationX(x2);
    y2 = Adafruit_RA8875::applyRotationY(y2);

    /* Set Point 0 */
    Adafruit_RA8875::writeCommand(0x91);
    Adafruit_RA8875::writeData(x0);
    Adafruit_RA8875::writeCommand(0x92);
    Adafruit_RA8875::writeData(x0 >> 8);
    Adafruit_RA8875::writeCommand(0x93);
    Adafruit_RA8875::writeData(y0);
    Adafruit_RA8875::writeCommand(0x94);
    Adafruit_RA8875::writeData(y0 >> 8);

    /* Set Point 1 */
    Adafruit_RA8875::writeCommand(0x95);
    Adafruit_RA8875::writeData(x1);
    Adafruit_RA8875::writeCommand(0x96);
    Adafruit_RA8875::writeData(x1 >> 8);
    Adafruit_RA8875::writeCommand(0x97);
    Adafruit_RA8875::writeData(y1);
    Adafruit_RA8875::writeCommand(0x98);
    Adafruit_RA8875::writeData(y1 >> 8);

    /* Set Point 2 */
    Adafruit_RA8875::writeCommand(0xA9);
    Adafruit_RA8875::writeData(x2);
    Adafruit_RA8875::writeCommand(0xAA);
    Adafruit_RA8875::writeData(x2 >> 8);
    Adafruit_RA8875::writeCommand(0xAB);
    Adafruit_RA8875::writeData(y2);
    Adafruit_RA8875::writeCommand(0xAC);
    Adafruit_RA8875::writeData(y2 >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(RA8875_DCR);
    if (filled) {
        Adafruit_RA8875::writeData(0xA1);
    } else {
        Adafruit_RA8875::writeData(0x81);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_DCR, RA8875_DCR_LINESQUTRI_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level ellipse drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::ellipseHelper(int16_t xCenter, int16_t yCenter,
                                                                        int16_t longAxis, int16_t shortAxis,
                                                                        uint16_t color, bool filled) {
    xCenter = Adafruit_RA8875::applyRotationX(xCenter);
    yCenter = Adafruit_RA8875::applyRotationY(yCenter);

    /* Set Center Point */
    Adafruit_RA8875::writeCommand(0xA5);
    Adafruit_RA8875::writeData(xCenter);
    Adafruit_RA8875::writeCommand(0xA6);
    Adafruit_RA8875::writeData(xCenter >> 8);
    Adafruit_RA8875::writeCommand(0xA7);
    Adafruit_RA8875::writeData(yCenter);
    Adafruit_RA8875::writeCommand(0xA8);
    Adafruit_RA8875::writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    Adafruit_RA8875::writeCommand(0xA1);
    Adafruit_RA8875::writeData(longAxis);
    Adafruit_RA8875::writeCommand(0xA2);
    Adafruit_RA8875::writeData(longAxis >> 8);
    Adafruit_RA8875::writeCommand(0xA3);
    Adafruit_RA8875::writeData(shortAxis);
    Adafruit_RA8875::writeCommand(0xA4);
    Adafruit_RA8875::writeData(shortAxis >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(0xA0);
    if (filled) {
        Adafruit_RA8875::writeData(0xC0);
    } else {
        Adafruit_RA8875::writeData(0x80);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level curve drawing code
*/
/**************************************************************************/
void Adafruit_RA8875::curveHelper(int16_t xCenter, int16_t yCenter,
                                                                    int16_t longAxis, int16_t shortAxis,
                                                                    uint8_t curvePart, uint16_t color,
                                                                    bool filled) {
    xCenter = Adafruit_RA8875::applyRotationX(xCenter);
    yCenter = Adafruit_RA8875::applyRotationY(yCenter);
    curvePart = (curvePart + _rotation) % 4;

    /* Set Center Point */
    Adafruit_RA8875::writeCommand(0xA5);
    Adafruit_RA8875::writeData(xCenter);
    Adafruit_RA8875::writeCommand(0xA6);
    Adafruit_RA8875::writeData(xCenter >> 8);
    Adafruit_RA8875::writeCommand(0xA7);
    Adafruit_RA8875::writeData(yCenter);
    Adafruit_RA8875::writeCommand(0xA8);
    Adafruit_RA8875::writeData(yCenter >> 8);

    /* Set Long and Short Axis */
    Adafruit_RA8875::writeCommand(0xA1);
    Adafruit_RA8875::writeData(longAxis);
    Adafruit_RA8875::writeCommand(0xA2);
    Adafruit_RA8875::writeData(longAxis >> 8);
    Adafruit_RA8875::writeCommand(0xA3);
    Adafruit_RA8875::writeData(shortAxis);
    Adafruit_RA8875::writeCommand(0xA4);
    Adafruit_RA8875::writeData(shortAxis >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(0xA0);
    if (filled) {
        Adafruit_RA8875::writeData(0xD0 | (curvePart & 0x03));
    } else {
        Adafruit_RA8875::writeData(0x90 | (curvePart & 0x03));
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_ELLIPSE, RA8875_ELLIPSE_STATUS);
}

/**************************************************************************/
/*!
            Helper function for higher level rounded rectangle drawing code
 */
/**************************************************************************/

void Adafruit_RA8875::roundRectHelper(int16_t x, int16_t y, int16_t w,
                                                                            int16_t h, int16_t r, uint16_t color,
                                                                            bool filled) {
    x = Adafruit_RA8875::applyRotationX(x);
    y = Adafruit_RA8875::applyRotationY(y);
    w = Adafruit_RA8875::applyRotationX(w);
    h = Adafruit_RA8875::applyRotationY(h);
    if (x > w)
        Adafruit_RA8875::swap(x, w);
    if (y > h)
        Adafruit_RA8875::swap(y, h);

    /* Set X */
    Adafruit_RA8875::writeCommand(0x91);
    Adafruit_RA8875::writeData(x);
    Adafruit_RA8875::writeCommand(0x92);
    Adafruit_RA8875::writeData(x >> 8);

    /* Set Y */
    Adafruit_RA8875::writeCommand(0x93);
    Adafruit_RA8875::writeData(y);
    Adafruit_RA8875::writeCommand(0x94);
    Adafruit_RA8875::writeData(y >> 8);

    /* Set X1 */
    Adafruit_RA8875::writeCommand(0x95);
    Adafruit_RA8875::writeData(w);
    Adafruit_RA8875::writeCommand(0x96);
    Adafruit_RA8875::writeData((w) >> 8);

    /* Set Y1 */
    Adafruit_RA8875::writeCommand(0x97);
    Adafruit_RA8875::writeData(h);
    Adafruit_RA8875::writeCommand(0x98);
    Adafruit_RA8875::writeData((h) >> 8);

    Adafruit_RA8875::writeCommand(0xA1);
    Adafruit_RA8875::writeData(r);
    Adafruit_RA8875::writeCommand(0xA2);
    Adafruit_RA8875::writeData((r) >> 8);

    Adafruit_RA8875::writeCommand(0xA3);
    Adafruit_RA8875::writeData(r);
    Adafruit_RA8875::writeCommand(0xA4);
    Adafruit_RA8875::writeData((r) >> 8);

    /* Set Color */
    Adafruit_RA8875::writeCommand(0x63);
    Adafruit_RA8875::writeData((color & 0xf800) >> 11);
    Adafruit_RA8875::writeCommand(0x64);
    Adafruit_RA8875::writeData((color & 0x07e0) >> 5);
    Adafruit_RA8875::writeCommand(0x65);
    Adafruit_RA8875::writeData((color & 0x001f));

    /* Draw! */
    Adafruit_RA8875::writeCommand(RA8875_ELLIPSE);
    if (filled) {
        Adafruit_RA8875::writeData(0xE0);
    } else {
        Adafruit_RA8875::writeData(0xA0);
    }

    /* Wait for the command to finish */
    Adafruit_RA8875::waitPoll(RA8875_ELLIPSE, RA8875_DCR_LINESQUTRI_STATUS);
}
/**************************************************************************/
/*!
            Set the scroll window

            @param x    X position of the scroll window
            @param y    Y position of the scroll window
            @param w    Width of the Scroll Window
            @param h    Height of the Scroll window
            @param mode Layer to Scroll

 */
/**************************************************************************/
void Adafruit_RA8875::setScrollWindow(int16_t x, int16_t y, int16_t w,
                                                                            int16_t h, uint8_t mode) {
    // Horizontal Start point of Scroll Window
    Adafruit_RA8875::writeCommand(0x38);
    Adafruit_RA8875::writeData(x);
    Adafruit_RA8875::writeCommand(0x39);
    Adafruit_RA8875::writeData(x >> 8);

    // Vertical Start Point of Scroll Window
    Adafruit_RA8875::writeCommand(0x3a);
    Adafruit_RA8875::writeData(y);
    Adafruit_RA8875::writeCommand(0x3b);
    Adafruit_RA8875::writeData(y >> 8);

    // Horizontal End Point of Scroll Window
    Adafruit_RA8875::writeCommand(0x3c);
    Adafruit_RA8875::writeData(x + w);
    Adafruit_RA8875::writeCommand(0x3d);
    Adafruit_RA8875::writeData((x + w) >> 8);

    // Vertical End Point of Scroll Window
    Adafruit_RA8875::writeCommand(0x3e);
    Adafruit_RA8875::writeData(y + h);
    Adafruit_RA8875::writeCommand(0x3f);
    Adafruit_RA8875::writeData((y + h) >> 8);

    // Scroll function setting
    Adafruit_RA8875::writeCommand(0x52);
    Adafruit_RA8875::writeData(mode);
}

/**************************************************************************/
/*!
        Scroll in the X direction

        @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875::scrollX(int16_t dist) {
    Adafruit_RA8875::writeCommand(0x24);
    Adafruit_RA8875::writeData(dist);
    Adafruit_RA8875::writeCommand(0x25);
    Adafruit_RA8875::writeData(dist >> 8);
}

/**************************************************************************/
/*!
         Scroll in the Y direction

         @param dist The distance to scroll

 */
/**************************************************************************/
void Adafruit_RA8875::scrollY(int16_t dist) {
    Adafruit_RA8875::writeCommand(0x26);
    Adafruit_RA8875::writeData(dist);
    Adafruit_RA8875::writeCommand(0x27);
    Adafruit_RA8875::writeData(dist >> 8);
}

/************************* Mid Level ***********************************/

/**************************************************************************/
/*!
        Set the Extra General Purpose IO Register

        @param on Whether to turn Extra General Purpose IO on or not

 */
/**************************************************************************/
void Adafruit_RA8875::GPIOX(bool on) {
    if (on)
        Adafruit_RA8875::writeReg(RA8875_GPIOX, 1);
    else
        Adafruit_RA8875::writeReg(RA8875_GPIOX, 0);
}

/**************************************************************************/
/*!
        Set the duty cycle of the PWM 1 Clock

        @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875::PWM1out(uint8_t p) { Adafruit_RA8875::writeReg(RA8875_P1DCR, p); }

/**************************************************************************/
/*!
         Set the duty cycle of the PWM 2 Clock

         @param p The duty Cycle (0-255)
*/
/**************************************************************************/
void Adafruit_RA8875::PWM2out(uint8_t p) { Adafruit_RA8875::writeReg(RA8875_P2DCR, p); }

/**************************************************************************/
/*!
        Configure the PWM 1 Clock

        @param on Whether to enable the clock
        @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875::PWM1config(bool on, uint8_t clock) {
    if (on) {
        Adafruit_RA8875::writeReg(RA8875_P1CR, RA8875_P1CR_ENABLE | (clock & 0xF));
    } else {
        Adafruit_RA8875::writeReg(RA8875_P1CR, RA8875_P1CR_DISABLE | (clock & 0xF));
    }
}

/**************************************************************************/
/*!
         Configure the PWM 2 Clock

         @param on Whether to enable the clock
         @param clock The Clock Divider
*/
/**************************************************************************/
void Adafruit_RA8875::PWM2config(bool on, uint8_t clock) {
    if (on) {
        Adafruit_RA8875::writeReg(RA8875_P2CR, RA8875_P2CR_ENABLE | (clock & 0xF));
    } else {
        Adafruit_RA8875::writeReg(RA8875_P2CR, RA8875_P2CR_DISABLE | (clock & 0xF));
    }
}


/**************************************************************************/
/*!
            Checks if a touch event has occured

            @return    True is a touch event has occured (reading it via
                             touchRead() will clear the interrupt in memory)
*/
/**************************************************************************/
bool Adafruit_RA8875::touched(void) {
    if (Adafruit_RA8875::readReg(RA8875_INTC2) & RA8875_INTC2_TP)
        return true;
    return false;
}

/**************************************************************************/
/*!
            Reads the last touch event

            @param x    Pointer to the uint16_t field to assign the raw X value
            @param y    Pointer to the uint16_t field to assign the raw Y value

            @return True if successful

            @note Calling this function will clear the touch panel interrupt on
                        the RA8875, resetting the flag used by the 'touched' function
*/
/**************************************************************************/
bool Adafruit_RA8875::touchRead(uint16_t *x, uint16_t *y) {
    uint16_t tx, ty;
    uint8_t temp;

    tx = Adafruit_RA8875::readReg(RA8875_TPXH);
    ty = Adafruit_RA8875::readReg(RA8875_TPYH);
    temp = Adafruit_RA8875::readReg(RA8875_TPXYL);
    tx <<= 2;
    ty <<= 2;
    tx |= temp & 0x03;                // get the bottom x bits
    ty |= (temp >> 2) & 0x03; // get the bottom y bits

    *x = tx;
    *y = ty;

    /* Clear TP INT Status */
    Adafruit_RA8875::writeReg(RA8875_INTC2, RA8875_INTC2_TP);

    return true;
}

/**************************************************************************/
/*!
            Turns the display on or off

            @param on Whether to turn the display on or not
*/
/**************************************************************************/
void Adafruit_RA8875::displayOn(bool on) {
    if (on)
        Adafruit_RA8875::writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPON);
    else
        Adafruit_RA8875::writeReg(RA8875_PWRR, RA8875_PWRR_NORMAL | RA8875_PWRR_DISPOFF);
}

/**************************************************************************/
/*!
        Puts the display in sleep mode, or disables sleep mode if enabled

        @param sleep Whether to sleep or not
*/
/**************************************************************************/
void Adafruit_RA8875::sleep(bool sleep) {
    if (sleep)
        Adafruit_RA8875::writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF | RA8875_PWRR_SLEEP);
    else
        Adafruit_RA8875::writeReg(RA8875_PWRR, RA8875_PWRR_DISPOFF);
}

/************************* Low Level ***********************************/

/**************************************************************************/
/*!
        Write data to the specified register

        @param reg Register to write to
        @param val Value to write
*/
/**************************************************************************/
void Adafruit_RA8875::writeReg(uint8_t reg, uint8_t val) {
    Adafruit_RA8875::writeCommand(reg);
    Adafruit_RA8875::writeData(val);
}

/**************************************************************************/
/*!
        Set the register to read from

        @param reg Register to read

        @return The value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875::readReg(uint8_t reg) {
    Adafruit_RA8875::writeCommand(reg);
    return Adafruit_RA8875::readData();
}

/**************************************************************************/
/*!
        Write data to the current register

        @param d Data to write
*/
/**************************************************************************/
void Adafruit_RA8875::writeData(uint8_t d) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//    SPI.transfer(RA8875_DATAWRITE);
	uint8_t data2 = 0x00;
	HAL_SPI_Transmit(&_DIS_HSPI, &data2, 1, HAL_MAX_DELAY);
//    SPI.transfer(d);
	HAL_SPI_Transmit(&_DIS_HSPI, &d, 1, HAL_MAX_DELAY);
//    spi_end();
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}



/**************************************************************************/
/*!
        Read the data from the current register

        @return The Value
*/
/**************************************************************************/
uint8_t Adafruit_RA8875::readData(void) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//
//    SPI.transfer(RA8875_DATAREAD);
	 uint8_t data3 = 0x40;
	HAL_SPI_Transmit(&_DIS_HSPI, &data3, 1, HAL_MAX_DELAY);
//    uint8_t x = SPI.transfer(0x0);
	uint8_t data4 = 0x00;
	uint8_t x;
	HAL_SPI_TransmitReceive(&_DIS_HSPI, &data4, &x, 1, HAL_MAX_DELAY);
//    spi_end();
//
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
    return x;
}

/**************************************************************************/
/*!
        Write a command to the current register

        @param d The data to write as a command
 */
/**************************************************************************/
void Adafruit_RA8875::writeCommand(uint8_t d) {

	// TODO: implement SPI and write functions for this
//    digitalWrite(_cs, LOW);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_RESET);
//    spi_begin();
//
//    SPI.transfer(RA8875_CMDWRITE);
    // TODO: clean this up
    uint8_t data1 = 0x80;
	HAL_SPI_Transmit(&_DIS_HSPI, &data1, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&_DIS_HSPI, &d, 1, HAL_MAX_DELAY);
//    SPI.transfer(d);
//    spi_end();

//
//    digitalWrite(_cs, HIGH);
	HAL_GPIO_WritePin(DIS_CS_GPIO_Port, DIS_CS_Pin, GPIO_PIN_SET);
}

/**************************************************************************/
/*!
        Read the status from the current register

        @return The value
 */
/**************************************************************************/
uint8_t Adafruit_RA8875::readStatus(void) {

	// TODO: implement SPI and write functions for this
	//    digitalWrite(_cs, LOW);
//    spi_begin();
//    SPI.transfer(RA8875_CMDREAD);
//    uint8_t x = SPI.transfer(0x0);
//    spi_end();
//
//    digitalWrite(_cs, HIGH);
//    return x;
	return 1;
}

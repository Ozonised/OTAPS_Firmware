#include "ssd1306.h"
#include <math.h>
#include <stdlib.h>
#include <string.h> // For memcpy


void Oled_Reset(void)
{
    /* for I2C - do nothing */
}

// Send a byte to the command register
void Oled_WriteCommand(SSD1306 *oled, uint8_t byte)
{
    HAL_I2C_Mem_Write(oled->hi2c, oled->address, 0x00, 1, &byte, 1,HAL_MAX_DELAY);
}

// Send data
void Oled_WriteData(SSD1306 *oled, uint8_t *buffer, size_t buff_size)
{
    HAL_I2C_Mem_Write(oled->hi2c, oled->address, 0x40, 1, buffer, buff_size, HAL_MAX_DELAY);
}


/* Fills the Screenbuffer with values from a given buffer of a fixed length */
SSD1306_Error_t Oled_FillBuffer(SSD1306 *oled, uint8_t *buf, uint32_t len)
{
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len <= oled->bufferSize)
    {
        memcpy(oled->screenBuffer, buf, len);
        ret = SSD1306_OK;
    }
    return ret;
}

void SSD1306_Init(SSD1306 *oled, I2C_HandleTypeDef *i2cHandle, uint16_t width, uint8_t height, uint8_t i2cAddress, uint8_t *screenBuffer, uint16_t screenBufferSize)
{
    oled->hi2c = i2cHandle;
    oled->width = width;
    oled->height = height;
    oled->address = i2cAddress << 1;
    oled->screenBuffer = screenBuffer;
    oled->bufferSize = screenBufferSize;
}

/* Initialize the oled screen */
void Oled_Init(SSD1306 *oled)
{
    // Reset OLED
    Oled_Reset();

    // Wait for the screen to boot
    HAL_Delay(100);

    // Init OLED
    Oled_SetDisplayOn(oled, 0); // display off

    Oled_WriteCommand(oled, 0x20); // Set Memory Addressing Mode
    Oled_WriteCommand(oled, 0x00);       // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                      // 10b,Page Addressing Mode (RESET); 11b,Invalid

    Oled_WriteCommand(oled, 0xB0); // Set Page Start Address for Page Addressing Mode,0-7

#ifdef SSD1306_MIRROR_VERT
    Oled_WriteCommand(oled, 0xC0); // Mirror vertically
#else
    Oled_WriteCommand(oled, 0xC8); // Set COM Output Scan Direction
#endif

    Oled_WriteCommand(oled, 0x00); //---set low column address
    Oled_WriteCommand(oled, 0x10); //---set high column address

    Oled_WriteCommand(oled, 0x40); //--set start line address - CHECK

    Oled_SetContrast(oled, 0xFF);

#ifdef SSD1306_MIRROR_HORIZ
    Oled_WriteCommand(oled, 0xA0); // Mirror horizontally
#else
    Oled_WriteCommand(oled, 0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef SSD1306_INVERSE_COLOR
    Oled_WriteCommand(oled, 0xA7); //--set inverse color
#else
    Oled_WriteCommand(oled, 0xA6); //--set normal color
#endif

// Set multiplex ratio.
if (oled->height == 128)
    // Found in the Luma Python lib for SH1106.
    Oled_WriteCommand(oled, 0xFF);
else
    Oled_WriteCommand(oled, 0xA8); //--set multiplex ratio(1 to 64) - CHECK


if (oled->height == 32)
    Oled_WriteCommand(oled, 0x1F); //
else if (oled->height == 64)
    Oled_WriteCommand(oled, 0x3F); //
else if (oled->height == 128)
    Oled_WriteCommand(oled, 0x3F); // Seems to work for 128px high displays too.
else
{
    /* Only 32, 64, or 128 lines of height are supported! */
}


    Oled_WriteCommand(oled, 0xA4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content

    Oled_WriteCommand(oled, 0xD3); //-set display offset - CHECK
    Oled_WriteCommand(oled, 0x00); //-not offset

    Oled_WriteCommand(oled, 0xD5); //--set display clock divide ratio/oscillator frequency
    Oled_WriteCommand(oled, 0xF0); //--set divide ratio

    Oled_WriteCommand(oled, 0xD9); //--set pre-charge period
    Oled_WriteCommand(oled, 0x22); //

    Oled_WriteCommand(oled, 0xDA); //--set com pins hardware configuration - CHECK
if (oled->height == 32)
    Oled_WriteCommand(oled, 0x02);
else if (oled->height == 64)
    Oled_WriteCommand(oled, 0x12);
else if (oled->height == 128)
    Oled_WriteCommand(oled, 0x12);
else
{
    /* Only 32, 64, or 128 lines of height are supported */
}


    Oled_WriteCommand(oled, 0xDB); //--set vcomh
    Oled_WriteCommand(oled, 0x20); // 0x20,0.77xVcc

    Oled_WriteCommand(oled, 0x8D); //--set DC-DC enable
    Oled_WriteCommand(oled, 0x14); //
    Oled_SetDisplayOn(oled, 1);    //--turn on SSD1306 panel

    // Clear screen
    Oled_Fill(oled, Black);

    // Flush buffer to screen
    Oled_UpdateScreen(oled);

    // Set default values for screen object
    oled->ssd1306_t.CurrentX = 0;
    oled->ssd1306_t.CurrentY = 0;

    oled->ssd1306_t.Initialized = 1;
}

/* Fill the whole screen with the given color */
void Oled_Fill(SSD1306 *oled, SSD1306_COLOR color)
{
    memset(oled->screenBuffer, (color == Black) ? 0x00 : 0xFF, oled->bufferSize);
}

/* Write the screenbuffer with changed to the screen */
void Oled_UpdateScreen(SSD1306 *oled)
{
    // Write data to each page of RAM. Number of pages
    // depends on the screen height:
    //
    //  * 32px   ==  4 pages
    //  * 64px   ==  8 pages
    //  * 128px  ==  16 pages
    for (uint8_t i = 0; i < oled->height / 8; i++)
    {
        Oled_WriteCommand(oled, 0xB0 + i); // Set the current RAM page address.
        Oled_WriteCommand(oled, 0x00 + SSD1306_X_OFFSET_LOWER);
        Oled_WriteCommand(oled, 0x10 + SSD1306_X_OFFSET_UPPER);
        Oled_WriteData(oled, &(oled->screenBuffer[oled->width * i]), oled->width);
    }
}

/*
 * Draw one pixel in the screenbuffer
 * X => X Coordinate
 * Y => Y Coordinate
 * color => Pixel color
 */
void Oled_DrawPixel(SSD1306 *oled, uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= oled->width || y >= oled->height)
    {
        // Don't write outside the buffer
        return;
    }

    // Draw in the right color
    if (color == White)
    {
        oled->screenBuffer[x + (y / 8) * oled->width] |= 1 << (y % 8);
    }
    else
    {
        oled->screenBuffer[x + (y / 8) * oled->width] &= ~(1 << (y % 8));
    }
}

/*
 * Draw 1 char to the screen buffer
 * ch       => char om weg te schrijven
 * Font     => Font waarmee we gaan schrijven
 * color    => Black or White
 */
char Oled_WriteChar(SSD1306 *oled, char ch, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    // Check if character is valid
    if (ch < 32 || ch > 126)
        return 0;

    // Check remaining space on current line
    if (oled->width < (oled->ssd1306_t.CurrentX + Font.width) ||
        oled->height < (oled->ssd1306_t.CurrentY + Font.height))
    {
        // Not enough space on current line
        return 0;
    }

    // Use the font to write
    for (i = 0; i < Font.height; i++)
    {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++)
        {
            if ((b << j) & 0x8000)
            {
                Oled_DrawPixel(oled, oled->ssd1306_t.CurrentX + j, (oled->ssd1306_t.CurrentY + i), (SSD1306_COLOR)color);
            }
            else
            {
                Oled_DrawPixel(oled, oled->ssd1306_t.CurrentX + j, (oled->ssd1306_t.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    // The current space is now taken
    oled->ssd1306_t.CurrentX += Font.char_width ? Font.char_width[ch - 32] : Font.width;

    // Return written char for validation
    return ch;
}

/* Write full string to screenbuffer */
char Oled_WriteString(SSD1306 *oled, char *str, SSD1306_Font_t Font, SSD1306_COLOR color)
{
    while (*str)
    {
        if (Oled_WriteChar(oled, *str, Font, color) != *str)
        {
            // Char could not be written
            return *str;
        }
        str++;
    }

    // Everything ok
    return *str;
}

/* Position the cursor */
void Oled_SetCursor(SSD1306 *oled,uint8_t x, uint8_t y)
{
    oled->ssd1306_t.CurrentX = x;
    oled->ssd1306_t.CurrentY = y;
}

/* Draw line by Bresenhem's algorithm */
void Oled_Line(SSD1306 *oled,uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX = ((x1 < x2) ? 1 : -1);
    int32_t signY = ((y1 < y2) ? 1 : -1);
    int32_t error = deltaX - deltaY;
    int32_t error2;

    Oled_DrawPixel(oled, x2, y2, color);

    while ((x1 != x2) || (y1 != y2))
    {
        Oled_DrawPixel(oled, x1, y1, color);
        error2 = error * 2;
        if (error2 > -deltaY)
        {
            error -= deltaY;
            x1 += signX;
        }

        if (error2 < deltaX)
        {
            error += deltaX;
            y1 += signY;
        }
    }
    return;
}

/* Draw polyline */
void Oled_Polyline(SSD1306 *oled, const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
    uint16_t i;
    if (par_vertex == NULL)
    {
        return;
    }

    for (i = 1; i < par_size; i++)
    {
        Oled_Line(oled, par_vertex[i - 1].x, par_vertex[i - 1].y, par_vertex[i].x, par_vertex[i].y, color);
    }

    return;
}

/* Convert Degrees to Radians */
static float ssd1306_DegToRad(float par_deg)
{
    return par_deg * (3.14f / 180.0f);
}

/* Normalize degree to [0;360] */
static uint16_t ssd1306_NormalizeTo0_360(uint16_t par_deg)
{
    uint16_t loc_angle;
    if (par_deg <= 360)
    {
        loc_angle = par_deg;
    }
    else
    {
        loc_angle = par_deg % 360;
        loc_angle = (loc_angle ? loc_angle : 360);
    }
    return loc_angle;
}

/*
 * DrawArc. Draw angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle in degree
 * sweep in degree
 */
void Oled_DrawArc(SSD1306 *oled,uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    static const uint8_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1, xp2;
    uint8_t yp1, yp2;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = ssd1306_NormalizeTo0_360(sweep);

    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;
    while (count < approx_segments)
    {
        rad = ssd1306_DegToRad(count * approx_degree);
        xp1 = x + (int8_t)(sinf(rad) * radius);
        yp1 = y + (int8_t)(cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = ssd1306_DegToRad(count * approx_degree);
        }
        else
        {
            rad = ssd1306_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sinf(rad) * radius);
        yp2 = y + (int8_t)(cosf(rad) * radius);
        Oled_Line(oled, xp1, yp1, xp2, yp2, color);
    }

    return;
}

/*
 * Draw arc with radius line
 * Angle is beginning from 4 quart of trigonometric circle (3pi/2)
 * start_angle: start angle in degree
 * sweep: finish angle in degree
 */
void Oled_DrawArcWithRadiusLine(SSD1306 *oled, uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color)
{
    const uint32_t CIRCLE_APPROXIMATION_SEGMENTS = 36;
    float approx_degree;
    uint32_t approx_segments;
    uint8_t xp1;
    uint8_t xp2 = 0;
    uint8_t yp1;
    uint8_t yp2 = 0;
    uint32_t count;
    uint32_t loc_sweep;
    float rad;

    loc_sweep = ssd1306_NormalizeTo0_360(sweep);

    count = (ssd1306_NormalizeTo0_360(start_angle) * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_segments = (loc_sweep * CIRCLE_APPROXIMATION_SEGMENTS) / 360;
    approx_degree = loc_sweep / (float)approx_segments;

    rad = ssd1306_DegToRad(count * approx_degree);
    uint8_t first_point_x = x + (int8_t)(sinf(rad) * radius);
    uint8_t first_point_y = y + (int8_t)(cosf(rad) * radius);
    while (count < approx_segments)
    {
        rad = ssd1306_DegToRad(count * approx_degree);
        xp1 = x + (int8_t)(sinf(rad) * radius);
        yp1 = y + (int8_t)(cosf(rad) * radius);
        count++;
        if (count != approx_segments)
        {
            rad = ssd1306_DegToRad(count * approx_degree);
        }
        else
        {
            rad = ssd1306_DegToRad(loc_sweep);
        }
        xp2 = x + (int8_t)(sinf(rad) * radius);
        yp2 = y + (int8_t)(cosf(rad) * radius);
        Oled_Line(oled, xp1, yp1, xp2, yp2, color);
    }

    // Radius line
    Oled_Line(oled, x, y, first_point_x, first_point_y, color);
    Oled_Line(oled, x, y, xp2, yp2, color);
    return;
}

/* Draw circle by Bresenhem's algorithm */
void Oled_DrawCircle(SSD1306 *oled, uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= oled->width || par_y >= oled->height)
    {
        return;
    }

    do
    {
        Oled_DrawPixel(oled, par_x - x, par_y + y, par_color);
        Oled_DrawPixel(oled, par_x + x, par_y + y, par_color);
        Oled_DrawPixel(oled, par_x + x, par_y - y, par_color);
        Oled_DrawPixel(oled, par_x - x, par_y - y, par_color);
        e2 = err;

        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw filled circle. Pixel positions calculated using Bresenham's algorithm */
void Oled_FillCircle(SSD1306 *oled, uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    if (par_x >= oled->width || par_y >= oled->height)
    {
        return;
    }

    do
    {
        for (uint8_t _y = (par_y + y); _y >= (par_y - y); _y--)
        {
            for (uint8_t _x = (par_x - x); _x >= (par_x + x); _x--)
            {
                Oled_DrawPixel(oled, _x, _y, par_color);
            }
        }

        e2 = err;
        if (e2 <= y)
        {
            y++;
            err = err + (y * 2 + 1);
            if (-x == y && e2 <= x)
            {
                e2 = 0;
            }
        }

        if (e2 > x)
        {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);

    return;
}

/* Draw a rectangle */
void Oled_DrawRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    Oled_Line(oled, x2, y1, x2, y2, color);
    Oled_Line(oled, x1, y1, x2, y1, color);
    Oled_Line(oled, x2, y2, x1, y2, color);
    Oled_Line(oled, x1, y2, x1, y1, color);

    return;
}

/* Draw a filled rectangle */
void Oled_FillRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    uint8_t x_start = ((x1 <= x2) ? x1 : x2);
    uint8_t x_end = ((x1 <= x2) ? x2 : x1);
    uint8_t y_start = ((y1 <= y2) ? y1 : y2);
    uint8_t y_end = ((y1 <= y2) ? y2 : y1);

    for (uint8_t y = y_start; (y <= y_end) && (y < oled->height); y++)
    {
        for (uint8_t x = x_start; (x <= x_end) && (x < oled->width); x++)
        {
            Oled_DrawPixel(oled, x, y, color);
        }
    }
    return;
}

SSD1306_Error_t Oled_InvertRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2)
{
    if ((x2 >= oled->width) || (y2 >= oled->height))
    {
        return SSD1306_ERR;
    }
    if ((x1 > x2) || (y1 > y2))
    {
        return SSD1306_ERR;
    }
    uint32_t i;
    if ((y1 / 8) != (y2 / 8))
    {
        /* if rectangle doesn't lie on one 8px row */
        for (uint32_t x = x1; x <= x2; x++)
        {
            i = x + (y1 / 8) * oled->width;
            oled->screenBuffer[i] ^= 0xFF << (y1 % 8);
            i += oled->width;
            for (; i < x + (y2 / 8) * oled->width; i += oled->width)
            {
                oled->screenBuffer[i] ^= 0xFF;
            }
            oled->screenBuffer[i] ^= 0xFF >> (7 - (y2 % 8));
        }
    }
    else
    {
        /* if rectangle lies on one 8px row */
        const uint8_t mask = (0xFF << (y1 % 8)) & (0xFF >> (7 - (y2 % 8)));
        for (i = x1 + (y1 / 8) * oled->width;
             i <= (uint32_t)x2 + (y2 / 8) * oled->width; i++)
        {
            oled->screenBuffer[i] ^= mask;
        }
    }
    return SSD1306_OK;
}

/* Draw a bitmap */
void Oled_DrawBitmap(SSD1306 *oled, uint8_t x, uint8_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color)
{
    int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
    uint8_t byte = 0;

    if (x >= oled->width || y >= oled->height)
    {
        return;
    }

    for (uint8_t j = 0; j < h; j++, y++)
    {
        for (uint8_t i = 0; i < w; i++)
        {
            if (i & 7)
            {
                byte <<= 1;
            }
            else
            {
                byte = (*(const unsigned char *)(&bitmap[j * byteWidth + i / 8]));
            }

            if (byte & 0x80)
            {
                Oled_DrawPixel(oled, x + i, y, color);
            }
        }
    }
    return;
}

void Oled_SetContrast(SSD1306 *oled, const uint8_t value)
{
    const uint8_t kSetContrastControlRegister = 0x81;
    Oled_WriteCommand(oled, kSetContrastControlRegister);
    Oled_WriteCommand(oled, value);
}

void Oled_SetDisplayOn(SSD1306 *oled, const uint8_t on)
{
    uint8_t value;
    if (on)
    {
        value = 0xAF; // Display on
        oled->ssd1306_t.DisplayOn = 1;
    }
    else
    {
        value = 0xAE; // Display off
        oled->ssd1306_t.DisplayOn = 1;
    }
    Oled_WriteCommand(oled, value);
}

uint8_t Oled_GetDisplayOn(SSD1306 *oled)
{
    return oled->ssd1306_t.DisplayOn;
}

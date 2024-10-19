/**
 * This Library was originally written by Olivier Van den Eede (4ilo) in 2016.
 * Some refactoring was done and SPI support was added by Aleksander Alekseev (afiskon) in 2018.
 *
 * https://github.com/afiskon/stm32-ssd1306
 */

#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"
#include <stddef.h>
#include <stdint.h>
#include <_ansi.h>

_BEGIN_STD_C

#include "ssd1306_conf.h"

#if defined(STM32WB)
#include "stm32wbxx_hal.h"
#elif defined(STM32F0)
#include "stm32f0xx_hal.h"
#elif defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#elif defined(STM32L0)
#include "stm32l0xx_hal.h"
#elif defined(STM32L1)
#include "stm32l1xx_hal.h"
#elif defined(STM32L4)
#include "stm32l4xx_hal.h"
#elif defined(STM32L5)
#include "stm32l5xx_hal.h"
#elif defined(STM32F3)
#include "stm32f3xx_hal.h"
#elif defined(STM32H7)
#include "stm32h7xx_hal.h"
#elif defined(STM32F7)
#include "stm32f7xx_hal.h"
#elif defined(STM32G0)
#include "stm32g0xx_hal.h"
#elif defined(STM32G4)
#include "stm32g4xx_hal.h"
#else
#error "SSD1306 library was tested only on STM32F0, STM32F1, STM32F3, STM32F4, STM32F7, STM32L0, STM32L1, STM32L4, STM32H7, STM32G0, STM32G4, STM32WB MCU families. Please modify ssd1306.h if you know what you are doing. Also please send a pull request if it turns out the library works on other MCU's as well!"
#endif

#ifdef SSD1306_X_OFFSET
#define SSD1306_X_OFFSET_LOWER (SSD1306_X_OFFSET & 0x0F)
#define SSD1306_X_OFFSET_UPPER ((SSD1306_X_OFFSET >> 4) & 0x07)
#else
#define SSD1306_X_OFFSET_LOWER 0
#define SSD1306_X_OFFSET_UPPER 0
#endif


// SSD1306 OLED height in pixels
#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT 64
#endif

// SSD1306 width in pixels
#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH 128
#endif

#define GET_SCREEN_BUFFER_SIZE(WIDTH, HEIGHT) (WIDTH * HEIGHT / 8)

// Struct to store transformations
typedef struct
{
    uint16_t CurrentX;
    uint16_t CurrentY;
    uint8_t Initialized;
    uint8_t DisplayOn;
} SSD1306_t;

typedef struct
{
    SSD1306_t ssd1306_t;        // SSD1306_t object to store transformation
    I2C_HandleTypeDef *hi2c;    // pointer to I2C_HandleTypeDef object
    uint16_t width;             // width of the screen
    uint16_t height;            // height of the screen
    uint16_t bufferSize;        // size of the screen buffer
    uint8_t address;            // address of the I2C oled display
    uint8_t *screenBuffer;      // pointer to the screen buffer
} SSD1306;

// Enumeration for screen colors
typedef enum
{
    Black = 0x00, // Black color, no pixel
    White = 0x01  // Pixel is set. Color depends on OLED
} SSD1306_COLOR;

typedef enum
{
    SSD1306_OK = 0x00,
    SSD1306_ERR = 0x01 // Generic error.
} SSD1306_Error_t;


typedef struct
{
    uint8_t x;
    uint8_t y;
} SSD1306_VERTEX;

/** Font */
typedef struct
{
    const uint8_t width;             /**< Font width in pixels */
    const uint8_t height;            /**< Font height in pixels */
    const uint16_t *const data;      /**< Pointer to font data array */
    const uint8_t *const char_width; /**< Proportional character width in pixels (NULL for monospaced) */
} SSD1306_Font_t;

// Procedure definitions
void SSD1306_Init(SSD1306 *oled, I2C_HandleTypeDef *i2cHandle, uint16_t width, uint8_t height, uint8_t i2cAddress, uint8_t *frameBuffer, uint16_t frameBufferSize);
void Oled_Init(SSD1306 *oled);
void Oled_Fill(SSD1306 *oled, SSD1306_COLOR color);
void Oled_UpdateScreen(SSD1306 *oled);
void Oled_DrawPixel(SSD1306 *oled,uint8_t x, uint8_t y, SSD1306_COLOR color);
char Oled_WriteChar(SSD1306 *oled, char ch, SSD1306_Font_t Font, SSD1306_COLOR color);
char Oled_WriteString(SSD1306 *oled, char *str, SSD1306_Font_t Font, SSD1306_COLOR color);
void Oled_SetCursor(SSD1306 *oled,uint8_t x, uint8_t y);
void Oled_Line(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void Oled_DrawArc(SSD1306 *oled, uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void Oled_DrawArcWithRadiusLine(SSD1306 *oled, uint8_t x, uint8_t y, uint8_t radius, uint16_t start_angle, uint16_t sweep, SSD1306_COLOR color);
void Oled_DrawCircle(SSD1306 *oled, uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR color);
void Oled_FillCircle(SSD1306 *oled, uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color);
void Oled_Polyline(SSD1306 *oled, const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color);
void Oled_DrawRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);
void Oled_FillRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color);

/**
 * @brief Invert the color of pixels in a specified rectangular area, including the border.
 *
 * @param oled Pointer to an SSD1306 object representing the OLED display.
 * @param x1 X Coordinate of the top-left corner of the rectangle.
 * @param y1 Y Coordinate of the top-left corner of the rectangle.
 * @param x2 X Coordinate of the bottom-right corner of the rectangle.
 * @param y2 Y Coordinate of the bottom-right corner of the rectangle.
 * @return SSD1306_Error_t Status of the operation.
 *         - SSD1306_OK: Operation was successful.
 *         - SSD1306_ERR: An error occurred during the operation.
 *
 * @note Ensure the coordinates (x1, y1) and (x2, y2) are within the display's boundaries.
 *       The function assumes that x2 >= x1 and y2 >= y1. It is recommended to validate the
 *       coordinates before calling this function to avoid unexpected behavior.
 *
 */
SSD1306_Error_t Oled_InvertRectangle(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);

void Oled_DrawBitmap(SSD1306 *oled, uint8_t x, uint8_t y, const unsigned char *bitmap, uint8_t w, uint8_t h, SSD1306_COLOR color);

/**
 * @brief Sets the contrast of the display.
 * @param SSD1306 *oled pointer to SSD1306 object.
 * @param[in] value contrast to set.
 * @note Contrast increases as the value increases.
 * @note RESET = 7Fh.
 */
void Oled_SetContrast(SSD1306 *oled, const uint8_t value);

/**
 * @brief Set Display ON/OFF.
 * @param SSD1306 *oled pointer to SSD1306 object.
 * @param SSD1306 *oled pointer to SSD1306 object.
 * @param[in] on 0 for OFF, any for ON.
 */
void Oled_SetDisplayOn(SSD1306 *oled, const uint8_t on);

/**
 * @brief Reads DisplayOn state.
 * @param SSD1306 *oled pointer to SSD1306 object.
 * @return  0: OFF.
 *          1: ON.
 */
uint8_t Oled_GetDisplayOn(SSD1306 *oled);

// Low-level procedures
void Oled_Reset(void);
void Oled_WriteCommand(SSD1306 *oled, uint8_t byte);
void Oled_WriteData(SSD1306 *oled, uint8_t *buffer, size_t buff_size);
SSD1306_Error_t Oled_FillBuffer(SSD1306 *oled, uint8_t *buf, uint32_t len);

_END_STD_C

#endif // __SSD1306_H__

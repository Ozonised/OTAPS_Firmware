#include "ui.h"

SSD1306 oled;
uint8_t frameBuffer[GET_SCREEN_BUFFER_SIZE(SSD1306_WIDTH, SSD1306_HEIGHT)];

static void centerCursor(SSD1306 *oled, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t noOfCharacter, uint8_t fontWidth, uint8_t fontHeight)
{
    float midX = (x2 - x1) / 2.00;
    float midY = (y2 - y1) / 2.00;

    float charCenterX = (fontWidth * noOfCharacter) / 2.00;
    float charCenterY = fontHeight / 2.00;

    oled->ssd1306_t.CurrentX = x1 + roundf((midX - charCenterX) + 1);
    oled->ssd1306_t.CurrentY = y1 + roundf((midY - charCenterY)) + 1;
}

void UI_Init(SSD1306 *oled)
{
    SSD1306_Init(oled, &hi2c1, 128, 64, 0x3C, frameBuffer, sizeof(frameBuffer));
    Oled_Init(oled);
}

void updateUI(SSD1306 *oled, Locomotive *loco)
{
    // creating table for node and loco state
    Oled_DrawRectangle(oled, 0, 0, 61, 40, White);
    Oled_DrawRectangle(oled, 61, 0, 124, 40, White);
    Oled_Line(oled, 0, 12, 124, 12, White);

    // NODE
    Oled_SetCursor(oled, 18, 3);
    Oled_WriteString(oled, "NODE", Font_6x8, White);

    Oled_SetCursor(oled, 4, 15);
    Oled_WriteString(oled, "NO  :", Font_6x8, White);
    Oled_WriteChar(oled, '0' + loco->comNodeNo, Font_6x8, White);
    Oled_SetCursor(oled, 4, 27);

    Oled_WriteString(oled, "RDY :", Font_6x8, White);
    Oled_WriteString(oled, loco->isComNodeReady ? "YES" : "NO ", Font_6x8, White);

    // LOCO
    Oled_SetCursor(oled, 78, 3);
    Oled_WriteString(oled, "STATE", Font_6x8, White);

    Oled_SetCursor(oled, 65, 15);
    Oled_WriteString(oled, "LOCO:", Font_6x8, White);

    switch (loco->state)
    {
    case STOP:
        Oled_WriteString(oled, "STOP", Font_6x8, White);
        break;

    case SLOW_DOWN:
        Oled_WriteString(oled, "BRK ", Font_6x8, White);
        break;

    case GO:
        Oled_WriteString(oled, "GO  ", Font_6x8, White);
        break;

    default:
        break;
    }

    Oled_SetCursor(oled, 65, 27);
    Oled_WriteString(oled, "LINE:", Font_6x8, White);
    // print line state
    switch (loco->line)
    {
    case CLEAR:
        Oled_WriteString(oled, "CLR", Font_6x8, White);
        break;

    case BLOCK:
        Oled_WriteString(oled, "BLK", Font_6x8, White);
        break;

    default:
        Oled_WriteString(oled, "---", Font_6x8, White);
        break;
    }

    // // drawing rectangles for the signal state
    // for (size_t x1 = 0, x2 = 31; x2 <= oled.width; x1 += 31, x2 += 31)
    // {
    //     if (x1 < 93)
    //     {
    //         Oled_DrawRectangle(oled, x1, 43, x2, 63, White);
    //     }
    //     else
    //     {
    //         Oled_FillRectangle(oled, x1, 43, x2, 63, White);
    //     }
    // }

    // print the signal states
    for (size_t i = 0, x1 = 0, x2 = 31; i < sizeof(loco->signalData) && x2 <= oled->width; i++, x1 += 31, x2 += 31)
    {

        switch (loco->signalData[i])
        {
        case GREEN:
            Oled_FillRectangle(oled, x1, 43, x2, 63, Black);
            Oled_DrawRectangle(oled, x1, 43, x2, 63, White);
            centerCursor(oled, x1, 43, x2, 63, 1, 11, 18);
            Oled_WriteChar(oled, 'G', Font_11x18, White);
            break;

        case DOUBLE_YELLOW:
            Oled_FillRectangle(oled, x1, 43, x2, 63, Black);
            Oled_DrawRectangle(oled, x1, 43, x2, 63, White);
            centerCursor(oled, x1, 43, x2, 63, 2, 11, 18);
            Oled_WriteString(oled, "DY", Font_11x18, White);
            break;

        case YELLOW:
            Oled_FillRectangle(oled, x1, 43, x2, 63, Black);
            Oled_DrawRectangle(oled, x1, 43, x2, 63, White);
            centerCursor(oled, x1, 43, x2, 63, 1, 11, 18);
            Oled_WriteChar(oled, 'Y', Font_11x18, White);
            break;

        case RED:
            Oled_FillRectangle(oled, x1, 43, x2, 63, White);
            centerCursor(oled, x1, 43, x2, 63, 1, 11, 18);
            Oled_WriteChar(oled, 'R', Font_11x18, Black);
            break;

        default:
            Oled_FillRectangle(oled, x1, 43, x2, 63, Black);
            Oled_DrawRectangle(oled, x1, 43, x2, 63, White);
            centerCursor(oled, x1, 43, x2, 63, 1, 11, 18);
            Oled_WriteChar(oled, 'X', Font_11x18, White);
            break;
        }
    }

    Oled_UpdateScreen(oled);
}

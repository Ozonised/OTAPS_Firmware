#ifndef INC_UI_H_
#define INC_UI_H_

#include "main.h"
#include <math.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include "locomotive.h"

void UI_Init(SSD1306 *oled);
void updateUI(SSD1306 *oled, Locomotive *loco);
#endif /* INC_UI_H_ */

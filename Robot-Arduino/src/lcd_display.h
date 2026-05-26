#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

// LCD I2C configuration — Frank de Brabander library
#define LCD_I2C_ADDR 0x27
#define LCD_COLS     16
#define LCD_ROWS     2

// Initialize LCD, backlight, and show splash screen for 1500 ms
void lcd_init();

// Show current station, target station, and operating mode string
void lcd_show_status(uint8_t currentStation, uint8_t targetStation, const char* mode);

// Show obstacle-detected warning on both lines
void lcd_show_obstacle();

// Show box-open prompt with countdown timer for a given station
void lcd_show_waiting_box(uint8_t station, uint16_t secondsLeft);

// Display any two arbitrary strings on line 1 and line 2
void lcd_show_message(const char* line1, const char* line2);

// Show emergency-stop message triggered by remote override
void lcd_show_estop();

// Show "returning home" status message
void lcd_show_home();

#endif

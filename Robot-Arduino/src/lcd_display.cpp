#include "lcd_display.h"

// Module-level LCD object — shared by all display functions
static LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLS, LCD_ROWS);

// Initialize LCD, enable backlight, show splash screen, then clear after 1500 ms
void lcd_init() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Delivery Robot");
    lcd.setCursor(0, 1);
    lcd.print("Initializing...");
    delay(1500);
    lcd.clear();
}

// Show navigation status: current room, target room, and operating mode
void lcd_show_status(uint8_t currentStation, uint8_t targetStation, const char* mode) {
    char line1[LCD_COLS + 1];
    // Format: "Room:X->Y  MODE" — mode string is right-padded to fill the display
    snprintf(line1, sizeof(line1), "Room:%d->%d  %-4s", currentStation, targetStation, mode);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    // Line 2 cleared (spaces) to erase any previous content
    lcd.setCursor(0, 1);
    lcd.print("                ");
}

// Show obstacle warning — displayed while robot is halted waiting for clearance
void lcd_show_obstacle() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("!! OBSTACLE !!");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
}

// Show box-open prompt with a live countdown for the given station number
void lcd_show_waiting_box(uint8_t station, uint16_t secondsLeft) {
    char line1[LCD_COLS + 1];
    char line2[LCD_COLS + 1];
    snprintf(line1, sizeof(line1), "Room %d: Open Box", station);
    snprintf(line2, sizeof(line2), "Timeout: %3ds   ", secondsLeft);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

// Generic display helper — show any two strings on lines 1 and 2
void lcd_show_message(const char* line1, const char* line2) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(line1);
    lcd.setCursor(0, 1);
    lcd.print(line2);
}

// Show emergency-stop state triggered by remote override command
void lcd_show_estop() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("** E-STOP **");
    lcd.setCursor(0, 1);
    lcd.print("Remote Override");
}

// Show homing status while robot navigates back to base station
void lcd_show_home() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Returning Home");
    lcd.setCursor(0, 1);
    lcd.print("                ");
}

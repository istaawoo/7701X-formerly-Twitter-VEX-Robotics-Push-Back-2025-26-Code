#include "screen.hpp"
#include "main.h"
#include "auton.hpp"

#define LINE_HEIGHT 40
#define PADDING 5

// Display the list of autons with a highlight on the selected one
void displayAutons() {
    pros::screen::erase();

    for (int i = 0; i < autons.size(); i++) {
        int y = i * LINE_HEIGHT;

        // Draw highlight for selected auton
        if (i == selectedAuton) {
            pros::screen::set_pen(pros::Color::alice_blue); // blue
            pros::screen::draw_rect(0, y, 480, y + LINE_HEIGHT); // Brain width = 480
        }

        // Draw the auton name
        pros::screen::set_pen(pros::Color::white); // white text
        pros::screen::print(pros::E_TEXT_MEDIUM, PADDING, y + PADDING, autons[i].name.c_str());
    }
}

// Detect taps and select the auton
void handleTouch() {
    int lastPressCount = pros::screen::touch_status().press_count;

    while (true) {
        pros::screen_touch_status_s_t touch = pros::screen::touch_status();

        if (touch.press_count > lastPressCount) {
            int index = touch.y / LINE_HEIGHT;
            if (index >= 0 && index < autons.size()) {
                selectedAuton = index;
                displayAutons();
            }
            lastPressCount = touch.press_count;
        }

        pros::delay(50); // small delay to reduce CPU usage
    }
}

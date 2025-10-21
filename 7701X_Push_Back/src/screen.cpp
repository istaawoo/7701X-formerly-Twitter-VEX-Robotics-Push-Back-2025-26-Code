//include main
#include "main.h" 

//include pros files
#include "liblvgl/llemu.h" // IWYU pragma: keep
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp" // IWYU pragma: keep
#include "pros/adi.h" // IWYU pragma: keep
#include "pros/adi.hpp" // IWYU pragma: keep
#include "pros/colors.hpp" // IWYU pragma: keep
#include "pros/device.hpp" // IWYU pragma: keep
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/imu.h"  // IWYU pragma: keep
#include "pros/imu.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h" // IWYU pragma: keep
#include "pros/motors.hpp" // IWYU pragma: keep
#include "pros/rotation.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include "pros/screen.hpp" // IWYU pragma: keep

//general C++ utilities 
#include <cmath>
#include <iostream> // IWYU pragma: keep
#include <stdio.h> // IWYU pragma: keep

//include PID/movement utilities
//#include "pid.cpp"
#include "mcl/pid.hpp"
//#include "mcl.cpp"
#include "mcl/mcl.hpp"
#include "mcl/auton.hpp"
#include "mcl/screen.hpp"
//#include "robot.cpp"
// Sensor UI data structure
struct SensorIcon {
    int x, y, r; // position and radius
    const char* name;
    const char* description;
    bool enabled;
};

// Example sensor icons (positions are illustrative)
SensorIcon sensor_icons[] = {
    {60, 60, 15, "IMU", "Measures heading/rotation", true},
    {120, 60, 15, "Distance", "Measures distance to objects", true},
    {90, 100, 15, "Rotation", "Tracks wheel rotation", true},
};
int selected_sensor = -1;

extern pros::MotorGroup right_motors;
extern pros::MotorGroup left_motors;
extern pros::Motor intake_left;
extern pros::Motor intake_right;

enum ScreenPage {
    HOME,
    MOTORS,
    SENSORS,
    FIELD,
    GENERAL,
    AUTONS
};

ScreenPage current_page = HOME;
bool showing_motor_info = false;
int selected_motor = -1;

struct MotorButton {
    int x, y, w, h;
    int index;
};
MotorButton motor_buttons[8] = {
    {10, 10, 60, 30, 0}, {10, 50, 60, 30, 1}, {10, 90, 60, 30, 2},
    {80, 10, 60, 30, 3}, {80, 50, 60, 30, 4}, {80, 90, 60, 30, 5},
    {150, 10, 60, 30, 6}, {150, 50, 60, 30, 7}
};

void drawRoundedRect(int x, int y, int w, int h, int radius, uint32_t color) {
    pros::screen::set_pen(color);
    pros::screen::draw_rect(x, y, x+w, y+h); // Draw main rect
    // Draw corners (simple circle approximation)
    pros::screen::draw_circle(x+radius, y+radius, radius);
    pros::screen::draw_circle(x+w-radius, y+radius, radius);
    pros::screen::draw_circle(x+radius, y+h-radius, radius);
    pros::screen::draw_circle(x+w-radius, y+h-radius, radius);
}

void drawHomeScreen() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "Robot Dashboard");
    drawRoundedRect(10, 40, 100, 40, 10, 0x0000FF); // Motors (blue)
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Motors");
    drawRoundedRect(120, 40, 100, 40, 10, 0x00FF00); // Sensors (green)
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Sensors");
    drawRoundedRect(10, 90, 100, 40, 10, 0xFFFF00); // Field (yellow)
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Field");
    drawRoundedRect(120, 90, 100, 40, 10, 0xFF0000); // General (red)
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "General Info");
    drawRoundedRect(10, 140, 210, 40, 10, 0x888888); // Autons (gray)
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Autons");
}
void drawAutonsPage() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "Autonomous Selector");
    // Show list of autons, highlight selected
    for (int i = 0; i < autons.size() && i < 7; i++) {
        int y = 40 + i * 25;
        if (i == selectedAuton) {
            drawRoundedRect(10, y, 200, 22, 8, 0x00AAFF); // Highlight selected
            pros::screen::print(pros::E_TEXT_MEDIUM, i+1, "> %s", autons[i].name.c_str());
        } else {
            drawRoundedRect(10, y, 200, 22, 8, 0xCCCCCC);
            pros::screen::print(pros::E_TEXT_MEDIUM, i+1, "  %s", autons[i].name.c_str());
        }
    }
    drawRoundedRect(10, 220, 70, 30, 10, 0xFF0000); // Back button
    pros::screen::print(pros::E_TEXT_MEDIUM, 7, "Back");
}

void drawMotorButtons() {
    pros::screen::erase();
    for (int i = 0; i < 8; ++i) {
    drawRoundedRect(motor_buttons[i].x, motor_buttons[i].y, motor_buttons[i].w, motor_buttons[i].h, 8, 0x0000FF); // blue
        pros::screen::print(pros::E_TEXT_MEDIUM, i, "Motor %d", i+1);
    }
}

void drawMotorInfo(int idx) {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "Motor %d Info", idx+1);
    double temp, vel, pos, current;
    if (idx < 3) {
        int port = right_motors.get_port(idx);
        pros::Motor m(port);
        temp = m.get_temperature();
        vel = m.get_actual_velocity();
        pos = m.get_position();
        current = m.get_current_draw();
    } else if (idx < 6) {
        int port = left_motors.get_port(idx-3);
        pros::Motor m(port);
        temp = m.get_temperature();
        vel = m.get_actual_velocity();
        pos = m.get_position();
        current = m.get_current_draw();
    } else if (idx == 6) {
        temp = intake_left.get_temperature();
        vel = intake_left.get_actual_velocity();
        pos = intake_left.get_position();
        current = intake_left.get_current_draw();
    } else {
        temp = intake_right.get_temperature();
        vel = intake_right.get_actual_velocity();
        pos = intake_right.get_position();
        current = intake_right.get_current_draw();
    }
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Temp: %.1f", temp);
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Vel: %.1f", vel);
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Pos: %.1f", pos);
    pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Current: %.1f", current);
    drawRoundedRect(10, 200, 70, 30, 10, 0xFF0000); // Back button (red)
    pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Back");
}

void drawSensorsPage() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "Sensors Overview");
    // Draw robot outline (simple rectangle)
    drawRoundedRect(50, 50, 80, 60, 10, 0x888888);
    // Draw sensor icons
    for (int i = 0; i < sizeof(sensor_icons)/sizeof(SensorIcon); i++) {
        auto& icon = sensor_icons[i];
        pros::screen::set_pen(icon.enabled ? 0x00FF00 : 0x888888);
        pros::screen::draw_circle(icon.x, icon.y, icon.r);
        pros::screen::print(pros::E_TEXT_SMALL, i+1, icon.name);
    }
    pros::screen::print(pros::E_TEXT_SMALL, 7, "Tap a sensor icon to view details");
    // If a sensor is selected, show popup
    if (selected_sensor != -1) {
        auto& icon = sensor_icons[selected_sensor];
        drawRoundedRect(30, 130, 160, 60, 10, 0x00AAFF);
        pros::screen::set_pen(0x000000);
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "Name: %s", icon.name);
        pros::screen::print(pros::E_TEXT_SMALL, 6, "Desc: %s", icon.description);
        // Example live value
        pros::screen::print(pros::E_TEXT_SMALL, 8, "Value: ...");
        pros::screen::print(pros::E_TEXT_SMALL, 9, icon.enabled ? "[Disable Sensor]" : "[Enable Sensor]");
        pros::screen::print(pros::E_TEXT_SMALL, 10, "[Back]");
    }
}

void drawFieldPage() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "Field Position");
    // TODO: Draw field and robot/target positions
}

void drawGeneralPage() {
    pros::screen::erase();
    pros::screen::print(pros::E_TEXT_LARGE, 0, "General Info");
    // TODO: Add team, battery, uptime, heading, etc.
}

void screenUpdate() {
    switch (current_page) {
        case HOME:
            drawHomeScreen(); break;
        case MOTORS:
            if (!showing_motor_info) drawMotorButtons();
            else if (selected_motor != -1) drawMotorInfo(selected_motor);
            break;
        case SENSORS:
            drawSensorsPage(); break;
        case FIELD:
            drawFieldPage(); break;
        case GENERAL:
            drawGeneralPage(); break;
        case AUTONS:
            drawAutonsPage(); break;
    }
}

void screenLoop() {
    while (true) {
        screenUpdate();
        pros::delay(250);
    }
}

void handleTouch() {
    while (true) {
        auto touch = pros::screen::touch_status();
    if (touch.x != -1 && touch.y != -1) {
            if (current_page == HOME) {
                // Home screen navigation
                if (touch.x >= 10 && touch.x <= 110 && touch.y >= 40 && touch.y <= 80) current_page = MOTORS;
                else if (touch.x >= 120 && touch.x <= 220 && touch.y >= 40 && touch.y <= 80) current_page = SENSORS;
                else if (touch.x >= 10 && touch.x <= 110 && touch.y >= 90 && touch.y <= 130) current_page = FIELD;
                else if (touch.x >= 120 && touch.x <= 220 && touch.y >= 90 && touch.y <= 130) current_page = GENERAL;
                else if (touch.x >= 10 && touch.x <= 220 && touch.y >= 140 && touch.y <= 180) current_page = AUTONS;
            } else if (current_page == SENSORS) {
                if (selected_sensor == -1) {
                    // Check if a sensor icon was tapped
                    for (int i = 0; i < sizeof(sensor_icons)/sizeof(SensorIcon); i++) {
                        auto& icon = sensor_icons[i];
                        int dx = touch.x - icon.x;
                        int dy = touch.y - icon.y;
                        if (dx*dx + dy*dy <= icon.r*icon.r) {
                            selected_sensor = i;
                            break;
                        }
                    }
                } else {
                    // Popup: check for disable/enable or back
                    if (touch.x >= 30 && touch.x <= 190 && touch.y >= 130 && touch.y <= 190) {
                        // If tap near bottom, toggle enable/disable or back
                        if (touch.y >= 170 && touch.y <= 190) {
                            selected_sensor = -1; // Back
                        } else if (touch.y >= 150 && touch.y <= 170) {
                            sensor_icons[selected_sensor].enabled = !sensor_icons[selected_sensor].enabled;
                        }
                    }
                }
            } else if (current_page == AUTONS) {
                // Autons page touch
                // Select auton by row
                for (int i = 0; i < autons.size() && i < 7; i++) {
                    int y = 40 + i * 25;
                    if (touch.x >= 10 && touch.x <= 210 && touch.y >= y && touch.y <= y+22) {
                        selectedAuton = i;
                        break;
                    }
                }
                // Back button
                if (touch.x >= 10 && touch.x <= 80 && touch.y >= 220 && touch.y <= 250) {
                    current_page = HOME;
                }
            } else if (current_page == MOTORS) {
                if (!showing_motor_info) {
                    for (int i = 0; i < 8; ++i) {
                        if (touch.x >= motor_buttons[i].x && touch.x <= motor_buttons[i].x + motor_buttons[i].w &&
                            touch.y >= motor_buttons[i].y && touch.y <= motor_buttons[i].y + motor_buttons[i].h) {
                            showing_motor_info = true;
                            selected_motor = i;
                            break;
                        }
                    }
                } else {
                    // Back button
                    if (touch.x >= 10 && touch.x <= 80 && touch.y >= 200 && touch.y <= 230) {
                        showing_motor_info = false;
                        selected_motor = -1;
                    }
                }
            } else {
                // Back to home from other pages
                if (touch.x >= 10 && touch.x <= 80 && touch.y >= 200 && touch.y <= 230) {
                    current_page = HOME;
                }
            }
        }
        pros::delay(50);
    }
}

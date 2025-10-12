#pragma once
#include "pros/apix.h"
#include <functional>

class Button {
public:
    Button(int x, int y, int width, int height, const char* text, std::function<void()> onPress);

    void draw() const;
    void update(); // checks for presses

private:
    int x, y, width, height;
    const char* text;
    std::function<void()> callback;
    bool wasPressedLast = false;

    bool isPressed() const;
};

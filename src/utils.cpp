#include "utils.h"
#include <iostream>
# include <cmath>

Coordinates::Coordinates() {
    this->x = 0;
    this->y = 0;
}

Coordinates::Coordinates(float x, float y) {
    this->x = x;
    this->y = y;
}

void Coordinates::setX(float x) {
    this->x = setPrecision(x);
}

void Coordinates::setY(float y) {
    this->y = setPrecision(y);
}

float Coordinates::getX() const {
    return x;
}


float Coordinates::getY() const {
    return y;
}
std::string Coordinates::toString() const {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
}

float Coordinates::setPrecision(float &value, int precision) {
    float factor = pow(10.0f, precision);
    value = roundf(value * factor) / factor;
    return value;
}
#ifndef UTILS_H
#define UTILS_H

#include <string>

class Coordinates {
public:
    float x, y;
    Coordinates();
    Coordinates(float x, float y);
    void setX(float x);
    void setY(float y);
    float getX() const;
    float getY() const;
    void getPrecision();
    std::string toString() const;
private:
    static const int DEFAULT_PRECISION = 3;
    float setPrecision(float &value, int precision = DEFAULT_PRECISION );
};

#endif // UTILS_H
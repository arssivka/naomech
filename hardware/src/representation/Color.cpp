//
// Created by arssivka on 12/21/15.
//

#include <rd/representation/Color.h>
#include <xmlrpc-c/abyss.h>
#include <string.h>

using namespace rd;


Color::Color() {
    memset(this->data, 0, sizeof(this->data));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Color::Color(const char r, const char g, const char b) {
    this->data[RED] = r;
    this->data[GREEN] = g;
    this->data[BLUE] = b;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Color::Color(const char *data) {
    memcpy(this->data, data, sizeof(this->data));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Color::setData(const char *data) {
    memcpy(this->data, data, sizeof(this->data));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Color::setRed(const char brightness) {
    this->data[RED] = brightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Color::setGreen(const char brightness) {
    this->data[GREEN] = brightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Color::setBlue(const char brightness) {
    this->data[BLUE] = brightness;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

const char *Color::getData() const {
    return this->data;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char Color::getRed() const {
    return this->data[RED];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char Color::getGreen() const {
    return this->data[GREEN];
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char Color::getBlue() const {
    return this->data[BLUE];
}

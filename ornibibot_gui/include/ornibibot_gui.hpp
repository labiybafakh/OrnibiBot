#ifndef ORNIBIBOT_GUI_HPP
#define ORNIBIBOT_GUI_HPP

#include <iostream>
#include <cstdio>

enum pattern{
    sine,
    square,
    triangle,
    saw,
    rev_saw
};

struct marker{
    float x;
    float y;
    float z;
};

typedef struct{
    uint8_t patterns;
    float frequency;
    int amplitude;
    int offset;
} flapping_param;

#endif
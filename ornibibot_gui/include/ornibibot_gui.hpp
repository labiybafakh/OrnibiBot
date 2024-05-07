#ifndef ORNIBIBOT_GUI_HPP
#define ORNIBIBOT_GUI_HPP

#include <iostream>
#include <cstdio>

enum pattern{
    sine,
    square,
    triangle,
    saw,
    rev_saw,
    adjusted_sine
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
    int down_stroke_periode;
} flapping_param;

#endif
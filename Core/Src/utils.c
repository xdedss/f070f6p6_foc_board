/*
 * utils.c
 *
 *  Created on: Mar 17, 2023
 *      Author: liuzh
 */

#include "utils.h"
#include "math.h"

float clip01(float f) {
    if (f < 0) return 0;
    if (f > 1) return 1;
    return f;
}

float clip11(float f) {
    if (f < -1) return -1;
    if (f > 1) return 1;
    return f;
}

float clip(float f, float min, float max) {
    if (f < min) return min;
    if (f > max) return max;
    return f;
}

float lerp(float f1, float f2, float t) {
    return f1 * (1 - t) + f2 * t;
}

float deltaAngle(float a1, float a2) {
    float res = a1 - a2;
    while (res < -PI) res += PI * 2;
    while (res >= PI) res -= PI * 2;
    return res;
}

float deltaLoop(float a1, float a2, float cap) {
    // loop in 0~cap, returns [-cap/2, cap/2)
    float res = a1 - a2;
    while (res < -cap / 2) res += cap;
    while (res >= cap / 2) res -= cap;
    return res;
}

float sinNarrow(float f) {
    // from 0 to PI/2
//    float f2 = f*f;
//    float f3 = f2 * f;
//    return f - 0.043 * f2 - 0.12 * f3;
    float f3 = f * f * f;
    return f - 0.1477 * f3;
}

float sinFast(float f) {
    f -= (floorf(f * (1 / (PI * 2))) * (PI * 2));
    if (f > PI) {
        if (f > (PI * 3 / 2)) {
            return -sinNarrow(PI * 2 - f);
        }
        else {
            return -sinNarrow(f - PI);
        }
    }
    else {
        if (f > (PI / 2)) {
            return sinNarrow(PI - f);
        }
        else {
            return sinNarrow(f);
        }
    }
}

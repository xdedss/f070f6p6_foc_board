/*
 * utils.h
 *
 *  Created on: Mar 17, 2023
 *      Author: liuzh
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define PI (3.14159265f)

float clip01(float f);
float clip11(float f);
float clip(float f, float max, float min);
float lerp(float f1, float f2, float t);
float deltaAngle(float a1, float a2);
float deltaLoop(float a1, float a2, float cap);
float sinFast(float f);

#endif /* INC_UTILS_H_ */

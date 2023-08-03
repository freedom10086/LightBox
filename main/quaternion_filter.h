//
// Created by yang on 2023/8/2.
//

#ifndef LIGHT_BOX_QUATERNION_FILTER_H
#define LIGHT_BOX_QUATERNION_FILTER_H

void madgwick_quaternion_update(float ax, float ay, float az, float gyrox, float gyroy, float gyroz);

void get_tait_bryan_angles(float *yaw, float *pitch, float *roll);

#endif //LIGHT_BOX_QUATERNION_FILTER_H

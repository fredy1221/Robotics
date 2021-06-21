#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "v5.h"
#include "v5_vcs.h"

using namespace vex;


void Drive_Back_encoder_speed(float distance, float pow);
void Drive_Front_encoder_speed(float distance, float pow);
void stop_drivetrain();
float EncoderRotation(float distance);
void rotateLeft(float angle);
void rotateRight(float anglee);

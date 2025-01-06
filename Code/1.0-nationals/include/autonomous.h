#pragma once

void loginator();
double meter_to_in (double meter);

void soloAWP_right_pos(); // red
void soloAWP_left_neg(); // red
void soloAWP_right_neg(); // blue
void soloAWP_left_pos(); // blue
void elims_right(); // red pos
void elims_left(); // blue pos

void pidtune();
void skills();

extern bool ejectEnabled;
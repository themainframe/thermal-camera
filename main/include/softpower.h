#ifndef SOFTPOWER_H
#define SOFTPOWER_H

// Pin number definitions for the softpower controls of the display & camera
#define PIN_NUM_SP_DISPLAY 26
#define PIN_NUM_SP_VOSPI 27

// Pin number definition for the power switch
#define PIN_NUM_SP_SWITCH 4

void softpower_init();
bool softpower_get_desired_state();
void softpower_deep_sleep();
void softpower_pf_on();
void softpower_pf_off();

#endif

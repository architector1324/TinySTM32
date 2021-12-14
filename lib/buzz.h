#ifndef _BUZZ_H_
#define _BUZZ_H_

#include "hal.h"

////////////////////////////////
//          SETTINGS          //
////////////////////////////////


////////////////////////////////
//         DEFINITION         //
////////////////////////////////

typedef struct {
    hal_timer_pwm_t pwm;
    hal_gpio_pin_t pin;
} buzz_t;

typedef enum {
    BUZZ_NOTE_NONE = 0,
    BUZZ_NOTE_C = 261,
    BUZZ_NOTE_D = 293,
    BUZZ_NOTE_E = 329,
    BUZZ_NOTE_F = 349,
    BUZZ_NOTE_G = 392,
    BUZZ_NOTE_A = 440,
    BUZZ_NOTE_B = 493
} buzz_note_t;

typedef struct {
    uint32_t dur;
    buzz_note_t note;
} buzz_tone_t;


////////////////////////////////
//       IMPLEMENTATION       //
////////////////////////////////

#endif
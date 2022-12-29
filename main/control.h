/**
 * ESP32 Blinkenlights.
 * Copyright (C) 2019-2022  Tido Klaassen <tido_blinken@4gh.eu>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef MAIN_CONTROL_H_
#define MAIN_CONTROL_H_

#include <freertos/queue.h>

enum ctrl_event_type {
    EVNT_NONE = 0,
    EVNT_0,
    EVNT_1,
    EVNT_2,
    EVNT_3,
    EVNT_4,
    EVNT_5,
    EVNT_6,
    EVNT_7,
    EVNT_8,
    EVNT_9,
    EVNT_PWR,
    EVNT_INFO,
    EVNT_BACK,
    EVNT_UP,
    EVNT_DOWN,
    EVNT_LEFT,
    EVNT_RIGHT,
    EVNT_OK,
    EVNT_VOLUP,
    EVNT_VOLDOWN,
    EVNT_MUTE,
    EVNT_PRGUP,
    EVNT_PRGDOWN,
    EVNT_MENU,
    EVNT_GUIDE,
    EVNT_RED,
    EVNT_GREEN,
    EVNT_YELLOW,
    EVNT_BLUE,
    EVNT_REWIND,
    EVNT_FASTFWD,
    EVNT_SKIPBCK,
    EVNT_SKIPFWD,
    EVNT_PLAY,
    EVNT_PAUSE,
    EVNT_STOP,
    EVNT_BTN0_S,
    EVNT_BTN0_L,
    EVNT_BTN1_S,
    EVNT_BTN1_L,
    EVNT_BTN2_S,
    EVNT_BTN2_L,
    EVNT_AIR_INIT,
    EVNT_AIR_GOOD,
    EVNT_AIR_NORMAL,
    EVNT_AIR_BAD,
};

struct ctrl_event {
    enum ctrl_event_type event;
    bool repeat;
};

QueueHandle_t blinken_ctrl_get_queue(void);
esp_err_t blinken_ctrl_start(void);

#endif /* MAIN_CONTROL_H_ */

#include "key_codes.h"

void update_modifier_byte(int keycode, int *modifier_byte) {
    switch (keycode) {
        case KEY_MOD_LCTRL:
            *modifier_byte |= KEY_MOD_LCTRL_MASK;
            break;
        case KEY_MOD_LSHIFT:
            *modifier_byte |= KEY_MOD_LSHIFT_MASK;
            break;
        case KEY_MOD_LALT:
            *modifier_byte |= KEY_MOD_LALT_MASK;
            break;
        case KEY_MOD_LMETA:
            *modifier_byte |= KEY_MOD_LMETA_MASK;
            break;
        case KEY_MOD_RCTRL:
            *modifier_byte |= KEY_MOD_RCTRL_MASK;
            break;
        case KEY_MOD_RSHIFT:
            *modifier_byte |= KEY_MOD_RSHIFT_MASK;
            break;
        case KEY_MOD_RALT:
            *modifier_byte |= KEY_MOD_RALT_MASK;
            break;
        case KEY_MOD_RMETA:
            *modifier_byte |= KEY_MOD_RMETA_MASK;
            break;
    }
}

int is_modifier(int keycode) {
    switch (keycode) {
        case KEY_MOD_LCTRL:
            return 1;
        case KEY_MOD_LSHIFT:
            return 1;
        case KEY_MOD_LALT:
            return 1;
        case KEY_MOD_LMETA:
            return 1;
        case KEY_MOD_RCTRL:
            return 1;
        case KEY_MOD_RSHIFT:
            return 1;
        case KEY_MOD_RALT:
            return 1;
        case KEY_MOD_RMETA:
            return 1;
    }

    return 0;
}

int is_number(int keycode){
	switch (keycode){
	case KEY_1:
		return 1;
	case KEY_2:
		return 1;
	case KEY_3:
		return 1;
	case KEY_4:
		return 1;
	case KEY_5:
		return 1;
	case KEY_6:
		return 1;
	case KEY_7:
		return 1;
	case KEY_8:
		return 1;
	case KEY_9:
		return 1;
	case KEY_0:
		return 1;
	case KEY_MINUS:
		return 1;
	case KEY_EQUAL:
		return 1;
	}
	return 0;
}

int return_fn(int keycode){
	switch (keycode){
	case KEY_1:
		return KEY_F1;
	case KEY_2:
		return KEY_F2;
	case KEY_3:
		return KEY_F3;
	case KEY_4:
		return KEY_F4;
	case KEY_5:
		return KEY_F5;
	case KEY_6:
		return KEY_F6;
	case KEY_7:
		return KEY_F7;
	case KEY_8:
		return KEY_F8;
	case KEY_9:
		return KEY_F9;
	case KEY_0:
		return KEY_F10;
	case KEY_MINUS:
		return KEY_F11;
	case KEY_EQUAL:
		return KEY_F12;
	}
	return 0;
}


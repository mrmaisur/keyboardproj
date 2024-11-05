#include <stdint.h>

#include "key_scan.h"
#include "main.h"
#include "key_map.h"
#include "key_codes.h"
#include "debounce.h"

int columns_pins[NUM_COLUMNS] = {
    GPIO_PIN_15,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_12,
    GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
	GPIO_PIN_8
};

GPIO_TypeDef *columns_ports[NUM_COLUMNS] = {
    GPIOA,
    GPIOB,
    GPIOB,
    GPIOB,
    GPIOB,
    GPIOB,
    GPIOA,
    GPIOA,
    GPIOA,
    GPIOA,
    GPIOB,
    GPIOB,
	GPIOB,
	GPIOB,
	GPIOA,
};

// all row inputs are on GPIOB
int rows[NUM_ROWS] = {
  GPIO_PIN_11,
  GPIO_PIN_10,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_0
};

void scan_keys(int keys[KEYS_PER_REPORT], int *modifier_byte) {
    int num_keys = 0;

    for (int col=0; col < NUM_COLUMNS; col++) {
        HAL_GPIO_WritePin(columns_ports[col], columns_pins[col], GPIO_PIN_SET);

        for (int row=0; row < NUM_ROWS; row++) {
            int keycode = BASE_LAYOUT[row][col];

            if (HAL_GPIO_ReadPin(ROW_PORT, rows[row]) == GPIO_PIN_SET) {
                if (col == 0 && row == 0) {  // dead key for now
        		    continue;
        	    }

                if (is_modifier(keycode)) {
                    update_modifier_byte(keycode, modifier_byte);
                } else if (num_keys < KEYS_PER_REPORT) { // add a check to see if debounced as well
                    keys[num_keys] = keycode;
                    num_keys++;
                }
            }
        }
        HAL_GPIO_WritePin(columns_ports[col], columns_pins[col], GPIO_PIN_RESET);
    }
}

void replace_fn(int keys[KEYS_PER_REPORT]){
	for (int i = 0; i < KEYS_PER_REPORT; i++){
		if (keys[i] == KEY_MOD_FN){
			for (int j = 0; j < KEYS_PER_REPORT; j++){
				if (is_number(keys[j])){
					int keycode = keys[j];
					keys[j] = return_fn(keycode);
				}
			}
		}
	}
}



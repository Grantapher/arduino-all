/**************************************************************************************************
 *    _____    __      __        __           __        ______   _______   _        _    ___     _
 *  / _____)  /  \    /   \     /  \         / /       / _____) |   ____| | |      | |  |   \   | |
 * | |_____  | |\ \  / /| |    / /\ \       / /____   | |_____  |  |____  | |      | |  | |\ \  | |
 * |  _____| | | \ \/ / | |   / /  \ \     / / ___/   | |_____| |  |____| | |      | |  | | \ \ | |
 * | |_____  | |  \__/  | |  / /****\ \   / /\ \____  | |_____  |  |      | \______/ |  | |  \ \| |
 *  \______) |_|        |_| /_/      \_\ /_/  \_____\  \______) |__|       \ ______ /   |_|   \___|
 *
 *
 * Emakefun Tech firmware
 *
 * Copyright (C) 2015-2020
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, in version 3.
 * learn more you can see <http://www.gnu.org/licenses/>.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * [object]     LED_Running_Lights
 * [diagram]
 *            PIN 2  ================== LED0 ( left )
 *            PIN 3  ================== LED1
 *            PIN 4  ================== LED2
 *            PIN 5  ================== LED3
 *            PIN 6  ================== LED4
 *            PIN 7  ================== LED5
 *            PIN 8  ================== LED6
 *            PIN 9  ================== LED7 ( right )
 */


#define START_LED 2
#define NUM_LEDS 4

#define KEYPAD_PIN 12

int led_array[NUM_LEDS];
int flash_speed = 100;

/* flash led form left to right one by one */
void led_flash(void)
{
    int i;
    for(i = 0; i < NUM_LEDS; i++ )
    {
        digitalWrite(led_array[i],LOW);
        delay(flash_speed);
        digitalWrite(led_array[i],HIGH);
    }
}

/* turn on all led form left to right */
void led_turn_on(void)
{
    int i;
    for (i = 0; i < NUM_LEDS; i++ )
    {
        digitalWrite(led_array[i],LOW);
        delay(flash_speed);
    }
}

/* turn off all led  */
void led_turn_off(void)
{
    int i;
    for (i = 0; i < NUM_LEDS; i++ ) {
        digitalWrite(led_array[i],HIGH);
        delay(flash_speed);
    }
}
void setup() 
{
  // put your setup code here, to run once:
    int i;
    Serial.begin(115200);

    for (i = 0; i < NUM_LEDS; i++) {
        led_array[i] = i + START_LED;
    }
    
    for (i = 0; i < NUM_LEDS; i++ ) {
        pinMode(led_array[i],OUTPUT);
        digitalWrite(led_array[i],HIGH);    // set led control pin defalut HIGH turn off all LED
    }

    pinMode(12, INPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:
    if (digitalRead(KEYPAD_PIN) == HIGH) {
        Serial.println("Button read, starting.");
        led_flash();  
        led_turn_on();
        led_turn_off();
        Serial.println("Finished!");
    }
}

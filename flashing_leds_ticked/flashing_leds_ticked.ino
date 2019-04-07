/****************************************************
    [object]     LED_Running_Lights
    [diagram]
              PIN 2  ================== LED0 ( left )
              PIN 3  ================== LED1
              PIN 4  ================== LED2
              PIN 5  ================== LED3
              PIN 6  ================== LED4
              PIN 7  ================== LED5
              PIN 8  ================== LED6
              PIN 9  ================== LED7 ( right )
*/

#define START_LED 2
#define NUM_LEDS 8

#define KEYPAD_PIN 12

#define OUTPUT_A 11
#define OUTPUT_B 10

#define ROTARY_MAX 40
#define ROTARY_MIN 0
#define ROTARY_START 0

#define TICK_MIN 30
#define TICK_MAX 1000

int led_array[NUM_LEDS];
int flash_speed = 100;

int rotaryCount = ROTARY_START;
int aState;
int aLastState;

unsigned long startMs;
unsigned long tickSize;
unsigned int tickState = -1;

/* Main setup code. */
void setup() {
    Serial.begin(115200);

    // organize LEDs
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        led_array[i] = i + START_LED;
    }

    // Turn off all LEDs
    for (i = 0; i < NUM_LEDS; i++ ) {
        pinMode(led_array[i], OUTPUT);
        digitalWrite(led_array[i], HIGH);   // set led control pin defalut HIGH turn off all LED
    }

    // Setup input pins
    pinMode(KEYPAD_PIN, INPUT);
    pinMode(OUTPUT_A, INPUT);
    pinMode(OUTPUT_B, INPUT);

    // Reads the initial state of rotary
    aLastState = digitalRead(OUTPUT_A);

    // start clock
    startMs = millis();

    // initialize tick size
    updateTickSize(ROTARY_START);
}

/* update rotary encoder */
void rotaryStateUpdate(void) {
    aState = digitalRead(OUTPUT_A); // Reads the "current" state of the outputA

    // If the previous and the current state of the outputA are different, that means a Pulse has occured
    if (aState != aLastState) {
        int bState = digitalRead(OUTPUT_B);

        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (bState != aState) {
            rotaryCount = min(ROTARY_MAX, rotaryCount + 1);
        } else {
            rotaryCount = max(ROTARY_MIN, rotaryCount - 1);
        }
        updateTickSize(rotaryCount);

    }
    aLastState = aState; // Updates the previous state of the outputA with the current state

}

/* Update the tick size from the rotary */
void updateTickSize(int rotary) {
    tickSize = map(rotary, ROTARY_MIN, ROTARY_MAX, TICK_MAX, TICK_MIN);
}

/* Turns off all LEDs */
void resetLeds() {
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        digitalWrite(led_array[i], HIGH);
    }
}

/* Turns on the given LED and off the previous LED (if not LED 0). */
void ledFlashAdvance(int led) {
    Serial.print("Advancing Flash! LED ");
    Serial.print(led);

    if (led != 0) {
        digitalWrite(led_array[led - 1], HIGH);
    }
    digitalWrite(led_array[led], LOW);
}

/* Turns on the given LED */
void ledOnAdvance(int led) {
    Serial.print("Advancing on!    LED ");
    Serial.print(led);

    digitalWrite(led_array[led], LOW);
}

/* Turns off the given LED */
void ledOffAdvance(int led) {
    Serial.print("Advancing Off!   LED ");
    Serial.print(led);

    digitalWrite(led_array[led], HIGH);
}

/* Prints the LED pin state to Serial. */
void printPins(int ray[]) {
    Serial.print("[ ");
    int i;
    for (i = 0; i < NUM_LEDS; i++) {
        Serial.print(digitalRead(ray[i]));
        if (i != NUM_LEDS - 1) {
            Serial.print(", ");
        }
    }
    Serial.print(" ]. ");
}

/* Handle going from one state to the next. */
void advanceState() {
    tickState = (tickState + 1) % (NUM_LEDS * 3);
    char buff[2];
    sprintf(buff, "%2d", tickState);
    Serial.print("Tick State: ");
    Serial.print(buff);
    Serial.print(". ");
    byte ledState = tickState % NUM_LEDS;
    if (tickState == 8) resetLeds();
    if (tickState < NUM_LEDS) {
        ledFlashAdvance(ledState);
    } else if (tickState < NUM_LEDS * 2) {
        ledOnAdvance(ledState);
    } else {
        ledOffAdvance(ledState);
    }
    Serial.print(". LED array: ");
    printPins(led_array);
    sprintf(buff, "%2d", rotaryCount);
    Serial.print("Counter: ");
    Serial.print(buff);
    Serial.print(". ");
    Serial.println();
}

/* Check if a new tick has passed. */
bool updateTick() {
    unsigned long currentMs = millis();
    bool tickUpdate = currentMs - startMs > tickSize;
    if (tickUpdate) {
        startMs = currentMs;
    }
    return tickUpdate;
}

/* Main loop code */
void loop() {
    rotaryStateUpdate();
    bool newTick = updateTick();
    if (newTick) advanceState();
}

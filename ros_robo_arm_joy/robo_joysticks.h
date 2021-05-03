int joystickLXPin = A0;
int joystickLYPin = A1;
int joystickLButtonPin = 2;

int joystickRXPin = A2;
int joystickRYPin = A3;
int joystickRButtonPin = 3;

unsigned long lastLButtonPress;
unsigned long lastRButtonPress;

int lxValue;
int lyValue;
bool lbuttonPressed;
int rxValue;
int ryValue;
bool rbuttonPressed;

bool readJoystickButton(int pinToRead, unsigned long lastButtonPress){
    int SW_state;
    SW_state = digitalRead(pinToRead);
    // make sure button can only be pressed once ever 400 ms or so
    if (SW_state == 0 && millis() - lastButtonPress > 400) {
      // button press increments selected servo, could resetting to 1 if we cycle through
      lastButtonPress = millis();
      return 1;
    }
    return 0;
}

int readJoystickDirection(int pinToRead, int lowerMidCutoff, int upperMidCutoff, int lowerMap, int upperMap){
    int value;
    value = analogRead(pinToRead);
    // often a bit of zone around middle joystick that you want to ignore to cut out noise
    if (value > upperMidCutoff || value < lowerMidCutoff) {
      value = map(value, 0, 1023, lowerMap, upperMap);
    }
    else {
      value = 0;
    }
    // our joysticks are placed on their side
    // so we get opposite values as we expect if we move joystick
    // moving x to the right should increase, moving y up should increase, so let's make it so
    value *= -1;
    return value;
  }

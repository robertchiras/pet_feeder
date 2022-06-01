#include "PinCtrl.h"

#define TRACE_LEVEL 1
#define TRACE(lvl, ...) if (lvl <= TRACE_LEVEL) Serial.printf(__VA_ARGS__)

/* PinControl */
PinControl::PinControl(u8 num) {
  numPins = num; 
  pins = (PinData*)malloc(sizeof(PinData) * num);
  memset(pins, 0, sizeof(PinData) * num);
}

PinData *PinControl::getFreePin() {
  for (u8 i = 0; i < numPins; i++) {
    LOG(4, "@getFreePin: pin[%d].pin = %d\n", i, pins[i].pin);
    if (!pins[i].pin)
      return &pins[i];
  }

  return NULL;
}

PinData *PinControl::getPin(u8 pin) {
  for (u8 i = 0; i < numPins; i++)
    if (pins[i].pin == pin)
      return &pins[i];

  return NULL;
}

bool PinControl::addPin(u8 pin, u8 state) {
  PinData *p = getFreePin();

  if (p) {
    p->pin = pin;
    p->state = state;
    digitalWrite(pin, state);
    LOG(4, "Added pin %d with default state %d\n", pin, state);
    return true;
  }
 
  p = (PinData*)realloc(pins, sizeof(PinData) * (numPins + 1));
  if (!p)
    return false;
    
  pins = p;
  numPins++;
  pins[numPins - 1].pin = pin;
  pins[numPins - 1].state = state;
  digitalWrite(pin, state);
  LOG(4, "[Realloc] Added pin %d with default state %d\n", pin, state);
  
  return true;
}

bool PinControl::startLedFlash(const u8 pin, u16 delay1, u8 num1, u16 off1, u16 delay2, u8 num2, u16 off2, u8 state) {
  if (bitRead(activePins, pin))
    return true;

  PinData *p = getPin(pin);
  if (!p)
    return false;
    
  memset(p, 0, sizeof(p));
  p->pin = pin;
  p->trigger_time = millis();
  
  p->delay = p->delay1 = delay1;
  p->delay2 = delay2;
  p->off1 = off1 - (delay1 * (num1 * 2 - 1));
  p->off2 = off2 - (delay2 * (num2 * 2 - 1));
  p->num1 = num1;
  p->num2 = num2;
  p->cur_num = 1;
  p->type = FLASHING;
  p->state = state;
  digitalWrite(pin, state);
  bitSet(activePins, pin);
  LOG(4, "Starting to flash led %d: [%u, %u, %u]\n", pin, delay1, num1, off1);
    
  return true;  
}

bool PinControl::startBeacon(const u8 pin1, const u8 pin2, u16 flash_delay, u8 state) {
  if (startLedFlash(pin1, flash_delay, 0, 0, 0, 0, 0, state)) {
    PinData *p = getPin(pin1);
    p->beacon_pin = pin2;
    if (state == HIGH)
      digitalWrite(pin2, LOW);
    else
      digitalWrite(pin2, HIGH);
    return true;
  }

  return false;
}

/*
 * dim_delay: led goes LOW-HIGH-LOW (dimming) during this time
 */
bool PinControl::startLedDim(const u8 pin, u16 dim_delay) {
  if (bitRead(activePins, pin))
    return true;

  PinData *p = getPin(pin);
  if (!p)
    return false;
    
  memset(p, 0, sizeof(p));
  p->pin = pin;
  p->trigger_time = millis();
  p->off1 = dim_delay / 2;
  p->delay = (p->off1 + 254) / 255;
  p->cur_num = 1;
  p->type = DIMMING;
  p->state = LOW;
  digitalWrite(pin, LOW);
  bitSet(activePins, pin);
  LOG(4, "Starting to dim led %d: [%u, %u, %u]\n", pin, dim_delay, p->off1, p->delay1);
}

bool PinControl::stopLed(const u8 pin, u8 state) {
  digitalWrite(pin, state);
  if (!bitRead(activePins, pin))
    return false;

  PinData *p = getPin(pin);
  if (!p)
    return false;

  bitClear(activePins, pin);
  LOG(4, "Stopped to flash led %d, state=%d\n", pin, state);
  return true;
}

void PinControl::stopLeds(u8 state) {
  for (u8 i = 0; i < numPins; i++) {
    digitalWrite(pins[i].pin, state);
    if (!bitRead(activePins, pins[i].pin))
      continue;
    bitClear(activePins, pins[i].pin);
    LOG(4, "Stopped to flash led %d, state=%d\n", pins[i].pin, state);
  }
}

void PinControl::run() {
  unsigned long now = millis();
  PinData *p;
  
  for (u8 i = 0; i < numPins; i++) {
    if (bitRead(activePins, pins[i].pin)) {
      p = &pins[i];
      u16 elapsed = now - p->trigger_time;
      if (p->num1 && (p->cur_num == p->num1 * 2)) {
        //LOG(4, "@run(num1 reached): now=%lu, active pin[%d]: trigger_time=%lu, num=%u, cur_num=%u, delay=%u, off1=%u\n", now, p->pin, p->trigger_time, p->num1, p->cur_num, p->delay, p->off1);
        if (p->off1 && (elapsed >= p->off1)) {
          p->trigger_time = now;
          if (p->num2) {
            p->cur_num++;
            p->delay = p->delay2;
          } else {
            p->cur_num = 1;
          }
          p->state = HIGH;
          digitalWrite(p->pin, HIGH);
        } else {
          stopLed(i);
        }
      } else if (p->num2 && (p->cur_num == (p->num1 * 2 + p->num2 * 2))) {
        //LOG(4, "@run(num2 reached): now=%lu, active pin[%d]: trigger_time=%lu, num=%u, cur_num=%u, delay=%u, off2=%u\n", now, p->pin, p->trigger_time, p->num2, p->cur_num, p->delay, p->off2);
        if (p->off2 && (elapsed >= p->off2)) {
          p->trigger_time = now;
          p->cur_num = 1;
          p->delay = p->delay1;
          p->state = HIGH;
          digitalWrite(p->pin, HIGH);
        } else {
          stopLed(i);
        }
      } else if (elapsed >= p->delay) {
        switch (p->type) {
          case FLASHING:
            // Here, we transition the LOW/HIGH states
            u8 newState;
            p->trigger_time = now;
            p->cur_num++;
            //LOG(4, "@run(delay reached): now=%lu, active pin[%d]: trigger_time=%lu, num=%u, cur_num=%u, delay=%u\n", now, p->pin, p->trigger_time, p->num1, p->cur_num, p->delay);
    
            if (p->state == LOW)
              newState = HIGH;
            else
              newState = LOW;
            digitalWrite(p->pin, newState);
            if (p->beacon_pin)
              digitalWrite(p->beacon_pin, p->state);
            p->state = newState;
            break;
          case DIMMING:
            int dim_value = elapsed / p->delay;
            if (!p->dim) {
              if (dim_value > 255) {
                p->dim = 1;
                dim_value = 255;
                p->trigger_time = now;
              }
            } else {
              if (dim_value > 255) {
                p->dim = 0;
                dim_value = 0;
                p->trigger_time = now;
              } else {
                dim_value = 255 - dim_value;
              }
            }
            //LOG(4, "@run(dimming): now=%lu, trigger_time=%lu, active pin[%d]: delay=%u, elapsed=%u, dim_value:%u\n", now, p->trigger_time, p->pin, p->delay, elapsed, dim_value);
            analogWrite(p->pin, dim_value);
            break;
        }
      }
    }
  }
}
/* END PinControl */

/* DeviceState */
/*
 * Possible states and status led actions:
 * always on: IDLE, WAKE_UP states
 * 1 blink each 3s: SLEEPING
 * 1 blink, long pause (1s total):
 * 2 blinks, long pause (1s total): needs configuration (wifi cannot connect)
 * 3 blinks, long pause (1.5s total): button pressed, waiting for release
 * short fast blink: pump is running
 * long slow blink: wifi connection in progress
 * short slow blink: waiting for NTP response
 * slowly dimming up&down: in OTA mode
 * 
 */
void DeviceState::setState(DevState state, DevState success, DevState fail) {
  LOG(1, "Status changed: %d -> %d (maxWait: %u)\n", this->state.state, state.state, state.maxWait);
  
  this->state = state;
  this->stateSuccess = success;
  this->stateFail = fail;
  this->triggerTime = millis();
  
  pinCtrl->stopLed(statusLed);
  if (powerLed) {
    pinCtrl->stopLed(powerLed);
    if (state.state == SLEEPING)
      analogWrite(powerLed, 10);
      //digitalWrite(powerLed, LOW);
    else
      digitalWrite(powerLed, HIGH);
  }
  
  switch (state.state) {
    case RUN_IDLE:
    case CONFIGURED:
    case WAKE_UP:
    case BTN_WAKE_UP:
      digitalWrite(statusLed, HIGH);
      break;
    case RUN_SERVO:
      pinCtrl->startLedFlash(statusLed, 200);
      break;
    case PUSH_BUTTON:
      pinCtrl->startLedFlash(statusLed, 100);
      break;
    case CONFIGURE:
    case CONFIGURING:
      pinCtrl->startLedFlash(statusLed, 100, 2, 1000);
      break;
    case WIFI_TEST:
    case WIFI_CONNECT:
      pinCtrl->startLedFlash(statusLed, 500);
      break;
    case GET_NTP_TIME:
    case WAIT_NTP_TIME:
      pinCtrl->startLedFlash(statusLed, 50);
      break;
    case SLEEPING:
      break;
    case OTA_UPDATE:
      if (powerLed)
        pinCtrl->startBeacon(powerLed, statusLed, 500);
      else
        pinCtrl->startLedDim(statusLed, 3000);
      break;     
    default:
      break;
  }
}

bool DeviceState::setSuccess() {
  if (stateSuccess.state == NO_STATE)
    return false;

  setState(stateSuccess);
}

bool DeviceState::setFailed() {
  if (stateFail.state == NO_STATE)
    return false;

  setState(stateFail);
}
/* END DeviceState */

/* Button Management */
ButtonManager buttons[MAX_BUTTONS];

u8 ICACHE_RAM_ATTR getPinState(u8 pin) {
  u8 lastState = digitalRead(pin);
  u8 numReads = 0;
  u16 totalDelay = 0;

  while (numReads < 5 && totalDelay < 100) {
    delayMicroseconds(10);
    totalDelay += 10;
    u8 state = digitalRead(pin);
    if (state == lastState)
      numReads++;
    else
      numReads = 0;
    lastState = state;
  }

  if (numReads < 5)
    return 0xff;
    
  return lastState;
}

bool buttonPressed(ButtonManager *btn, u8 state) {
  if (state == 0xff)
    state = digitalRead(btn->pin);
  if (btn->pull == BTN_PULLDOWN)
    return (state == HIGH);
    
  return (state == LOW);
}

#define FLIP_DELAY 400
#define MIN_DELAY 20
// Checks if button changed state
ICACHE_RAM_ATTR void checkButtonState(ButtonManager &btn) {
  if (!btn.pin)
    return;
    
  //u8 newState = getPinState(btn.pin);
  u8 newState = digitalRead(btn.pin);

  if (newState == btn.curState)
    return;

  u64 trigger = millis();
  u32 curDelay = trigger - btn.lastTrigger;

  /* Debouncing mechanism */
  if (curDelay < MIN_DELAY) {
    //LOG(3, "Debouncing state: %d -> %d (delay: %u)\n", btn.curState, newState, curDelay);
    if (btn.bounceAction == BTN_ACT_UNDEFINED)
      btn.bounceAction = btn.lastAction;
    btn.lastAction = buttonPressed(&btn, newState)?BTN_ACT_PRESSED:BTN_ACT_RELEASED;
    btn.curState = newState;
    return;
  }
  if (btn.bounceAction != BTN_ACT_UNDEFINED) {
    btn.lastAction = btn.bounceAction;
    btn.bounceAction = BTN_ACT_UNDEFINED;
  }
  /* End Debouncing */
  
  u16 flip_delay = FLIP_DELAY;
 
  if (buttonPressed(&btn, newState) && !buttonPressed(&btn, btn.curState) &&
      buttonPressed(&btn, btn.prevState) && (curDelay < flip_delay)) {
    if (btn.type == BTN_TYPE_LOCKING) {
      LOG(3, "LOCKING: Button flipped ON! (delay: %u)\n", curDelay);
      btn.lastAction = BTN_ACT_FLIPPED_ON;
    } else {
      LOG(3, "MOMENTARY: Button pressed after single tap! (delay: %u)\n", curDelay);
      btn.lastAction = BTN_ACT_SINGLE_TAP_ON;
    }
  } else if (!buttonPressed(&btn, newState) && buttonPressed(&btn, btn.curState) &&
             !buttonPressed(&btn, btn.prevState) && (curDelay < flip_delay)) {
    if (btn.type == BTN_TYPE_LOCKING) {
      LOG(3, "LOCKING: Button flipped OFF! (delay: %u)\n", curDelay);
      btn.lastAction = BTN_ACT_FLIPPED_OFF;
    } else {
      if (btn.lastAction == BTN_ACT_SINGLE_TAP_ON) {
        LOG(3, "MOMENTARY: Button double tap! (delay: %u)\n", curDelay);
        btn.lastAction = BTN_ACT_DOUBLE_TAP;
      } else {
        LOG(3, "MOMENTARY: Button single tap! (delay: %u)\n", curDelay);
        btn.lastAction = BTN_ACT_SINGLE_TAP;
      }
    }
  } else if (buttonPressed(&btn, btn.curState) && !buttonPressed(&btn, newState)) {
    LOG(3, "Button released! (delay: %u)\n", curDelay);
    btn.lastAction = BTN_ACT_RELEASED;
  } else if (!buttonPressed(&btn, btn.curState) && buttonPressed(&btn, newState)) {
      LOG(3, "Button pressed! (delay: %u)\n", curDelay);
      btn.lastAction = BTN_ACT_PRESSED;
  }
  
  btn.lastTrigger = trigger;
  btn.prevState = btn.curState;
  btn.curState = newState;
}

ICACHE_RAM_ATTR void checkButtonState_0() {
  return checkButtonState(buttons[0]); 
}

ICACHE_RAM_ATTR void checkButtonState_1() {
  return checkButtonState(buttons[1]);
}

ButtonAction getButtonAction(ButtonManager *btn) {
  noInterrupts();
  ButtonAction act = btn->lastAction;
  interrupts();
  return act;
}

void setButtonAction(ButtonManager *btn, ButtonAction action) {
  noInterrupts();
  btn->lastAction = action;
  interrupts();
}

bool waitForButtonAction(ButtonManager *btn, ButtonAction action) {
  u16 mDelay = 0;
  while (btn->lastAction == action && mDelay <= FLIP_DELAY) {
      delay(10);
      mDelay += 10;
  }

  return (mDelay >= FLIP_DELAY);
}

bool getAction(ButtonManager *btn, ButtonAction action) {
  if (btn->type == BTN_TYPE_LOCKING)
    return (getButtonAction(btn) == action);

  
  if (!waitForButtonAction(btn, BTN_ACT_PRESSED))
    if (!waitForButtonAction(btn, BTN_ACT_SINGLE_TAP))
      waitForButtonAction(btn, BTN_ACT_SINGLE_TAP_ON);
  
  if (getButtonAction(btn) == action) {
    String actionName;
    
    switch (action) {
      case BTN_ACT_PRESSED: actionName = "PRESSED"; break;
      case BTN_ACT_RELEASED: actionName = "RELEASED"; break;
      case BTN_ACT_SINGLE_TAP: actionName = "SINGLE_TAP"; break;
      case BTN_ACT_SINGLE_TAP_ON: actionName = "BTN_ACT_SINGLE_TAP_ON"; break;
      case BTN_ACT_DOUBLE_TAP: actionName = "DOUBLE_TAP"; break;
    }
    LOG(1, "Button action detected: %s\n", actionName);
    setButtonAction(btn, BTN_ACT_UNDEFINED);
    return true;
  }
  
  return false;
}

ButtonManager *installButton(u8 pin, ButtonType type, ButtonPull pull) {
  ButtonManager *button;
  u8 id;
  for (id = 0; id < MAX_BUTTONS; id++) {
    button = &buttons[id];
    if (!button->pin)
      break;
  }
  if (button->pin)
    return NULL;

  button->type = type;
  button->pull = pull;
  button->pin = pin;
  button->curState = digitalRead(pin);
  button->bounceAction = BTN_ACT_UNDEFINED;
  
  switch (id) {
    case 0:
      attachInterrupt(digitalPinToInterrupt(pin), checkButtonState_0, CHANGE);
      break;
    case 1:
      attachInterrupt(digitalPinToInterrupt(pin), checkButtonState_1, CHANGE);
      break;
    default:
      break;
  }

  return button;
}

void unInstallButton(ButtonManager *btn) {
 detachInterrupt(digitalPinToInterrupt(btn->pin));
 btn->pin = 0;
 btn->curState = BTN_ACT_UNDEFINED;
}
/* END Button Management */

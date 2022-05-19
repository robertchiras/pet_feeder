#ifndef _PinCtrl_H_
#define _PinCtrl_H_

#include "Arduino.h"
#include "Logger.h"

/* PinControl */
typedef enum flash_type {
  FLASHING = 0,
  DIMMING,
} FlashType;

typedef struct pin_data {
  u8 pin;
  FlashType type;
  unsigned long trigger_time; //time when pin started the transition
  u16 delay;
  u16 delay1; //how long to keep a state
  u8 num1; //how many times to change the state
  u16 off1; //how long to keep the pin in off state
  /* second set is to allow blinkX-off-blinkY-off transitions (ex: blink-wait-blink-blink) */
  u16 delay2;
  u8 num2;
  u16 off2;

  u8 state;
  u8 cur_num;
  u8 dim;
  u8 beacon_pin;
} PinData;

class PinControl {
  private:
    PinData *pins;
    u8 numPins;
    u32 activePins;

    PinData *getFreePin();
    PinData *getPin(const u8 pin);
 
  public:
    PinControl(u8 num);
    bool addPin(const u8 pin, u8 state);
    bool startLedFlash(const u8 pin, u16 delay1, u8 num1 = 0, u16 off1 = 0, u16 delay2 = 0, u8 num2 = 0, u16 off2 = 0, u8 state = HIGH);
    bool startBeacon(const u8 pin1, const u8 pin2, u16 flash_delay, u8 state = HIGH);
    bool startLedDim(const u8 pin, u16 dim_delay);
    bool stopLed(const u8 pin, u8 state = LOW);
    void stopLeds(u8 state = LOW);
    void run();
};
/* END PinControl */

/* DeviceState */
typedef enum {
  RUN_IDLE        = 0,
  START_SERVO     = 1,
  RUN_SERVO       = 2,
  PUSH_BUTTON     = 3,
  TAP_BUTTON      = 4,
  CONFIGURE       = 5,
  CONFIGURING     = 6,
  CONFIGURED      = 7,
  WIFI_TEST       = 8,
  WIFI_CONNECT    = 9,
  GET_NTP_TIME    = 10,
  WAIT_NTP_TIME   = 11,
  SLEEPING        = 12,
  WAKE_UP         = 13,
  BTN_WAKE_UP     = 14,
  BEGIN_OTA       = 15,
  OTA_UPDATE      = 16,
  TEST_LEDS,
  NO_STATUS,
} StateEnum;

typedef struct dev_state {
  StateEnum state;
  u32 maxWait;
  u32 extraWait;
} DevState;

class DeviceState {
  private:
    PinControl *pinCtrl;
    DevState state;
    DevState nextState;
    DevState stateSuccess;
    DevState stateFail;
    u64 triggerTime;
    u8 powerLed;
    u8 statusLed;

  public:
    DeviceState(PinControl *pinCtrl, u8 s_led, u8 p_led = 0):
        pinCtrl(pinCtrl),
        powerLed(p_led),
        statusLed(s_led) {}

    StateEnum getState() { return state.state; }
    DevState getStateSuccess() { return stateSuccess; }
    DevState getStateFail() { return stateFail; }
    u32 getWaitTime() { return state.maxWait; }
    u32 getExtraTime() { return state.extraWait; }
    u64 getElapsed() {
      return millis() - triggerTime;
    }
    void extendWaitTime(u32 waitTime) { state.extraWait += waitTime; }
    bool expired() { return state.maxWait && getElapsed() > (state.maxWait + state.extraWait); }
    void setState(DevState state, DevState success = { NO_STATUS, 0}, DevState fail = { NO_STATUS, 0});
    void setState(StateEnum state) { setState({state, 0}); }
    void setStateWithNext(DevState state, DevState nextState) {
      setState(state);
      this->nextState = nextState;
    }
    bool setNextState() {
      if (nextState.state == NO_STATUS)
        return false;
      setState(nextState);
      nextState = { NO_STATUS, 0};
      return true;
    }
    bool setSuccess();
    bool setFailed();
};
/* END DeviceState */

/* Button Management */
#define MAX_BUTTONS 2 // handle up to 2 buttons
typedef enum button_type {
  BTN_TYPE_MOMENTARY,
  BTN_TYPE_LOCKING
} ButtonType;

typedef enum button_pull {
  BTN_PULLUP,
  BTN_PULLDOWN
} ButtonPull;

typedef enum button_action {
  BTN_ACT_UNDEFINED,
  BTN_ACT_PRESSED,
  BTN_ACT_RELEASED,
  BTN_ACT_FLIPPED_ON,
  BTN_ACT_FLIPPED_OFF,
  BTN_ACT_SINGLE_TAP,
  BTN_ACT_SINGLE_TAP_ON,
  BTN_ACT_DOUBLE_TAP
} ButtonAction;

typedef struct button_manager {
  u8 pin;
  ButtonPull pull;
  ButtonType type;
  ButtonAction lastAction;
  u64 lastTrigger;
  u8 prevState;
  u8 curState;
} ButtonManager;

u8 getPinState(u8 pin);
ICACHE_RAM_ATTR void checkButtonState(ButtonManager &btn);
ICACHE_RAM_ATTR void checkButtonState_0();
ICACHE_RAM_ATTR void checkButtonState_1();
ButtonAction getButtonAction(ButtonManager *btn);
void setButtonAction(ButtonManager *btn, ButtonAction action);
bool getAction(ButtonManager *btn, ButtonAction action);
ButtonManager *installButton(u8 pin, ButtonType type, ButtonPull pull = BTN_PULLDOWN);
void unInstallButton(ButtonManager *btn);
/* END button management */

#endif

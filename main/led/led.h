#ifndef _LED_H_
#define _LED_H_

class Led {
public:
    virtual ~Led() = default;
    // Set the led state based on the device state
    virtual void OnStateChanged() = 0;
    // Turn the LED into a bright white "torch" while the camera captures (on=true),
    // then restore the normal state color (on=false). Default: no-op.
    virtual void OnCameraFlash(bool on) { (void)on; }
};


class NoLed : public Led {
public:
    virtual void OnStateChanged() override {}
};

#endif // _LED_H_

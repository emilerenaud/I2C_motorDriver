#ifndef ENCODER_WRAPPER_H
#define ENCODER_WRAPPER_H

#include <Arduino.h>
#include <Encoder.h>


// #define PRESSED 1
// #define RELEASED 2
// #define LONG_PRESSED 3

class encoderWrapper
{
    public:
        encoderWrapper(uint8_t pinA);
        encoderWrapper(uint8_t pinA, uint8_t pinB);
        encoderWrapper(uint8_t pinA, uint8_t pinB, uint8_t pinSW);

        void init(void);
        int read(void);
        bool getSwitch(void);
        long getCount(void);
        int getRetCount(void);
        bool check(void);
        double getSpeed(void);
        void calculSpeed(void);

        enum eEncoderType
        {
            IR,
            quadrature,
            quadratureWithSW
        } _encoderType;

        enum eSwitchState
        {
            pressed = 1,
            released = 2,
            longPressed = 3
        };
    
    private:
        Encoder* _encoder = nullptr;

        uint8_t _pinSW = -1;
        uint8_t _swState = -1;
        bool _lastSwState = false;
        uint8_t _debounce = 100; // 100ms
        uint32_t _lastDebounceTime = 0;
        uint16_t _longPress = 800;
        uint32_t _lastLongPressTime = 0;
        uint8_t _pressedReleased = 0;

        uint32_t _encoderCount = 0;
        uint32_t _lastEncoderCount = 0;
        uint16_t _retCount = 0;

        uint8_t _pinIR = -1;
        uint8_t _lastPinIRState = 0;

        uint32_t _lastMillisSpd = 0;
        uint32_t _lastEncoderCountSpd = 0;
        float _speed = 0;
        float _bufSpeed[10];
        int _bufSpeedIndex = 0;

};

#endif
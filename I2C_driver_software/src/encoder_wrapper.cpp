#include "encoder_wrapper.h"

// One pin encoder like IR sensor
encoderWrapper::encoderWrapper(uint8_t pinA)
{
    _encoder = nullptr;
    _encoderType = encoderWrapper::IR;
    _pinIR = pinA;
    pinMode(_pinIR, INPUT_PULLUP);
    _encoderCount = 1;

}

// Two pins encoder like Pololu motor
encoderWrapper::encoderWrapper(uint8_t pinA, uint8_t pinB)
{
    _encoder = new Encoder(pinA, pinB);
    _encoderType = encoderWrapper::quadrature;
}

// Two pins encoder + switch like rotary encoder
encoderWrapper::encoderWrapper(uint8_t pinA, uint8_t pinB, uint8_t pinSW)
{
    _encoder = new Encoder(pinA, pinB);
    _pinSW = pinSW;
    pinMode(pinSW, INPUT_PULLUP);
    _encoderType = encoderWrapper::quadratureWithSW;
}

void encoderWrapper::init()
{
    // reset var
    _encoderCount = 0;
    _lastEncoderCount = 0;
}

// read encoder count, switch state and calculate speed. Return the switch state
int encoderWrapper::read()
{
    if(_encoder != nullptr)
    {
        _encoderCount = _encoder->read()/4;
    }
    else
    {
        // check one edge
        if(_lastPinIRState != digitalRead(_pinIR))
        {
            if(_lastPinIRState == 0)
            {
                _encoderCount++;
            }
            _lastPinIRState = digitalRead(_pinIR);
        }
    }

    // Calculate speed each 2ms
    calculSpeed();

    // read switch, include normal debouce 100ms + detect long press 800ms
    if(_pinSW != -1)
    {
        _swState = !digitalRead(_pinSW); // active low
        if(_swState != _lastSwState)
        {
            if(_debounce == 0)
            {
                _debounce = 100;
                if(_swState == 1)
                {
                    // Serial.println("Switch pressed");
                    _lastSwState = _swState;
                    _pressedReleased = encoderWrapper::pressed;
                    return encoderWrapper::pressed;
                }
                else
                {
                    _lastSwState = _swState;
                    _pressedReleased = encoderWrapper::released;
                    return encoderWrapper::released;
                }
            }
            else
            {
                if(millis() - _lastDebounceTime > 10)
                {
                    _debounce -= 10;
                    _lastDebounceTime = millis();
                }
            }
        }
        else if(_pressedReleased == encoderWrapper::pressed)
        {
            // if button isnt released yet
            if(_longPress == 0)
            {
                _longPress = 800;
                _pressedReleased = encoderWrapper::longPressed;
                return encoderWrapper::longPressed;
            }
            else
            {
                if(millis() - _lastLongPressTime > 10)
                {
                    _longPress -= 10;
                    _lastLongPressTime = millis();
                }
            }
        }
    }

    return -1;
}

void encoderWrapper::calculSpeed()
{
    bool newSpeed = 0;
    if(_encoderType == eEncoderType::quadrature)
    {
        if((millis() - _lastMillisSpd > 2) && (_lastEncoderCountSpd != _encoderCount))
        {
    
            float deltaCount = _encoderCount - _lastEncoderCountSpd;
            _lastEncoderCountSpd = _encoderCount;

            float deltaTime = (millis() - _lastMillisSpd)/1000.0; // in s
            _lastMillisSpd = millis();

            _speed = (deltaCount/(20.4 * 48.0 * deltaTime))*100.0; // idk why *100. Speed in RPM
            newSpeed = 1;
        }
    }
    else if(_encoderType == eEncoderType::IR)
    {
        if(_encoderCount != _lastEncoderCountSpd)
        {
            float revolutions = _encoderCount - _lastEncoderCountSpd;
            _lastEncoderCountSpd = _encoderCount;

            float deltaTime = millis() - _lastMillisSpd;
            _lastMillisSpd = millis();

            _speed = (revolutions/deltaTime)*60000.0; // in RPM
            // WARNING(_speed);
            newSpeed = 1;
        }

        // if(millis() - _lastMillisSpd > 50)
        // {
        //     float deltaT = millis() - _lastMillisSpd;
        //     _lastMillisSpd = millis();
        //     // motorPos.update();
        //     _speed = (_encoderCount/deltaT)*60000.0;
        //     // Serial.print("Rev : " + String(_encoderCount));
        //     // Serial.println("    RPM : " + String(_speed));
        //     _encoderCount = 0;
        //     newSpeed = 1;
        // }

    }

    if(newSpeed)
    {
        // Average speed on 10 samples.
        _bufSpeed[_bufSpeedIndex] = _speed;
        _bufSpeedIndex++;
        if(_bufSpeedIndex == 10)
        {
            _bufSpeedIndex = 0;
        }
        for(int i=0; i<10; i++)
        {
            _speed += _bufSpeed[i];
        }
        _speed /= 10.0;
        newSpeed = 0;
    }
}

double encoderWrapper::getSpeed()
{
    return _speed;
}

bool encoderWrapper::getSwitch()
{
    return _swState;
}

long encoderWrapper::getCount()
{
    return _encoderCount;
}

int encoderWrapper::getRetCount()
{
    _retCount = _encoderCount - _lastEncoderCount;
    _lastEncoderCount = _encoderCount;
    return _retCount;
}

bool encoderWrapper::check()
{
    if(_encoderCount != _lastEncoderCount)
    {
        return true;
    }
    else
    {
        return false;
    }
}

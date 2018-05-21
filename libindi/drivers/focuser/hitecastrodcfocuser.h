/*******************************************************************************
  Copyright(c) 2018 Jasem Mutlaq. All rights reserved.
  Copyright(c) 2016 Andy Kirkham. All rights reserved.

 Hitechastro DC Focus driver

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.
 .
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.
 .
 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include "hidapi.h"
#include "indifocuser.h"
#include "indiusbdevice.h"

class HitecAstroDCFocuser : public INDI::Focuser, public INDI::USBDevice
{
  public:
    HitecAstroDCFocuser();
    virtual ~HitecAstroDCFocuser() override;

    virtual void ISGetProperties(const char *dev) override;
    virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) override;

    // Commands
    enum
    {
        GET_MOTOR_BRAKE = 38,
        SET_MOTOR_BRAKE = 40,
        SET_STEP_POS = 42,
        GET_STEP_POS = 44,
        SET_MOTOR_REV = 46,
        GET_MOTOR_REV = 48,
        SET_PWM_FREQ = 50,
        GET_PWM_FREQ = 54,
        SET_PWM_DUTY = 52,
        GET_PWM_DUTY = 53,
        STEP_INWARD = 80,
        STEP_OUTWARD = 82,
        START_INWARD = 84,
        STOP_INWARD = 184,
        START_OUTWARD = 86,
        STOP_OUTWARD = 186
    };

    // Responses
    enum
    {
        RES_SUCCESS = 33,
        RES_VALUE,
        RES_ERROR,
        RES_STARTED,
        RES_FINISHED,
    };

protected:
    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;
    virtual bool saveConfigItems(FILE *fp) override;

    bool Connect() override;
    bool Disconnect() override;

    void TimerHit() override;

    virtual IPState MoveAbsFocuser(uint32_t ticks) override;
    virtual IPState MoveRelFocuser(FocusDirection dir, uint32_t ticks) override;
    virtual IPState MoveFocuser(FocusDirection dir, int speed, uint16_t duration) override;
    virtual bool AbortFocuser() override;

  private:
    /**
     * @brief sendCommand Generic send command with parameter & get response.
     * @param command Command to be set to controller
     * @param param optional parameter to be sent to controller.
     * @param response if not nullptr, store the reponse from the controller in this value
     * @return True if command is successful. False if command failed to be sent, or response was a failure.
     */
    bool sendCommand(uint8_t command, uint32_t param=0, uint32_t *response=nullptr);

    /**
     * @brief getStartupData Get initial controller data
     * @return True on success, false otherwise.
     */
    bool getStartupData();

    void updateFocusRange();

    // PWM
    bool setPWM(uint8_t param, double value);
    bool getPWM(uint8_t param, double &value);

    // Braking
    bool setBrake(uint8_t param);
    bool getBrake(uint8_t &value);

    // Position
    bool setSteps(uint32_t steps);
    bool getSteps(uint32_t &steps);

    // Direction
    bool setDirection(uint8_t param);
    bool getDirection(uint8_t &value);

    // Sync
    bool sync(uint32_t steps);

    hid_device *hidHandle { nullptr};

    // PWM Controls
    INumber PWMN[2];
    INumberVectorProperty PWMNP;
    enum
    {
        PWM_FREQ,
        PWM_DUTY
    };

    // Motor Brake High or Low
    ISwitch BrakeS[2];
    ISwitchVectorProperty BrakeSP;
    enum
    {
        BRAKE_LOW,
        BRAKE_HIGH
    };

    // Reverse Direction?
    ISwitch ReverseDirectionS[2];
    ISwitchVectorProperty ReverseDirectionSP;
    enum
    {
        REVERSE_DISABLED,
        REVERSE_ENABLED
    };

    // Continuous vs. Steps
    ISwitch MotionModeS[2];
    ISwitchVectorProperty MotionModeSP;
    enum
    {
        MODE_STEPS,
        MODE_CONTINUOUS
    };

    // Max Travel
    INumber MaxTravelN[1];
    INumberVectorProperty MaxTravelNP;

    // Sync
    INumber SyncN[1];
    INumberVectorProperty SyncNP;
};

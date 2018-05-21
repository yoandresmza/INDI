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

#include "hitecastrodcfocuser.h"

#include <cstring>
#include <memory>
#include <math.h>

#define HID_TIMEOUT    10000

#define FOCUS_SETTINGS_TAB "Settings"

static std::unique_ptr<HitecAstroDCFocuser> hitecastroDcFocuser(new HitecAstroDCFocuser());

void ISPoll(void *p);

void ISGetProperties(const char *dev)
{
    hitecastroDcFocuser->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    hitecastroDcFocuser->ISNewSwitch(dev, name, states, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n)
{
    hitecastroDcFocuser->ISNewText(dev, name, texts, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    hitecastroDcFocuser->ISNewNumber(dev, name, values, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[],
               char *names[], int n)
{
    INDI_UNUSED(dev);
    INDI_UNUSED(name);
    INDI_UNUSED(sizes);
    INDI_UNUSED(blobsizes);
    INDI_UNUSED(blobs);
    INDI_UNUSED(formats);
    INDI_UNUSED(names);
    INDI_UNUSED(n);
}

void ISSnoopDevice(XMLEle *root)
{
    hitecastroDcFocuser->ISSnoopDevice(root);
}

HitecAstroDCFocuser::HitecAstroDCFocuser()
{
    FI::SetCapability(FOCUSER_CAN_REL_MOVE | FOCUSER_CAN_ABS_MOVE | FOCUSER_CAN_ABORT);
    setConnection(CONNECTION_NONE);

    setVersion(1,1);
}

HitecAstroDCFocuser::~HitecAstroDCFocuser()
{
    if (hidHandle != nullptr)
        hid_close(hidHandle);

    hid_exit();
}

bool HitecAstroDCFocuser::Connect()
{
    if (hid_init() != 0)
    {
        LOG_ERROR("hid_init() failed.");
        return false;
    }

    hidHandle = hid_open(0x04D8, 0xFAC2, nullptr);

    if (hidHandle == nullptr)
        hidHandle = hid_open(0x04D8, 0xF53A, nullptr);

    if (hidHandle != nullptr)
    {
        wchar_t productString[MAXINDINAME] = {0}, manufacturerSerial[MAXINDINAME] = {0};
        hid_get_product_string(hidHandle, productString, MAXINDINAME);
        hid_get_manufacturer_string(hidHandle, manufacturerSerial, MAXINDINAME);
        LOGF_INFO("Connected to Hitechastro focuser. Product: %ls %ls", manufacturerSerial, productString);
        return true;
    }

    LOGF_ERROR("Failed to connect to focuser: %s", hid_error(hidHandle));
    return false;
}

bool HitecAstroDCFocuser::Disconnect()
{
    if (hidHandle != nullptr)
    {
        hid_close(hidHandle);
        hidHandle = nullptr;
    }

    return true;
}

const char *HitecAstroDCFocuser::getDefaultName()
{
    return static_cast<const char *>("HitecAstro DC");
}

bool HitecAstroDCFocuser::initProperties()
{
    INDI::Focuser::initProperties();

    // PWM Controls
    IUFillNumber(&PWMN[PWM_FREQ], "Freq", "Freq", "%.f", 400, 6400, 400., 400);
    IUFillNumber(&PWMN[PWM_DUTY], "Duty", "Duty", "%.f", 0, 100, 10., 50);
    IUFillNumberVector(&PWMNP, PWMN, 2, getDeviceName(), "PWM", "PWM",
                       FOCUS_SETTINGS_TAB, IP_RW, 0, IPS_IDLE);

    // Brake controls
    IUFillSwitch(&BrakeS[BRAKE_LOW], "Low", "Low", ISS_OFF);
    IUFillSwitch(&BrakeS[BRAKE_HIGH], "High", "High", ISS_ON);
    IUFillSwitchVector(&BrakeSP, BrakeS, 2, getDeviceName(), "Brake", "Brake",
                       FOCUS_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Reverse direction?
    IUFillSwitch(&ReverseDirectionS[REVERSE_DISABLED], "Normal", "Normal", ISS_ON);
    IUFillSwitch(&ReverseDirectionS[REVERSE_ENABLED], "Reversed", "Reversed", ISS_OFF);
    IUFillSwitchVector(&ReverseDirectionSP, ReverseDirectionS, 2, getDeviceName(), "Direction", "Direction",
                       FOCUS_SETTINGS_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);


    // Continuous vs. Steps
    IUFillSwitch(&MotionModeS[MODE_STEPS], "Step", "Step", ISS_ON);
    IUFillSwitch(&MotionModeS[MODE_CONTINUOUS], "Continuous", "Continuous", ISS_OFF);
    IUFillSwitchVector(&MotionModeSP, MotionModeS, 2, getDeviceName(), "Mode", "Mode",
                       MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Maximum Travel Distance. Arbitrary and decided by the user.
    IUFillNumber(&MaxTravelN[0], "Steps", "", "%.f", 0, 500000, 0., 50000);
    IUFillNumberVector(&MaxTravelNP, MaxTravelN, 1, getDeviceName(), "MAX_POSITION", "Max position",
                       MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // Sync
    IUFillNumber(&SyncN[0], "FOCUS_SYNC_OFFSET", "Offset", "%6.0f", 0, 500000., 0., 0.);
    IUFillNumberVector(&SyncNP, SyncN, 1, getDeviceName(), "FOCUS_SYNC", "Sync", MAIN_CONTROL_TAB, IP_RW, 0, IPS_IDLE);

    // Update range based on Max Travel
    updateFocusRange();

    addDebugControl();

    setDefaultPollingPeriod(500);

    return true;
}

void HitecAstroDCFocuser::ISGetProperties(const char *dev)
{
    INDI::Focuser::ISGetProperties(dev);

    defineNumber(&MaxTravelNP);
    loadConfig(true, "MAX_POSITION");
}

bool HitecAstroDCFocuser::updateProperties()
{
    INDI::Focuser::updateProperties();

    if (isConnected())
    {
        getStartupData();

        defineNumber(&PWMNP);
        defineSwitch(&MotionModeSP);
        defineSwitch(&ReverseDirectionSP);
        defineSwitch(&BrakeSP);
        defineNumber(&SyncNP);
    }
    else
    {
        deleteProperty(PWMNP.name);
        deleteProperty(MotionModeSP.name);
        deleteProperty(ReverseDirectionSP.name);
        deleteProperty(BrakeSP.name);
        deleteProperty(SyncNP.name);
    }

    return true;
}

void HitecAstroDCFocuser::TimerHit()
{
    if (FocusAbsPosNP.s == IPS_BUSY)
    {
        uint8_t res[1] = {0};
        bool rc = hid_read_timeout(hidHandle, res, 1, 500);
        if (rc)
        {
            if (res[0] == RES_FINISHED)
            {
                uint32_t steps=0;
                rc = sendCommand(GET_STEP_POS, 0, &steps);
                if (rc)
                {
                    FocusAbsPosN[0].value = steps;
                    FocusAbsPosNP.s = IPS_OK;
                    IDSetNumber(&FocusAbsPosNP, nullptr);
                    if (FocusRelPosNP.s == IPS_BUSY)
                    {
                        FocusRelPosNP.s = IPS_OK;
                        IDSetNumber(&FocusRelPosNP, nullptr);
                    }
                }
            }
            else if (res[0] == RES_ERROR)
            {
                LOG_ERROR("Failed to move. Check frequency settings or motor jam.");
                FocusAbsPosNP.s = IPS_ALERT;
                IDSetNumber(&FocusAbsPosNP, nullptr);
                if (FocusRelPosNP.s == IPS_BUSY)
                {
                    FocusRelPosNP.s = IPS_ALERT;
                    IDSetNumber(&FocusRelPosNP, nullptr);
                }

                return;
            }
        }

        SetTimer(POLLMS);
    }
}

bool HitecAstroDCFocuser::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // Focus IN/OUT
        // We process it in this driver because the action depends on the mode (step vs. cont)
        if (!strcmp(name, FocusMotionSP.name))
        {
            FocusDirection prevDirection = FocusMotionS[FOCUS_INWARD].s == ISS_ON ? FOCUS_INWARD : FOCUS_OUTWARD;
            IPState prevState = FocusMotionSP.s;

            IUUpdateSwitch(&FocusMotionSP, states, names, n);

            FocusDirection targetDirection = FocusMotionS[FOCUS_INWARD].s == ISS_ON ? FOCUS_INWARD : FOCUS_OUTWARD;

            // Continuous acts like a dumb DC motor
            if (MotionModeS[MODE_CONTINUOUS].s == ISS_ON)
            {
                // If we are reversing direction let's issue abort first.
                if (prevDirection != targetDirection && prevState == IPS_BUSY)
                    AbortFocuser();

                FocusMotionSP.s = MoveFocuser(targetDirection, 0, 0);
            }
            else
                FocusMotionSP.s = IPS_OK;

            IDSetSwitch(&FocusMotionSP, nullptr);
            return true;
        }

        // Reverse Direction
        if (!strcmp(name, ReverseDirectionSP.name))
        {
            int prevIndex = IUFindOnSwitchIndex(&ReverseDirectionSP);
            IUUpdateSwitch(&ReverseDirectionSP, states, names, n);

            bool rc = setDirection(ReverseDirectionS[REVERSE_ENABLED].s == ISS_ON ? REVERSE_ENABLED : REVERSE_DISABLED);
            if (rc)
                ReverseDirectionSP.s = IPS_OK;
            else
            {
                IUResetSwitch(&ReverseDirectionSP);
                ReverseDirectionS[prevIndex].s = ISS_ON;
                ReverseDirectionSP.s = IPS_ALERT;
            }

            IDSetSwitch(&ReverseDirectionSP, nullptr);
            return true;
        }

        // Braking
        if (!strcmp(name, BrakeSP.name))
        {
            int prevIndex = IUFindOnSwitchIndex(&BrakeSP);
            IUUpdateSwitch(&BrakeSP, states, names, n);

            bool rc = setBrake(BrakeS[BRAKE_LOW].s == ISS_ON ? BRAKE_LOW : BRAKE_HIGH);
            if (rc)
                BrakeSP.s = IPS_OK;
            else
            {
                IUResetSwitch(&BrakeSP);
                BrakeS[prevIndex].s = ISS_ON;
                BrakeSP.s = IPS_ALERT;
            }

            IDSetSwitch(&BrakeSP, nullptr);
            return true;
        }

        // Direction Mode
        if (!strcmp(name, MotionModeSP.name))
        {
            IUUpdateSwitch(&MotionModeSP, states, names, n);
            MotionModeSP.s = IPS_OK;
            IDSetSwitch(&MotionModeSP, nullptr);
            return true;
        }
    }
    return INDI::Focuser::ISNewSwitch(dev, name, states, names, n);
}

bool HitecAstroDCFocuser::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n)
{
    if (dev != nullptr && strcmp(dev, getDeviceName()) == 0)
    {
        // PWM settings
        if (strcmp(name, PWMNP.name) == 0)
        {
            double freq = PWMN[PWM_FREQ].value;
            double duty = PWMN[PWM_DUTY].value;

            IUUpdateNumber(&PWMNP, values, names, n);

            if (PWMN[PWM_FREQ].value != 400.0 ||
                PWMN[PWM_FREQ].value != 800.0 ||
                PWMN[PWM_FREQ].value != 1600.0 ||
                PWMN[PWM_FREQ].value != 3200.0 ||
                PWMN[PWM_FREQ].value != 6400.0)
            {
                LOG_ERROR("Invalid frequency requested. Valid values: 400, 800, 1600, 3200, 6400");
                PWMN[PWM_FREQ].value = freq;
                PWMN[PWM_DUTY].value = duty;
                PWMNP.s = IPS_ALERT;
                IDSetNumber(&PWMNP, nullptr);
                return true;
            }

            bool rc1 = setPWM(PWM_FREQ, PWMN[PWM_FREQ].value);
            bool rc2 = setPWM(PWM_DUTY, PWMN[PWM_DUTY].value);

            PWMNP.s = (rc1 && rc2) ? IPS_OK : IPS_ALERT;
            if (PWMNP.s == IPS_ALERT)
            {
                PWMN[PWM_FREQ].value = freq;
                PWMN[PWM_DUTY].value = duty;
            }

            IDSetNumber(&PWMNP, nullptr);
            return true;
        }

        // Max Travel
        if (strcmp(name, MaxTravelNP.name) == 0)
        {
            IUUpdateNumber(&MaxTravelNP, values, names, n);
            MaxTravelNP.s = IPS_OK;

            updateFocusRange();

            // If we're already connected, update the min/max values.
            if (isConnected())
            {
                IUUpdateMinMax(&FocusAbsPosNP);
                IUUpdateMinMax(&FocusRelPosNP);
            }

            IDSetNumber(&MaxTravelNP, nullptr);
            return true;
        }
    }
    return INDI::Focuser::ISNewNumber(dev, name, values, names, n);
}

IPState HitecAstroDCFocuser::MoveAbsFocuser(uint32_t ticks)
{
    bool rc = setSteps(ticks);
    if (rc)
       SetTimer(POLLMS);

    return (rc ? IPS_BUSY : IPS_ALERT);
}

IPState HitecAstroDCFocuser::MoveRelFocuser(FocusDirection dir, uint32_t ticks)
{
    int currentSteps = static_cast<int>(FocusAbsPosN[0].value);
    int targetTicks  = static_cast<int>(ticks);

    int targetSteps = currentSteps + ((dir == FOCUS_INWARD) ? (targetTicks*-1) : targetTicks);

    FocusAbsPosNP.s = MoveAbsFocuser(static_cast<uint32_t>(targetSteps));
    IDSetNumber(&FocusAbsPosNP, nullptr);

    return FocusAbsPosNP.s;
}

IPState HitecAstroDCFocuser::MoveFocuser(FocusDirection dir, int speed, uint16_t duration)
{
    INDI_UNUSED(speed);
    INDI_UNUSED(duration);

    bool rc = sendCommand(dir == FOCUS_INWARD ? START_INWARD : START_OUTWARD);

    return (rc ? IPS_BUSY : IPS_ALERT);
}

bool HitecAstroDCFocuser::sync(uint32_t steps)
{
    return sendCommand(SET_STEP_POS, steps);
}

bool HitecAstroDCFocuser::setSteps(uint32_t steps)
{
    double diff = static_cast<int>(steps) - static_cast<int>(FocusAbsPosN[0].value);

    return sendCommand(diff < 0 ? STEP_INWARD : STEP_OUTWARD, static_cast<uint32_t>(fabs(diff)));
}

bool HitecAstroDCFocuser::getSteps(uint32_t &steps)
{
    return sendCommand(GET_STEP_POS, 0, &steps);
}

bool HitecAstroDCFocuser::AbortFocuser()
{
    bool rc = false;

    if (FocusMotionS[FOCUS_INWARD].s == ISS_ON)
        rc = sendCommand(STOP_INWARD);
    else if (FocusMotionS[FOCUS_OUTWARD].s == ISS_ON)
        rc = sendCommand(STOP_OUTWARD);
    else
    {
        bool rc1 = sendCommand(STOP_INWARD);
        bool rc2 = sendCommand(STOP_OUTWARD);

        rc = rc1 && rc2;
    }

    return rc;
}

bool HitecAstroDCFocuser::getStartupData()
{
    double freq=0, duty=0;
    bool rc1 = getPWM(PWM_FREQ, freq);
    bool rc2 = getPWM(PWM_DUTY, duty);
    if (rc1 && rc2)
    {
        PWMNP.s = IPS_OK;
        PWMN[PWM_FREQ].value = freq;
        PWMN[PWM_DUTY].value = duty;
    }
    else
        PWMNP.s = IPS_ALERT;

    uint8_t dir = REVERSE_DISABLED;
    bool rc3 = getDirection(dir);
    if (rc3)
    {
        IUResetSwitch(&ReverseDirectionSP);
        ReverseDirectionS[dir].s = ISS_ON;
        ReverseDirectionSP.s = IPS_OK;
    }
    else
        ReverseDirectionSP.s = IPS_ALERT;

    uint8_t brake=0;
    bool rc4 = getBrake(brake);
    if (rc4)
    {
        BrakeSP.s = IPS_OK;
        IUResetSwitch(&BrakeSP);
        BrakeS[brake].s = ISS_ON;
    }
    else
        BrakeSP.s = IPS_ALERT;

    uint32_t steps=0;
    bool rc5 = getSteps(steps);
    if (rc5)
    {
        FocusAbsPosN[0].value = steps;
    }

    return (rc1 && rc2 && rc3 && rc4 && rc5);
}

bool HitecAstroDCFocuser::sendCommand(uint8_t command, uint32_t param, uint32_t *response)
{
    uint8_t cmd[65] = {0};
    uint8_t res[8] = {0};
    uint8_t resLen = 1;

    // Mandotary report byte
    cmd[0] = 0x0;

    // Actual command
    cmd[1] = command;

    switch (command)
    {
    case GET_MOTOR_BRAKE:
    case GET_MOTOR_REV:
    case GET_PWM_DUTY:
        resLen=3;
        break;

    case GET_STEP_POS:
    case GET_PWM_FREQ:
        resLen = 4;
        break;

    case SET_MOTOR_BRAKE:
        cmd[2] = (param == BRAKE_HIGH) ? 100 : 50;
        break;

    case SET_STEP_POS:
    case SET_PWM_FREQ:
    case STEP_INWARD:
    case STEP_OUTWARD:
        // Send in high-byte & low-byte order
        cmd[2] = (param & 0xFF00) >> 8;
        cmd[3] = (param & 0x00FF);
        break;

    case SET_MOTOR_REV:
        cmd[2] = (param == REVERSE_ENABLED) ? 1 : 0;
        break;

    case SET_PWM_DUTY:
    case START_INWARD:
    case START_OUTWARD:
        cmd[2] = static_cast<uint8_t>(param);
        break;

    default:
        LOGF_ERROR("Command %#02X is not recognized.", command);
        return false;
    }

    LOGF_DEBUG("CMD <%#02X %#02X %#02X>", cmd[1], cmd[2], cmd[3]);

    int rc = hid_write(hidHandle, cmd, 65);
    if (rc != 65)
    {
        LOGF_ERROR("Controller write error for command: %#02X (%s)", command, hid_error(hidHandle));
        return false;
    }

    // We're done if abort or step commands
    if (command == STOP_INWARD || command == STOP_OUTWARD || command == STEP_INWARD || command == STEP_OUTWARD)
        return true;

    rc = hid_read_timeout(hidHandle, res, resLen, HID_TIMEOUT);
    //rc = hid_read(hidHandle, res, resLen);
    if (rc <= 0)
    {
        if (rc == 0)
            LOGF_ERROR("Controller timed out for command: %#02X", command);
        else
            LOGF_ERROR("Controller read error for command: %#02X (%s)", command, hid_error(hidHandle));
        return false;
    }

    LOGF_DEBUG("RES <%#02X %#02X %#02X %#02X>", res[0], res[1], res[2], res[3]);

    // If no response required, then simply check the return value.
    if (response == nullptr)
    {
        return (res[0] == RES_SUCCESS);
    }

    switch (command)
    {
    case GET_MOTOR_BRAKE:
    case GET_MOTOR_REV:
    case GET_PWM_DUTY:
    case START_INWARD:
    case START_OUTWARD:
        *response = res[1];
        break;

    case GET_PWM_FREQ:
    case GET_STEP_POS:
        *response = res[1]*256 + res[2];
        break;

    default:
        LOGF_ERROR("Command %#02X is not recognized.", command);
        return false;
    }

    return true;
}

bool HitecAstroDCFocuser::setPWM(uint8_t param, double value)
{
    switch (param)
    {
        case PWM_FREQ:
            return sendCommand(SET_PWM_FREQ, static_cast<uint32_t>(value));

        case PWM_DUTY:
            return sendCommand(SET_PWM_DUTY, static_cast<uint32_t>(value));

        default:
            break;
    }

    return false;
}

bool HitecAstroDCFocuser::getPWM(uint8_t param, double &value)
{
    uint32_t response=0;
    bool rc = false;
    switch (param)
    {
        case PWM_FREQ:
            rc = sendCommand(GET_PWM_FREQ, 0, &response);
            break;

        case PWM_DUTY:
            rc = sendCommand(GET_PWM_DUTY, 0, &response);
        break;

        default:
            break;
    }

    value = response;
    return rc;
}

// Braking
bool HitecAstroDCFocuser::setBrake(uint8_t param)
{
    return sendCommand(SET_MOTOR_BRAKE, param);
}

bool HitecAstroDCFocuser::getBrake(uint8_t &value)
{
    uint32_t response=0;
    if (sendCommand(GET_MOTOR_BRAKE, 0, &response))
    {
        value = (response == 100) ? BRAKE_HIGH : BRAKE_LOW;
        return true;
    }

    return false;
}

// Direction
bool HitecAstroDCFocuser::setDirection(uint8_t param)
{
    return sendCommand(SET_MOTOR_REV, param);
}

bool HitecAstroDCFocuser::getDirection(uint8_t &value)
{
    uint32_t response=0;
    if (sendCommand(GET_MOTOR_REV, 0, &response))
    {
        value = (response == 0) ? REVERSE_DISABLED : REVERSE_ENABLED;
        return true;
    }

    return false;
}

void HitecAstroDCFocuser::updateFocusRange()
{
    FocusAbsPosN[0].min   = 0;
    FocusAbsPosN[0].max   = MaxTravelN[0].value;
    FocusAbsPosN[0].step  = MaxTravelN[0].value / 50.0;
    FocusAbsPosN[0].value = MaxTravelN[0].value / 2;

    FocusRelPosN[0].min   = 1;
    FocusRelPosN[0].max   = (FocusAbsPosN[0].max - FocusAbsPosN[0].min) / 2;
    FocusRelPosN[0].step  = FocusRelPosN[0].max / 100.0;
    FocusRelPosN[0].value = 100;
}

bool HitecAstroDCFocuser::saveConfigItems(FILE *fp)
{
    INDI::Focuser::saveConfigItems(fp);

    IUSaveConfigNumber(fp, &MaxTravelNP);

    if (isConnected())
    {
        IUSaveConfigNumber(fp, &PWMNP);
        IUSaveConfigSwitch(fp, &MotionModeSP);
        IUSaveConfigSwitch(fp, &ReverseDirectionSP);
        IUSaveConfigSwitch(fp, &BrakeSP);
    }

    return true;
}

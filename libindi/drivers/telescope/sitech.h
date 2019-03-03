/*******************************************************************************
 Copyright(c) 2019 Jasem Mutlaq. All rights reserved.

 SiTech Driver.

 The driver communicates with SiTechExe Application via TCP socket.

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

#include "indibase/indiguiderinterface.h"
#include "indibase/inditelescope.h"

#include <map>

class SiTech : public INDI::Telescope, public INDI::GuiderInterface
{
public:
    SiTech();

    virtual const char *getDefaultName() override;
    virtual bool initProperties() override;
    virtual bool updateProperties() override;

    virtual bool ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n) override;
    virtual bool ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n) override;

    protected:

    virtual bool Handshake() override;
    virtual bool ReadScopeStatus() override;

    ///////////////////////////////////////////////////////////////////////////////
    /// Motion Commands
    ///////////////////////////////////////////////////////////////////////////////
    virtual bool MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command) override;
    virtual bool MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command) override;
    virtual bool Abort() override;

    ///////////////////////////////////////////////////////////////////////////////
    /// GOTO Commands
    ///////////////////////////////////////////////////////////////////////////////
    virtual bool Goto(double,double) override;
    virtual bool Park() override;
    virtual bool UnPark() override;
    virtual bool Sync(double ra, double dec) override;

    ///////////////////////////////////////////////////////////////////////////////
    /// Guiding Commands
    ///////////////////////////////////////////////////////////////////////////////
    virtual IPState GuideNorth(uint32_t ms) override;
    virtual IPState GuideSouth(uint32_t ms) override;
    virtual IPState GuideEast(uint32_t ms) override;
    virtual IPState GuideWest(uint32_t ms) override;

    private:
    bool setTracking(bool enable, bool isSidereal, double raRate, double deRate);

    ///////////////////////////////////////////////////////////////////////////////
    /// Utility Functions
    ///////////////////////////////////////////////////////////////////////////////
    bool parseStatusResponse(const char *response);
    bool sendCommand(const char * cmd, char * res = nullptr, int cmd_len = -1, int res_len = -1);
    bool getStartupValues();
    void hexDump(char * buf, const char * data, int size);

    typedef enum
    {
        ST_INITIALIZED          = 1 << 0,   ///< Bit 00 (AND with    1) Scope Is Initialized
        ST_TRACKING             = 1 << 1,   ///< Bit 01 (AND with    2) Scope Is Tracking (remains true when slewing)
        ST_SLEWING              = 1 << 2,   ///< Bit 02 (AND with    4) Scope is Slewing
        ST_PARKING              = 1 << 3,   ///< Bit 03 (AND with    8) Scope is Parking
        ST_PARKED               = 1 << 4,   ///< Bit 04 (AND with   16) Scope is Parked
        ST_LOOKING_EAST         = 1 << 5,   ///< Bit 05 (AND with   32) Scope is "Looking East" (GEM mount);
        ST_BLINKY               = 1 << 6,   ///< Bit 06 (AND with   64) ServoController is in "Blinky" (Manual) mode, one or both axis's
        ST_COM_FAULT            = 1 << 7,   ///< Bit 07 (AND with  128) There is a communication fault between SiTechExe and the ServoController
        ST_LIMIT_PRIMARY_PLUS   = 1 << 8,   ///< Bit 08 (AND with  256) Limit Swith is activated (Primary Plus) (ServoII and Brushless)
        ST_LIMIT_PRIMARY_MINUS  = 1 << 9,   ///< Bit 09 (AND with  512) Limit Swith is activated (Primary Minus) (ServoII and Brushless)
        ST_LIMIT_SECONDARY_PLUS = 1 << 10,  ///< Bit 10 (AND with 1024) Limit Swith is activated (Secondary Plus) (ServoII and Brushless)
        ST_LIMIT_SECONDARY_MINUS= 1 << 11,  ///< Bit 11 (AND with 2048) Limit Swith is activated (Secondary Minus) (ServoII and Brushless)
        ST_HOMING_PRIMARY       = 1 << 12,  ///< Bit 12 (AND with 4096) Homing Switch Primary Axis is activated
        ST_HOMING_SECONDARY     = 1 << 13,  ///< Bit 13 (AND with 8192) Homing Switch Secondary Axis is activated
        ST_ROTATOR_GOTO         = 1 << 14,  ///< Bit 14 (AND with 16384) GoTo Commanded Rotator Position (if this is a rotator response)
        ST_TRACKING_OFFSET      = 1 << 15,  ///< Bit 15 (AND with 32768) Tracking at Offset Rate of some kind (non-sidereal)
    } StatusBits;

    std::map<StatusBits, bool> m_Status;

    /////////////////////////////////////////////////////////////////////////////
    /// Properties
    /////////////////////////////////////////////////////////////////////////////
    INumber GuideRateN[2];
    INumberVectorProperty GuideRateNP;

    /////////////////////////////////////////////////////////////////////////////
    /// Static Helper Values
    /////////////////////////////////////////////////////////////////////////////
    // LF is the stop char
    static const char DRIVER_STOP_CHAR { 0xA };
    // Wait up to a maximum of 3 seconds for serial input
    static constexpr const uint8_t DRIVER_TIMEOUT {3};
    // Maximum buffer for sending/receving.
    static constexpr const uint8_t DRIVER_LEN {64};

};

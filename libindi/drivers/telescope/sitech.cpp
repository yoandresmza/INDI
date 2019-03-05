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

#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <termio.h>
#include <cstring>
#include <memory>
#include <regex>

#include "sitech.h"
#include "indicom.h"

// Single instance of the driver class.
static std::unique_ptr<SiTech> sitech(new SiTech());

#define NAD -99999999.99

void ISGetProperties(const char *dev)
{
    sitech->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int num)
{
    sitech->ISNewSwitch(dev, name, states, names, num);
}

void ISNewText(	const char *dev, const char *name, char *texts[], char *names[], int num)
{
    sitech->ISNewText(dev, name, texts, names, num);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int num)
{
    sitech->ISNewNumber(dev, name, values, names, num);
}

void ISNewBLOB (const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n)
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
void ISSnoopDevice (XMLEle *root)
{
   sitech->ISSnoopDevice(root);
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
SiTech::SiTech()
{
    setVersion(0, 1);

    SetTelescopeCapability(TELESCOPE_CAN_PARK | TELESCOPE_CAN_SYNC | TELESCOPE_CAN_GOTO | TELESCOPE_CAN_ABORT,4);
    setTelescopeConnection(CONNECTION_TCP);

    m_Status[ST_INITIALIZED] = false;
    m_Status[ST_TRACKING] = false;
    m_Status[ST_SLEWING] = false;
    m_Status[ST_PARKING] = false;
    m_Status[ST_PARKED] = false;
    m_Status[ST_LOOKING_EAST] = false;
    m_Status[ST_BLINKY] = false;
    m_Status[ST_COM_FAULT] = false;
    m_Status[ST_LIMIT_PRIMARY_PLUS] = false;
    m_Status[ST_LIMIT_PRIMARY_MINUS] = false;
    m_Status[ST_LIMIT_SECONDARY_PLUS] = false;
    m_Status[ST_LIMIT_SECONDARY_MINUS] = false;
    m_Status[ST_HOMING_PRIMARY] = false;
    m_Status[ST_HOMING_SECONDARY] = false;
    m_Status[ST_ROTATOR_GOTO] = false;
    m_Status[ST_TRACKING_OFFSET] = false;

    m_Data.reserve(MOUNT_N);
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
const char * SiTech::getDefaultName()
{
    return "SiTech";
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::initProperties()
{
    INDI::Telescope::initProperties();

    // Guide Rate
    IUFillNumber(&GuideRateN[AXIS_RA], "GUIDE_RATE_WE", "W/E Rate", "%g", 0, 1, 0.1, 0.3);
    IUFillNumber(&GuideRateN[AXIS_DE], "GUIDE_RATE_NS", "N/S Rate", "%g", 0, 1, 0.1, 0.3);
    IUFillNumberVector(&GuideRateNP, GuideRateN, 2, getDeviceName(), "GUIDE_RATE", "Guiding Rate", MOTION_TAB, IP_RW, 0, IPS_IDLE);

    // Slew Rates
    IUFillSwitch(&SlewRateS[SLEW_GUIDE], "SLEW_GUIDE", "Guide", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_CENTERING], "SLEW_CENTERING", "Centering", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_FIND], "SLEW_FIND", "Find", ISS_OFF);
    IUFillSwitch(&SlewRateS[SLEW_MAX], "SLEW_MAX", "Max", ISS_ON);
    IUFillSwitchVector(&SlewRateSP, SlewRateS, 4, getDeviceName(), "TELESCOPE_SLEW_RATE", "Slew Rate", MOTION_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Tracking Mode
    IUFillSwitch(&TrackModeS[TRACK_SIDEREAL], "TRACK_SIDEREAL", "Sidereal", ISS_OFF);
    IUFillSwitch(&TrackModeS[TRACK_SOLAR], "TRACK_SOLAR", "Solar", ISS_OFF);
    IUFillSwitch(&TrackModeS[TRACK_LUNAR], "TRACK_LUNAR", "Lunar", ISS_OFF);
    IUFillSwitch(&TrackModeS[TRACK_CUSTOM], "TRACK_CUSTOM", "Custom", ISS_OFF);
    IUFillSwitchVector(&TrackModeSP, TrackModeS, 4, getDeviceName(), "TELESCOPE_TRACK_MODE", "Track Mode", MAIN_CONTROL_TAB, IP_RW, ISR_ATMOST1, 0, IPS_IDLE);

    // Sync Options
    IUFillSwitch(&SyncOptionS[SYNC_INIT], "SYNC_INIT", "Init", ISS_OFF);
    IUFillSwitch(&SyncOptionS[SYNC_OFFSET], "SYNC_OFFSET", "Offset", ISS_ON);
    IUFillSwitch(&SyncOptionS[SYNC_LOAD_CALIBRATION], "SYNC_LOAD_CALIBRATION", "Load Calibration", ISS_OFF);
    IUFillSwitchVector(&SyncOptionSP, SyncOptionS, 3, getDeviceName(), "TELESCOPE_SYNC_OPTIONS", "Sync Options", MAIN_CONTROL_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Park Options
    IUFillSwitch(&ParkGotoS[PARK_1], "PARK_1", "Park 1", ISS_ON);
    IUFillSwitch(&ParkGotoS[PARK_2], "PARK_2", "Park 2", ISS_OFF);
    IUFillSwitch(&ParkGotoS[PARK_3], "PARK_3", "Park 3", ISS_OFF);
    IUFillSwitchVector(&ParkGotoSP, ParkGotoS, 3, getDeviceName(), "TELESCOPE_PARK_GOTO", "Park Setting", SITE_TAB, IP_RW, ISR_1OFMANY, 0, IPS_IDLE);

    // Custom Tracking Rate
    IUFillNumber(&TrackRateN[0],"TRACK_RATE_RA","RA (arcsecs/s)","%.6f",-16384.0, 16384.0, 0.000001, 15.041067);
    IUFillNumber(&TrackRateN[1],"TRACK_RATE_DE","DE (arcsecs/s)","%.6f",-16384.0, 16384.0, 0.000001, 0);
    IUFillNumberVector(&TrackRateNP, TrackRateN,2,getDeviceName(),"TELESCOPE_TRACK_RATE","Track Rates", MAIN_CONTROL_TAB, IP_RW,60,IPS_IDLE);

    TrackState=SCOPE_IDLE;

    // TODO
    //enum TelescopeParkData  { PARK_NONE, PARK_RA_DEC, PARK_AZ_ALT, PARK_RA_DEC_ENCODER, PARK_AZ_ALT_ENCODER };
    //  SetParkDataType(PARK_RA_DEC);

    initGuiderProperties(getDeviceName(), MOTION_TAB);

    addDebugControl();
    setDriverInterface(getDriverInterface() | GUIDER_INTERFACE);
    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::updateProperties()
{
    INDI::Telescope::updateProperties();

    if (isConnected())
    {
        getStartupValues();

        defineSwitch(&SyncOptionSP);

        defineSwitch(&TrackModeSP);
        defineNumber(&TrackRateNP);

        defineSwitch(&ParkGotoSP);

        defineNumber(&GuideNSNP);
        defineNumber(&GuideWENP);
        defineNumber(&GuideRateNP);
    }
    else
    {
        deleteProperty(SyncOptionSP.name);

        deleteProperty(TrackModeSP.name);
        deleteProperty(TrackRateNP.name);

        deleteProperty(ParkGotoSP.name);

        deleteProperty(GuideNSNP.name);
        deleteProperty(GuideWENP.name);
        deleteProperty(GuideRateNP.name);
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::Handshake()
{
    char res[DRIVER_RES]={0};

    if (sendCommand("ReadScopeStatus", res) == false)
        return false;

    if (parseStatusResponse(res) == false)
        return false;

    if (m_Status[ST_COM_FAULT])
    {
        LOG_ERROR("No communication with Sitech Controller");
        return false;
    }

    if (m_Status[ST_BLINKY])
    {
        LOG_ERROR("Controller is in manual blinky mode.");
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
/* All numbers are separated by a ';'
     * First is an integer, with bits in it as follows:
     * (001) Bit  0 = true if initialized
     * (002) Bit  1 = true if tracking
     * (004) Bit  2 = true if slewing, or slew settling time
     * (008) Bit  3 = true if Parking
     * (016) Bit  4 = true if Parked
     * (032) Bit  5 = true if "looking east" (GEM scope)
     * (064) Bit  6 = true if either servo is in manual (blinky mode)
     * (128) Bit  7 = true if Comm to Controller is bad
     * (256) Bit  8 = true if Limit Switch + in Primary Axis Activated
     * (512) Bit  9 = true if Limit Switch - in Primary Axis Activated
     *(1024) Bit 10 = true if Limit Switch + in Secondary Axis Activated
     *(2048) Bit 11 = true if Limit Switch - in Secondary Axis Activated
     *(4096) Bit 12 = true if Home Switch Primary Axis Activated
     *(8192) Bit 13 = true if Home Switch Secondary Axis Activated
    *(16384) Bit 14 = true if HandPad Pan Mode
    *(32768) Bit 15 = true if HandPad Guide Mode
    * Next is RA (hours)
    * Next is Dec (Hours)
    * Next is Altitude (degs)
    * Next is Azimuth (degs)
    * Next is Primary axisPositionDegsPrimary
    * Next is Primary axisPositionDegsSecondary
    * Next is Scope Sidereal Time (hours).
    * Next is Scope Julian Day.
    * Next is ScopeTime (hours)
    * Next is a possible string message.
*/
///////////////////////////////////////////////////////////////////////////
bool SiTech::parseStatusResponse(const char *response)
{
    std::vector<std::string> result = split(response, ";");
    if (result.size() < 10)
    {
        LOG_WARN("Received wrong number of detailed mount data. Retrying...");
        return false;
    }

    if (result == m_LastMountResponse)
        return true;

    // Update Mount Status
    uint32_t mountStatus = std::stoi(result[0]);
    for (auto &x : m_Status)
    {
        x.second = (x.first & mountStatus);
    }

    // Update Mount Data
    m_Data[MOUNT_RA] = std::stod(result[1]);
    m_Data[MOUNT_DE] = std::stod(result[2]);
    m_Data[MOUNT_AT] = std::stod(result[3]);
    m_Data[MOUNT_AZ] = std::stod(result[4]);

    m_Data[MOUNT_AXIS_PRIMARY] = std::stod(result[5]);
    m_Data[MOUNT_AXIS_SECONDARY] = std::stod(result[6]);

    m_Data[MOUNT_LST] = std::stod(result[7]);
    m_Data[MOUNT_JD] = std::stod(result[8]);
    m_Data[MOUNT_LT] = std::stod(result[9]);

    // Update Mount Message, if any.
    if (result.size() > 10)
    {
        m_Message = result[10];
        LOGF_DEBUG("Mount message: %s", m_Message.c_str());
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::ReadScopeStatus()
{
    char res[DRIVER_RES]={0};

    if (sendCommand("ReadScopeStatus", res) == false)
        return false;

    if (parseStatusResponse(res) == false)
        return false;

    if (TrackState == SCOPE_SLEWING)
    {
        if (m_Status[ST_TRACKING])
        {
            TrackState = SCOPE_TRACKING;
            LOG_INFO("Slew is complete. Tracking...");
        }
    }
    else if (TrackState == SCOPE_PARKING)
    {
        if (m_Status[ST_PARKED])
        {
            SetParked(true);
        }
    }

    NewRaDec(m_Data[MOUNT_RA], m_Data[MOUNT_DE]);

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::Goto(double ra , double de)
{
    char cmd[DRIVER_CMD]={0}, res[DRIVER_RES]={0};
    snprintf(cmd, DRIVER_CMD, "Goto %f %f", ra, de);

    if (sendCommand(cmd, res) == false)
        return false;

    if (m_Message != "Accepted")
    {
        LOGF_ERROR("Goto failed: %s", m_Message.c_str());
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::Sync(double ra, double de)
{
    char cmd[DRIVER_CMD]={0}, res[DRIVER_RES]={0};
    snprintf(cmd, DRIVER_CMD, "Sync %f %f %d", ra, de, IUFindOnSwitchIndex(&SyncOptionSP));

    if (sendCommand(cmd, res) == false)
        return false;

    if (m_Message != "Accepted")
    {
        LOGF_ERROR("Sync failed: %s", m_Message.c_str());
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::Park()
{
    char cmd[DRIVER_CMD]={0}, res[DRIVER_RES]={0};
    snprintf(cmd, DRIVER_CMD, "Park %d", IUFindOnSwitchIndex(&ParkGotoSP));

    if (sendCommand(cmd, res) == false)
        return false;

    if (m_Message != "Park")
    {
        LOGF_ERROR("Park failed: %s", m_Message.c_str());
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::UnPark()
{    
    return false;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::setTracking(bool enable, bool isSidereal, double raRate, double deRate)
{    
    return false;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::ISNewNumber (const char *dev, const char *name, double values[], char *names[], int n)
{
    //  first check if it's for our device

    if(strcmp(dev,getDeviceName())==0)
    {
        // Tracking Rate
        if (!strcmp(name, TrackRateNP.name))
        {
            IUUpdateNumber(&TrackRateNP, values, names, n);
            if (IUFindOnSwitchIndex(&TrackModeSP) != TRACK_CUSTOM)
            {
                DEBUG(INDI::Logger::DBG_ERROR, "Can only set tracking rate if track mode is custom.");
                TrackRateNP.s = IPS_ALERT;
            }
            else
            {
                if (setTracking(true, false, TrackRateN[AXIS_RA].value, TrackRateN[AXIS_DE].value))
                    TrackRateNP.s = IPS_OK;
                else
                    TrackRateNP.s = IPS_ALERT;
                IDSetNumber(&TrackRateNP, nullptr);
            }

            IDSetNumber(&TrackRateNP, nullptr);
            return true;
        }

        // Guide Rate
        if(!strcmp(name,"GUIDE_RATE"))
        {
             IUUpdateNumber(&GuideRateNP, values, names, n);
             GuideRateNP.s = IPS_OK;
             IDSetNumber(&GuideRateNP, nullptr);
             return true;
        }

        // Guiding Pulse
        if (!strcmp(name,GuideNSNP.name) || !strcmp(name,GuideWENP.name))
        {
            processGuiderProperties(name, values, names, n);
            return true;
        }
    }

    return INDI::Telescope::ISNewNumber(dev,name,values,names,n);
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::ISNewSwitch (const char *dev, const char *name, ISState *states, char *names[], int n)
{
    if(strcmp(dev,getDeviceName())==0)
    {
        // Sync Options
        if (!strcmp(SyncOptionSP.name, name))
        {
            IUUpdateSwitch(&SyncOptionSP, states, names, n);
            SyncOptionSP.s = IPS_OK;
            IDSetSwitch(&SyncOptionSP, nullptr);
            return true;
        }

        // Park Goto Settings
        if (!strcmp(ParkGotoSP.name, name))
        {
            IUUpdateSwitch(&ParkGotoSP, states, names, n);
            ParkGotoSP.s = IPS_OK;
            IDSetSwitch(&ParkGotoSP, nullptr);
            return true;
        }

        // Tracking Mode
        if (!strcmp(TrackModeSP.name, name))
        {
            // TODO
            return true;
        }

        // Slew Rate
        if (!strcmp (name, SlewRateSP.name))
        {
          if (IUUpdateSwitch(&SlewRateSP, states, names, n) < 0)
              return false;
          SlewRateSP.s = IPS_OK;
          IDSetSwitch(&SlewRateSP, nullptr);
          return true;
        }
    }

    //  Nobody has claimed this, so, ignore it
    return INDI::Telescope::ISNewSwitch(dev,name,states,names,n);
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::Abort()
{    
    return false;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::MoveNS(INDI_DIR_NS dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::MoveWE(INDI_DIR_WE dir, TelescopeMotionCommand command)
{
    if (TrackState == SCOPE_PARKED)
    {
        LOG_ERROR("Please unpark the mount before issuing any motion commands.");
        return false;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
IPState SiTech::GuideNorth(uint32_t ms)
{
    return IPS_BUSY;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
IPState SiTech::GuideSouth(uint32_t ms)
{
    return IPS_BUSY;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
IPState SiTech::GuideEast(uint32_t ms)
{
    return IPS_BUSY;

}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
IPState SiTech::GuideWest(uint32_t ms)
{
    return IPS_BUSY;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::getStartupValues()
{
    return false;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::sendCommand(const char * cmd, char * res, int cmd_len, int res_len)
{
    int nbytes_written = 0, nbytes_read = 0, rc = -1;
    char fullCommand[DRIVER_RES]={0};

    tcflush(PortFD, TCIOFLUSH);

    if (cmd_len > 0)
    {
        char hex_cmd[DRIVER_RES * 3] = {0};
        hexDump(hex_cmd, cmd, cmd_len);
        LOGF_DEBUG("CMD <%s>", hex_cmd);
        memcpy(fullCommand, cmd, cmd_len);
        // Append /r/n
        fullCommand[cmd_len+1] = 0xD;
        fullCommand[cmd_len+2] = 0xA;
        rc = tty_write(PortFD, fullCommand, cmd_len+2, &nbytes_written);
    }
    else
    {
        LOGF_DEBUG("CMD <%s>", cmd);
        cmd_len = static_cast<int>(strlen(cmd));
        memcpy(fullCommand, cmd, cmd_len);
        // Append /r/n
        fullCommand[cmd_len+1] = 0xD;
        fullCommand[cmd_len+2] = 0xA;
        rc = tty_write(PortFD, fullCommand, cmd_len+2, &nbytes_written);
    }

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial write error: %s.", errstr);
        return false;
    }

    if (res == nullptr)
        return true;

    if (res_len > 0)
        rc = tty_read(PortFD, res, res_len, DRIVER_TIMEOUT, &nbytes_read);
    else
        rc = tty_nread_section(PortFD, res, DRIVER_RES, DRIVER_STOP_CHAR, DRIVER_TIMEOUT, &nbytes_read);

    if (rc != TTY_OK)
    {
        char errstr[MAXRBUF] = {0};
        tty_error_msg(rc, errstr, MAXRBUF);
        LOGF_ERROR("Serial read error: %s.", errstr);
        return false;
    }

    // Remove Extra \n
    res[nbytes_read - 1] = 0;

    if (res_len > 0)
    {
        char hex_res[DRIVER_RES * 3] = {0};
        hexDump(hex_res, res, res_len);
        LOGF_DEBUG("RES <%s>", hex_res);
    }
    else
    {
        LOGF_DEBUG("RES <%s>", res);
    }

    tcflush(PortFD, TCIOFLUSH);

    return true;
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
void SiTech::hexDump(char * buf, const char * data, int size)
{
    for (int i = 0; i < size; i++)
        sprintf(buf + 3 * i, "%02X ", static_cast<uint8_t>(data[i]));

    if (size > 0)
        buf[3 * size - 1] = '\0';
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
std::vector<std::string> SiTech::split(const std::string &input, const std::string &regex)
{
    // passing -1 as the submatch index parameter performs splitting
    std::regex re(regex);
    std::sregex_token_iterator
    first{input.begin(), input.end(), re, -1},
          last;
    return {first, last};
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
bool SiTech::saveConfigItems(FILE *fp)
{
    INDI::Telescope::saveConfigItems(fp);

    IUSaveConfigSwitch(fp, &SyncOptionSP);
    IUSaveConfigSwitch(fp, &ParkGotoSP);

    return true;
}

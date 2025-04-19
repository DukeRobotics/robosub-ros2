/** @file PDD_Helper.c
 *  @brief PD helper functions implementation
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#include "PDD_Helper.h"

#ifdef __cplusplus
namespace tdym
{
#endif

/** @brief User configurable value to mark invalid data (default 99999).  Set it to NAN, INF, 99999, -32768, etc.
 */
static double PDD_INVALID_VALUE = 99999;

/** @brief Internal scale factor to convert "DECI" to base SI units, for example decimeters to meters
*/
#define PDD_TENTH (0.1)
/** @brief Internal scale factor to convert "CENTI" to base SI units, for example centimeters to meters
*/
#define PDD_HUNDREDTH (0.01)

/** @brief Internal scale factor to convert "MILLI" to base SI units, for example millimeters to meters
*/
#define PDD_THOUSANDTH (0.001)

/** @brief Internal scale factor to convert hundreds of seconds to nanoseconds
*/
#define PDD_HS_TO_NANO (1000000L)

/** @brief Internal value for Narrow band mode
*/
#define PDD_NB_MODE (10)

/** @brief Set the number user wants to mark invalid data in helper functions
*/
PDD_DECL void PDD_SetInvalidValue(double invalid)
{
    PDD_INVALID_VALUE = invalid;
}

/** @brief Internal function to get water profile index as they can get swapped
* 
*   @returns Ping index for water profiles or index to first valid fixed leader for other ping types
*/
static int GetWPIndex(const PDD_PD0Ensemble* pEns, e_PDPingType ping)
{
    if (ping >= 0 && ping < PDD_NumPingTypes)
    {
        int index;
        for (index = 0; index < 2; index++)
        {
            PDD_FixedLeader* vl = pEns->fixedLeader[index];
            if (vl)
            {
                int mode = vl->mode;
                switch (ping)
                {
                case PDBB:
                    if (mode != PDD_NB_MODE)
                    {
                        return index;
                    }
                    break;
                case PDNB:
                    if (mode == PDD_NB_MODE)
                    {
                        return index;
                    }
                    break;
                default:
                    // For other pings point to valid leader
                    return index;
                }
            }
        }
    }
    return -1;
}

/** @brief Internal function to decode beam angle in degrees form configuration
*/
static double GetBeamAngle(unsigned short config, unsigned char fwVersion)
{
    double angle;
    switch (config & ANGLE_MASK)
    {
    case ANGLE_15DEG:
        angle = 15.0;
        break;

    case ANGLE_20DEG:
        angle = 20.0;
        break;

    case ANGLE_30DEG:
        angle = 30.0;
        break;
    default:
        angle = ((fwVersion == WH_HORIZONTAL_11) || (fwVersion == SENTINELV_66)) ? 25.0 : 40.0;
    }
    return angle;
}

/** @brief Internal function to decode exact ADCP frequency in kHz from configuration
*/
static double GetADCPFrequency(unsigned short config, unsigned char fwVersion)
{
    double freq;
    int index = config & FREQ_MASK;
    freq = 0;

    switch (fwVersion)
    {
    case STREAMPRO_31: // StreamPro
        freq = 2000;
        break;
    case PINNACLE_61: // Pinnacle
        freq = 44;
        break;
    case SENTINELV_47:
    case SENTINELV_66: // SentinelV
        if (index >= 0 && index < 7)
        {
            double f[7] = { 38.4, 76.8, 153.6, 307.2, 491.52, 983.04, 2457.6 };
            freq = f[index];
        }
        break;
    default:
        if (index >= 0 && index < 7)
        {
            double f[7] = { 38.4, 76.8, 153.6, 307.2, 614.4, 1228.8, 2457.6 };
            freq = f[index];
        }
        break;
    }

    return freq;
}

/** @brief Get ADCP information
*
*   @param pEns Pointer to ensemble structure
*   @param pInfo Pointer to info structure
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetADCPInfo(const PDD_PD0Ensemble* pEns, PDD_ADCPInfo* pInfo)
{
    // Get valid index to fixed leader
    int pingIndex = GetWPIndex(pEns, PDBT);
    if (pingIndex >= 0 && pInfo)
    {
        PDD_FixedLeader* fl = pEns->fixedLeader[pingIndex];
        if (fl > 0)
        {
            pInfo->serialNumber = fl->serialNumber;
            pInfo->firmwareVersion = fl->fwVersion + fl->fwRevision * PDD_HUNDREDTH;
            pInfo->frequency = GetADCPFrequency(fl->config, fl->fwVersion);
            pInfo->beamAngle = GetBeamAngle(fl->config, fl->fwVersion);
            pInfo->beamsConvex = (fl->config & PDD_CFG_CONVEX_MASK) ? 1 : 0;
            pInfo->tiltConfig = (fl->config && PDD_CFG_TILT_CFG_MASK) >> PDD_CFG_TILT_CFG_LSB;
            return 1;
        }
    }
    return 0;
}

/** @brief Get ping setup
* 
*   @param pEns Pointer to ensemble structure
*   @param pSetup Pointer to setup structure
*   @param ping Type of ping
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetPingSetup(const PDD_PD0Ensemble* pEns, PDD_PingSetup* pSetup, e_PDPingType ping)
{
    PDD_FixedLeader* fl;
    int frame;
    int pingIndex = GetWPIndex(pEns, ping);
    fl = pEns->fixedLeader[pingIndex];
    if (pSetup == NULL || fl == NULL)
    {
        return 0;
    }
    memset(pSetup, 0, sizeof(PDD_PingSetup));
    frame = (fl->xform & PDD_XFORM_COORD_MASK) >> PDD_XFORM_COORD_LSB;
    pSetup->frame = frame;
    switch (ping)
    {
    case PDBT:
        {
            PDD_BottomTrack* bt = pEns->bottomTrack;
            if (bt)
            {
                 pSetup->mode = bt->mode;
                pSetup->pingCount = bt->pingCount;
                return 1;
            }
        }
        break;
    case PDVP:
    case PDVR:
        {
            PDD_VertBeamLeader* vbl = pEns->vbLeader;
            if (vbl)
            {
                pSetup->mode = vbl->mode;
                pSetup->cellCount = vbl->cellCount;
                pSetup->pingCount = vbl->pingCount;
                pSetup->cellSize = vbl->cellSize * PDD_HUNDREDTH;
                pSetup->blank = 0;
                pSetup->firstCellRange = vbl->firstCellRange * PDD_HUNDREDTH;
                pSetup->transmit = vbl->transmit * PDD_HUNDREDTH;
                pSetup->lag = vbl->lag * PDD_HUNDREDTH;
                return 1;
            }
        }
        break;
    default:

        if (pingIndex >= 0)
        {
            PDD_FixedLeader* fl = pEns->fixedLeader[pingIndex];
            if (fl)
            {
                pSetup->mode = fl->mode;
                pSetup->cellCount = fl->cellCount;
                pSetup->pingCount = fl->pingCount;
                pSetup->cellSize = fl->cellSize * PDD_HUNDREDTH;
                pSetup->blank = fl->blank * PDD_HUNDREDTH;
                pSetup->firstCellRange = fl->firstCellRange * PDD_HUNDREDTH;
                pSetup->transmit = fl->transmit * PDD_HUNDREDTH;
                pSetup->lag = fl->lag * PDD_HUNDREDTH;
                return 1;
            }
        }
        break;
    }
    return 0;
}

/** @brief Get ensemble number
* 
*   @param pEns Pointer to ensemble structure
*   @param ping Type of ping - use PDBB for broad band of default, PDNB for narrow band ping (only valid for certain setups)
*
*   @returns Ensemble number
*/
PDD_DECL unsigned int PDD_GetEnsembleNumber(const PDD_PD0Ensemble* pEns, e_PDPingType ping)
{
    int pingIndex = GetWPIndex(pEns, ping);
    if (pingIndex >= 0)
    {
        PDD_VariableLeader* vl = pEns->varLeader[pingIndex];
        if (vl)
        {
            return (vl->ensNumberLsb & 0x0FFFF) + (vl->ensNumberMsb << 16);;
        }
    }

    return 0;
}

/** @brief Get ensemble time stamp
* 
*   @param pEns Pointer to ensemble structure
*   @param ping Type of ping - use PDBB for broad band of default, PDNB for narrow band ping (only valid for certain setups)
*
*   @returns timespec structure
*/
PDD_DECL struct timespec PDD_GetTimestamp(const PDD_PD0Ensemble* pEns, e_PDPingType ping)
{
    struct timespec timestamp;
    struct tm tm;
    int pingIndex = GetWPIndex(pEns, ping);

    memset(&timestamp, 0, sizeof(timestamp));
    memset(&tm, 0, sizeof(tm));

    if (pingIndex >= 0 && pingIndex < 2)
    {
        PDD_VariableLeader* vl = pEns->varLeader[pingIndex];
        if (vl && vl->ensTime.month != 0)
        {
            int year;
            time_t result;
            year = vl->ensTime.year;
            year += (year >= 70 ? 1900 : 2000);
            if (year >= 2038)
            {
                year = 1970; // to mark it bad for now
            }
            tm.tm_year = year - 1900;
            tm.tm_mon = vl->ensTime.month - 1;
            tm.tm_mday = vl->ensTime.day;
            tm.tm_hour = vl->ensTime.hour;
            tm.tm_min = vl->ensTime.minute;
            tm.tm_sec = vl->ensTime.second;
            result = mktime(&tm);
            if (result > 0)
            {
                timestamp.tv_sec = result;
                timestamp.tv_nsec = vl->ensTime.sec100 * PDD_HS_TO_NANO;
            }
        }
    }

    return timestamp;
}

/** @brief Get heading, pitch, roll from ping attitude sample structure
*/
PDD_DECL void GetHPRFromSample(PDD_HPRSensor* hpr, PDD_PingAttitudeSample* pSample)
{
    hpr->heading = pSample->attitude[0] * PDD_THOUSANDTH;
    hpr->pitch = pSample->attitude[1] * PDD_THOUSANDTH;
    hpr->roll = pSample->attitude[2] * PDD_THOUSANDTH;
    hpr->headingRate = pSample->rate[0] * PDD_THOUSANDTH;
    hpr->pitchRate = pSample->rate[1] * PDD_THOUSANDTH;
    hpr->rollRate = pSample->rate[2] * PDD_THOUSANDTH;
}

/** @brief Get heading, pitch, roll structure (from variable leader of ping attitude structure)
* 
*   @param pEns Pointer to ensemble structure
*   @param pHpr Pointer to compass data structure
*   @param ping Type of ping
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetHPRSensor(const PDD_PD0Ensemble* pEns, PDD_HPRSensor* pHpr, e_PDPingType ping)
{
    enum e_PingAttitudePingType{NB=1, BB, BT};
    int pingIndex;
    PDD_PingAttitude* pingAttitude;
    if (pHpr == NULL)
    {
        return 0;
    }
    memset(pHpr, 0, sizeof(PDD_HPRSensor));
    pingAttitude = pEns->pingAttitude;
    if (pingAttitude)
    {
        int i;

        for (i = 0; i < pingAttitude->count; i++)
        {
            PDD_PingAttitudeSample* sample = &pingAttitude->samples[i];
            switch (sample->id)
            {
            case NB: // NB
                if (ping == PDNB && sample->valid)
                {
                    GetHPRFromSample(pHpr, sample);
                    return 1;
                }
                break;
            case BB: // BB
                if (ping == PDBB && sample->valid)
                {
                    GetHPRFromSample(pHpr, sample);
                    return 1;
                }
                break;
            case BT: // BT
                if (ping == PDBT && sample->valid)
                {
                    GetHPRFromSample(pHpr, sample);
                    return 1;
                }
                break;
            }
        }
    }

    pingIndex = GetWPIndex(pEns, ping);
    if (pingIndex >= 0 && pingIndex < 2)
    {
        PDD_VariableLeader* vl = pEns->varLeader[pingIndex];
        if (vl)
        {
            pHpr->heading = vl->heading * PDD_HUNDREDTH;
            pHpr->pitch = vl->pitch * PDD_HUNDREDTH;
            pHpr->roll = vl->roll * PDD_HUNDREDTH;
            return 1;
        }
    }
    return 0;
}

/** @brief Internal function to get voltage offset
*/
PDD_DECL double GetVoltageOffset(unsigned char fwVersion)
{
    double offset = 0;
    switch (fwVersion)
    {
    case RIVERRAY_44: // RiverRay
        offset = 0;
        break;
    case PINNACLE_61: // Pinnacle
        offset = 24;
        break;
    default:
        offset = 0;
        break;
    }
    return offset;
}

/** @brief Internal function to get voltage scale factor
*/
PDD_DECL double GetVoltageScaleFactor(unsigned char fwVersion)
{
    double scale = PDD_TENTH;
    if (fwVersion == TASMAN_74 || fwVersion == PATHFINDER_67)
    {
        // Some firmware versions just round voltage value
        scale = 1;
    }

    return scale;
}

/** @brief Get sensors
* 
*   @param pEns Pointer to ensemble structure
*   @param pSensors Pointer to sensors structure
*   @param ping Type of ping - use PDBB for broad band of default, PDNB for narrow band ping
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetSensors(const PDD_PD0Ensemble* pEns, PDD_Sensors* pSensors, e_PDPingType ping)
{
    int pingIndex = GetWPIndex(pEns, ping);
    if (pSensors == NULL)
    {
        return 0;
    }
    memset(pSensors, 0, sizeof(PDD_Sensors));
    if (pingIndex >= 0 && pingIndex < 2)
    {
        PDD_FixedLeader* fl = pEns->fixedLeader[pingIndex];
        PDD_VariableLeader* vl = pEns->varLeader[pingIndex];
        if (vl && fl)
        {
            static int VL_WITHOUT_HEM = 65;
            pSensors->temperature = vl->temperature * PDD_HUNDREDTH;
            pSensors->pressure = vl->pressure * PDD_HUNDREDTH;
            pSensors->salinity = vl->salinity;
            pSensors->depth = vl->depth * PDD_TENTH;
            pSensors->speedOfSound = vl->speedOfSound;
            pSensors->voltage = vl->adc[1] * GetVoltageScaleFactor(fl->fwVersion) + GetVoltageOffset(fl->fwVersion);
            if (fl->fwVersion == CHANNELMASTER_28 || fl->fwVersion == WH_HORIZONTAL_11)
            {
                pSensors->orientation = PDD_HORIZONTAL;
            }
            else
            {
                pSensors->orientation = (fl->config & PDD_CFG_UP_MASK) ? PDD_UP : PDD_DOWN;
            }
            pSensors->hemStatus = (pEns->varLeaderSize[pingIndex] > VL_WITHOUT_HEM) ? vl->hem.hemStatus : -1;
            return 1;
        }
    }
    return 0;
}

/** @brief Internal function to get vector of velocities in m/s from array of short
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
static int InternalGetVelFromShort(short* source, int sourceCount, double* destination, int destCount, double scaleFactor)
{
    if (source == NULL)
    {
        sourceCount = 0;
    }
    if (destination)
    {
        int i;
        for (i = 0; i < destCount; i++)
        {
            if (i < sourceCount)
            {
                double vel = source[i];
                destination[i] = (vel == PD0_INVALID_VALUE) ? PDD_INVALID_VALUE : scaleFactor * (vel * PDD_THOUSANDTH);
            }
            else
            {
                destination[i] = PDD_INVALID_VALUE;
            }
        }
        return sourceCount > 0 ? 1 : 0;
    }
    return 0;
}

/** @brief Internal function to get vector of velocities in m/s from array of int
*
*   @returns 1 if successful (data are present), 0 otherwise
*/
static int InternalGetVelFromInt(int* source, int sourceCount, double* destination, int destCount, double scaleFactor)
{
    if (source == NULL)
    {
        sourceCount = 0;
    }
    if (destination)
    {
        int i;
        for (i = 0; i < destCount; i++)
        {
            if (i < sourceCount)
            {
                int vel = source[i];
                destination[i] = (vel == PD0_HI_RES_INVALID_VALUE) ? 
                    PDD_INVALID_VALUE : scaleFactor * (vel * PDD_THOUSANDTH * PDD_HUNDREDTH);
            }
            else
            {
                destination[i] = PDD_INVALID_VALUE;
            }
        }
        return sourceCount > 0 ? 1 : 0;
    }
    return 0;
}


/** @brief Get vector of profile velocities in m/s (frame depends on coordinate system)
*
*   @param pEns Pointer to ensemble structure
*   @param vel Vector of velocity components in m/s, PDD_SetInvalidValue sets data invalid value
*   @param size Velocities vector size
*   @param ping Type of ping - use PDBB for broad band of default, PDNB for narrow band ping (only valid for certain setups)
* 
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetProfileVelocities(const PDD_PD0Ensemble* pEns, double* vel, int size, e_PDPingType ping)
{
    PDD_ShortArray* velocity;
    int pingIndex = GetWPIndex(pEns, ping);
    if (ping == PDVP)
    {
        pingIndex = 2;
    }
    // Only for profiling pings
    if (pingIndex >= 0 && pingIndex < 2)
    {
        PDD_FixedLeader* fl;
        fl = pEns->fixedLeader[pingIndex];
        velocity = pEns->velocity[pingIndex];

        if (fl)
        {
            int count = fl->cellCount * FOUR_BEAMS;
            return InternalGetVelFromShort(velocity == NULL ? NULL : velocity->data, 
                count, vel, size, 1);
        }
    }
    else if (pingIndex == 2)
    {
        PDD_VertBeamLeader* vbLeader = pEns->vbLeader;
        velocity = pEns->velocity[pingIndex];
        if (vbLeader)
        {
            int count = vbLeader->cellCount;
            return InternalGetVelFromShort(velocity == NULL ? NULL : velocity->data, 
                count, vel, size, 1);
        }
    }
    return 0;
}

/** @brief Get vector of vessel velocities in m/s (frame depends on coordinate system)
*
*   @param pEns Pointer to ensemble structure
*   @param velocities Vector of 4 velocity components in m/s, PDD_SetInvalidValue sets data invalid value
* 
*   @returns 1 if successful (data are present), 0 otherwise
*/
PDD_DECL int PDD_GetVesselVelocities(const PDD_PD0Ensemble* pEns, double velocities[4])
{
    PDD_BTHighResVelocity* hiResVelocity;
    PDD_BottomTrack* bt = pEns->bottomTrack;
    hiResVelocity = pEns->btHiResVel;
    if (hiResVelocity)
    {
        // High resolution velocity agree with vessel movement (scale = 1)
        return InternalGetVelFromInt(hiResVelocity->velocity, 4, velocities, 4, 1);
    }

    // Regular bottom track velocity is opposite to vessel movement (scale = -1)
    return InternalGetVelFromShort(bt == NULL ? NULL : bt->velocity, 4, velocities, 4, -1);
}

/** @brief Get range to bottom in meters
*
*   @param pEns Pointer to ensemble structure
*   @param beamRange Vector of 4 beam ranges to bottom in meters, 0 meters = data are invalid
* 
*   @returns average range to bottom if successful (3 beams valid), 0 otherwise
*/
PDD_DECL double PDD_GetRangeToBottom(const PDD_PD0Ensemble* pEns, double beamRange[4])
{
    int i;
    PDD_BottomTrackRange* btRange;
    PDD_BottomTrack* bt = pEns->bottomTrack;
    btRange = pEns->btRange;
    memset(beamRange, 0, sizeof(beamRange));
    if (btRange)
    {
        for (i = 0; i < FOUR_BEAMS; i++)
        {
            beamRange[i] = btRange->rawRange[i] * PDD_TENTH * PDD_THOUSANDTH;
        }
        return btRange->verticalRange * PDD_TENTH * PDD_THOUSANDTH;
    }

    if (bt)
    {
        double br;
        double sum = 0;
        int good = 0;
        // Minimum number of good beams to calculate average range
        static int MIN_GOOD = 3;
        for (i = 0; i < FOUR_BEAMS; i++)
        {
            int r = bt->range[i];
            // Check if decoded structure is big enough
            if (pEns->bottomTrackSize >= 81)
            {
                r += ((int)bt->rangeMsb[i]) * 65536;
            }

            br = r * PDD_HUNDREDTH;
            if (r > 0)
            {
                good++;
                sum += br;
            }
            beamRange[i] = br;

        }
        return good >= MIN_GOOD ? sum / good : 0.0;
    }
    return 0.0;
}

#ifdef __cplusplus
}
#endif
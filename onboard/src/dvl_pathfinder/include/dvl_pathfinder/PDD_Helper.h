/** @file PDD_Helper.h
 *  @brief Helper functions to extract data from ensembles
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#ifndef PDD_HELPER_H
#define PDD_HELPER_H

#include "PDD_Definitions.h"
#include "PDD_Ensemble.h"
#include <time.h>

#pragma pack(push, 1)

#ifdef __cplusplus
extern "C"
{
    namespace tdym
    {
#endif

        /** @brief Structure that contains ADCP information
        */
        typedef struct PDD_ADCPInfo
        {
            double firmwareVersion;         //!< Firmware version
            double frequency;               //!< System frequency in kHz
            double beamAngle;               //!< Beam angle in degrees
            int serialNumber;               //!< ADCP Serial number
            int beamsConvex;                //!< Beams geometry
            int tiltConfig;                 //!< Tilt configuration

        } PDD_ADCPInfo;                     // ADCP information structure

        /** @brief Structure that contains Ping setup information
         */
        typedef struct PDD_PingSetup
        {
            int frame;                      //!< Coordinate frame - e_CoordinateType
            int mode;                       //!< Ping mode
            int cellCount;                  //!< Number of depth cells
            int pingCount;                  //!< Number of pings per ensemble
            double cellSize;                //!< Cell size in meters
            double blank;                   //!< Blank after transmit in meters
            double firstCellRange;          //!< First call range in meters
            double transmit;                //!< Transmit length in meters
            double lag;                     //!< Lag length in meters

        } PDD_PingSetup;                    // Ping setup information structure

        /** @brief Compass heading, pitch, and roll structure
         */
        typedef struct PDD_HPRSensor
        {
            double heading;                 //!< Heading in deg
            double pitch;                   //!< Pitch in deg
            double roll;                    //!< Roll in deg
            double headingRate;             //!< Heading in deg/seconds
            double pitchRate;               //!< Pitch in deg/seconds
            double rollRate;                //!< Roll in deg/seconds

        }  PDD_HPRSensor;                   // Compass HPR structure

        /** @brief Other sensors structure
         */
        typedef struct PDD_Sensors
        {
            double temperature;             //!< Temperature at transducer in deg C
            double pressure;                //!< Pressure at transducer in kPa
            double depth;                   //!< Depth of the transducer in meters
            double voltage;                 //!< Voltage in V
            double salinity;                //!< Salinity
            double speedOfSound;            //!< Speed of sound in m/s
            int orientation;                //!< Orientation
            int hemStatus;                  //!< -1 not present, > 0 check bit flags

        }  PDD_Sensors;                     // Other sensors structure

        PDD_DECL void PDD_SetInvalidValue(double invalid);                                                              // Set invalid value
        PDD_DECL unsigned int PDD_GetEnsembleNumber(const PDD_PD0Ensemble* pEns, e_PDPingType ping);                    // Get ensemble number
        PDD_DECL struct timespec PDD_GetTimestamp(const PDD_PD0Ensemble* pEns, e_PDPingType ping);                      // Get ensemble time
        PDD_DECL int PDD_GetADCPInfo(const PDD_PD0Ensemble* pEns, PDD_ADCPInfo* pInfo);                                 // Get ADCP info
        PDD_DECL int PDD_GetPingSetup(const PDD_PD0Ensemble* pEns, PDD_PingSetup* pSetup, e_PDPingType ping);           // Get ping setup
        PDD_DECL int PDD_GetHPRSensor(const PDD_PD0Ensemble* pEns, PDD_HPRSensor* pHpr, e_PDPingType ping);             // Get compass data in degrees
        PDD_DECL int PDD_GetSensors(const PDD_PD0Ensemble* pEns, PDD_Sensors* pSensors, e_PDPingType ping);             // Get sensor data in SI units
        PDD_DECL int PDD_GetProfileVelocities(const PDD_PD0Ensemble* pEns, double* vel, int size, e_PDPingType ping);   // Get profile velocity in m/s
        PDD_DECL int PDD_GetVesselVelocities(const PDD_PD0Ensemble* pEns, double vel[4]);                               // Get vessel velocity in m/s
        PDD_DECL double PDD_GetRangeToBottom(const PDD_PD0Ensemble* pEns, double beamRange[4]);                         // Get range to bottom in meters

#ifdef __cplusplus
    }
}
#endif

#pragma pack(pop)

#endif

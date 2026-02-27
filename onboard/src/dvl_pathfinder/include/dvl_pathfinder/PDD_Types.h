/** @file PDD_Types.h
 *  @brief Definition of data structures and types available in TRDI ensembles
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */

#ifndef PDD_TYPES_H
#define PDD_TYPES_H

#include <stdint.h>

#pragma pack(push, 1)

#ifdef __cplusplus
extern "C"
{
    namespace tdym
    {
#endif
        /** @brief Defines 4 beam constant
        */
#define FOUR_BEAMS (4)

        /** @brief Maximum number of depth cells in ensemble
        */
#define PDD_MAX_CELLS (255)

        /** @brief Maximum size for profile arrays in ensemble 4*255 = 1020
        */
#define PDD_ARRAY_SIZE (1020)

        /** @brief Minimum ensemble size
        */
#define PDD_MIN_ENSEMBLE_SIZE (10)

        /** @brief Maximum ensemble size
        */
#define PDD_MAX_ENSEMBLE_SIZE (5400)

        /** @brief Value that is used to mark invalid velocities
        */
#define PD0_INVALID_VALUE (-32768)

        /** @brief Value that is used to mark high resolution invalid velocities
        */
#define PD0_HI_RES_INVALID_VALUE ((int)(0x80000000))

        /** @brief Ensemble header structure
        */
        typedef struct PDD_Header
        {
            uint16_t id;                                //!< Ensemble ID (PD0 - 0x7F7F)
            uint16_t checksumOffset;                    //!< Checksum offset
            uint8_t spare;                              //!< Internal use
            uint8_t dataCount;                          //!< Number of data structure in the ensemble
            uint16_t offset[255];                       //!< Offset to each data structure 
        } PDD_Header;

        /** @brief PD time structure used in fixed and variable leader
        */
        typedef struct PDD_Time
        {
            uint8_t minute;                             //!< Minutes of the day (0-59)
            uint8_t second;                             //!< Seconds of the day (0-59)
            uint8_t sec100;                             //!< Hundreds of second (0-99)
        } PDD_Time;

        /** @brief Date and time structure used in variable leader
        */
        typedef struct PDD_DateTime
        {
            uint8_t year;                               //!< Year of the century (0-99)
            uint8_t month;                              //!< Month of the year (1-12)
            uint8_t day;                                //!< Day of the month (1-31)
            uint8_t hour;                               //!< Hour of the day (0-24)
            uint8_t minute;                             //!< Minutes of the day (0-59)
            uint8_t second;                             //!< Seconds of the day (0-59)
            uint8_t sec100;                             //!< Hundreds of second (0-99)
        } PDD_DateTime;

        /** @brief Date and time structure that contains century used in variable leader
        */
        typedef struct PDD_Y2KDateTime
        {
            uint8_t century;                            //!< Century (19-20)
            uint8_t year;                               //!< Year of the century (0-99)
            uint8_t month;                              //!< Month of the year (1-12)
            uint8_t day;                                //!< Day of the month (1-31)
            uint8_t hour;                               //!< Hour of the day (0-24)
            uint8_t minute;                             //!< Minutes of the day (0-59)
            uint8_t second;                             //!< Seconds of the day (0-59)
            uint8_t sec100;                             //!< Hundreds of second (0-99)
        } PDD_Y2KDateTime;

        /** @brief Fixed leader structure
        */
        typedef struct PDD_FixedLeader
        {
            uint16_t id;                                //!< Fixed Leader ID 0x0000
            uint8_t fwVersion;                          //!< Major firmware version number
            uint8_t fwRevision;                         //!< Firmware revision number
            uint16_t config;                            //!< System configuration
            uint8_t dataSource;                         //!< Data source: 1 - fake data, 0 - real data
            uint8_t codeCount;                          //!< Number of code elements
            uint8_t beamCount;                          //!< Number of profiling beams (it should be 4)
            uint8_t cellCount;                          //!< Number of depth cells
            uint16_t pingCount;                         //!< Number of pings per ensemble
            uint16_t cellSize;                          //!< Cell size in cm
            uint16_t blank;                             //!< Blank after transmit in cm
            uint8_t mode;                               //!< Water profiling mode
            uint8_t minCorr;                            //!< Minimum correlation threshold
            uint8_t ncodeReps;                          //!< Number of code repetitions
            uint8_t minPerc;                            //!< Minimum percent good
            uint16_t maxErrVel;                         //!< Error velocity threshold mm/s
            PDD_Time pingTime;                          //!< Time between pings
            uint8_t xform;                              //!< Coordinate transform bit flags configuration
            int16_t headingAlignment;                   //!< Heading alignment in hundreds of degrees
            int16_t headingBias;                        //!< Magnetic variation/bias in hundreds of degrees
            uint8_t sensorSource;                       //!< Bit-map of sensor used in calculations
            uint8_t sensorsAvail;                       //!< Bit-map of installed sensors
            uint16_t firstCellRange;                    //!< Range in cm to middle of first cell
            uint16_t transmit;                          //!< Transmit pulse length in cm
            uint8_t refMinCell;                         //!< First cell used for reference layer
            uint8_t refMaxCell;                         //!< Last cell used for reference layer
            uint8_t falseTargetTh;                      //!< False target threshold
            uint8_t spare;                              //!< Spare byte
            uint16_t lag;                               //!< Lag in cm
            uint8_t cpuSerialNum[8];                    //!< CPU serial number
            uint16_t bandwidth;                         //!< Bandwidth
            uint8_t powerLevel;                         //!< Power level
            uint8_t spare2;                             //!< Spare byte
            uint32_t serialNumber;                      //!< Serial number
            uint8_t beamAngle;                          //!< System beam angle in degrees
            uint8_t reserved;                           //!< Reserved
            uint16_t pingPerEns;                        //!< Pings per ensemble
            uint8_t freq[3];                            //!< Exact Frequency
        } PDD_FixedLeader;

        /** @brief Health and environment monitoring structure
        */
        typedef struct PDD_HEMData
        {
            uint8_t hemStatus;                          //!< HEM status mask - HEM data not always present
            uint16_t leakACounts;                       //!< HEM leak sensor A counts
            uint16_t leakBCounts;                       //!< HEM leak sensor B counts
            int16_t xmtVoltage;                         //!< HEM transmit voltage Milli-Volts, -1 if not available
            int16_t xmtCurrent;                         //!< HEM transmit current Milli-Amps, -1 if not available
            int16_t xmtImpedance;                       //!< HEM transmit impedance Milli-Ohms, -1 if not available
        } PDD_HEMData;

        /** @brief Variable leader structure
        */
        typedef struct PDD_VariableLeader
        {
            uint16_t id;                                //!< ID 0x0080
            uint16_t ensNumberLsb;                      //!< Least significant word for ensemble number
            PDD_DateTime ensTime;                       //!< Ensemble date and time
            uint8_t ensNumberMsb;                       //!< Most significant byte for ensemble number
            uint16_t  bit;                              //!< Built in Test results
            uint16_t speedOfSound;                      //!< Speed of sound in m/s
            int16_t depth;                              //!< Transducer depth in decimeters
            uint16_t heading;                           //!< Heading in 0.01 deg
            int16_t pitch;                              //!< Pitch in 0.01 deg
            int16_t roll;                               //!< Roll 0.01 deg
            uint16_t salinity;                          //!< Salinity
            int16_t temperature;                        //!< Water Temperature in 0.01 deg C
            PDD_Time minPrePingTime;                    //!< Min ping wait time
            uint8_t headingStd;                         //!< Heading standard deviation in deg
            uint8_t pitchStd;                           //!< Pitch standard deviation in 0.1 deg
            uint8_t rollStd;                            //!< Roll standard deviation in 0.1 deg
            uint8_t adc[8];                             //!< ADC channels 0-7
            uint32_t activeBit;                         //!< Error status bits
            uint16_t reserved;                          //!< Reserved
            int32_t pressure;                           //!< Pressure in kPa
            int32_t pressureStd;                        //!< Pressure standard deviation
            uint8_t pctWm4Pings;                        //!< Percent of WM4 pings
            PDD_Y2KDateTime y2KdateTime;                //!< Y2K compatible time
            uint8_t lagNearBottom;                      //!< Lag near bottom flag
            PDD_HEMData hem;                            //!< Hem data extension for some systems (not always present)
            uint16_t pingNumber;                        //!< Ping sequence number in ensemble or burst (not always present)
        } PDD_VariableLeader;

        /** @brief Bottom track leader structure
        */
        typedef struct PDD_BottomTrack
        {
            uint16_t id;                                //!< ID 0x6000
            uint16_t pingCount;                         //!< Number of bottom track pings per ensemble
            uint16_t reacquireDelay;                    //!< Reacquire delay in ensemble
            uint8_t minCorrelation;                     //!< Minimum correlation magnitude
            uint8_t minIntensity;                       //!< Minimum evaluation intensity
            uint8_t minPercGood;                        //!< Minimum percent good
            uint8_t mode;                               //!< Bottom track mode
            uint16_t maxErrVelocity;                    //!< Error velocity threshold
            uint16_t searchPings;                       //!< Number of search pings issued
            uint16_t trackingPings;                     //!< Number of tracking pings issued
            uint16_t range[4];                          //!< Bottom range to each beam in cm
            int16_t velocity[4];                        //!< Bottom velocity for each beam mm/s
            uint8_t correlation[4];                     //!< Correlation magnitude for each beam
            uint8_t intensity[4];                       //!< Filter evaluation intensity
            uint8_t percGood[4];                        //!< Percent good pings
            uint16_t refLayerMin;                       //!< Minimum reference layer in cm
            uint16_t refLayerNear;                      //!< Near reference layer in cm
            uint16_t refLayerFar;                       //!< Far reference layer in cm
            int16_t refLayerVelocity[4];                //!< Reference layer velocities mm/s
            uint8_t refLayerCorrlation[4];              //!< Reference layer correlation
            uint8_t refLayerIntensity[4];               //!< Reference layer echo intensity
            uint8_t refLayerPercGood[4];                //!< Reference layer percent good
            uint16_t searchDepth;                       //!< Max tracking depth in cm
            uint8_t rssi[4];                            //!< Bottom track RSSI value
            uint8_t gain;                               //!< Gain (1 = high, 0 = low)
            uint8_t rangeMsb[4];                        //!< Most significant byte of range
        } PDD_BottomTrack;

        /** @brief Profile int16_t data array structure
        */
        typedef struct PDD_ShortArray
        {
            uint16_t id;                                //!< ID
            int16_t data[1020];                         //!< Data - usually velocity
        } PDD_ShortArray;

        /** @brief Profile byte data array structure
        */
        typedef struct PDD_ByteArray
        {
            uint16_t id;                                //!< ID
            uint8_t data[1020];                         //!< Data - usually correlation, intensity and percentage good
        } PDD_ByteArray;

        /** @brief Vertical beam leader structure
        */
        typedef struct PDD_VertBeamLeader
        {
            uint16_t id;                                //!< ID 0x0F01
            uint16_t cellCount;                         //!< Number of depth cells
            uint16_t pingCount;                         //!< Number of pings
            uint16_t cellSize;                          //!< Depth cell size in cm
            uint16_t firstCellRange;                    //!< First cell range in cm
            uint16_t mode;                              //!< Vertical profiling mode
            uint16_t transmit;                          //!< Transmit in cm
            uint16_t lag;                               //!< Lag in cm
            uint16_t lagElements;                       //!< Number of transmit elements
            uint16_t rssiThresh;                        //!< RSSI threshold
            uint16_t shallCell;                         //!< Reserved
            uint16_t startCell;                         //!< Reserved
            uint16_t shallRssiCell;                     //!< Reserved
            uint16_t maxCorThresh;                      //!< Reserved
            uint16_t minCorThresh;                      //!< Reserved
            uint16_t pingOffsetTime;                    //!< Ping offset in milliseconds
            uint16_t spare;                             //!< Reserved
            uint16_t depthScreen;                       //!< Reserved
            uint16_t pctGdThresh;                       //!< Reserved
            uint16_t doProofing;                        //!< Reserved
        }  PDD_VertBeamLeader;

        /** @brief Navigation parameters data structure
        */
        typedef struct PDD_NavigationParams
        {
            uint16_t id;                                //!< ID 0x2013
            uint32_t timeToBottom[4];                   //!< Time to bottom
            int16_t btStd[4];                           //!< BT std deviation in mm/s
            int8_t shallowFlag;                         //!< Shallow mode flag
            uint32_t timeToRefLayer[4];                 //!< Time to ref layer
            uint16_t rangeRefLayer;                     //!< Range to ref layer in meters
            int16_t wtStd[4];                           //!< WT std deviation in mm/s
            uint32_t btValTime[4];                      //!< BT time of validity in microseconds
            uint32_t wtValTime[4];                      //!< WT time of validity in microseconds
        } PDD_NavigationParams;

        /** @brief Transformation calibration matrix
        */
        typedef struct PDD_TransformationMatrix
        {
            uint16_t id;                                //!< ID 0x3200
            int16_t data[16];                           //!< 4x4 transformation matrix
        } PDD_TransformationMatrix;


        /** @brief Vertical beam range structure
        */
        typedef struct PDD_VertBeamRange
        {
            uint16_t id;                                //!< ID 0x4100
            uint8_t evalAmplitude;                      //!< Evaluation amplitude
            uint8_t rssiAmplitude;                      //!< RSSI amplitude
            uint32_t range;                             //!< Range to bottom in mm
            uint8_t status;                             //!< Status bit flags
        } PDD_VertBeamRange;

        /** @brief Bottom track high resolution velocity structure
        */
        typedef struct PDD_BTHighResVelocity
        {
            uint16_t id;                                //!< ID 0x5803
            int32_t velocity[4];                        //!< High resolution BT velocity in 0.01 mm/s
            int32_t distMadeGood[4];                    //!< Distance made good in 0.01 mm
            int32_t refLayerVelocity[4];                //!< Reference layer velocity in 0.01 mm/s
            int32_t refLayerDistMadeGood[4];            //!< Reference layer distance made good in 0.01 mm
            int32_t speedOfSound;                       //!< Speed of sound * 10^6 m/s
        } PDD_BTHighResVelocity;

        /** @brief Bottom track range structure
        */
        typedef struct PDD_BottomTrackRange
        {
            uint16_t id;                                //!< ID 0x5804
            uint32_t slanRange;                         //!< Slant range in 0.1 mm
            int32_t axisDeltaRange;                     //!< Difference in slant range in 0.1 mm
            uint32_t verticalRange;                     //!< Average vertical range in 0.1 mm
            uint8_t percGood4Bm;                        //!< Percent good slant range solutions 
            uint8_t percGoodBm12;                       //!< Percent good for beam 1 & 2
            uint8_t percGoodBm34;                       //!< Percent good for beam 3 & 4
            uint32_t rawRange[4];                       //!< Raw range in 0.1 mm
            uint8_t maxBTFilter[4];                     //!< Raw max BT filter
            uint8_t maxBTAmplitude[4];                  //!< BT amplitude at measured range
        } PDD_BottomTrackRange;

        /** @brief ISM calibration sample
        */
        typedef struct PDD_ISMDataSample
        {
            int32_t heading;                            //!< Heading in 0.001 degrees
            int32_t pitch;                              //!< Pitch in 0.001 degrees
            int32_t roll;                               //!< Roll in 0.001 degrees
            int32_t accel[3];                           //!< Axis accelerometer reading in 0.001 G's
            uint16_t magnetometer[3];                   //!< Axis raw magnetometer counts
        } PDD_ISMDataSample;

        /** @brief ISM post magnetic calibration data
        */
        typedef struct PDD_ISMData
        {
            uint16_t id;                                //!< ID 0x5901
            uint8_t valid;                              //!< Flag 0 if invalid, else valid
            PDD_ISMDataSample samples[2];               //!< Sample data
            uint32_t timeBetweenSamples;                //!< Time between samples 0.001 milliseconds
        } PDD_ISMData;

        /** @brief Attitude data for a single ping
        */
        typedef struct PDD_PingAttitudeSample
        {
            uint8_t id;                                 //!< Ping ID, NB = 1, BB = 2, BT = 3
            uint8_t valid;                              //!< Data valid flag
            int32_t attitude[3];                        //!< HPR 0.001 degrees
            int32_t rate[3];                            //!< HPR change rate, 0.001 degrees/second
        } PDD_PingAttitudeSample;

        /** @brief Attitude data for multiple ping types
        */
        typedef struct PDD_PingAttitude
        {
            uint16_t id;                                //!< ID 0x5902
            uint8_t count;                              //!< Number of ping attitudes reported
            PDD_PingAttitudeSample samples[4];          //!< Ping attitude samples (check count before access)
        } PDD_PingAttitude;

        /** @brief Extra firmware information not provided in fixed leader
         */
        typedef struct PDD_FirmwareInfo
        {
            uint16_t id;                                //!< ID 0x4400
            uint8_t alpha;                              //!< Alpha version character
            char version[16];                           //!< Branch version number
            uint32_t testData;                          //!< Test data selected
            uint32_t testSwitch;                        //!< Test switch selected
            uint16_t fpgaVersion;                       //!< FPGA version
            uint8_t reserved;                           //!< Reserved
        } PDD_FirmwareInfo;

        /** @brief ADC data structure
         */
        typedef struct PDD_ADCData
        {
            uint16_t id;                                //!< ID 0x7001
            uint16_t vin;                               //!< ADC channel data - input voltage in Volts
            uint16_t iin;                               //!< ADC channel data
            uint16_t vio;                               //!< ADC channel data
            uint16_t vcore;                             //!< ADC channel data
            uint16_t vdd1;                              //!< ADC channel data
            uint16_t vref;                              //!< ADC channel data
            uint16_t vgg;                               //!< ADC channel data
        }  PDD_ADCData;

        /** @brief Waves first leader structure
         */
        typedef struct PDD_WavesFirstLeader
        {
            uint16_t id;                                //!< ID 0x0103
            uint8_t fwVersion;                          //!< Major firmware version number
            uint8_t fwRevision;                         //!< Firmware revision number
            uint16_t configuration;                     //!< Configuration
            uint8_t cellCount;                          //!< Number of depth cells
            uint16_t waveRecPings;                      //!< Wave record ping count
            uint16_t cellSize;                          //!< Cells size in  cm
            uint16_t timeBetweenWavePings;              //!< Time between waves pings in 0.01 seconds
            uint16_t timeBetweenBursts;                 //!< Time between bursts in seconds
            uint16_t firstCellRange;                    //!< Range in cm to middle of first cell
            uint8_t cellOutput;                         //!< Number of cells output (20 max.)
            uint16_t selectedData;                      //!< Selected data
            int32_t dwsFlags[4];                        //!< Bitmap flags for directional waves
            int32_t velFlags[4];                        //!< Bitmap flags for non-directional waves
            PDD_Y2KDateTime startTime;                  //!< Start time
            uint32_t waveRecordNumber;                  //!< Wave record number
            uint8_t cpuSerial[8];                       //!< CPU serial number
            uint16_t temperature;                       //!< Temperature in deg C
            uint16_t reserved;                          //!< Reserved
        }  PDD_WavesFirstLeader;

        /** @brief Waves data structure
         */
        typedef struct PDD_WavesData
        {
            uint16_t id;                                //!< ID 0x0203
            uint16_t pingNumber;                        //!< Ping number in waves record
            uint32_t timeSinceStart;                    //!< Time since start in 0.01 s
            int32_t pressure;                           //!< Pressure in 10 Pascal
            int32_t distToSurf[4];                      //!< Distance to surface in mm, -1 = invalid
            int16_t velocity[20 * 4];                   //!< Beam velocity in mm/s, -32768 = invalid
        }  PDD_WavesData;

        /** @brief Waves last leader structure
         */
        typedef struct PDD_WavesLastLeader
        {
            uint16_t id;                                //!< ID 0x0303
            uint16_t avgDepth;                          //!< Average depth in decimeters
            uint16_t avgSpeedOfSound;                   //!< Average speed of sound in m/s
            uint16_t avgTemp;                           //!< Average temperature in 0.1 deg C
            uint16_t avgHeading;                        //!< Average heading in 0.1 deg
            uint16_t stdHeading;                        //!< Standard deviation of heading in 0.1 deg
            int16_t avgPitch;                           //!< Average pitch in 0.1 deg
            uint16_t stdPitch;                          //!< Standard deviation of pitch in 0.1 deg
            int16_t avgRoll;                            //!< Average roll in 0.1 deg
            uint16_t stdRoll;                           //!< Standard deviation of roll 0.1 deg
        }  PDD_WavesLastLeader;

        /** @brief Waves heading, pitch, and roll structure
         */
        typedef struct PDD_HPRData
        {
            uint16_t id;                                //!< ID 0x0403
            uint16_t heading;                           //!< Heading in 0.1 deg
            int16_t pitch;                              //!< Pitch in 0.1 deg
            int16_t roll;                               //!< Roll in 0.1 deg
        }  PDD_HPRData;

        /** @brief System frequency flags of configuration in fixed leader
         */
        typedef enum e_FrequencyType
        {
            FREQ_75KHZ = 0,                             //!< 75kHz system
            FREQ_150KHZ = 1,                            //!< 150 kHz system
            FREQ_300KHZ = 2,                            //!< 150 kHz system
            FREQ_600KHZ = 3,                            //!< 600 kHz system
            FREQ_1200KHZ = 4,                           //!< 1200 kHz system
            FREQ_2400KHZ = 5,                           //!< 2400 kHz system
            FREQ_38KHZ = 6,                             //!< Low frequency system 
            FREQ_MASK = 7                               //!< Used to mask the system frequency bits
        } e_FrequencyType;

        /** @brief Beam angle flags of configuration in fixed leader
         */
        typedef enum e_BeamAngleType
        {
            ANGLE_15DEG = 0x0000,                       //!< 15 degrees system
            ANGLE_20DEG = 0x0100,                       //!< 20 degrees system
            ANGLE_30DEG = 0x0200,                       //!< 30 degrees system
            ANGLE_MASK = 0x0300,                        //!< Mask or other
        } e_BeamAngleType;

        /** @brief Velocities coordinate frame type
         */
        typedef enum e_CoordinateType
        {
            PDD_BEAM_COORD = 0,                         //!< Beam coordinate system
            PDD_XYZ_COORD,                              //!< Instrument (XYZ) coordinate system
            PDD_SHIP_COORD,                             //!< Ship coordinate system
            PDD_EARTH_COORD,                            //!< Earth coordinate system
        } e_CoordinateType;

        /** @brief Transformation bit masks
        */
        typedef enum e_XformBitType
        {
            PDD_XFORM_BIN_MAP_MASK = 0x01,              //!< Transformation bin mapping mask
            PDD_XFORM_3BEAM_MASK = 0x02,                //!< Transformation 3-beam solution mask
            PDD_XFORM_TILTS_MASK = 0x04,                //!< Transformation mask for using tilts
            PDD_XFORM_COORD_MASK = 0x18,                //!< Transformation mask for coordinate system 0x18
            PDD_XFORM_COORD_LSB = 3                     //!< Coordinate system LSB
        } e_XformBitType;

        /** @brief Transformation bit masks
        */
        typedef enum e_SysCfgBitType
        {
            PDD_CFG_CONVEX_MASK = 0x0008,               //!< Configuration convex mask
            PDD_CFG_UP_MASK = 0x0080,                   //!< Configuration up mask
            PDD_CFG_XDUCER_ATTACHED = 0x0040,           //!< Configuration transducer attached mask
            PDD_CFG_TILT_CFG_MASK = 0x0030,             //!< Configuration tilts mask
            PDD_CFG_TILT_CFG_LSB = 4                    //!< Configuration tilts LSB
        } e_SysCfgBitType;

        /** @brief Beams geometry type
         */
        typedef enum e_BeamsGeometryType
        {
            PDD_CONCAVE = 0,                            //!< Concave beams geometry
            PDD_CONVEX,                                 //!< Convex beams geometry
        } e_BeamsGeometryType;

        /** @brief ADCP orientation type
         */
        typedef enum e_OrientationType
        {
            PDD_DOWN = 0,                               //!< Beams are pointing down
            PDD_UP,                                     //!< Beams are pointing up
            PDD_HORIZONTAL                              //!< Beams are horizontal
        } e_OrientationType;

        /** @brief Tilt configuration type
         */
        typedef enum e_TiltConfigType
        {
            PDD_TILT_CONFIG_1,                          //!< Both tilt sensors ate fixed to instrument frame 
            PDD_TILT_CONFIG_2,                          //!< Roll axis is fixed, pitch is gimbaled inside the roll
            PDD_TILT_CONFIG_3,                          //!< Pitch axis is fixed, roll is gimbaled inside the pitch
        } e_TiltConfigType;

        /** @brief ADCP types based on firmware version
         */
        typedef enum e_ADCPType
        {
            BROADBAND_5 = 5,                            //!< Old broadband systems
            WORKHORSE_8 = 8,                            //!< Old Workhorse
            NAVIGATOR_9 = 9,                            //!< Navigator
            RIO_GRANDE_10 = 10,                         //!< RioGrande
            WH_HORIZONTAL_11 = 11,                      //!< Workhorse horizontal system
            OCEAN_SURVEYOR_14 = 14,                     //!< Ocean Surveyor
            WORKHORSE_16 = 16,                          //!< Workhorse
            NAVIGATOR_19 = 19,                          //!< Navigator
            OCEAN_SURVEYOR_23 = 23,                     //!< Ocean Surveyor
            CHANNELMASTER_28 = 28,                      //!< ChannelMaster
            STREAMPRO_31 = 31,                          //!< StreamPro
            EXPLORER_34 = 34,                           //!< Explorer
            NAVIGATOR_37 = 37,                          //!< Navigator
            DVS_41 = 41,                                //!< DVS
            WORKHORSE_43 = 43,                          //!< Workhorse
            RIVERRAY_44 = 44,                           //!< RiverRay
            SENTINELV_47 = 47,                          //!< SentinelV
            WORKHORSE_50 = 50,                          //!< Workhorse
            WORKHORSE_51 = 51,                          //!< Workhorse
            WORKHORSE_52 = 52,                          //!< Workhorse
            NAVIGATOR_53 = 53,                          //!< Workhorse
            DVS_55 = 55,                                //!< DVS
            RIVERPRO_56 = 56,                           //!< RiverPro(5 beams) or RioPro(4 beams)
            MERIDIAN_59 = 59,                           //!< Meridian
            PINNACLE_61 = 61,                           //!< Pinnacle
            SENTINELV_66 = 66,                          //!< SentinelV RT
            PATHFINDER_67 = 67,                         //!< Pathfinder
            PIONEER_73 = 73,                            //!< Pioneer
            TASMAN_74 = 74,                             //!< TASMAN
            WAYFINDER_76 = 76,                          //!< Wayfinder
            WORKHORSE_77 = 77,                          //!< Workhorse II
            WORKHORSE_78 = 78,                          //!< Workhorse II
        } e_ADCPType;

        /** @brief HEM status bits
        */
        typedef enum e_HEMStatusType
        {
            SENSORA_LEAK = 0x01,                        //!< Sensor A leak detected
            SENSORA_DISCONNECT = 0x02,                  //!< Sensor A disconnected
            SENSORB_LEAK = 0x04,                        //!< Sensor B leak detected
            SENSORB_DISCONNECT = 0x08,                  //!< Sensor B disconnected
            TX_VOLTAGE_UPDATED = 0x10,                  //!< Transmit voltage updated
            TX_CURRENT_UPDATED = 0x20,                  //!< Transmit current updated
            IMPEDANECE_UPDATED = 0x40                   //!< Impedance updated
        } e_HEMStatusType;

        /** @brief Ensemble types IDs
     */
        typedef enum e_PDEnsIdType
        {
            PD4DataID = 0x007D,                         //!<PD4 data ID
            PD5DataID = 0x017D,                         //!<PD5 data ID
            PD0DataID = 0x7F7F,                         //!<PD0 data ID
            WavePacketID = 0x797F                       //!<Wave packets data ID
        } e_PDIdType;

        /** @brief PD structures IDs
         */
        typedef enum e_PDDataIdType
        {
            FixedLeaderID = 0x000,                      //!<Fixed leader ID
            FixedLeader2ID = 0x001,                     //!<Secondary fixed leader ID
            VariableLeaderID = 0x0080,                  //!<Variable leader ID
            VariableLeader2ID = 0x0081,                 //!<Secondary variable leader ID
            BottomTrackID = 0x0600,                     //!<Bottom track ID
            VelocityProfileID = 0x0100,                 //!<Velocity profile ID
            VelocityProfile2ID = 0x0101,                //!<Secondary velocity profile ID
            CorrelationProfileID = 0x0200,              //!<Correlation profile ID
            CorrelationProfile2ID = 0x0201,             //!<Secondary correlation profile ID
            IntensityProfileID = 0x0300,                //!<Intensity profile ID
            IntensityProfile2ID = 0x0301,               //!<Secondary intensity profile ID
            PercentGoodProfileID = 0x0400,              //!<Percent good profile ID
            PercentGoodProfile2ID = 0x0401,             //!<Secondary percent good profile ID
            StatusProfileID = 0x0500,                   //!<Status profile ID
            StatusProfile2ID = 0x0501,                  //!<Secondary status profile ID
            VertBeamVelocityID = 0x0A00,                //!<Vertical beam velocity ID
            VertBeamCorrelationID = 0x0B00,             //!<Vertical beam correlation ID
            VertBeamIntensityID = 0x0C00,               //!<Vertical beam intensity ID
            VertBeamPercentGoodID = 0x0D00,             //!<Vertical beam percent good ID
            VertBeamLeaderID = 0x0F01,                  //!<Vertical beam leader
            NavigationParamsID = 0x2013,                //!<Navigation parameters data ID
            TransformationMatrixID = 0x3200,            //!<Transformation matrix data ID
            VertBeamRangeID = 0x4100,                   //!<Vertical beam range ID
            BTHighResVelocityID = 0x5803,               //!<Bottom track high resolution velocity ID
            BottomTrackRangeID = 0x5804,                //!<Bottom track range ID
            ISMDataID = 0x5901,                         //!<ISM data ID
            PingAttitudeID = 0x5902,                    //!<Ping attitude ID
            ADCDataID = 0x7001,                         //!<ADC data ID

            WavesFirstLeaderID = 0x0103,                //!<Waves first leader ID
            WavesDataID = 0x0203,                       //!<Waves data ID
            WavesLastLeaderID = 0x0303,                 //!<Waves last leader ID
            HPRDataID = 0x0403                          //!<Waves HPR data ID
        } e_PDDataIdType;

        /** @brief Ping types
         */
        typedef enum e_PDPingType
        {
            PDBB,                                       //!< Broad-band profile ping
            PDNB,                                       //!< Narrow-band profile ping
            PDVP,                                       //!< Vertical beam profile
            PDBT,                                       //!< Bottom track ping
            PDVR,                                       //!< Vertical range ping
            PDD_NumPingTypes                            //!< Number of ping types
        } e_PDPingType;

        /** @brief Internal function definition to set ensemble structures
         */
        typedef void (*SetStructureFunc)(void*, const uint8_t*, int, int);

        /** @brief Structure to define map to decode ensemble structures
         */
        typedef struct PDD_DataMap {
            const int key;                                    //!<Key that corresponds to structure ID
            const SetStructureFunc value;                     //!<Set function pointer
            const int index;                                  //!<Parameter to pass in set function
        } PDD_DataMap;

#ifdef __cplusplus
    }
}
#endif

#pragma pack(pop)

#endif
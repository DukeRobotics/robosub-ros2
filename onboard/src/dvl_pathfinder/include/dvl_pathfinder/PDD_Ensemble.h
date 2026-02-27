/** @file PDD_Ensemble.h
 *  @brief Ensemble structures and functions definitions
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#ifndef PDD_ENSEMBLE_H
#define PDD_ENSEMBLE_H

#include "PDD_Definitions.h"
#include "PDD_Types.h"

#pragma pack(push, 1)

#ifdef __cplusplus
extern "C"
{
    namespace tdym
    {
#endif

        /** @brief Waves ensemble structure
    */
        typedef struct PDD_WavesEnsemble
        {
            int ensSize;                                    //!< Ensemble size
            unsigned char data[PDD_MAX_ENSEMBLE_SIZE];      //!< Data buffer for ensemble (size 5400)
            int headerSize;                                 //!< Header size
            PDD_Header* header;                             //!< Pointer to header
            int firstLeaderSize;                            //!< Waves first leader size
            PDD_WavesFirstLeader* firstLeader;              //!< Pointer to waves first leader structure
            int wavesDataSize;                              //!< Waves data size
            PDD_WavesData* wavesData;                       //!< Pointer to waves data structure
            int lastLeaderSize;                             //!< Waves last leader size
            PDD_WavesLastLeader* lastLeader;                //!< Pointer to waves last leader
            int hprDataSize;                                //!< HPR data size
            PDD_HPRData* hprData;                           //!< Pointer to waves HPR data
        } PDD_WavesEnsemble;


        PDD_DECL int PDD_DecodeWaves(const unsigned char* data, int count, PDD_WavesEnsemble* ens);  // Decode ensemble

        /** @brief PD0 ensemble structure
        */
        typedef struct PDD_PD0Ensemble
        {
            int ensSize;                                    //!< Ensemble size
            unsigned char data[PDD_MAX_ENSEMBLE_SIZE];          //!< Data buffer for ensemble (size 5400)
            int headerSize;                                 //!< Header size
            PDD_Header* header;                             //!< Pointer to header
            int fixedLeaderSize[2];                         //!< Fixed leader sizes
            PDD_FixedLeader* fixedLeader[2];                //!< Pointers to fixed leader structure
            int varLeaderSize[2];                           //!< Variable leader sizes
            PDD_VariableLeader* varLeader[2];               //!< Pointers to variable leader structure
            int bottomTrackSize;                            //!< Bottom track size
            PDD_BottomTrack* bottomTrack;                   //!< Pointers to bottom track structure
            int velocitySize[3];                            //!< Velocity sizes
            PDD_ShortArray* velocity[3];                    //!< Pointers to velocity profiles
            int intensitySize[3];                           //!< Intensity sizes
            PDD_ByteArray* intensity[3];                    //!< Pointers to intensity profiles
            int correlationSize[3];                         //!< Correlation sizes
            PDD_ByteArray* correlation[3];                  //!< Pointers to correlation profiles
            int pctGoodSize[3];                             //!< Percent good sizes
            PDD_ByteArray* pctGood[3];                      //!< Pointers to percent good profiles
            int statusSize[3];                              //!< Status sizes
            PDD_ByteArray* status[3];                       //!< Pointers to status profiles
            int vbLeaderSize;                               //!< Vertical beam leader size
            PDD_VertBeamLeader* vbLeader;                   //!< Pointer to vertical beam leader
            int navParamsSize;                              //!< Navigation parameters size
            PDD_NavigationParams* navParams;                //!< Pointer to navigation parameters structure 
            int xformSize;                                  //!< Transformation matrix size
            PDD_TransformationMatrix* xform;                //!< Pointer to transformation matrix structure
            int vbRangeSize;                                //!< Vertical beam range size
            PDD_VertBeamRange* vbRange;                     //!< Pointer to vertical beam range structure
            int btHiResVelSize;                             //!< BT high resolution velocity size
            PDD_BTHighResVelocity* btHiResVel;              //!< Pointer to BT high resolution velocity structure
            int btRangeSize;                                //!< Bottom track range size
            PDD_BottomTrackRange* btRange;                  //!< Pointer to bottom track range 
            int ismDataSize;                                //!< ISM data size
            PDD_ISMData* ismData;                           //!< Pointer to ISM data
            int pingAttitudeSize;                           //!< Ping attitude size
            PDD_PingAttitude* pingAttitude;                 //!< Pointer to ping attitude structure
            int adcDataSize;                                //!< ADC Data size
            PDD_ADCData* adcData;                           //!< Pointer to ADC data

        } PDD_PD0Ensemble; // PD0 ensemble structure

        PDD_DECL int PDD_DecodePD0(const unsigned char* data, int count, PDD_PD0Ensemble* pEns); // Decode ensemble    

#ifndef DONT_USE_MALLOC
        PDD_DECL PDD_WavesEnsemble* PDD_CreateWavesEnsemble();                                   // Create ensemble
        PDD_DECL void PDD_DestroyWavesEnsemble(PDD_WavesEnsemble* pDec);                         // Destroy ensemble
        PDD_DECL PDD_PD0Ensemble* PDD_CreatePD0Ensemble();                                       // Create ensemble
        PDD_DECL void PDD_DestroyPD0Ensemble(PDD_PD0Ensemble* pDec);                             // Destroy ensemble
#endif

#ifdef __cplusplus
    }
}
#endif

#pragma pack(pop)

#endif

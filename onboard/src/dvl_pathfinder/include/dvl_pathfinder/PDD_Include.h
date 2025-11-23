/** @file PDD_Include.h
 *  @brief Main include for library users
 * 
 * Copyright (c) Teledyne Marine 2021. All rights reserved.
 *
 * This file is part of TRDI.Drivers.PDDecoder library.
 */
#ifndef PDD_INCLUDE_H
#define PDD_INCLUDE_H

#include "PDD_Decoder.h"
#include "PDD_Helper.h"

 /*! \mainpage Teledyne Marine PDDecoder Library in "C"
*
*<b> What is the Teledyne Marine PDDecoder library? </b>
*
*The Teledyne Marine PDDecoder library is an open source library written in C language to decode the PD0 data formats that are commonly output by Teledyne Marine/Teledyne RD Instruments ADCPs. The definition and details of the PD0 format can be found in any of the manuals under the section, Output Data Format.
*
*An example of the driver used in application code can be found in the /examples folder supplied with this driver.
*
*\b Getting \b Started
*
*Follow these steps to use the Teledyne Marine PDDecoder library:
*
*1. Include the file "PDD_Include.h" in your cpp/c file.
*2. Create a PDD_Decoder structure using the PDD_CreateDecoder() function.
*3. Initialize the PDD_Decoder structure using the PDD_InitializeDecoder() function.
*4. Initialize PDD_PD0Ensemble structure using the PDD_CreatePD0Ensemble() function.
*5. Read  PD0 formatted data from a file or from another stream.
*6. Add these data to the decoder using the PDD_AddDecoderData() function.
*7. Call the PDD_GetPD0Ensemble() function and if it succeeds, then use the ensemble to extract out the necessary data
*8. Repeat 7 until function fails
*9. Go to step 5 until no more data is available
*
*/

#endif
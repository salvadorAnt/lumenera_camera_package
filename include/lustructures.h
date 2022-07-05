//******************************************************************************
//
// Copyright (c) 2019-2020, TELEDYNE LUMENERA, a business unit of TELEDYNE
// DIGITAL IMAGING, INCORPORATED. All rights reserved.
//
// This program is subject to the terms and conditions defined in file 'LICENSE'
// , which is part of this Linux LuCam SDK package.
//
//******************************************************************************

#pragma once

#if !defined(LUMENERA_MAC_API) && !defined(LUMENERA_LINUX_API)
    #ifndef LUMENERA_WINDOWS_API
        #define LUMENERA_WINDOWS_API
    #endif
#endif

#include "lucommon.h"

//****************************************************************************************************************
//****************************************************************************************************************
//  Structures
//****************************************************************************************************************
//****************************************************************************************************************

// Version Information
//----------------------------------------------------------------------------------------------------------------
// Version Information: Camera and Host
// For use with QueryVersion and EnumCameras
typedef struct LUCAM_VERSION {
    ULONG firmware;     // Camera firmware version.      Not available with LucamEnumCameras
    ULONG fpga;         // Camera FPGA version.          Not available with LucamEnumCameras
    ULONG api;          // API version (lucamapi.dll, lucamapi.so.*)
    ULONG driver;       // Device driver version.        Not available with LucamEnumCameras
    ULONG serialnumber; // Unique serial number of a camera.
    ULONG cameraid;     // Also known as camera model id.
} LUCAM_VERSION;
//----------------------------------------------------------------------------------------------------------------

// Frame Format
//----------------------------------------------------------------------------------------------------------------
typedef struct LUCAM_FRAME_FORMAT {
    ULONG xOffset;         // X coordinate on imager of top left corner of subwindow, in pixels
    ULONG yOffset;         // Y coordinate on imager of top left corner of subwindow, in pixels
    ULONG width;           // Width  of subwindow, in pixels
    ULONG height;          // Height of subwindow, in pixls
    ULONG pixelFormat;     // Pixel format LUCAM_PF
    union {
        USHORT subSampleX; // Sub-sample ratio in x direction, in pixels (x:1)
        USHORT binningX;   // Binning ratio in x direction, in pixels (x:1
    };
    USHORT flagsX;         // LUCAM_FRAME_FORMAT_FLAGS_*
    union {
        USHORT subSampleY; // Sub-sample ratio in y direction, in pixels (y:1)
        USHORT binningY;   // Binning ratio in y direction, in pixels (y:1)
    };
    USHORT flagsY;         // LUCAM_FRAME_FORMAT_FLAGS_*
} LUCAM_FRAME_FORMAT;
//----------------------------------------------------------------------------------------------------------------

// Snapshot Settings
//----------------------------------------------------------------------------------------------------------------
// See TakeSnapshot, EnableFastFrames, and EnableSynchronousSnapshots.
typedef struct LUCAM_SNAPSHOT {
    FLOAT exposure;            // Exposure in milliseconds
    FLOAT gain;                // Overall gain as a multiplicative factor
    union {
        struct {
            FLOAT gainRed;     // Gain for Red pixels as multiplicative factor
            FLOAT gainBlue;    // Gain for Blue pixels as multiplicative factor
            FLOAT gainGrn1;    // Gain for Green pixels on Red rows as multiplicative factor
            FLOAT gainGrn2;    // Gain for Green pixels on Blue rows as multiplicative factor
         };
         struct {
             FLOAT gainMag;    // Gain for Magenta pixels as multiplicative factor
             FLOAT gainCyan;   // Gain for Cyan pixels as multiplicative factor
             FLOAT gainYel1;   // Gain for Yellow pixels on Magenta rows as multiplicative factor
             FLOAT gainYel2;   // Gain for Yellow pixels on Cyan rows as multiplicative factor
         };
    };
    union {
        BOOL  useStrobe;       // For backward compatibility
        ULONG strobeFlags;     // Use LUCAM_PROP_FLAG_USE and/or LUCAM_PROP_FLAG_STROBE_FROM_START_OF_EXPOSURE
    };
    FLOAT strobeDelay;         // Time interval from when exposure starts to time the flash is fired in milliseconds
    BOOL  useHwTrigger;        // Wait for hardware trigger
    FLOAT timeout;             // Maximum time to wait for hardware trigger prior to returning from function in milliseconds
    LUCAM_FRAME_FORMAT format; // Frame format for data
    ULONG shutterType;
    FLOAT exposureDelay;
    union {
        BOOL  bufferlastframe; // Set to TRUE if you want TakeFastFrame to return an already received frame.
        ULONG ulReserved1;
    };
    ULONG ulReserved2;         // Must be set to 0
    FLOAT flReserved1;         // Must be set to 0
    FLOAT flReserved2;         // Must be set to 0
} LUCAM_SNAPSHOT;
//----------------------------------------------------------------------------------------------------------------

// Conversion Settings
//----------------------------------------------------------------------------------------------------------------
// For use with LucamConvertFrame*
typedef struct LUCAM_CONVERSION {
    ULONG DemosaicMethod;     // LUCAM_DM_*
    ULONG CorrectionMatrix;   // LUCAM_CM_*
} LUCAM_CONVERSION;
//----------------------------------------------------------------------------------------------------------------

// Conversion Parameters
//----------------------------------------------------------------------------------------------------------------
// For the ConvertFrameTo*Ex functions
typedef struct LUCAM_CONVERSION_PARAMS {
    ULONG Size;               // Must be set to sizeof this struct
    ULONG DemosaicMethod;     // LUCAM_DM_*
    ULONG CorrectionMatrix;   // LUCAM_CM_*
    BOOL  FlipX;
    BOOL  FlipY;
    FLOAT Hue;
    FLOAT Saturation;
    BOOL  UseColorGainsOverWb;
    union {
        struct {
            FLOAT DigitalGain;
            FLOAT DigitalWhiteBalanceU;
            FLOAT DigitalWhiteBalanceV;
        };
        struct {
            FLOAT DigitalGainRed;
            FLOAT DigitalGainGreen;
            FLOAT DigitalGainBlue;
        };
    };
} LUCAM_CONVERSION_PARAMS, *PLUCAM_CONVERSION_PARAMS;
//----------------------------------------------------------------------------------------------------------------

// Image Format
//----------------------------------------------------------------------------------------------------------------
typedef struct LUCAM_IMAGE_FORMAT {
    ULONG Size;         // Must be set to sizeof this struct
    ULONG Width;
    ULONG Height;
    ULONG PixelFormat;  // LUCAM_PF_*
    ULONG ImageSize;

    ULONG LucamReserved[8];

} LUCAM_IMAGE_FORMAT, *PLUCAM_IMAGE_FORMAT;
//----------------------------------------------------------------------------------------------------------------

//******************************************************************************
// Stream Statistics
//******************************************************************************
typedef struct _LUCAM_STREAM_STATS
{
    ULONG FramesCompleted;
    ULONG FramesDropped;
    ULONG ActualFramesDropped;
    union
    {
        struct
        {
            ULONG ShortErrors;
            ULONG XactErrors;
            ULONG BabbleErrors;
            ULONG OtherErrors;
        }USB;
        struct
        {
            ULONG ShortErrors;
            ULONG XactErrors;
            ULONG BabbleErrors;
            ULONG OtherErrors;


            ULONG TransfersOutOfOrderErrors;
            ULONG PendingFrames;
            ULONG PendingUsbTransfers;
        }USB2; // Version 2 of this.
        struct
        {
            ULONG ExpectedResend;
            ULONG LostPacket;
            ULONG DataOverrun;
            ULONG PartialLineMissing;
            ULONG FullLineMissing;
            ULONG OtherErrors;

            ULONG ExpectedSingleResend;
            ULONG UnexpectedResend;
            ULONG ResendGroupRequested;
            ULONG ResendPacketRequested;
            ULONG IgnoredPacket;
            ULONG RedundantPacket;
            ULONG PacketOutOfOrder;
            ULONG BlocksDropped;
            ULONG BlockIDsMissing;
            struct
            {
                ULONG ImageError;
                ULONG MissingPackets;
                ULONG StateError;
                ULONG TooManyResends;
                ULONG TooManyConsecutiveResends;
                ULONG ResendsFailure;
            }Result;
        }GEV;
        struct
        {
            ULONG InputBufferAcqSuccess;
            ULONG InputBufferAcqFailures; // Same as FramesDropped and ActualFramesDropped
            ULONG FramesCompletedSuccess; // Same as FramesCompleted
            ULONG FramesCompletedError;

            ULONG PktReceived;
            ULONG PktInLastBlockError;
            ULONG PktInNextBlockError;
            ULONG BlockIdWayAheadError;
            ULONG NonSeqJumpAhead;
            ULONG NonSeqJumpBack;
            ULONG SegmentOverflowError;
            ULONG SegmentCreatedOnDesynch;
            ULONG PktOnlyPrecedingSegment;
            ULONG ResendOnSkip;
            ULONG ResendOnCountdown;
            ULONG PktAlreadyReceived;
            ULONG DesynchFixed;
            ULONG PktDroppedForAcqFailureCur;
            ULONG PktDroppedForAcqFailureNext;
            ULONG PktDiscardedForPreviousFailure;
            ULONG InvalidGvspHeader;
            ULONG InvalidPayloadSize;
            ULONG GvspStatusError;
            ULONG GvspStatusWarning;
            ULONG GvspLeaderReceived;
            ULONG GvspTrailerReceived;
            ULONG GvspPayloadReceived;
        }LSGEV;
    };
} LUCAM_STREAM_STATS, *PLUCAM_STREAM_STATS;
//----------------------------------------------------------------------------------------------------------------

// Subsampling and Binning Description - Used for WINDOWS or MAC
//----------------------------------------------------------------------------------------------------------------
typedef struct _LUCAM_SS_BIN_DESC
{
   UCHAR flags ; // 0x80: X and Y settings must be the same
   UCHAR reserved ;
   UCHAR ssNot1Count ;
   UCHAR binNot1Count ;
   UCHAR ssFormatsNot1[8] ; //
   UCHAR binFormatsNot1[8] ;//
}LUCAM_SS_BIN_DESC, *PLUCAM_SS_BIN_DESC ;
//----------------------------------------------------------------------------------------------------------------

#if defined(LUMENERA_WINDOWS_API) || defined(LUMENERA_LINUX_API)
// IP Configuration - Used for WINDOWS only
//----------------------------------------------------------------------------------------------------------------
typedef struct LGCAM_IP_CONFIGURATION {
    ULONG IPAddress;
    ULONG SubnetMask;
    ULONG DefaultGateway;
} LGCAM_IP_CONFIGURATION;
typedef LGCAM_IP_CONFIGURATION *PLGCAM_IP_CONFIGURATION;
//----------------------------------------------------------------------------------------------------------------
#endif // LUMENERA_WINDOWS_API
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************


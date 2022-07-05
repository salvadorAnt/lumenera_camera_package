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

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
// COMMON Section: Begin
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************

//****************************************************************************************************************
//  Properties
//****************************************************************************************************************
#define LUCAM_PROP_BRIGHTNESS                                       0
#define LUCAM_PROP_CONTRAST                                         1
#define LUCAM_PROP_HUE                                              2
#define LUCAM_PROP_SATURATION                                       3
#define LUCAM_PROP_GAMMA                                            5
#define LUCAM_PROP_EXPOSURE                                         20
#define LUCAM_PROP_IRIS                                             21
#define LUCAM_PROP_FOCUS                                            22
#define LUCAM_PROP_GAIN                                             40
#define LUCAM_PROP_GAIN_RED                                         41
#define LUCAM_PROP_GAIN_BLUE                                        42
#define LUCAM_PROP_GAIN_GREEN1                                      43
#define LUCAM_PROP_GAIN_GREEN2                                      44

// PROP_STILL_*
#define LUCAM_PROP_STILL_EXPOSURE                                   50
#define LUCAM_PROP_STILL_GAIN                                       51
#define LUCAM_PROP_STILL_GAIN_RED                                   52
#define LUCAM_PROP_STILL_GAIN_GREEN1                                53
#define LUCAM_PROP_STILL_GAIN_GREEN2                                54
#define LUCAM_PROP_STILL_GAIN_BLUE                                  55
#define LUCAM_PROP_STILL_STROBE_DELAY                               56

#define LUCAM_PROP_DEMOSAICING_METHOD                               64
#define LUCAM_PROP_CORRECTION_MATRIX                                65
#define LUCAM_PROP_FLIPPING                                         66

// all PROP_DIGITAL_*
#define LUCAM_PROP_DIGITAL_SATURATION                               67 // factor of 1.0
#define LUCAM_PROP_DIGITAL_HUE                                      68 // from -180 to +180
#define LUCAM_PROP_DIGITAL_WHITEBALANCE_U                           69 // from -100 to 100
#define LUCAM_PROP_DIGITAL_WHITEBALANCE_V                           70 // from -100 to 100
#define LUCAM_PROP_DIGITAL_GAIN                                     71 // from 0 to 2, 1 means a gain of 1.0
#define LUCAM_PROP_DIGITAL_GAIN_RED                                 72 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE. SATURATION must not be low.
#define LUCAM_PROP_DIGITAL_GAIN_GREEN                               73 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE. SATURATION must not be low.
#define LUCAM_PROP_DIGITAL_GAIN_BLUE                                74 // from 0 to 2.5, 1 means a gain of 1.0. Relates to GAIN_Y and WHITEBALANCE. SATURATION must not be low.
#define LUCAM_PROP_COLOR_FORMAT                                     80 // Read-only
#define LUCAM_PROP_MAX_WIDTH                                        81 // Read-only
#define LUCAM_PROP_MAX_HEIGHT                                       82 // Read-only
#define LUCAM_PROP_UNIT_WIDTH                                       83 // Read-only
#define LUCAM_PROP_UNIT_HEIGHT                                      84 // Read-only
#define LUCAM_PROP_ABS_FOCUS                                        85 // Requires the auto lens to be initialized
#define LUCAM_PROP_BLACK_LEVEL                                      86
#define LUCAM_PROP_KNEE1_EXPOSURE                                   96
#define LUCAM_PROP_STILL_KNEE1_EXPOSURE                             LUCAM_PROP_KNEE1_EXPOSURE
#define LUCAM_PROP_KNEE2_EXPOSURE                                   97
#define LUCAM_PROP_STILL_KNEE2_EXPOSURE                             LUCAM_PROP_KNEE2_EXPOSURE
#define LUCAM_PROP_STILL_KNEE3_EXPOSURE                             98
#define LUCAM_PROP_VIDEO_KNEE                                       99
#define LUCAM_PROP_KNEE1_LEVEL                                      LUCAM_PROP_VIDEO_KNEE
#define LUCAM_PROP_STILL_EXPOSURE_DELAY                             100
#define LUCAM_PROP_THRESHOLD                                        101
#define LUCAM_PROP_AUTO_EXP_TARGET                                  103
#define LUCAM_PROP_TIMESTAMPS                                       105
#define LUCAM_PROP_SNAPSHOT_CLOCK_SPEED                             106 // 0 is the fastest
#define LUCAM_PROP_AUTO_EXP_MAXIMUM                                 107
#define LUCAM_PROP_TEMPERATURE                                      108
#define LUCAM_PROP_TRIGGER                                          110
#define LUCAM_PROP_TRIGGER_PIN                                      LUCAM_PROP_TRIGGER // Alias
#define LUCAM_PROP_EXPOSURE_INTERVAL                                113
#define LUCAM_PROP_STILL_STROBE_DURATION                            116
#define LUCAM_PROP_SNAPSHOT_COUNT                                   120
#define LUCAM_PROP_AUTO_IRIS_MAX                                    123 // N/A for linux
#define LUCAM_PROP_VIDEO_CLOCK_SPEED                                126 // 0 is the fastest. Check for read-only flag
#define LUCAM_PROP_KNEE2_LEVEL                                      163
#define LUCAM_PROP_STROBE_PIN                                       172
#define LUCAM_PROP_TAP_CONFIGURATION                                176
#define LUCAM_PROP_STILL_TAP_CONFIGURATION                          177
#define LUCAM_PROP_JPEG_QUALITY                                     256 // N/A for linux
//****************************************************************************************************************

//****************************************************************************************************************
//  Property Flags
//****************************************************************************************************************
#define LUCAM_PROP_FLAG_USE                                         0x80000000
#define LUCAM_PROP_FLAG_AUTO                                        0x40000000
#define LUCAM_PROP_FLAG_STROBE_FROM_START_OF_EXPOSURE               0x20000000
#define LUCAM_PROP_FLAG_POLARITY                                    0x10000000
#define LUCAM_PROP_FLAG_BUSY                                        0x00100000
#define LUCAM_PROP_FLAG_ALTERNATE                                   0x00080000
#define LUCAM_PROP_FLAG_UNKNOWN_MAXIMUM                             0x00020000
#define LUCAM_PROP_FLAG_UNKNOWN_MINIMUM                             0x00010000
//****************************************************************************************************************

//****************************************************************************************************************
//  Property-Specific Flags
//****************************************************************************************************************
#define LUCAM_PROP_FLAG_LITTLE_ENDIAN                               0x80000000 // for LUCAM_PROP_COLOR_FORMAT
#define LUCAM_PROP_FLAG_MASTER                                      0x40000000 // for LUCAM_PROP_SYNC_MODE
#define LUCAM_PROP_FLAG_BACKLASH_COMPENSATION                       0x20000000 // for LUCAM_PROP_IRIS and LUCAM_PROP_FOCUS
#define LUCAM_PROP_FLAG_MEMORY_READBACK                             0x08000000 // for LUCAM_PROP_MEMORY
#define LUCAM_PROP_FLAG_USE_FOR_SNAPSHOTS                           0x04000000 // for LUCAM_PROP_IRIS
#define LUCAM_PROP_FLAG_READONLY                                    0x00010000 // for flags param of GetPropertyRange
//****************************************************************************************************************

//****************************************************************************************************************
//  Camera-Specific Flags
//****************************************************************************************************************
// These flags can be used with  LUCAM_PROP_GAMMA, LUCAM_PROP_BRIGHTNESS, and LUCAM_PROP_CONTRAST
// They are available on specific cameras only.
#define LUCAM_PROP_FLAG_RED                                         0x00000001
#define LUCAM_PROP_FLAG_GREEN1                                      0x00000002
#define LUCAM_PROP_FLAG_GREEN2                                      0x00000004
#define LUCAM_PROP_FLAG_BLUE                                        0x00000008
//****************************************************************************************************************

//****************************************************************************************************************
//  Pixel Formats
//****************************************************************************************************************
#define LUCAM_PF_8                                                  0
#define LUCAM_PF_16                                                 1
#define LUCAM_PF_24                                                 2
#define LUCAM_PF_32                                                 6
#define LUCAM_PF_48                                                 7
#define LUCAM_PF_COUNT                                              4
#define LUCAM_PF_FILTER                                             5
//****************************************************************************************************************

//****************************************************************************************************************
//  Color Patterns
//****************************************************************************************************************
// For the LUCAM_PROP_COLOR_FORMAT property
#define LUCAM_CF_MONO                                               0
#define LUCAM_CF_BAYER_RGGB                                         8
#define LUCAM_CF_BAYER_GRBG                                         9
#define LUCAM_CF_BAYER_GBRG                                         10
#define LUCAM_CF_BAYER_BGGR                                         11
//****************************************************************************************************************

//****************************************************************************************************************
//  Flipping Directions
//****************************************************************************************************************
// For the LUCAM_PROP_FLIPPING property
#define LUCAM_PROP_FLIPPING_NONE                                    0
#define LUCAM_PROP_FLIPPING_X                                       1
#define LUCAM_PROP_FLIPPING_Y                                       2
#define LUCAM_PROP_FLIPPING_XY                                      3
//****************************************************************************************************************

//****************************************************************************************************************
//  Tap Configurations
//****************************************************************************************************************
// For LUCAM_PROP_TAP_CONFIGURATION and LUCAM_PROP_STILL_TAP_CONFIGURATION
#define TAP_CONFIGURATION_1X1                                       0
#define TAP_CONFIGURATION_2X1                                       1
#define TAP_CONFIGURATION_1X2                                       2
#define TAP_CONFIGURATION_2X2                                       4
#define TAP_CONFIGURATION_SINGLE                                    TAP_CONFIGURATION_1X1
#define TAP_CONFIGURATION_DUAL                                      TAP_CONFIGURATION_2X1
#define TAP_CONFIGURATION_QUAD                                      TAP_CONFIGURATION_2X2
//****************************************************************************************************************

//****************************************************************************************************************
//  Video Streaming Modes
//****************************************************************************************************************
#define STOP_STREAMING                                              0
#define START_STREAMING                                             1
#define START_DISPLAY                                               2
#define PAUSE_STREAM                                                3
#define START_RGBSTREAM                                             6
//****************************************************************************************************************

//****************************************************************************************************************
//  Demosaicing Methods
//****************************************************************************************************************
#define LUCAM_DM_NONE                                               0
#define LUCAM_DM_FAST                                               1
#define LUCAM_DM_HIGH_QUALITY                                       2
#define LUCAM_DM_HIGHER_QUALITY                                     3
#define LUCAM_DM_SIMPLE                                             8
//****************************************************************************************************************

//****************************************************************************************************************
//  Color Correction Matrices
//****************************************************************************************************************
#define LUCAM_CM_NONE                                               0
#define LUCAM_CM_FLUORESCENT                                        1
#define LUCAM_CM_DAYLIGHT                                           2
#define LUCAM_CM_INCANDESCENT                                       3
#define LUCAM_CM_XENON_FLASH                                        4
#define LUCAM_CM_HALOGEN                                            5
#define LUCAM_CM_LED                                                6
#define LUCAM_CM_DAYLIGHT_H_AND_E                                   7
#define LUCAM_CM_LED_H_AND_E                                        8
#define LUCAM_CM_IDENTITY                                           14
#define LUCAM_CM_CUSTOM                                             15
//****************************************************************************************************************

//****************************************************************************************************************
//  Shutter Types
//****************************************************************************************************************
#define LUCAM_SHUTTER_TYPE_GLOBAL                                   0
#define LUCAM_SHUTTER_TYPE_ROLLING                                  1
//****************************************************************************************************************

//****************************************************************************************************************
//  External Interfaces
//****************************************************************************************************************
// See QueryExternInterface, and SelectExternInterface
#define LUCAM_EXTERN_INTERFACE_USB1                                 1
#define LUCAM_EXTERN_INTERFACE_USB2                                 2
#define LUCAM_EXTERN_INTERFACE_USB3                                 3
#define LUCAM_EXTERN_INTERFACE_GIGEVISION                           4
//****************************************************************************************************************

//****************************************************************************************************************
//  Frame Format Flag(s)
//****************************************************************************************************************
#define LUCAM_FRAME_FORMAT_FLAGS_BINNING                            0x0001

//****************************************************************************************************************
//  Device Notifications
//****************************************************************************************************************
// For use with LucamRegisterEventNotification via callbacks
#define LUCAM_EVENT_START_OF_READOUT                                2
#define LUCAM_EVENT_DEVICE_SURPRISE_REMOVAL                         32
//****************************************************************************************************************


//****************************************************************************************************************
//  Image Formats
//****************************************************************************************************************
// The RGB format for images produced by functions such as
// AddRgbPreviewCallback and ConvertFrameToRgb24
#define LUCAM_RGB_FORMAT_RGB                                        0    // Standard R,G,B
#define LUCAM_RGB_FORMAT_BMP                                        1    // B,G,R, row-inverted, in the format of a Windows BMP.
//****************************************************************************************************************

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
// COMMON Section: End
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************

#if !defined(LUMENERA_MAC_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
// Applicable to Windows & Linux Only: Begin
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************

    //****************************************************************************************************************
    //  Properties
    //****************************************************************************************************************

    #define LUCAM_PROP_LSC_X                                            121
    #define LUCAM_PROP_LSC_Y                                            122

    #define LUCAM_PROP_AUTO_GAIN_MAXIMUM                                170

    #define LUCAM_PROP_TRIGGER_MODE                                     173 // See TRIGGER_MODE_* below
    #define LUCAM_PROP_FOCAL_LENGTH                                     174
    #define LUCAM_PROP_MAX_FRAME_RATE                                   184
    #define LUCAM_PROP_AUTO_GAIN_MINIMUM                                186

    #define LUCAM_PROP_IRIS_STEPS_COUNT                                 188
    #define LUCAM_PROP_GAINHDR                                          189
    #define LUCAM_PROP_STILL_GAINHDR                                    190
    //****************************************************************************************************************


    //****************************************************************************************************************
    //  Property Flags
    //****************************************************************************************************************
    #define LUCAM_PROP_FLAG_SEQUENCABLE                                 0x08000000
    //****************************************************************************************************************







    //****************************************************************************************************************
    //  Trigger Modes
    //****************************************************************************************************************
    // For the LUCAM_PROP_TRIGGER_MODE property.
    #define TRIGGER_MODE_NORMAL                                         0
    #define TRIGGER_MODE_BULB                                           1
    //****************************************************************************************************************




    //****************************************************************************************************************
    //  Metadata Flags
    //****************************************************************************************************************
    // For use with LucamGetMetadata
    // Metadata that may be embedded within an image.
    // Capabilities vary from model to model.
    #define LUCAM_METADATA_FRAME_COUNTER                                1
    #define LUCAM_METADATA_TIMESTAMP                                    2
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  HDR Modes
    //****************************************************************************************************************
    #define HDR_DISABLED                                                0
    #define HDR_ENABLED_PRIMARY_IMAGE                                   1
    #define HDR_ENABLED_SECONDARY_IMAGE                                 2
    #define HDR_ENABLED_MERGED_IMAGE                                    3
    #define HDR_ENABLED_AVERAGED_IMAGE                                  4 // not supported
    #define HDR_PIECEWISE_LINEAR_RESPONSE                               5
    //****************************************************************************************************************

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
// Applicable to Windows & Linux Only: End
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#endif // !defined(LUMENERA_MAC_API)


    //****************************************************************************************************************
    //  Properties
    //****************************************************************************************************************
    #define LUCAM_PROP_SHARPNESS                                        4

    #define LUCAM_PROP_PAN                                              16
    #define LUCAM_PROP_TILT                                             17
    #define LUCAM_PROP_ROLL                                             18
    #define LUCAM_PROP_ZOOM                                             19

    #define LUCAM_PROP_GAIN_MAGENTA                                     LUCAM_PROP_GAIN_RED
    #define LUCAM_PROP_GAIN_CYAN                                        LUCAM_PROP_GAIN_BLUE
    #define LUCAM_PROP_GAIN_YELLOW1                                     LUCAM_PROP_GAIN_GREEN1
    #define LUCAM_PROP_GAIN_YELLOW2                                     LUCAM_PROP_GAIN_GREEN2

    #define LUCAM_PROP_STILL_GAIN_MAGENTA                               LUCAM_PROP_STILL_GAIN_RED
    #define LUCAM_PROP_STILL_GAIN_YELLOW1                               LUCAM_PROP_STILL_GAIN_GREEN1
    #define LUCAM_PROP_STILL_GAIN_YELLOW2                               LUCAM_PROP_STILL_GAIN_GREEN2
    #define LUCAM_PROP_STILL_GAIN_CYAN                                  LUCAM_PROP_STILL_GAIN_BLUE


    #define LUCAM_PROP_FRAME_GATE                                       112
    #define LUCAM_PROP_PWM                                              114
    #define LUCAM_PROP_MEMORY                                           115 // Read-only. Represents number of frames in memory.

    #define LUCAM_PROP_THRESHOLD                                        101
    #define LUCAM_PROP_FAN                                              118
    #define LUCAM_PROP_SYNC_MODE                                        119
    #define LUCAM_PROP_LENS_STABILIZATION                               124
    #define LUCAM_PROP_VIDEO_TRIGGER                                    125

    #define LUCAM_PROP_THRESHOLD_LOW                                    165
    #define LUCAM_PROP_THRESHOLD_HIGH                                   166
    #define LUCAM_PROP_TEMPERATURE2                                     167
    #define LUCAM_PROP_LIGHT_FREQUENCY                                  168
    #define LUCAM_PROP_LUMINANCE                                        169
    #define LUCAM_PROP_IRIS_LATENCY                                     175

    #define LUCAM_PROP_HOST_AUTO_WB_ALGORITHM                           258
    #define LUCAM_PROP_HOST_AUTO_EX_ALGORITHM                           259

    //****************************************************************************************************************
    //  GigE/GEV Specific Properties
    //****************************************************************************************************************
    #define LUCAM_PROP_GEV_IPCONFIG_LLA                                 512 /* Use LUCAM_PROP_FLAG_USE flag */
    #define LUCAM_PROP_GEV_IPCONFIG_DHCP                                513 /* Use LUCAM_PROP_FLAG_USE flag */
    #define LUCAM_PROP_GEV_IPCONFIG_PERSISTENT                          514 /* Use LUCAM_PROP_FLAG_USE flag */
    #define LUCAM_PROP_GEV_IPCONFIG_PERSISTENT_IPADDRESS                515 /* Use a non-cast */
    #define LUCAM_PROP_GEV_IPCONFIG_PERSISTENT_SUBNETMASK               516 /* Use a non-cast */
    #define LUCAM_PROP_GEV_IPCONFIG_PERSISTENT_DEFAULTGATEWAY           517 /* Use a non-cast */
    #define LUCAM_PROP_GEV_SCPD                                         518


#if defined(LUMENERA_WINDOWS_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Windows Only: Begin
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************


    #define LUCAM_API_RGB24_FORMAT                                      LUCAM_RGB_FORMAT_BMP
    #define LUCAM_API_RGB32_FORMAT                                      LUCAM_RGB_FORMAT_BMP
    #define LUCAM_API_RGB48_FORMAT                                      LUCAM_RGB_FORMAT_BMP
    //****************************************************************************************************************

    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Algorithms
    //****************************************************************************************************************
    // For LUCAM_PROP_HOST_AUTO_WB_ALGORITHM and LUCAM_PROP_HOST_AUTO_EX_ALGORITHM
    #define AUTO_ALGORITHM_SIMPLE_AVERAGING                             0
    #define AUTO_ALGORITHM_HISTOGRAM                                    1
    #define AUTO_ALGORITHM_MACROBLOCKS                                  2
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  AVI Streaming Modes
    //****************************************************************************************************************
    #define STOP_AVI                                                    0
    #define START_AVI                                                   1
    #define PAUSE_AVI                                                   2
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  AVI Types
    //****************************************************************************************************************
    #define AVI_RAW_LUMENERA                                            0
    #define AVI_STANDARD_24                                             1
    #define AVI_STANDARD_32                                             2
    #define AVI_XVID_24                                                 3
    #define AVI_STANDARD_8                                              4 // For monochrome only
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Device Notifications - for Lgcam only
    //****************************************************************************************************************
    // For use with LucamRegisterEventNotification via callbacks
    #define LUCAM_EVENT_GPI1_CHANGED                                    4
    #define LUCAM_EVENT_GPI2_CHANGED                                    5
    #define LUCAM_EVENT_GPI3_CHANGED                                    6
    #define LUCAM_EVENT_GPI4_CHANGED                                    7
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Property-Specific Flags
    //****************************************************************************************************************
    #define LUCAM_PROP_FLAG_HW_ENABLE                                   0x40000000 // for VIDEO_TRIGGER (also uses LUCAM_PROP_FLAG_USE)
    #define LUCAM_PROP_FLAG_SW_TRIGGER                                  0x00200000 // for VIDEO_TRIGGER (also uses LUCAM_PROP_FLAG_USE) // Self-cleared
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Pixel Formats
    //****************************************************************************************************************
    #define LUCAM_PF_YUV422                                             3
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Color Patterns
    //****************************************************************************************************************
    // For the LUCAM_PROP_COLOR_FORMAT property
    #define LUCAM_CF_BAYER_CYYM                                         16
    #define LUCAM_CF_BAYER_YCMY                                         17
    #define LUCAM_CF_BAYER_YMCY                                         18
    #define LUCAM_CF_BAYER_MYYC                                         19
    //****************************************************************************************************************

    //****************************************************************************************************************
    //  Color Correction Matrices
    //****************************************************************************************************************
    #define LUCAM_CM_LED                                                6
    //****************************************************************************************************************

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Windows Only: End
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#elif defined(LUMENERA_MAC_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Mac Only: Begin
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************

    //****************************************************************************************************************
    //  Properties
    //****************************************************************************************************************
    #define LUCAM_PROP_MIN_WIDTH                                        LUCAM_PROP_UNIT_WIDTH   // alias
    #define LUCAM_PROP_MIN_HEIGHT                                       LUCAM_PROP_UNIT_HEIGHT  // alias

    //****************************************************************************************************************
    //  Color Patterns
    //****************************************************************************************************************
    // For the LUCAM_PROP_COLOR_FORMAT property
    // Aliases
    #define LUCAM_COLOR_FORMAT_MONO                                     LUCAM_CF_MONO
    #define LUCAM_COLOR_FORMAT_BAYER_RGGB                               LUCAM_CF_BAYER_RGGB
    #define LUCAM_COLOR_FORMAT_BAYER_GRBG                               LUCAM_CF_BAYER_GRBG
    #define LUCAM_COLOR_FORMAT_BAYER_GBRG                               LUCAM_CF_BAYER_GBRG
    #define LUCAM_COLOR_FORMAT_BAYER_BGGR                               LUCAM_CF_BAYER_BGGR
    //****************************************************************************************************************

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Mac Only: End
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#elif defined(LUMENERA_LINUX_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Linux Only: Begin
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************

    //****************************************************************************************************************
    //  Properties
    //****************************************************************************************************************
    #define LUCAM_PROP_TIMESTAMP_HW_RESET                               187
    //****************************************************************************************************************

    #define LUCAM_API_RGB24_FORMAT                                      LUCAM_RGB_FORMAT_RGB
    #define LUCAM_API_RGB32_FORMAT                                      LUCAM_RGB_FORMAT_RGB
    #define LUCAM_API_RGB48_FORMAT                                      LUCAM_RGB_FORMAT_RGB

    //****************************************************************************************************************
    //  Flipping Directions
    //****************************************************************************************************************
    // For the LUCAM_PROP_FLIPPING property
    // Aliases
    #define LUCAM_FLIPPING_NONE                                         LUCAM_PROP_FLIPPING_NONE
    #define LUCAM_FLIPPING_X                                            LUCAM_PROP_FLIPPING_X
    #define LUCAM_FLIPPING_Y                                            LUCAM_PROP_FLIPPING_Y
    #define LUCAM_FLIPPING_XY                                           LUCAM_PROP_FLIPPING_XY
    //****************************************************************************************************************

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  Applicable to Linux Only: End
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#endif





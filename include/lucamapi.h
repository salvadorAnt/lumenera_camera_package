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

#include "ludefines.h"
#include "lucommon.h"
#include "lustructures.h"
#include "lucamerr.h"

#if !defined(LUMENERA_MAC_API) && !defined(LUMENERA_LINUX_API)
    #ifndef LUMENERA_WINDOWS_API
        #define LUMENERA_WINDOWS_API
    #endif
#endif
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  COMMON Section
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#ifdef LUMENERA_WINDOWS_API
LUCAM_API LONG   LUCAM_EXPORT LucamNumCameras                               (VOID);
#else
LUCAM_API LONG   LUCAM_EXPORT LucamNumCameras                               (void);
#endif // LUMENERA_WINDOWS_API

LUCAM_API LONG   LUCAM_EXPORT LucamEnumCameras                              (LUCAM_VERSION *pVersionsArray, ULONG arrayCount);
LUCAM_API HANDLE LUCAM_EXPORT LucamCameraOpen                               (ULONG index);
LUCAM_API BOOL   LUCAM_EXPORT LucamCameraClose                              (HANDLE hCamera);
LUCAM_API BOOL   LUCAM_EXPORT LucamCameraReset                              (HANDLE hCamera);

// Querying error information
#ifdef LUMENERA_WINDOWS_API
LUCAM_API ULONG  LUCAM_EXPORT LucamGetLastError                             (VOID);
#else
LUCAM_API ULONG  LUCAM_EXPORT LucamGetLastError                             (void);
#endif // LUMENERA_WINDOWS_API
LUCAM_API ULONG  LUCAM_EXPORT LucamGetLastErrorForCamera                    (HANDLE hCamera);

LUCAM_API BOOL   LUCAM_EXPORT LucamQueryVersion                             (HANDLE hCamera, LUCAM_VERSION *pVersion);
LUCAM_API BOOL   LUCAM_EXPORT LucamQueryExternInterface                     (HANDLE hCamera, ULONG *pExternInterface);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetCameraId                              (HANDLE hCamera, ULONG *pId);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetHardwareRevision                      (HANDLE hCamera, ULONG *pRevision);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetProperty                              (HANDLE hCamera, ULONG propertyId, FLOAT *pValue, LONG *pFlags);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetProperty                              (HANDLE hCamera, ULONG propertyId, FLOAT   value, LONG   flags);
LUCAM_API BOOL   LUCAM_EXPORT LucamPropertyRange                            (HANDLE hCamera, ULONG propertyId, FLOAT *pMin, FLOAT *pMax, FLOAT *pDefault, LONG *pFlags);

LUCAM_API BOOL   LUCAM_EXPORT LucamSetFormat                                (HANDLE hCamera, LUCAM_FRAME_FORMAT *pFormat, FLOAT   frameRate);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetFormat                                (HANDLE hCamera, LUCAM_FRAME_FORMAT *pFormat, FLOAT *pFrameRate);

LUCAM_API ULONG  LUCAM_EXPORT LucamEnumAvailableFrameRates                  (HANDLE hCamera, ULONG entryCount, FLOAT *pAvailableFrameRates);

LUCAM_API BOOL   LUCAM_EXPORT LucamStreamVideoControl                       (HANDLE hCamera, ULONG controlType, HWND hWnd);

LUCAM_API BOOL   LUCAM_EXPORT LucamTakeVideo                                (HANDLE hCamera, LONG numFrames, BYTE *pData);
LUCAM_API BOOL   LUCAM_EXPORT LucamTakeVideoEx                              (HANDLE hCamera, BYTE *pData, ULONG *pLength, ULONG timeout);
LUCAM_API BOOL   LUCAM_EXPORT LucamCancelTakeVideo                          (HANDLE hCamera);

//
// TakeSnapshot is a convenience function, wrapping the following sequence:
//   BOOL rc = LucamEnableFastFrames(hCamera, pSettings);
//   if (FALSE == rc) { return rc; }
//   rc = LucamTakeFastFrame(hCamera, pData);
//   LucamDisableFastFrames(hCamera);
//   return rc;
//
LUCAM_API BOOL   LUCAM_EXPORT LucamTakeSnapshot                             (HANDLE hCamera, LUCAM_SNAPSHOT *pSettings, BYTE *pData);


LUCAM_API LONG   LUCAM_EXPORT LucamAddStreamingCallback                     (HANDLE hCamera, VOID (LUCAM_EXPORT *VideoFilter)(VOID *pContext, BYTE *pData, ULONG dataLength), VOID *pCBContext);
LUCAM_API BOOL   LUCAM_EXPORT LucamRemoveStreamingCallback                  (HANDLE hCamera, LONG callbackId);

LUCAM_API LONG   LUCAM_EXPORT LucamAddRgbPreviewCallback                    (HANDLE hCamera, VOID (LUCAM_EXPORT *RgbVideoFilter)(VOID *pContext, BYTE *pData, ULONG dataLength, ULONG unused), VOID *pContext, ULONG rgbPixelFormat);
LUCAM_API BOOL   LUCAM_EXPORT LucamRemoveRgbPreviewCallback                 (HANDLE hCamera, LONG callbackId);
LUCAM_API BOOL   LUCAM_EXPORT LucamQueryRgbPreviewPixelFormat               (HANDLE hCamera, ULONG *pRgbPixelFormat);

LUCAM_API LONG   LUCAM_EXPORT LucamAddSnapshotCallback                      (HANDLE hCamera, VOID (LUCAM_EXPORT *SnapshotCallback)(VOID *pContext, BYTE *pData, ULONG dataLength), VOID *pCBContext);
LUCAM_API BOOL   LUCAM_EXPORT LucamRemoveSnapshotCallback                   (HANDLE hCamera, LONG callbackId);

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale8Ex               (HANDLE hCamera, BYTE   *pDest, const BYTE   *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale16Ex              (HANDLE hCamera, USHORT *pDest, const USHORT *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb24                      (HANDLE hCamera, BYTE   *pDest, BYTE   *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb32                      (HANDLE hCamera, BYTE   *pDest, BYTE   *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb48                      (HANDLE hCamera, USHORT *pDest, USHORT *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb24Ex                    (HANDLE hCamera, BYTE   *pDest, const BYTE   *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb32Ex                    (HANDLE hCamera, BYTE   *pDest, const BYTE   *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb48Ex                    (HANDLE hCamera, USHORT *pDest, const USHORT *pSrc, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);

LUCAM_API BOOL   LUCAM_EXPORT LucamSetupCustomMatrix                        (HANDLE hCamera, FLOAT *pMatrix);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetCurrentMatrix                         (HANDLE hCamera, FLOAT *pMatrix);

LUCAM_API BOOL   LUCAM_EXPORT LucamEnableFastFrames                         (HANDLE hCamera, LUCAM_SNAPSHOT *pSettings);
LUCAM_API BOOL   LUCAM_EXPORT LucamTakeFastFrame                            (HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL   LUCAM_EXPORT LucamForceTakeFastFrame                       (HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL   LUCAM_EXPORT LucamTakeFastFrameNoTrigger                   (HANDLE hCamera, BYTE *pData);
LUCAM_API BOOL   LUCAM_EXPORT LucamDisableFastFrames                        (HANDLE hCamera);
LUCAM_API BOOL   LUCAM_EXPORT LucamTriggerFastFrame                         (HANDLE hCamera);

LUCAM_API BOOL   LUCAM_EXPORT LucamSetTriggerMode                           (HANDLE hCamera, BOOL useHwTrigger);
LUCAM_API BOOL   LUCAM_EXPORT LucamCancelTakeFastFrame                      (HANDLE hCamera);


LUCAM_API BOOL   LUCAM_EXPORT LucamGetTruePixelDepth                        (HANDLE hCamera, ULONG *pCount);

LUCAM_API BOOL   LUCAM_EXPORT LucamGpioRead                                 (HANDLE hCamera, BYTE *pGpoValues, BYTE *pGpiValues);
LUCAM_API BOOL   LUCAM_EXPORT LucamGpioWrite                                (HANDLE hCamera, BYTE   gpoValues);
LUCAM_API BOOL   LUCAM_EXPORT LucamGpoSelect                                (HANDLE hCamera, BYTE gpoEnable);    // Selects between GPO output or alternate function
LUCAM_API BOOL   LUCAM_EXPORT LucamGpioConfigure                            (HANDLE hCamera, BYTE enableOutput); // Enables output drive on a pin.

LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoExposure                      (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoGain                          (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoWhiteBalance                  (HANDLE hCamera,               ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamDigitalWhiteBalance                      (HANDLE hCamera,               ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoWhiteBalanceEx                (HANDLE hCamera, FLOAT redOverGreen, FLOAT blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetVideoImageFormat                      (HANDLE hCamera, LUCAM_IMAGE_FORMAT *pImageFormat);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetStillImageFormat                      (HANDLE hCamera, LUCAM_IMAGE_FORMAT *pImageFormat);

// On-host Tap Correction
LUCAM_API BOOL   LUCAM_EXPORT LucamPerformDualTapCorrection                 (HANDLE hCamera, BYTE *pFrame, const LUCAM_IMAGE_FORMAT *pImageFormat);
LUCAM_API BOOL   LUCAM_EXPORT LucamPerformMultiTapCorrection                (HANDLE hCamera, BYTE *pFrame, const LUCAM_IMAGE_FORMAT *pImageFormat);


LUCAM_API BOOL   LUCAM_EXPORT LucamSaveImageEx                              (HANDLE hCamera, ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const CHAR  *pFilename);
LUCAM_API BOOL   LUCAM_EXPORT LucamSaveImageWEx                             (HANDLE hCamera, ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const WCHAR *pFilename);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetSubsampleBinDescription               (HANDLE hCamera, LUCAM_SS_BIN_DESC *pDesc) ;

//****************************************************************************************************************
//  Deprecated API Function(s)
//****************************************************************************************************************
LUCAM_API LUCAM_DEPRECATED BOOL LUCAM_EXPORT LucamSaveImage                 (ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const CHAR  *pFilename);
LUCAM_API LUCAM_DEPRECATED BOOL LUCAM_EXPORT LucamSaveImageW                (ULONG width, ULONG height, ULONG pixelFormat, BYTE *pData, const WCHAR *pFilename);


//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************


#if defined(LUMENERA_WINDOWS_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  WINDOWS Section
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
LUCAM_API ULONG  LUCAM_EXPORT LucamQueryStats                               (HANDLE hCamera, BOOL stillStream, LUCAM_STREAM_STATS *pStats, ULONG sizeofStats);

LUCAM_API BOOL   LUCAM_EXPORT LucamDisplayPropertyPage                      (HANDLE hCamera, HWND hParentWnd);
LUCAM_API BOOL   LUCAM_EXPORT LucamDisplayVideoFormatPage                   (HANDLE hCamera, HWND hParentWnd);

LUCAM_API BOOL   LUCAM_EXPORT LucamQueryDisplayFrameRate                    (HANDLE hCamera, FLOAT *pValue);

#ifdef __cplusplus
LUCAM_API BOOL   LUCAM_EXPORT LucamCreateDisplayWindow                      (HANDLE hCamera, LPCSTR lpTitle = NULL, DWORD dwStyle = WS_OVERLAPPED|WS_MINIMIZEBOX|WS_CAPTION|WS_SYSMENU|WS_VISIBLE, int x = 0, int y = 0, int width = 0, int height = 0, HWND hParent = NULL, HMENU childId = NULL);
LUCAM_API BOOL   LUCAM_EXPORT LucamAdjustDisplayWindow                      (HANDLE hCamera, LPCSTR lpTitle = NULL, int x = 0, int y = 0, int width = 0, int height = 0);
#else
LUCAM_API BOOL   LUCAM_EXPORT LucamCreateDisplayWindow                      (HANDLE hCamera, LPCSTR lpTitle, DWORD dwStyle, int x, int y, int width, int height, HWND hParent, HMENU childId);
LUCAM_API BOOL   LUCAM_EXPORT LucamAdjustDisplayWindow                      (HANDLE hCamera, LPCSTR lpTitle, int x, int y, int width, int height);
#endif // __cplusplus
LUCAM_API BOOL   LUCAM_EXPORT LucamDestroyDisplayWindow                     (HANDLE hCamera);

LUCAM_API BOOL   LUCAM_EXPORT LucamReadRegister                             (HANDLE hCamera, LONG address, LONG numReg, LONG *pValue);
LUCAM_API BOOL   LUCAM_EXPORT LucamWriteRegister                            (HANDLE hCamera, LONG address, LONG numReg, LONG *pValue);

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale8                 (HANDLE hCamera, BYTE   *pDest, BYTE   *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale16                (HANDLE hCamera, USHORT *pDest, USHORT *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API VOID   LUCAM_EXPORT LucamConvertBmp24ToRgb24                      (UCHAR *pFrame, ULONG width, ULONG height);

LUCAM_API BOOL   LUCAM_EXPORT LucamStreamVideoControlAVI                    (HANDLE hCamera, ULONG controlType, LPCWSTR pFileName, HWND hWnd);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertRawAVIToStdVideo                  (HANDLE hCamera, const WCHAR *pOutputFileName, const WCHAR *pInputFileName, ULONG outputType);

LUCAM_API HANDLE LUCAM_EXPORT LucamPreviewAVIOpen                           (const WCHAR *pFileName);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIClose                          (HANDLE hAVI);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIControl                        (HANDLE hAVI, ULONG previewControlType, HWND previewWindow);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetDuration                    (HANDLE hAVI, LONGLONG *pDurationMinutes, LONGLONG *pDurationSeconds, LONGLONG *pDurationMilliseconds, LONGLONG *pDurationMicroSeconds);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetFrameCount                  (HANDLE hAVI, LONGLONG *pFrameCount);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetFrameRate                   (HANDLE hAVI, FLOAT *pFrameRate);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVISetPositionFrame               (HANDLE hAVI, LONGLONG  pPositionFrame);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetPositionFrame               (HANDLE hAVI, LONGLONG *pPositionFrame);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVISetPositionTime                (HANDLE hAVI, LONGLONG   positionMinutes, LONGLONG   positionSeconds, LONGLONG   positionMilliSeconds, LONGLONG   positionMicroSeconds);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetPositionTime                (HANDLE hAVI, LONGLONG *pPositionMinutes, LONGLONG *pPositionSeconds, LONGLONG *pPositionMilliSeconds, LONGLONG *pPositionMicroSeconds);
LUCAM_API BOOL   LUCAM_EXPORT LucamPreviewAVIGetFormat                      (HANDLE hAVI, LONG *width, LONG *height, LONG *fileType, LONG *bitDepth);

LUCAM_API HANDLE LUCAM_EXPORT LucamEnableSynchronousSnapshots               (ULONG numberOfCameras, HANDLE *phCameras, LUCAM_SNAPSHOT **ppSettings);
LUCAM_API BOOL   LUCAM_EXPORT LucamTakeSynchronousSnapshots                 (HANDLE syncSnapsHandle, BYTE **ppBuffers);
LUCAM_API BOOL   LUCAM_EXPORT LucamDisableSynchronousSnapshots              (HANDLE syncSnapsHandle);

LUCAM_API BOOL   LUCAM_EXPORT LucamLedSet                                   (HANDLE hCamera, ULONG led);

LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoExposureEx                    (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT lightingPeriod /* ms, should be 8.333 in North America*/);

LUCAM_API BOOL   LUCAM_EXPORT LucamDigitalWhiteBalanceEx                    (HANDLE hCamera, FLOAT redOverGreen, FLOAT blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamAdjustWhiteBalanceFromSnapshot           (HANDLE hCamera, LUCAM_SNAPSHOT *pSettings, BYTE *pData, FLOAT redOverGreen, FLOAT blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoIris                          (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height);
LUCAM_API BOOL   LUCAM_EXPORT LucamContinuousAutoExposureEnable             (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT lightingPeriod /* ms, should be 8.333 in North America */);
LUCAM_API BOOL   LUCAM_EXPORT LucamContinuousAutoExposureDisable            (HANDLE hCamera);

LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusStart                           (HANDLE hCamera, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT putZeroThere1, FLOAT putZeroThere2, FLOAT putZeroThere3, BOOL (LUCAM_EXPORT * ProgressCallback)(VOID *context, FLOAT percentageCompleted), VOID *contextForCallback);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusWait                            (HANDLE hCamera, DWORD timeout);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusStop                            (HANDLE hCamera);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusQueryProgress                   (HANDLE hCamera, FLOAT *pPercentageCompleted);
LUCAM_API BOOL   LUCAM_EXPORT LucamInitAutoLens                             (HANDLE hCamera, BOOL force);

// Lookup table
LUCAM_API BOOL   LUCAM_EXPORT LucamSetup8bitsLUT                            (HANDLE hCamera, UCHAR *pLut, ULONG length);   // Length must be 0 or 256
LUCAM_API BOOL   LUCAM_EXPORT LucamSetup8bitsColorLUT                       (HANDLE hCamera, UCHAR *pLut, ULONG length, BOOL applyOnRed, BOOL applyOnGreen1, BOOL applyOnGreen2 , BOOL applyOnBlue);   // Length must be 0 or 256

// RS-232
LUCAM_API int    LUCAM_EXPORT LucamRs232Transmit                            (HANDLE hCamera, CHAR *pData, int length);
LUCAM_API int    LUCAM_EXPORT LucamRs232Receive                             (HANDLE hCamera, CHAR *pData, int maxLength);
LUCAM_API BOOL   LUCAM_EXPORT LucamAddRs232Callback                         (HANDLE hCamera, VOID (LUCAM_EXPORT * callback)(VOID *), VOID *context);
LUCAM_API VOID   LUCAM_EXPORT LucamRemoveRs232Callback                      (HANDLE hCamera);

// In-camera persistent buffers
LUCAM_API BOOL   LUCAM_EXPORT LucamPermanentBufferRead                      (HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);
LUCAM_API BOOL   LUCAM_EXPORT LucamPermanentBufferWrite                     (HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);

LUCAM_API BOOL   LUCAM_EXPORT LucamSetTimeout                               (HANDLE hCamera, BOOL still, FLOAT timeout);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetTimestampFrequency                    (HANDLE hCamera, ULONGLONG* pTimestampTickFrequency);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetTimestamp                             (HANDLE hCamera, ULONGLONG* pTimestamp);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetTimestamp                             (HANDLE hCamera, ULONGLONG   timestamp);
LUCAM_API BOOL   LUCAM_EXPORT LucamEnableTimestamp                          (HANDLE hCamera, BOOL     enable);
LUCAM_API BOOL   LUCAM_EXPORT LucamIsTimestampEnabled                       (HANDLE hCamera, BOOL* pIsEnabled);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetMetadata                              (HANDLE hCamera, BYTE* pImageBuffer, LUCAM_IMAGE_FORMAT* pFormat, ULONG metaDataIndex, ULONGLONG* pMetaData);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetDualGainFactor                        (HANDLE hCamera, BYTE *pValue);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetDualGainFactor                        (HANDLE hCamera, BYTE value);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetPiecewiseLinearResponseParameters     (HANDLE hCamera, BYTE *pKneepoint, ULONG *pGainDivider);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetPiecewiseLinearResponseParameters     (HANDLE hCamera, BYTE kneepoint, ULONG gainDivider);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetHdrMode                               (HANDLE hCamera, BYTE *pValue);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetHdrMode                               (HANDLE hCamera, BYTE value);

LUCAM_API PVOID  LUCAM_EXPORT LucamRegisterEventNotification                (HANDLE hCamera, DWORD eventId, HANDLE hEvent);
LUCAM_API BOOL   LUCAM_EXPORT LucamUnregisterEventNotification              (HANDLE hCamera, PVOID pEventInformation);

// On-host Tap Correction
LUCAM_API BOOL   LUCAM_EXPORT LucamPerformMonoGridCorrection                (HANDLE hCamera, BYTE *pFrame, const LUCAM_IMAGE_FORMAT *pImageFormat);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetImageIntensity                        (HANDLE hCamera, BYTE *pFrame, LUCAM_IMAGE_FORMAT *pImageFormat , ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT *pIntensity, FLOAT *pRedIntensity, FLOAT *pGreen1Intensity, FLOAT *pGreen2Intensity, FLOAT *pBlueIntensity);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoRoiGet                               (HANDLE hCamera, LONG *pStartX, LONG *pStartY, LONG *pWidth, LONG *pHeight);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoRoiSet                               (HANDLE hCamera, LONG   startX, LONG   startY, LONG   width, LONG   height);
LUCAM_API BOOL   LUCAM_EXPORT LucamDataLsbAlign                             (HANDLE hCamera, LUCAM_IMAGE_FORMAT *pLif, UCHAR *pData);
LUCAM_API BOOL   LUCAM_EXPORT LucamEnableInterfacePowerSpecViolation        (HANDLE hCamera, BOOL     enable);
LUCAM_API BOOL   LUCAM_EXPORT LucamIsInterfacePowerSpecViolationEnabled     (HANDLE hCamera, BOOL* pIsEnabled);
LUCAM_API BOOL   LUCAM_EXPORT LucamSelectExternInterface                    (ULONG externInterface); // The API defaults to USB

LUCAM_API BOOL   LUCAM_EXPORT LgcamGetIPConfiguration                       (ULONG index, UCHAR cameraMac[6], LGCAM_IP_CONFIGURATION *pCameraConfiguration, UCHAR hostMac[6], LGCAM_IP_CONFIGURATION *pHostConfiguration);
LUCAM_API BOOL   LUCAM_EXPORT LgcamSetIPConfiguration                       (ULONG index, LGCAM_IP_CONFIGURATION *pCameraConfiguration);


//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************


#elif defined(LUMENERA_MAC_API)
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  MAC Section
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
LUCAM_API BOOL   LUCAM_EXPORT LucamOneShotAutoExposureEx                    (HANDLE hCamera, UCHAR target, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT lightingPeriod /* ms, should be 8.333 in North America*/);

LUCAM_API LONG   LUCAM_EXPORT LucamRegisterCallbackNotification             (HANDLE hCamera, DWORD eventId, int (*callback)(void *context, ULONG eventId), void *contextForCallback);
LUCAM_API BOOL   LUCAM_EXPORT LucamUnregisterCallbackNotification           (HANDLE hCamera, LONG callbackId);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetSubsampleBinDescription               (HANDLE hCamera, LUCAM_SS_BIN_DESC *pDesc) ;

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale8                 (HANDLE hCamera, BYTE   *pDest, BYTE   *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale16                (HANDLE hCamera, USHORT *pDest, USHORT *pSrc, ULONG width, ULONG height, ULONG pixelFormat, LUCAM_CONVERSION *pParams);

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#else
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//  LINUX Section
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
LUCAM_API ULONG  LUCAM_EXPORT LucamQueryStats                               (HANDLE hCamera, BOOL stillStream, LUCAM_STREAM_STATS *pStats, ULONG sizeofStats);

LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusStart                           (HANDLE hCamera, ULONG startX, ULONG startY, ULONG width, ULONG height, FLOAT putZeroThere1, FLOAT putZeroThere2, FLOAT putZeroThere3, BOOL (LUCAM_EXPORT * ProgressCallback)(VOID *context, FLOAT percentageCompleted), VOID *contextForCallback);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusWait                            (HANDLE hCamera, DWORD timeout);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusStop                            (HANDLE hCamera);
LUCAM_API BOOL   LUCAM_EXPORT LucamAutoFocusQueryProgress                   (HANDLE hCamera, FLOAT *pPercentageCompleted);
LUCAM_API BOOL   LUCAM_EXPORT LucamInitAutoLens                             (HANDLE hCamera, BOOL force);

// In-camera persistent buffers
LUCAM_API BOOL   LUCAM_EXPORT LucamPermanentBufferRead                      (HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);
LUCAM_API BOOL   LUCAM_EXPORT LucamPermanentBufferWrite                     (HANDLE hCamera, UCHAR *pBuf, ULONG offset, ULONG length);

LUCAM_API BOOL   LUCAM_EXPORT LucamAdjustWhiteBalanceFromSnapshot           (HANDLE hCamera, LUCAM_SNAPSHOT *pSettings, BYTE *pData, FLOAT redOverGreen, FLOAT blueOverGreen, ULONG startX, ULONG startY, ULONG width, ULONG height);

LUCAM_API BOOL   LUCAM_EXPORT LucamSetTimeout                               (HANDLE hCamera, BOOL still, FLOAT timeout);

// Lookup table
LUCAM_API BOOL   LUCAM_EXPORT LucamSetup8bitsLUT                            (HANDLE hCamera, UCHAR *pLut, ULONG length);   // Length must be 0 or 256
LUCAM_API BOOL   LUCAM_EXPORT LucamSetup8bitsColorLUT                       (HANDLE hCamera, UCHAR *pLut, ULONG length, BOOL applyOnRed, BOOL applyOnGreen1, BOOL applyOnGreen2 , BOOL applyOnBlue);   // Length must be 0 or 256

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale8Ex               (HANDLE hCamera, BYTE   *pDest, const BYTE   *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToGreyscale16Ex              (HANDLE hCamera, USHORT *pDest, const USHORT *pSrc, LUCAM_IMAGE_FORMAT *pImageFormat, LUCAM_CONVERSION_PARAMS *pParams);

LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb24NC                    (BYTE *pDest, const BYTE *pSrc, const BYTE *pImageProperties);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb32NC                    (BYTE *pDest, const BYTE *pSrc, const BYTE *pImageProperties);
LUCAM_API BOOL   LUCAM_EXPORT LucamConvertFrameToRgb48NC                    (USHORT *pDest, const USHORT *pSrc, const BYTE *pImageProperties);
LUCAM_API ULONG  LUCAM_EXPORT LucamGetImageProperties                       (HANDLE hCamera, BYTE *pImgProp, ULONG size, const LUCAM_IMAGE_FORMAT *pImageFormat, const LUCAM_CONVERSION_PARAMS *pParams);

LUCAM_API BOOL   LUCAM_EXPORT LucamGetTimestampFrequency                    (HANDLE hCamera, ULONGLONG* pTimestampTickFrequency);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetTimestamp                             (HANDLE hCamera, ULONGLONG* pTimestamp);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetTimestamp                             (HANDLE hCamera, ULONGLONG   timestamp);
LUCAM_API BOOL   LUCAM_EXPORT LucamEnableTimestamp                          (HANDLE hCamera, BOOL     enable);
LUCAM_API BOOL   LUCAM_EXPORT LucamIsTimestampEnabled                       (HANDLE hCamera, BOOL* pIsEnabled);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetMetadata                              (HANDLE hCamera, BYTE* pImageBuffer, LUCAM_IMAGE_FORMAT* pFormat, ULONG metaDataIndex, ULONGLONG* pMetaData);
LUCAM_API BOOL   LUCAM_EXPORT LucamLedSet                                   (HANDLE hCamera, ULONG led);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetDualGainFactor                        (HANDLE hCamera, BYTE *pValue);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetDualGainFactor                        (HANDLE hCamera, BYTE value);
LUCAM_API BOOL   LUCAM_EXPORT LucamGetHdrMode                               (HANDLE hCamera, BYTE *pValue);
LUCAM_API BOOL   LUCAM_EXPORT LucamSetHdrMode                               (HANDLE hCamera, BYTE value);

LUCAM_API BOOL   LUCAM_EXPORT LgcamGetIPConfiguration                       (ULONG index, UCHAR cameraMac[6], LGCAM_IP_CONFIGURATION *pCameraConfiguration, UCHAR hostMac[6], LGCAM_IP_CONFIGURATION *pHostConfiguration);
LUCAM_API BOOL   LUCAM_EXPORT LgcamSetIPConfiguration                       (ULONG index, LGCAM_IP_CONFIGURATION *pCameraConfiguration);

//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
//****************************************************************************************************************
#endif

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */
#ifndef XSSYNCFUNCTION_H
#define XSSYNCFUNCTION_H

/*!	\addtogroup enums Global enumerations
	@{
*/
/*! \brief Actions to be taken on input triggers */
enum XsSyncFunction
{
	XSF_StartRecording,						/*!< \brief Start recording on trigger or emit trigger when first recording frame is started. \remark Applies to Awinda Station. */
	XSF_StopRecording,						/*!< \brief Stop recording on trigger or emit trigger when recording is stopped. \remark Applies to Awinda Station. */
	XSF_ResetTimer,							/*!< \brief On input trigger, the outgoing timer of the station will be set to 0. \remark Applies to Awinda Station. */
	XSF_TriggerIndication,					/*!< \brief An indication is sent to the driver when trigger is detected. \remark Applies to Awinda Station. */

	XSF_IntervalTransitionMeasurement,		/*!< \brief Emit trigger on an interval transition during measurement and recording. */
	XSF_IntervalTransitionRecording,		/*!< \brief Emit trigger on an interval transition during recording. */
	XSF_GotoOperational,					/*!< \brief Emit trigger when going to Operational mode \remark Applies to Awinda Station. */

	XSF_SampleAndSend,						/*!< \brief Sample a sample and send the MT Data message. \remark Applies to Mt. */
	XSF_SendLatest,							/*!< \brief Send the latest sample. \remark Applies to Mt. */
	XSF_ClockBiasEstimation,				/*!< \brief Do a clock bias estimation on trigger. \remark Applies to Mti-G. */

	XSF_PulseWithModulation,				/*!< \brief Do interval transition measurement with pulse widht modulation. \remark Applies only to Xbusmaster. */

	XSF_StartSampling,						/*!< \brief Start sampling. Data will only be transmitted after this trigger has been recieved. \remark Applies only to Mk4. */

	XSF_Invalid,							/*!< \brief Invalid action \details This indicates the trigger action is not usable. */
	XSF_Count = XSF_Invalid					/*!< \brief Amount of trigger actions */
};
/*! @} */
typedef enum XsSyncFunction XsSyncFunction;

#endif // file guard

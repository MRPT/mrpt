/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/serialization/CSerializable.h>
#include <mrpt/obs/CObservation.h>

namespace mrpt::obs
{
/** Store raw data from a Data Acquisition (DAQ) device, such that input or
 * output analog and digital channels, counters from encoders, etc. at one
 * sampling instant.
 *  All analog values are assumed to be volts.
 * On timing:
 * - CObservation::timestamp corresponds to the time of the first samples in
 * each of the vectors.
 * - CObservationRawDAQ::sample_rate is the sampling rate, in samples per second
 * per channel, as stored by the source driver.
 *
 * \sa CObservation
 * \ingroup mrpt_obs_grp
 */
class CObservationRawDAQ : public CObservation
{
	DEFINE_SERIALIZABLE(CObservationRawDAQ)
   public:
	/** Constructor */
	inline CObservationRawDAQ() = default;
	/** Destructor */
	~CObservationRawDAQ() override = default;
	std::vector<uint8_t> AIN_8bits; /** Readings from 8-bit analog input (ADCs)
									   channels (vector length=channel count) in
									   ADC units. */
	std::vector<uint16_t> AIN_16bits; /** Readings from 16-bit analog input
										 (ADCs) channels (vector length=channel
										 count) in ADC units. */
	std::vector<uint32_t> AIN_32bits; /** Readings from 32-bit analog input
										 (ADCs) channels (vector length=channel
										 count) in ADC units. */
	std::vector<float> AIN_float; /** Readings from analog input (ADCs) channels
									 (vector length=channel count) in Volts. */
	std::vector<double> AIN_double; /** Readings from analog input (ADCs)
									   channels (vector length=channel count) in
									   Volts. */

	/** How many different ADC channels are present in the AIN_* vectors. */
	uint16_t AIN_channel_count{0};
	/** Whether the channels are interleaved (A0 A1 A2 A0 A1 A2...) or not (A0
	 * A0 A0 A1 A1 A1 A2 A2 A2...) in the AIN_* vectors. */
	bool AIN_interleaved{true};

	std::vector<uint8_t> AOUT_8bits; /** Present output values for 8-bit analog
										output (DACs) channels (vector
										length=channel count) in DAC units.*/
	std::vector<uint16_t> AOUT_16bits; /** Present output values for 16-bit
										  analog output (DACs) channels (vector
										  length=channel count) in DAC units.*/
	std::vector<float> AOUT_float; /** Present output values for 16-bit analog
									  output (DACs) channels (vector
									  length=channel count) in volts.*/
	std::vector<double> AOUT_double; /** Present output values for 16-bit analog
										output (DACs) channels (vector
										length=channel count) in volts.*/

	std::vector<uint8_t> DIN; /** Readings from digital inputs; each byte stores
								 8 digital inputs, or 8-bit port. */
	std::vector<uint8_t> DOUT; /** Present digital output values; each byte
								  stores 8 digital inputs, or 8-bit port. */

	std::vector<uint32_t> CNTRIN_32bits; /** Readings from ticks counters, such
											as quadrature encoders. (vector
											length=channel count) in ticks. */
	std::vector<double> CNTRIN_double; /** Readings from ticks counters, such as
										  quadrature encoders. (vector
										  length=channel count) in radians,
										  degrees or any other unit (depends on
										  the source driver). */

	/** The sampling rate, in samples per second per channel */
	double sample_rate{0};

	/** Not used in this class */
	void getSensorPose(mrpt::poses::CPose3D&) const override {}
	/** Not used in this class */
	void setSensorPose(const mrpt::poses::CPose3D&) override {}
	// See base class docs
	void getDescriptionAsText(std::ostream& o) const override;

};  // End of class def.

}  // namespace mrpt::obs

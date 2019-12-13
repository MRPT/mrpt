/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/core/integer_select.h>
#include <mrpt/serialization/CSerializable.h>
#include <map>
#include <memory>
#include <string>

namespace mrpt::obs
{
/** Virtual interface to all pixel-label semantic information structs.
 *
 * See CObservation3DRangeScan::pixelLabels
 * \ingroup mrpt_obs_grp
 */
struct TPixelLabelInfoBase
{
	TPixelLabelInfoBase(unsigned int BITFIELD_BYTES_)
		: BITFIELD_BYTES(BITFIELD_BYTES_)
	{
	}
	virtual ~TPixelLabelInfoBase();

	/** Used in CObservation3DRangeScan::pixelLabels */
	using Ptr = std::shared_ptr<TPixelLabelInfoBase>;
	using TMapLabelID2Name = std::map<uint32_t, std::string>;

	/** The 'semantic' or human-friendly name of the i'th bit in
	 * pixelLabels(r,c) can be found in pixelLabelNames[i] as a std::string
	 */
	TMapLabelID2Name pixelLabelNames;

	const std::string& getLabelName(unsigned int label_idx) const
	{
		auto it = pixelLabelNames.find(label_idx);
		if (it == pixelLabelNames.end())
			throw std::runtime_error("Error: label index has no defined name");
		return it->second;
	}
	void setLabelName(unsigned int label_idx, const std::string& name)
	{
		pixelLabelNames[label_idx] = name;
	}
	/** Check the existence of a label by returning its associated index.
	 * -1 if it does not exist. */
	int checkLabelNameExistence(const std::string& name) const
	{
		std::map<uint32_t, std::string>::const_iterator it;
		for (it = pixelLabelNames.begin(); it != pixelLabelNames.end(); it++)
			if (it->second == name) return it->first;
		return -1;
	}

	/** Resizes the matrix pixelLabels to the given size, setting all
	 * bitfields to zero (that is, all pixels are assigned NONE category).
	 */
	virtual void setSize(const int NROWS, const int NCOLS) = 0;
	/** Mark the pixel(row,col) as classified in the category \a label_idx,
	 * which may be in the range 0 to MAX_NUM_LABELS-1
	 * Note that 0 is a valid label index, it does not mean "no label" \sa
	 * unsetLabel, unsetAll */
	virtual void setLabel(const int row, const int col, uint8_t label_idx) = 0;
	virtual void getLabels(const int row, const int col, uint8_t& labels) = 0;
	/** For the pixel(row,col), removes its classification into the category
	 * \a label_idx, which may be in the range 0 to 7
	 * Note that 0 is a valid label index, it does not mean "no label" \sa
	 * setLabel, unsetAll */
	virtual void unsetLabel(
		const int row, const int col, uint8_t label_idx) = 0;
	/** Removes all categories for pixel(row,col)  \sa setLabel, unsetLabel
	 */
	virtual void unsetAll(const int row, const int col, uint8_t label_idx) = 0;
	/** Checks whether pixel(row,col) has been clasified into category \a
	 * label_idx, which may be in the range 0 to 7
	 * \sa unsetLabel, unsetAll */
	virtual bool checkLabel(
		const int row, const int col, uint8_t label_idx) const = 0;

	void writeToStream(mrpt::serialization::CArchive& out) const;
	static TPixelLabelInfoBase* readAndBuildFromStream(
		mrpt::serialization::CArchive& in);

	/// std stream interface
	friend std::ostream& operator<<(
		std::ostream& out, const TPixelLabelInfoBase& obj)
	{
		obj.Print(out);
		return out;
	}

	/** Minimum number of bytes required to hold MAX_NUM_DIFFERENT_LABELS
	 * bits. */
	const uint8_t BITFIELD_BYTES;

   protected:
	virtual void internal_readFromStream(mrpt::serialization::CArchive& in) = 0;
	virtual void internal_writeToStream(
		mrpt::serialization::CArchive& out) const = 0;
	virtual void Print(std::ostream&) const = 0;
};

/** Pixel-wise semantic label struct.
 *
 * See CObservation3DRangeScan::pixelLabels
 * \ingroup mrpt_obs_grp
 */
template <unsigned int BYTES_REQUIRED_>
struct TPixelLabelInfo : public TPixelLabelInfoBase
{
	using Ptr = std::shared_ptr<TPixelLabelInfo>;
	constexpr static unsigned int BYTES_REQUIRED = BYTES_REQUIRED_;

	/** Automatically-determined integer type of the proper size such that
	 * all labels fit as one bit (max: 64)  */
	using bitmask_t =
		typename mrpt::uint_select_by_bytecount<BYTES_REQUIRED>::type;

	/** Each pixel may be assigned between 0 and MAX_NUM_LABELS-1 'labels'
	 * by
	 * setting to 1 the corresponding i'th bit [0,MAX_NUM_LABELS-1] in the
	 * byte in pixelLabels(r,c).
	 * That is, each pixel is assigned an 8*BITFIELD_BYTES bit-wide
	 * bitfield of possible categories.
	 * \sa hasPixelLabels
	 */
	using TPixelLabelMatrix = mrpt::math::CMatrixDynamic<bitmask_t>;
	TPixelLabelMatrix pixelLabels;

	void setSize(const int NROWS, const int NCOLS) override
	{
		pixelLabels = TPixelLabelMatrix::Zero(NROWS, NCOLS);
	}
	void setLabel(const int row, const int col, uint8_t label_idx) override
	{
		pixelLabels(row, col) |= static_cast<bitmask_t>(1) << label_idx;
	}
	void getLabels(const int row, const int col, uint8_t& labels) override
	{
		labels = pixelLabels(row, col);
	}

	void unsetLabel(const int row, const int col, uint8_t label_idx) override
	{
		pixelLabels(row, col) &= ~(static_cast<bitmask_t>(1) << label_idx);
	}
	void unsetAll(
		const int row, const int col,
		[[maybe_unused]] uint8_t label_idx) override
	{
		pixelLabels(row, col) = 0;
	}
	bool checkLabel(
		const int row, const int col, uint8_t label_idx) const override
	{
		return (pixelLabels(row, col) &
				(static_cast<bitmask_t>(1) << label_idx)) != 0;
	}

	// Ctor: pass identification to parent for deserialization
	TPixelLabelInfo() : TPixelLabelInfoBase(BYTES_REQUIRED_) {}

   protected:
	void internal_readFromStream(mrpt::serialization::CArchive& in) override;
	void internal_writeToStream(
		mrpt::serialization::CArchive& out) const override;
	void Print(std::ostream& out) const override;
};  // end TPixelLabelInfo

}  // namespace mrpt::obs

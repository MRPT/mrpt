/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/opengl/CRenderizable.h>

namespace mrpt::opengl
{
class CCylinder;
/** A cylinder or cone whose base lies in the XY plane.
 * \sa opengl::COpenGLScene,opengl::CDisk
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CCylinder </td> <td> \image html
 * preview_CCylinder.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_opengl_grp
 */
class CCylinder : public CRenderizable
{
	DEFINE_SERIALIZABLE(CCylinder, mrpt::opengl)
	DEFINE_SCHEMA_SERIALIZABLE()
   protected:
	/**
	 * Cylinder's radii. If mBaseRadius==mTopRadius, then the object is an
	 * actual cylinder. If both differ, it's a truncated cone. If one of the
	 * radii is zero, the object is a cone.
	 */
	float mBaseRadius{1}, mTopRadius{1};
	/**
	 * Cylinder's height
	 */
	float mHeight{1};
	/**
	 * Implementation parameters on which depend the number of actually
	 * rendered polygons.
	 */
	uint32_t mSlices{10}, mStacks{10};
	/**
	 * Boolean parameters about including the bases in the object. If both
	 * mHasTopBase and mHasBottomBase are set to false, only the lateral area is
	 * displayed.
	 */
	bool mHasTopBase{true}, mHasBottomBase{true};

   public:
	void render() const override;
	void renderUpdateBuffers() const override;

	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	/**
	 * Configuration of the cylinder's bases display.
	 */
	inline void setHasBases(bool top = true, bool bottom = true)
	{
		mHasTopBase = top;
		mHasBottomBase = bottom;
		CRenderizable::notifyChange();
	}
	/**
	 * Check whether top base is displayed.
	 * \sa hasBottomBase
	 */
	inline bool hasTopBase() const { return mHasTopBase; }
	/**
	 * Check whether bottom base is displayed.
	 * \sa hasTopBase
	 */
	inline bool hasBottomBase() const { return mHasBottomBase; }
	/**
	 * Sets both radii to a single value, thus configuring the object as a
	 * cylinder.
	 * \sa setRadii
	 */
	inline void setRadius(float radius)
	{
		mBaseRadius = mTopRadius = radius;
		CRenderizable::notifyChange();
	}
	/**
	 * Sets both radii independently.
	 * \sa setRadius
	 */
	inline void setRadii(float bottom, float top)
	{
		mBaseRadius = bottom;
		mTopRadius = top;
		CRenderizable::notifyChange();
	}
	/**
	 * Chenges cylinder's height.
	 */
	inline void setHeight(float height)
	{
		mHeight = height;
		CRenderizable::notifyChange();
	}
	/**
	 * Gets the bottom radius.
	 */
	inline float getBottomRadius() const { return mBaseRadius; }
	/**
	 * Gets the top radius.
	 */
	inline float getTopRadius() const { return mTopRadius; }
	/**
	 * Gets the cylinder's height.
	 */
	inline float getHeight() const { return mHeight; }
	/**
	 * Gets how many slices are used in the cylinder's lateral area and in its
	 * bases.
	 */
	inline void setSlicesCount(uint32_t slices)
	{
		mSlices = slices;
		CRenderizable::notifyChange();
	}
	/**
	 * Gets how many stacks are used in the cylinder's lateral area.
	 */
	inline void setStacksCount(uint32_t stacks)
	{
		mStacks = stacks;
		CRenderizable::notifyChange();
	}
	/**
	 * Sets the amount of slices used to display the object.
	 */
	inline uint32_t getSlicesCount() const { return mSlices; }
	/**
	 * Sets the amount of stacks used to display the object.
	 */
	inline uint32_t getStacksCount() const { return mStacks; }
	/** Evaluates the bounding box of this object (including possible children)
	 * in the coordinate frame of the object parent. */
	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;
	/**
	 * Basic empty constructor. Set all parameters to default.
	 */
	CCylinder()

		= default;
	/**
	 * Complete constructor. Allows the configuration of every parameter.
	 */
	/** Constructor with two radii. Allows the construction of any cylinder. */
	CCylinder(
		const float baseRadius, const float topRadius, const float height = 1,
		const int slices = 10, const int stacks = 10)
		: mBaseRadius(baseRadius),
		  mTopRadius(topRadius),
		  mHeight(height),
		  mSlices(slices),
		  mStacks(stacks),
		  mHasTopBase(true),
		  mHasBottomBase(true){};
	/**
	 * Destructor.
	 */
	~CCylinder() override = default;

   private:
	/**
	 * Gets the radius of the circunference located at certain height,
	 * returning false if the cylinder doesn't get that high.
	 */
	inline bool getRadius(float Z, float& r) const
	{
		if (!reachesHeight(Z)) return false;
		r = (Z / mHeight) * (mTopRadius - mBaseRadius) + mBaseRadius;
		return true;
	}
	/**
	 * Checks whether the cylinder exists at some height.
	 */
	inline bool reachesHeight(float Z) const
	{
		return (mHeight < 0) ? (Z >= mHeight && Z <= 0)
							 : (Z <= mHeight && Z >= 0);
	}
};
}  // namespace mrpt::opengl

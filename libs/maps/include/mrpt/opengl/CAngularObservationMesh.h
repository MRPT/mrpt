/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2019, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/maps/CPointsMap.h>
#include <mrpt/math/CMatrixB.h>
#include <mrpt/math/CMatrixDynamic.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/opengl/CRenderizable.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CSetOfTriangles.h>

#include <mrpt/math/geometry.h>

namespace mrpt::opengl
{
/**
 * A mesh built from a set of 2D laser scan observations.
 * Each element of this set is a single scan through the yaw, given a specific
 * pitch.
 * Each scan has a mrpt::poses::CPose3D identifying the origin of the scan,
 * which ideally is the
 * same for every one of them.
 *
 *  <div align="center">
 *  <table border="0" cellspan="4" cellspacing="4" style="border-width: 1px;
 * border-style: solid;">
 *   <tr> <td> mrpt::opengl::CAngularObservationMesh </td> <td> \image html
 * preview_CAngularObservationMesh.png </td> </tr>
 *  </table>
 *  </div>
 *
 * \ingroup mrpt_maps_grp
 */
class CAngularObservationMesh : public CRenderizable
{
	DEFINE_SERIALIZABLE(CAngularObservationMesh, mrpt::opengl)
   public:
	/**
	 * Range specification type, with several uses.
	 */
	struct TDoubleRange
	{
	   private:
		/**
		 * Range type.
		 * If 0, it's specified by an initial and a final value, and an
		 * increment.
		 * If 1, it's specified by an initial and a final value, and a fixed
		 * size of samples.
		 * If 2, it's specified by an aperture, a fixed size of samples and a
		 * boolean variable controlling direction. This type is always
		 * zero-centered.
		 */
		char rangeType;
		/**
		 * Union type with the actual data.
		 * \sa rangeType
		 */
		union rd {
			struct
			{
				double initial;
				double final;
				double increment;
			} mode0;
			struct
			{
				double initial;
				double final;
				size_t amount;
			} mode1;
			struct
			{
				double aperture;
				size_t amount;
				bool negToPos;
			} mode2;
		} rangeData;

	   public:
		/**
		 * Constructor from initial value, final value and range.
		 */
		TDoubleRange(double a, double b, double c) : rangeType(0)
		{
			rangeData.mode0.initial = a;
			rangeData.mode0.final = b;
			rangeData.mode0.increment = c;
		}
		/**
		 * Constructor from initial value, final value and amount of samples.
		 */
		TDoubleRange(double a, double b, size_t c) : rangeType(1)
		{
			rangeData.mode1.initial = a;
			rangeData.mode1.final = b;
			rangeData.mode1.amount = c;
		}
		/**
		 * Constructor from aperture, amount of samples and scan direction.
		 */
		TDoubleRange(double a, size_t b, bool c) : rangeType(2)
		{
			rangeData.mode2.aperture = a;
			rangeData.mode2.amount = b;
			rangeData.mode2.negToPos = c;
		}
		/**
		 * Creates a range of values from the initial value, the final value
		 * and the increment.
		 * \throw std::logic_error if the increment is zero.
		 */
		inline static TDoubleRange CreateFromIncrement(
			double initial, double final, double increment)
		{
			if (increment == 0)
				throw std::logic_error("Invalid increment value.");
			return TDoubleRange(initial, final, increment);
		}
		/**
		 * Creates a range of values from the initial value, the final value
		 * and a desired amount of samples.
		 */
		inline static TDoubleRange CreateFromAmount(
			double initial, double final, size_t amount)
		{
			return TDoubleRange(initial, final, amount);
		}
		/**
		 * Creates a zero-centered range of values from an aperture, an amount
		 * of samples and a direction.
		 */
		inline static TDoubleRange CreateFromAperture(
			double aperture, size_t amount, bool negToPos = true)
		{
			return TDoubleRange(aperture, amount, negToPos);
		}
		/**
		 * Returns the total aperture of the range.
		 * \throw std::logic_error on invalid range type.
		 */
		inline double aperture() const
		{
			switch (rangeType)
			{
				case 0:
					return (mrpt::sign(rangeData.mode0.increment) ==
							mrpt::sign(
								rangeData.mode0.final -
								rangeData.mode0.initial))
							   ? fabs(
									 rangeData.mode0.final -
									 rangeData.mode0.initial)
							   : 0;
				case 1:
					return rangeData.mode1.final - rangeData.mode1.initial;
				case 2:
					return rangeData.mode2.aperture;
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
		/**
		 * Returns the first value of the range.
		 * \throw std::logic_error on invalid range type.
		 */
		inline double initialValue() const
		{
			switch (rangeType)
			{
				case 0:
				case 1:
					return rangeData.mode0.initial;
				case 2:
					return rangeData.mode2.negToPos
							   ? -rangeData.mode2.aperture / 2
							   : rangeData.mode2.aperture / 2;
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
		/**
		 * Returns the last value of the range.
		 * \throw std::logic_error on invalid range type.
		 */
		inline double finalValue() const
		{
			switch (rangeType)
			{
				case 0:
					return (mrpt::sign(rangeData.mode0.increment) ==
							mrpt::sign(
								rangeData.mode0.final -
								rangeData.mode0.initial))
							   ? rangeData.mode0.final
							   : rangeData.mode0.initial;
				case 1:
					return rangeData.mode1.final;
				case 2:
					return rangeData.mode2.negToPos
							   ? rangeData.mode2.aperture / 2
							   : -rangeData.mode2.aperture / 2;
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
		/**
		 * Returns the increment between two consecutive values of the range.
		 * \throw std::logic_error on invalid range type.
		 */
		inline double increment() const
		{
			switch (rangeType)
			{
				case 0:
					return rangeData.mode0.increment;
				case 1:
					return (rangeData.mode1.final - rangeData.mode1.initial) /
						   static_cast<double>(rangeData.mode1.amount - 1);
				case 2:
					return rangeData.mode2.negToPos
							   ? rangeData.mode2.aperture /
									 static_cast<double>(
										 rangeData.mode2.amount - 1)
							   : -rangeData.mode2.aperture /
									 static_cast<double>(
										 rangeData.mode2.amount - 1);
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
		/**
		 * Returns the total amount of values in this range.
		 * \throw std::logic_error on invalid range type.
		 */
		inline size_t amount() const
		{
			switch (rangeType)
			{
				case 0:
					return (mrpt::sign(rangeData.mode0.increment) ==
							mrpt::sign(
								rangeData.mode0.final -
								rangeData.mode0.initial))
							   ? 1 + static_cast<size_t>(ceil(
										 (rangeData.mode0.final -
										  rangeData.mode0.initial) /
										 rangeData.mode0.increment))
							   : 1;
				case 1:
					return rangeData.mode1.amount;
				case 2:
					return rangeData.mode2.amount;
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
		/**
		 * Gets a vector with every value in the range.
		 * \throw std::logic_error on invalid range type.
		 */
		void values(std::vector<double>& vals) const;
		/**
		 * Returns the direction of the scan. True if the increment is
		 * positive, false otherwise.
		 * \throw std::logic_error on invalid range type.
		 */
		inline bool negToPos() const
		{
			switch (rangeType)
			{
				case 0:
					return mrpt::sign(rangeData.mode0.increment) > 0;
				case 1:
					return mrpt::sign(
							   rangeData.mode1.final -
							   rangeData.mode1.initial) > 0;
				case 2:
					return rangeData.mode2.negToPos;
				default:
					throw std::logic_error("Unknown range type.");
			}
		}
	};

	void getBoundingBox(
		mrpt::math::TPoint3D& bb_min,
		mrpt::math::TPoint3D& bb_max) const override;

   protected:
	/** Updates the mesh, if needed. It's a const method, but modifies mutable
	 * content. */
	void updateMesh() const;
	/** Actual set of triangles to be displayed. */
	mutable std::vector<mrpt::opengl::TTriangle> triangles;
	/** Internal method to add a triangle to the mutable mesh. */
	void addTriangle(
		const mrpt::math::TPoint3D& p1, const mrpt::math::TPoint3D& p2,
		const mrpt::math::TPoint3D& p3) const;
	/** Whether the mesh will be displayed wireframe or solid. */
	bool m_Wireframe{true};
	/** Mutable variable which controls if the object has suffered any change
	 * since last time the mesh was updated. */
	mutable bool meshUpToDate{false};
	/** Whether the object may present transparencies or not. */
	bool mEnableTransparency{true};
	/** Mutable object with the mesh's points. */
	mutable mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_data<double>>
		actualMesh;
	/** Scan validity matrix. */
	mutable mrpt::math::CMatrixB validityMatrix;
	/** Observation pitch range. When containing exactly two elements, they
	 * represent the bounds. */
	std::vector<double> pitchBounds;
	/** Actual scan set which is used to generate the mesh.  */
	std::vector<mrpt::obs::CObservation2DRangeScan> scanSet;

   public:
	/**
	 * Basic constructor.
	 */
	CAngularObservationMesh()
		: actualMesh(0, 0), validityMatrix(0, 0), pitchBounds(), scanSet()
	{
	}
	/** Empty destructor. */
	~CAngularObservationMesh() override = default;
	/**
	 * Returns whether the object is configured as wireframe or solid.
	 */
	inline bool isWireframe() const { return m_Wireframe; }
	/**
	 * Sets the display mode for the object. True=wireframe, False=solid.
	 */
	inline void setWireframe(bool enabled = true)
	{
		m_Wireframe = enabled;
		CRenderizable::notifyChange();
	}
	/**
	 * Returns whether the object may be transparent or not.
	 */
	inline bool isTransparencyEnabled() const { return mEnableTransparency; }
	/**
	 * Enables or disables transparencies.
	 */
	inline void enableTransparency(bool enabled = true)
	{
		mEnableTransparency = enabled;
		CRenderizable::notifyChange();
	}

	void render(const RenderContext& rc) const override;
	void renderUpdateBuffers() const override;

	/**
	 * Traces a ray to the object, returning the distance to a given pose
	 * through its X axis.
	 * \sa mrpt::opengl::CRenderizable,trace2DSetOfRays,trace1DSetOfRays
	 */
	bool traceRay(const mrpt::poses::CPose3D& o, double& dist) const override;
	/**
	 * Sets the pitch bounds for this range.
	 */
	void setPitchBounds(const double initial, const double final);
	/**
	 * Sets the pitch bounds for this range.
	 */
	void setPitchBounds(const std::vector<double>& bounds);
	/**
	 * Gets the initial and final pitch bounds for this range.
	 */
	void getPitchBounds(double& initial, double& final) const;
	/**
	 * Gets the pitch bounds for this range.
	 */
	void getPitchBounds(std::vector<double>& bounds) const;
	/**
	 * Gets the scan set.
	 */
	void getScanSet(
		std::vector<mrpt::obs::CObservation2DRangeScan>& scans) const;
	/**
	 * Sets the scan set.
	 */
	bool setScanSet(
		const std::vector<mrpt::obs::CObservation2DRangeScan>& scans);
	/**
	 * Gets the mesh as a set of triangles, for displaying them.
	 * \sa generateSetOfTriangles(std::vector<TPolygon3D>
	 * &),mrpt::opengl::CSetOfTriangles,mrpt::opengl::mrpt::opengl::TTriangle
	 */
	void generateSetOfTriangles(CSetOfTriangles::Ptr& res) const;
	/**
	 * Returns the scanned points as a 3D point cloud. The target pointmap must
	 * be passed as a pointer to allow the use of any derived class.
	 */
	void generatePointCloud(mrpt::maps::CPointsMap* out_map) const;
	/**
	 * Gets a set of lines containing the traced rays, for displaying them.
	 * \sa getUntracedRays,mrpt::opengl::CSetOfLines
	 */
	void getTracedRays(CSetOfLines::Ptr& res) const;
	/**
	 * Gets a set of lines containing the untraced rays, up to a specified
	 * distance, for displaying them.
	 * \sa getTracedRays,mrpt::opengl::CSetOfLines
	 */
	void getUntracedRays(CSetOfLines::Ptr& res, double dist) const;
	/**
	 * Gets the mesh as a set of polygons, to work with them.
	 * \sa generateSetOfTriangles(mrpt::opengl::CSetOfTriangles &)
	 */
	void generateSetOfTriangles(std::vector<mrpt::math::TPolygon3D>& res) const;
	/**
	 * Retrieves the full mesh, along with the validity matrix.
	 */
	void getActualMesh(
		mrpt::math::CMatrixDynamic<mrpt::math::TPoint3D_data<double>>& pts,
		mrpt::math::CMatrixBool& validity) const
	{
		if (!meshUpToDate) updateMesh();
		pts = actualMesh;
		validity = validityMatrix;
	}

   private:
	/**
	 * Internal functor class to trace a ray.
	 */
	template <class T>
	class FTrace1D
	{
	   protected:
		const mrpt::poses::CPose3D& initial;
		const T& e;
		std::vector<double>& values;
		std::vector<char>& valid;

	   public:
		FTrace1D(
			const T& s, const mrpt::poses::CPose3D& p, std::vector<double>& v,
			std::vector<char>& v2)
			: initial(p), e(s), values(v), valid(v2)
		{
		}
		void operator()(double yaw)
		{
			double dist;
			const mrpt::poses::CPose3D pNew =
				initial + mrpt::poses::CPose3D(0.0, 0.0, 0.0, yaw, 0.0, 0.0);
			if (e->traceRay(pNew, dist))
			{
				values.push_back(dist);
				valid.push_back(1);
			}
			else
			{
				values.push_back(0);
				valid.push_back(0);
			}
		}
	};
	/**
	 * Internal functor class to trace a set of rays.
	 */
	template <class T>
	class FTrace2D
	{
	   protected:
		const T& e;
		const mrpt::poses::CPose3D& initial;
		CAngularObservationMesh::Ptr& caom;
		const CAngularObservationMesh::TDoubleRange& yaws;
		std::vector<mrpt::obs::CObservation2DRangeScan>& vObs;
		const mrpt::poses::CPose3D& pBase;

	   public:
		FTrace2D(
			const T& s, const mrpt::poses::CPose3D& p,
			CAngularObservationMesh::Ptr& om,
			const CAngularObservationMesh::TDoubleRange& y,
			std::vector<mrpt::obs::CObservation2DRangeScan>& obs,
			const mrpt::poses::CPose3D& b)
			: e(s), initial(p), caom(om), yaws(y), vObs(obs), pBase(b)
		{
		}
		void operator()(double pitch)
		{
			std::vector<double> yValues;
			yaws.values(yValues);
			mrpt::obs::CObservation2DRangeScan o =
				mrpt::obs::CObservation2DRangeScan();
			const mrpt::poses::CPose3D pNew =
				initial + mrpt::poses::CPose3D(0, 0, 0, 0, pitch, 0);
			std::vector<double> values;
			std::vector<char> valid;
			size_t nY = yValues.size();
			values.reserve(nY);
			valid.reserve(nY);
			for_each(
				yValues.begin(), yValues.end(),
				FTrace1D<T>(e, pNew, values, valid));
			o.aperture = yaws.aperture();
			o.rightToLeft = yaws.negToPos();
			o.maxRange = 10000;
			o.sensorPose = pNew;
			o.deltaPitch = 0;
			o.resizeScan(values.size());
			for (size_t i = 0; i < values.size(); i++)
			{
				o.setScanRange(i, values[i]);
				o.setScanRangeValidity(i, valid[i] != 0);
			}
			vObs.push_back(o);
		}
	};

   public:
	/**
	 * 2D ray tracing (will generate a 3D mesh). Given an object and two
	 * ranges, realizes a scan from the initial pose and stores it in a
	 * CAngularObservationMesh object.
	 * The objective may be a COpenGLScene, a CRenderizable or any children of
	 * its.
	 * \sa mrpt::opengl::CRenderizable,mrpt::opengl::COpenGLScene.
	 */
	template <class T>
	static void trace2DSetOfRays(
		const T& e, const mrpt::poses::CPose3D& initial,
		CAngularObservationMesh::Ptr& caom, const TDoubleRange& pitchs,
		const TDoubleRange& yaws);
	/**
	 * 2D ray tracing (will generate a vectorial mesh inside a plane). Given an
	 * object and a range, realizes a scan from the initial pose and stores it
	 * in a CObservation2DRangeScan object.
	 * The objective may be a COpenGLScene, a CRenderizable or any children of
	 * its.
	 * \sa mrpt::opengl::CRenderizable,mrpt::opengl::COpenGLScene.
	 */
	template <class T>
	static void trace1DSetOfRays(
		const T& e, const mrpt::poses::CPose3D& initial,
		mrpt::obs::CObservation2DRangeScan& obs, const TDoubleRange& yaws)
	{
		std::vector<double> yValues;
		yaws.values(yValues);
		mrpt::aligned_std_vector<float> scanValues;
		mrpt::aligned_std_vector<char> valid;
		size_t nV = yaws.amount();
		scanValues.reserve(nV);
		valid.reserve(nV);
		for_each(
			yValues.begin(), yValues.end(),
			FTrace1D<T>(e, initial, scanValues, valid));
		obs.aperture = yaws.aperture();
		obs.rightToLeft = yaws.negToPos();
		obs.maxRange = 10000;
		obs.sensorPose = initial;
		obs.deltaPitch = 0;
		for (size_t i = 0; i < nV; i++)
		{
			obs.setScanRange(i, scanValues[i]);
			obs.setScanRangeValidity(i, valid[i]);
		}
	}
};

template <class T>
void CAngularObservationMesh::trace2DSetOfRays(
	const T& e, const mrpt::poses::CPose3D& initial,
	CAngularObservationMesh::Ptr& caom, const TDoubleRange& pitchs,
	const TDoubleRange& yaws)
{
	std::vector<double> pValues;
	pitchs.values(pValues);
	std::vector<mrpt::obs::CObservation2DRangeScan> vObs;
	vObs.reserve(pValues.size());
	for_each(
		pValues.begin(), pValues.end(),
		FTrace2D<T>(e, initial, caom, yaws, vObs, initial));
	caom->m_Wireframe = false;
	caom->mEnableTransparency = false;
	caom->setPitchBounds(pValues);
	caom->setScanSet(vObs);
}
}  // namespace mrpt::opengl

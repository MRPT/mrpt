/* +---------------------------------------------------------------------------+
   |                 The Mobile Robot Programming Toolkit (MRPT)               |
   |                                                                           |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2013, Individual contributors, see AUTHORS file        |
   | Copyright (c) 2005-2013, MAPIR group, University of Malaga                |
   | Copyright (c) 2012-2013, University of Almeria                            |
   | All rights reserved.                                                      |
   |                                                                           |
   | Redistribution and use in source and binary forms, with or without        |
   | modification, are permitted provided that the following conditions are    |
   | met:                                                                      |
   |    * Redistributions of source code must retain the above copyright       |
   |      notice, this list of conditions and the following disclaimer.        |
   |    * Redistributions in binary form must reproduce the above copyright    |
   |      notice, this list of conditions and the following disclaimer in the  |
   |      documentation and/or other materials provided with the distribution. |
   |    * Neither the name of the copyright holders nor the                    |
   |      names of its contributors may be used to endorse or promote products |
   |      derived from this software without specific prior written permission.|
   |                                                                           |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       |
   | 'AS IS' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED |
   | TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR|
   | PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS BE LIABLE |
   | FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL|
   | DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR|
   |  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)       |
   | HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,       |
   | STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN  |
   | ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           |
   | POSSIBILITY OF SUCH DAMAGE.                                               |
   +---------------------------------------------------------------------------+ */
#ifndef mrpt_CKinematicChain_H
#define mrpt_CKinematicChain_H

#include <mrpt/poses/CPose3D.h>
#include <mrpt/utils/CSerializable.h>
#include <mrpt/opengl/CSetOfObjects.h>

#include <mrpt/kinematics/link_pragmas.h>

namespace mrpt
{

	namespace kinematics
	{
		DEFINE_SERIALIZABLE_PRE_CUSTOM_BASE_LINKAGE( CKinematicChain, mrpt::utils::CSerializable, KINEMATICS_IMPEXP )

		/** An individual kinematic chain element (one link) which builds up a CKinematicChain.
		  * The parameterization of the SE(3) transformation from the starting point to the end point 
		  * follows a Denavit-Hartenberg standard parameterization: [theta, d, a, alpha].
		  */
		struct KINEMATICS_IMPEXP TKinematicLink
		{
			double  theta;  //!< Rotation from X_i to X_{i+1} (radians)
			double  d;      //!< Distance along Z_i to the common normal between Z_i and Z_{i+1}
			double  a;      //!< Distance along the common normal (in the same direction than the new X_{i+1})
			double  alpha;  //!< Rotation along X_{i+1} to transform Z_i into Z_{i+1}

			bool    is_prismatic; //!< "false": Is revolute ("q_i" is "theta"), "true": is prismatic ("q_i" is "d")

			TKinematicLink(double _theta,double _d, double _a, double _alpha, bool _is_prismatic) : theta(_theta),d(_d),a(_a),alpha(_alpha),is_prismatic(_is_prismatic) {}
			TKinematicLink() : theta(0),d(0),a(0),alpha(0),is_prismatic(false) { }
		};

		KINEMATICS_IMPEXP mrpt::utils::CStream &operator>>(mrpt::utils::CStream &in,TKinematicLink &o);
		KINEMATICS_IMPEXP mrpt::utils::CStream &operator<<(mrpt::utils::CStream &out,const TKinematicLink &o);

		/** A open-loop kinematic chain model, suitable to robotic manipulators.
		  *
		  * \sa CPose3D
		  * \ingroup kinematics_grp
		  */
		class KINEMATICS_IMPEXP CKinematicChain : public mrpt::utils::CSerializable
		{
			// This must be added to any CSerializable derived class:
			DEFINE_SERIALIZABLE( CKinematicChain )
		
		private:
			mutable std::vector<mrpt::opengl::CRenderizablePtr>  m_last_gl_objects; //!< Smart pointers to the last objects for each link, as returned in getAs3DObject(), for usage within update3DObject()

			std::vector<TKinematicLink>  m_links;  //!< The links of this robot arm

		public:
			
			/** Return the number of links */
			size_t size() const { return m_links.size(); }

			/** Erases all links and leave the robot arm empty. */
			void clear();

			/** Appends a new link to the robotic arm, with the given Denavit-Hartenberg parameters (see TKinematicLink for further details) */
			void addLink(double theta, double d, double a, double alpha, bool is_prismatic);

			/** Get a ref to a given link (read-only) */
			const TKinematicLink& getLink(const size_t idx) const;

			/** Get a ref to a given link (read-write) */
			TKinematicLink& getLinkRef(const size_t idx);

			/** Get all the DOFs of the arm at once, returning them in a vector with all the "q_i" values, which 
			  * can be interpreted as rotations (radians) or displacements (meters) depending on links being "revolute" or "prismatic". 
			  * The vector is automatically resized to the correct size (the number of links).
			  * \tparam VECTOR Can be any Eigen vector, mrpt::vector_double, or std::vector<double>
			  */
			template <class VECTOR>
			void getConfiguration(VECTOR &v) const
			{
				const size_t N=m_links.size();
				v.resize(N);
				for (size_t i=0;i<N;i++) {
					if (m_links[i].is_prismatic)
					     v[i] = m_links[i].d;
					else v[i] = m_links[i].theta;
				}
			}

			/** Set all the DOFs of the arm at once, from a vector with all the "q_i" values, which 
			  * are interpreted as rotations (radians) or displacements (meters) depending on links being "revolute" or "prismatic". 
			  * \exception std::exception If the size of the vector doesn't match the number of links.
			  * \tparam VECTOR Can be any Eigen vector, mrpt::vector_double, or std::vector<double>
			  */
			template <class VECTOR>
			void setConfiguration(const VECTOR &v)
			{
				ASSERT_EQUAL_(v.size(),this->size())
				const size_t N=m_links.size();
				for (size_t i=0;i<N;i++) {
					if (m_links[i].is_prismatic)
					     m_links[i].d = v[i];
					else m_links[i].theta = v[i];
				}
			}

			/** Constructs a 3D representation of the kinematic chain, in its current state.
			  * You can call update3DObject() to update the kinematic state of these OpenGL objects in the future, since 
			  * an internal list of smart pointers to the constructed objects is kept internally. This is more efficient 
			  * than calling this method again to build a new representation. 
			  * \sa update3DObject
			  */
			void getAs3DObject(mrpt::opengl::CSetOfObjectsPtr &inout_gl_obj) const;

			/** Read getAs3DObject() for a description. */
			void update3DObject() const;

			/** Go thru all the links of the chain and compute the global pose of each link. The "ground" link pose "pose0" defaults to the origin of coordinates, 
			  * but anything else can be passed as the optional argument. */
			void recomputeAllPoses( mrpt::aligned_containers<mrpt::poses::CPose3D>::vector_t & poses, const mrpt::poses::CPose3D & pose0 = mrpt::poses::CPose3D() )const;


		}; // End of class def.

	} // End of namespace


	// Specialization must occur in the same namespace
	// (This is to ease serialization)
	namespace utils
	{
		using namespace ::mrpt::kinematics;

		MRPT_DECLARE_TTYPENAME(TKinematicLink)

	} // End of namespace

} // End of namespace

#endif

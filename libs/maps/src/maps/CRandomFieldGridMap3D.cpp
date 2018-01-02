/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

#include "maps-precomp.h"  // Precomp header

#include <mrpt/maps/CRandomFieldGridMap3D.h>
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/system/CTicTac.h>
#include <mrpt/io/CFileOutputStream.h>

#include <mrpt/config.h>

#if MRPT_HAS_VTK
#include <vtkStructuredGrid.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkVersion.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkSmartPointer.h>
#endif

using namespace mrpt;
using namespace mrpt::maps;
using namespace std;

IMPLEMENTS_SERIALIZABLE(CRandomFieldGridMap3D, CSerializable, mrpt::maps)

/*---------------------------------------------------------------
						Constructor
  ---------------------------------------------------------------*/
CRandomFieldGridMap3D::CRandomFieldGridMap3D(
	double x_min, double x_max, double y_min, double y_max, double z_min,
	double z_max, double voxel_size, bool call_initialize_now)
	: CDynamicGrid3D<TRandomFieldVoxel>(
		  x_min, x_max, y_min, y_max, z_min, z_max, voxel_size /*xy*/,
		  voxel_size /*z*/),
	  COutputLogger("CRandomFieldGridMap3D")
{
	if (call_initialize_now) this->internal_initialize();
}

/** Changes the size of the grid, erasing previous contents */
void CRandomFieldGridMap3D::setSize(
	const double x_min, const double x_max, const double y_min,
	const double y_max, const double z_min, const double z_max,
	const double resolution_xy, const double resolution_z,
	const TRandomFieldVoxel* fill_value)
{
	MRPT_START;

	CDynamicGrid3D<TRandomFieldVoxel>::setSize(
		x_min, x_max, y_min, y_max, z_min, z_max, resolution_xy, resolution_z,
		fill_value);
	this->internal_initialize();

	MRPT_END;
}

void CRandomFieldGridMap3D::resize(
	double new_x_min, double new_x_max, double new_y_min, double new_y_max,
	double new_z_min, double new_z_max,
	const TRandomFieldVoxel& defaultValueNewCells,
	double additionalMarginMeters)
{
	MRPT_START;

	CDynamicGrid3D<TRandomFieldVoxel>::resize(
		new_x_min, new_x_max, new_y_min, new_y_max, new_z_min, new_z_max,
		defaultValueNewCells, additionalMarginMeters);
	this->internal_initialize(false);

	MRPT_END;
}

void CRandomFieldGridMap3D::clear()
{
	mrpt::containers::CDynamicGrid3D<TRandomFieldVoxel>::clear();
	internal_initialize();
}

void CRandomFieldGridMap3D::internal_initialize(bool erase_prev_contents)
{
	if (erase_prev_contents)
	{
		// Set the gridmap (m_map) to initial values:
		TRandomFieldVoxel def(0, 0);  // mean, std
		fill(def);
	}

	// Reset gmrf:
	m_gmrf.setVerbosityLevel(this->getMinLoggingLevel());
	if (erase_prev_contents)
	{
		m_gmrf.clear();
		m_mrf_factors_activeObs.clear();
	}
	else
	{
		// Only remove priors, leave observations:
		m_gmrf.clearAllConstraintsByType_Binary();
	}
	m_mrf_factors_priors.clear();

	// Initiating prior:
	const size_t nodeCount = m_map.size();
	ASSERT_EQUAL_(nodeCount, m_size_x * m_size_y * m_size_z);
	ASSERT_EQUAL_(m_size_x_times_y, m_size_x * m_size_y);

	MRPT_LOG_DEBUG_STREAM(
		"[internal_initialize] Creating priors for GMRF with "
		<< nodeCount << " nodes." << std::endl);
	CTicTac tictac;
	tictac.Tic();

	m_mrf_factors_activeObs.resize(nodeCount);  // Alloc space for obs
	m_gmrf.initialize(nodeCount);

	ConnectivityDescriptor* custom_connectivity =
		m_gmrf_connectivity
			.get();  // Use a raw ptr to avoid the cost in the inner loops

	size_t cx = 0, cy = 0, cz = 0;
	for (size_t j = 0; j < nodeCount; j++)
	{
		// add factors between this node and:
		// 1) the right node: j +1
		// 2) the back node:  j+m_size_x
		// 3) the top node: j+m_size_x*m_size*y
		//-------------------------------------------------
		const size_t c_idx_to_check[3] = {cx, cy, cz};
		const size_t c_idx_to_check_limits[3] = {m_size_x - 1, m_size_y - 1,
												 m_size_z - 1};
		const size_t c_neighbor_idx_incr[3] = {1, m_size_x, m_size_x_times_y};

		for (int dir = 0; dir < 3; dir++)
		{
			if (c_idx_to_check[dir] >= c_idx_to_check_limits[dir]) continue;

			const size_t i = j + c_neighbor_idx_incr[dir];
			ASSERT_(i < nodeCount);

			double edge_lamdba = .0;
			if (custom_connectivity != nullptr)
			{
				const bool is_connected =
					custom_connectivity->getEdgeInformation(
						this, cx, cy, cz, cx + (dir == 0 ? 1 : 0),
						cy + (dir == 1 ? 1 : 0), cz + (dir == 2 ? 1 : 0),
						edge_lamdba);
				if (!is_connected) continue;
			}
			else
			{
				edge_lamdba = insertionOptions.GMRF_lambdaPrior;
			}

			TPriorFactorGMRF new_prior(*this);
			new_prior.node_id_i = i;
			new_prior.node_id_j = j;
			new_prior.Lambda = edge_lamdba;

			m_mrf_factors_priors.push_back(new_prior);
			m_gmrf.addConstraint(
				*m_mrf_factors_priors.rbegin());  // add to graph
		}

		// Increment coordinates:
		if (++cx >= m_size_x)
		{
			cx = 0;
			if (++cy >= m_size_y)
			{
				cy = 0;
				cz++;
			}
		}
	}  // end for "j"

	MRPT_LOG_DEBUG_STREAM(
		"[internal_initialize] Prior built in " << tictac.Tac() << " s\n"
												<< std::endl);
}

/*---------------------------------------------------------------
					TInsertionOptions
 ---------------------------------------------------------------*/
CRandomFieldGridMap3D::TInsertionOptions::TInsertionOptions()
	: GMRF_lambdaPrior(0.01f),  // [GMRF model] The information (Lambda) of
	  // fixed map constraints
	  GMRF_skip_variance(false)
{
}

void CRandomFieldGridMap3D::TInsertionOptions::dumpToTextStreamstd::ostream& in, int version)
{
	switch (version)
	{
		case 0:
		{
			dyngridcommon_readFromStream(in);

			// To assure compatibility: The size of each cell:
			uint32_t n;
			in >> n;

			ASSERT_EQUAL_(n, static_cast<uint32_t>(sizeof(TRandomFieldVoxel)));
			// Load the map contents:
			in >> n;
			m_map.resize(n);

// Read the note in writeToStream()
#if MRPT_IS_BIG_ENDIAN
			for (uint32_t i = 0; i < n; i++)
				in >> m_map[i].mean_value >> m_map[i].stddev_value;
#else
			// Little endian: just read all at once:
			in.ReadBuffer(&m_map[0], sizeof(m_map[0]) * m_map.size());
#endif
			in >> insertionOptions.GMRF_lambdaPrior >>
				insertionOptions.GMRF_skip_variance;
		}
		break;
		default:
			MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
	};
}

void CRandomFieldGridMap3D::getAsVtkStructuredGrid(
	vtkStructuredGrid* output, const std::string& label_mean,
	const std::string& label_stddev) const
{
	MRPT_START;
#if MRPT_HAS_VTK

	const size_t nx = this->getSizeX(), ny = this->getSizeY(),
				 nz = this->getSizeZ();

	const int num_values = nx * ny * nz;

	vtkPoints* newPoints = vtkPoints::New();

	vtkDoubleArray* newData = vtkDoubleArray::New();
	newData->SetNumberOfComponents(3);
	newData->SetNumberOfTuples(num_values);

	vtkDoubleArray* mean_arr = vtkDoubleArray::New();
	mean_arr->SetNumberOfComponents(1);
	mean_arr->SetNumberOfTuples(num_values);

	vtkDoubleArray* std_arr = vtkDoubleArray::New();
	std_arr->SetNumberOfComponents(1);
	std_arr->SetNumberOfTuples(num_values);

	vtkIdType numtuples = newData->GetNumberOfTuples();

	{
		size_t cx = 0, cy = 0, cz = 0;
		for (vtkIdType cc = 0; cc < numtuples; cc++)
		{
			const double x = idx2x(cx), y = idx2y(cy), z = idx2z(cz);

			newData->SetComponent(cc, 0, x);
			newData->SetComponent(cc, 1, y);
			newData->SetComponent(cc, 2, z);

			mean_arr->SetComponent(cc, 0, m_map[cc].mean_value);
			std_arr->SetComponent(cc, 0, m_map[cc].stddev_value);

			// Increment coordinates:
			if (++cx >= m_size_x)
			{
				cx = 0;
				if (++cy >= m_size_y)
				{
					cy = 0;
					cz++;
				}
			}
		}
		ASSERT_(size_t(m_map.size()) == size_t(numtuples));
	}

	newPoints->SetData(newData);
	newData->Delete();

	output->SetExtent(0, nx - 1, 0, ny - 1, 0, nz - 1);
	output->SetPoints(newPoints);
	newPoints->Delete();

	mean_arr->SetName(label_mean.c_str());
	std_arr->SetName(label_stddev.c_str());
	output->GetPointData()->AddArray(mean_arr);
	output->GetPointData()->AddArray(std_arr);

	mean_arr->Delete();
	std_arr->Delete();

#else
	THROW_EXCEPTION("This method requires building MRPT against VTK!");
#endif  // VTK
	MRPT_END;
}

// ============ TObservationGMRF ===========
double CRandomFieldGridMap3D::TObservationGMRF::evaluateResidual() const
{
	return m_parent->m_map[this->node_id].mean_value - this->obsValue;
}
double CRandomFieldGridMap3D::TObservationGMRF::getInformation() const
{
	return this->Lambda;
}
void CRandomFieldGridMap3D::TObservationGMRF::evalJacobian(double& dr_dx) const
{
	dr_dx = 1.0;
}
// ============ TPriorFactorGMRF ===========
double CRandomFieldGridMap3D::TPriorFactorGMRF::evaluateResidual() const
{
	return m_parent->m_map[this->node_id_i].mean_value -
		   m_parent->m_map[this->node_id_j].mean_value;
}
double CRandomFieldGridMap3D::TPriorFactorGMRF::getInformation() const
{
	return this->Lambda;
}
void CRandomFieldGridMap3D::TPriorFactorGMRF::evalJacobian(
	double& dr_dx_i, double& dr_dx_j) const
{
	dr_dx_i = +1.0;
	dr_dx_j = -1.0;
}

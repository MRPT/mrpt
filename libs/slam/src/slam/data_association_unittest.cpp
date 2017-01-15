/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */


#include <mrpt/slam/data_association.h>
#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace std;


TEST(DataAssociation, TestNoICs)
{
	// Try to do DA when no individual compatible pairings exist.
	// Based on test proposed by Mauricio Soto Alvarez:
	// See: https://sourceforge.net/tracker/?func=detail&aid=3562885&group_id=205280&atid=993006

	CMatrixDouble y, y_cov, z;
	y.setSize(1,1);
	y_cov.setSize(1,1);
	z.setSize(1,1);

	y(0,0) = 0.0;
	y_cov(0,0) = 1.0;
	z(0,0) = 10.0;

	const TDataAssociationMethod dams[2]   = { assocNN, assocJCBB };
	const TDataAssociationMetric damets[2] = { metricMaha, metricML };

	for (unsigned int da_metric=0;da_metric<sizeof(damets)/sizeof(damets[0]);++da_metric)
	{
		const TDataAssociationMetric damet = damets[da_metric];

		for (unsigned int da_method=0;da_method<sizeof(dams)/sizeof(dams[0]);++da_method)
		{
			const TDataAssociationMethod dam = dams[da_method];

			TDataAssociationResults	DAresults;
			data_association_independent_predictions(z, y, y_cov, DAresults, dam, damet, 0.99, true, std::vector<prediction_index_t>(), metricMaha, 0.0);

			EXPECT_EQ(0u,DAresults.associations.size())
				<< "For da_method="<< da_method << " and da_metric="<<da_metric<< endl;
		}
	}

}

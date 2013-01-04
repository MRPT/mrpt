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


#include <mrpt/slam.h>
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

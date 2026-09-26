"""
mrpt.slam — SLAM and localization algorithms for MRPT.

Provides:
  - CICP                   : Iterative Closest Point algorithm for map alignment
  - CICPOptions            : Parameters for the CICP algorithm
  - TICPReturnInfo         : Result information from CICP.AlignPDF()
  - CMetricMapBuilder      : Abstract base for SLAM map builders
  - CMetricMapBuilderICP   : Simple ICP-based SLAM map builder
  - CMetricMapBuilderICPOptions : Options for CMetricMapBuilderICP
  - TICPAlgorithm          : Enum: icpClassic, icpLevenbergMarquardt
  - TICPCovarianceMethod   : Enum: icpCovLinealMSE, icpCovFiniteDifferences
  - CMonteCarloLocalization2D/3D, TMonteCarloLocalizationParams, TKLDParams :
                             particle filter localization (run it with
                             mrpt.bayes.CParticleFilter.executeOn())
  - CMetricMapBuilderRBPF  : Rao-Blackwellized particle filter SLAM

Basic ICP-SLAM usage::

    import mrpt.slam as slam
    import mrpt.obs as obs
    import mrpt.maps

    builder = slam.CMetricMapBuilderICP()
    builder.ICP_options.insertionLinDistance = 0.5
    builder.initialize()

    # In a loop:
    builder.processObservation(my_scan_obs)
    pose_pdf = builder.getCurrentPoseEstimation()
"""

import mrpt.rtti          # noqa: F401
import mrpt.serialization  # noqa: F401
import mrpt.poses          # noqa: F401
import mrpt.bayes          # noqa: F401  (CParticleFilter.executeOn() is attached here)
import mrpt.maps           # noqa: F401
import mrpt.viz            # noqa: F401

from mrpt.slam._bindings import (
    CICP,
    CICPOptions,
    CMetricMapBuilder,
    CMetricMapBuilderICP,
    CMetricMapBuilderICPOptions,
    TICPAlgorithm,
    TICPCovarianceMethod,
    TICPReturnInfo,
    icpClassic,
    icpLevenbergMarquardt,
    icpCovLinealMSE,
    icpCovFiniteDifferences,
    TKLDParams,
    TMonteCarloLocalizationParams,
    CMonteCarloLocalization2D,
    CMonteCarloLocalization3D,
    TPredictionParams,
    CMetricMapBuilderRBPF,
)

__all__ = [
    "CICP",
    "CICPOptions",
    "CMetricMapBuilder",
    "CMetricMapBuilderICP",
    "CMetricMapBuilderICPOptions",
    "TICPAlgorithm",
    "TICPCovarianceMethod",
    "TICPReturnInfo",
    "icpClassic",
    "icpLevenbergMarquardt",
    "icpCovLinealMSE",
    "icpCovFiniteDifferences",
    "TKLDParams",
    "TMonteCarloLocalizationParams",
    "CMonteCarloLocalization2D",
    "CMonteCarloLocalization3D",
    "TPredictionParams",
    "CMetricMapBuilderRBPF",
]

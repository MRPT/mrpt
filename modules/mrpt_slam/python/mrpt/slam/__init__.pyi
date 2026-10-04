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
from __future__ import annotations
import mrpt as mrpt
from mrpt.slam._bindings import CICP as CICP
from mrpt.slam._bindings import CICPOptions as CICPOptions
from mrpt.slam._bindings import CMetricMapBuilder as CMetricMapBuilder
from mrpt.slam._bindings import CMetricMapBuilderICP as CMetricMapBuilderICP
from mrpt.slam._bindings import CMetricMapBuilderICPOptions as CMetricMapBuilderICPOptions
from mrpt.slam._bindings import CMetricMapBuilderRBPF as CMetricMapBuilderRBPF
from mrpt.slam._bindings import CMonteCarloLocalization2D as CMonteCarloLocalization2D
from mrpt.slam._bindings import CMonteCarloLocalization3D as CMonteCarloLocalization3D
from mrpt.slam._bindings import TICPAlgorithm as TICPAlgorithm
from mrpt.slam._bindings import TICPCovarianceMethod as TICPCovarianceMethod
from mrpt.slam._bindings import TICPReturnInfo as TICPReturnInfo
from mrpt.slam._bindings import TKLDParams as TKLDParams
from mrpt.slam._bindings import TMonteCarloLocalizationParams as TMonteCarloLocalizationParams
from mrpt.slam._bindings import TPredictionParams as TPredictionParams
from . import _bindings
__all__: list = ['CICP', 'CICPOptions', 'CMetricMapBuilder', 'CMetricMapBuilderICP', 'CMetricMapBuilderICPOptions', 'TICPAlgorithm', 'TICPCovarianceMethod', 'TICPReturnInfo', 'icpClassic', 'icpLevenbergMarquardt', 'icpCovLinealMSE', 'icpCovFiniteDifferences', 'TKLDParams', 'TMonteCarloLocalizationParams', 'CMonteCarloLocalization2D', 'CMonteCarloLocalization3D', 'TPredictionParams', 'CMetricMapBuilderRBPF']
icpClassic: _bindings.TICPAlgorithm
icpCovFiniteDifferences: _bindings.TICPCovarianceMethod
icpCovLinealMSE: _bindings.TICPCovarianceMethod
icpLevenbergMarquardt: _bindings.TICPAlgorithm

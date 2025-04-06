# Eigen Alignment to make MRPT compatible with different compiler build flags.
# EIGEN_MAX_ALIGN_BYTES & EIGEN_MAX_STATIC_ALIGN_BYTES
# Just go for the safer side: align=32 bytes
# ------------------------------------------------------
if (MRPT_ARCH_INTEL_COMPATIBLE)
    set(EIGEN_MAX_ALIGN_BYTES 32)
    set(EIGEN_MAX_STATIC_ALIGN_BYTES 32)
else()
    # for aarch64, etc.
    set(EIGEN_MAX_ALIGN_BYTES 16)
    set(EIGEN_MAX_STATIC_ALIGN_BYTES 16)
endif()

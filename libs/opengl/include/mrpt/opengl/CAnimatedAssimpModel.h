/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */
#pragma once

#include <mrpt/math/CMatrixFixed.h>
#include <mrpt/opengl/CAssimpModel.h>

#include <map>
#include <string>
#include <vector>

// Forward declarations for Assimp types (avoid leaking assimp headers)
struct aiNode;

namespace mrpt::opengl
{

/**
 * Extension of CAssimpModel with skeletal animation support.
 *
 * Supports models that contain bone hierarchies and keyframed animations
 * (FBX, glTF/GLB, Collada, etc. via Assimp).
 *
 * CPU-side vertex skinning is performed each time the animation time
 * changes: bind-pose vertex positions are stored on first load and then
 * transformed through the active bone matrices every frame.
 *
 * Usage:
 * \code
 *   auto model = CAnimatedAssimpModel::Create();
 *   model->loadScene("character.glb");
 *
 *   // In simulation loop:
 *   model->setAnimationTime(simTime);  // updates bone transforms
 *   // (buffers are rebuilt automatically on next render)
 * \endcode
 *
 * \ingroup mrpt_opengl_grp
 */
class CAnimatedAssimpModel : public CAssimpModel
{
  DEFINE_SERIALIZABLE(CAnimatedAssimpModel, mrpt::opengl)

 public:
  CAnimatedAssimpModel() = default;

  // ========== Animation Control ==========

  /** Set current animation time in seconds.
   *  Updates all bone transforms for the active animation and marks
   *  render buffers for rebuild. */
  void setAnimationTime(double timeSeconds);

  /** Get animation duration in seconds. */
  double getAnimationDuration(size_t animIndex = 0) const;

  /** Get number of animations in the model. */
  size_t getAnimationCount() const;

  /** Get animation name by index. */
  std::string getAnimationName(size_t animIndex) const;

  /** Select which animation to play (by index). */
  void setActiveAnimation(size_t animIndex);

  /** Select which animation to play (by name).
   *  Does nothing if the name is not found. */
  void setActiveAnimation(const std::string& animName);

  /** Enable/disable animation looping. */
  void setLooping(bool loop) { looping_ = loop; }

  /** Get current normalized animation progress [0,1]. */
  double getAnimationProgress() const;

  // ========== Bone Access (for procedural animation) ==========

  /** Get number of bones. */
  size_t getBoneCount() const { return bones_.size(); }

  /** Get bone index by name, or -1 if not found. */
  int getBoneIndex(const std::string& boneName) const;

  /** Override a bone's local transform (for procedural animation). */
  void setBoneLocalTransform(size_t boneIndex, const mrpt::math::CMatrixDouble44& localTransform);

  /** Clear all bone overrides, return to animation-driven transforms. */
  void clearBoneOverrides();

 protected:
  // Override to apply bone transforms before generating triangles
  void renderUpdateBuffers() const override;

  /** Called after loadScene() / deserialization.
   *  Extracts skeleton, animations, and caches bind-pose geometry. */
  void onAfterLoadScene() override;

 private:
  // ========== Internal Bone Data ==========

  struct Bone
  {
    std::string name;
    int parentIndex = -1;  //!< -1 for root bones

    // Bind pose (rest position)
    mrpt::math::CMatrixDouble44 offsetMatrix;  //!< mesh space → bone space

    // Current animated transform
    mrpt::math::CMatrixDouble44 localTransform;   //!< relative to parent
    mrpt::math::CMatrixDouble44 globalTransform;  //!< world space
    mrpt::math::CMatrixDouble44 finalTransform;   //!< for skinning

    // Optional override (for procedural animation)
    bool hasOverride = false;
    mrpt::math::CMatrixDouble44 overrideTransform;
  };

  struct VertexBoneData
  {
    static constexpr int MAX_BONES_PER_VERTEX = 4;
    int boneIds[MAX_BONES_PER_VERTEX] = {-1, -1, -1, -1};
    float weights[MAX_BONES_PER_VERTEX] = {0.f, 0.f, 0.f, 0.f};

    void addBoneWeight(int boneId, float weight);
  };

  // Animation keyframe data
  struct VectorKey
  {
    double time = 0.0;
    mrpt::math::TPoint3Df value;
  };

  struct QuatKey
  {
    double time = 0.0;
    float w = 1.f, x = 0.f, y = 0.f, z = 0.f;
  };

  struct BoneAnimation
  {
    std::string boneName;
    int boneIndex = -1;
    std::vector<VectorKey> positionKeys;
    std::vector<QuatKey> rotationKeys;
    std::vector<VectorKey> scalingKeys;
  };

  struct Animation
  {
    std::string name;
    double duration = 0.0;  //!< in ticks
    double ticksPerSecond = 25.0;
    std::vector<BoneAnimation> channels;
  };

  // ========== Bind-Pose Storage for CPU Skinning ==========

  struct MeshBindPose
  {
    std::vector<mrpt::math::TPoint3Df> positions;
    std::vector<mrpt::math::TPoint3Df> normals;
  };
  std::vector<MeshBindPose> meshBindPoses_;

  // ========== Member Variables ==========

  std::vector<Bone> bones_;
  std::map<std::string, size_t> boneNameToIndex_;

  /** Per-vertex skinning data.  Key = (meshIdx << 20) | vertIdx.
   *  Populated during extractSkeletonData(). */
  std::map<uint64_t, VertexBoneData> vertexBoneMap_;

  std::vector<Animation> animations_;
  size_t activeAnimation_ = 0;
  double currentTime_ = 0.0;
  bool looping_ = true;
  bool hasSkeleton_ = false;

  mrpt::math::CMatrixDouble44 globalInverseTransform_;

  // ========== Internal Methods ==========

  /** Called by onAfterLoadScene() to extract skeleton data from aiScene. */
  void extractSkeletonData();

  /** Build parent-child relationships by walking the aiNode tree. */
  void buildBoneHierarchy(const aiNode* node, int parentBoneIdx);

  void captureBindPose();
  void applySkinningToScene() const;
  void restoreBindPoseToScene() const;

  /** Update all bone transforms for current animation time. */
  void updateBoneTransforms();

  /** Recursively compute global transforms from local transforms. */
  void computeGlobalTransforms(size_t boneIndex);

  /** Interpolate position at given time. */
  mrpt::math::TPoint3Df interpolatePosition(const BoneAnimation& channel, double animTime) const;

  /** Interpolate rotation (SLERP) at given time. */
  void interpolateRotation(
      const BoneAnimation& channel, double animTime, float& qw, float& qx, float& qy, float& qz)
      const;

  /** Interpolate scale at given time. */
  mrpt::math::TPoint3Df interpolateScaling(const BoneAnimation& channel, double animTime) const;

  /** Build a 4×4 TRS matrix from position, quaternion, scale. */
  static mrpt::math::CMatrixDouble44 buildTransformMatrix(
      const mrpt::math::TPoint3Df& pos,
      float qw,
      float qx,
      float qy,
      float qz,
      const mrpt::math::TPoint3Df& scale);

  /** Apply skinning to transform a vertex position. */
  mrpt::math::TPoint3Df skinVertex(const mrpt::math::TPoint3Df& vertex, uint64_t vertexKey) const;

  /** Apply skinning to transform a vertex normal. */
  mrpt::math::TPoint3Df skinNormal(const mrpt::math::TPoint3Df& normal, uint64_t vertexKey) const;

  /** Encode a (mesh, vertex) pair into a single lookup key. */
  static uint64_t vertexKey(unsigned int meshIdx, unsigned int vertIdx)
  {
    return (static_cast<uint64_t>(meshIdx) << 20) | static_cast<uint64_t>(vertIdx);
  }
};

}  // namespace mrpt::opengl

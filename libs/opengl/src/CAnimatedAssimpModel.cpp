/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2026, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "opengl-precomp.h"  // Precompiled header
//
#include <mrpt/opengl/CAnimatedAssimpModel.h>
#include <mrpt/serialization/CArchive.h>

#if MRPT_HAS_ASSIMP
#include <assimp/anim.h>
#include <assimp/scene.h>
#endif

#include <Eigen/Dense>
#include <cmath>

using namespace mrpt::opengl;
using namespace mrpt::math;

IMPLEMENTS_SERIALIZABLE(CAnimatedAssimpModel, CAssimpModel, mrpt::opengl)

// ============================================================================
// Serialization  (delegates to base; skeleton is re-extracted on load)
// ============================================================================

uint8_t CAnimatedAssimpModel::serializeGetVersion() const { return 0; }

void CAnimatedAssimpModel::serializeTo(mrpt::serialization::CArchive& out) const
{
  // Delegate to base class which stores the assimp blob:
  out << CAssimpModel::serializeGetVersion();
  CAssimpModel::serializeTo(out);
  // Animation state (cheap to store):
  out << activeAnimation_ << currentTime_ << looping_;
}

void CAnimatedAssimpModel::serializeFrom(mrpt::serialization::CArchive& in, uint8_t version)
{
  switch (version)
  {
    case 0:
    {
      uint8_t baseVersion = in.ReadAs<uint8_t>();
      CAssimpModel::serializeFrom(in, baseVersion);
      in >> activeAnimation_ >> currentTime_ >> looping_;
      // Skeleton will be re-extracted via onAfterLoadScene() triggered
      // by the base class deserialization path.
    }
    break;
    default:
      MRPT_THROW_UNKNOWN_SERIALIZATION_VERSION(version);
  }
}

// ============================================================================
// Animation Control
// ============================================================================

void CAnimatedAssimpModel::setAnimationTime(double timeSeconds)
{
  if (!hasSkeleton_ || animations_.empty())
  {
    return;
  }

  currentTime_ = timeSeconds;

  // Update bone local transforms from animation
  updateBoneTransforms();

  // Mark buffers as needing update
  notifyChange();
}

double CAnimatedAssimpModel::getAnimationDuration(size_t animIndex) const
{
  if (animIndex >= animations_.size())
  {
    return 0.0;
  }
  const auto& anim = animations_[animIndex];
  const double ticksPerSec = (anim.ticksPerSecond > 0.0) ? anim.ticksPerSecond : 25.0;
  return anim.duration / ticksPerSec;
}

size_t CAnimatedAssimpModel::getAnimationCount() const { return animations_.size(); }

std::string CAnimatedAssimpModel::getAnimationName(size_t animIndex) const
{
  if (animIndex >= animations_.size())
  {
    return {};
  }
  return animations_[animIndex].name;
}

void CAnimatedAssimpModel::setActiveAnimation(size_t animIndex)
{
  if (animIndex < animations_.size())
  {
    activeAnimation_ = animIndex;
    currentTime_ = 0.0;
  }
}

void CAnimatedAssimpModel::setActiveAnimation(const std::string& animName)
{
  for (size_t i = 0; i < animations_.size(); ++i)
  {
    if (animations_[i].name == animName)
    {
      setActiveAnimation(i);
      return;
    }
  }
  // Not found — do nothing (user can check getAnimationCount / names).
}

double CAnimatedAssimpModel::getAnimationProgress() const
{
  if (animations_.empty() || activeAnimation_ >= animations_.size())
  {
    return 0.0;
  }
  const double dur = getAnimationDuration(activeAnimation_);
  if (dur < 1e-9)
  {
    return 0.0;
  }
  const double t = looping_ ? std::fmod(currentTime_, dur) : currentTime_;
  return std::clamp(t / dur, 0.0, 1.0);
}

// ========== Bone Access =====================================================

int CAnimatedAssimpModel::getBoneIndex(const std::string& boneName) const
{
  auto it = boneNameToIndex_.find(boneName);
  if (it == boneNameToIndex_.end())
  {
    return -1;
  }
  return static_cast<int>(it->second);
}

void CAnimatedAssimpModel::setBoneLocalTransform(
    size_t boneIndex, const CMatrixDouble44& localTransform)
{
  if (boneIndex >= bones_.size())
  {
    return;
  }
  bones_[boneIndex].hasOverride = true;
  bones_[boneIndex].overrideTransform = localTransform;
}

void CAnimatedAssimpModel::clearBoneOverrides()
{
  for (auto& bone : bones_)
  {
    bone.hasOverride = false;
  }
}

// ============================================================================
// onAfterLoadScene — called by base class after loadScene / deserialize
// ============================================================================

void CAnimatedAssimpModel::onAfterLoadScene() { extractSkeletonData(); }

// ============================================================================
// Skeleton Extraction
// ============================================================================

#if MRPT_HAS_ASSIMP
namespace
{
CMatrixDouble44 aiMatrixToMrpt(const aiMatrix4x4& m)
{
  CMatrixDouble44 M;
  M(0, 0) = m.a1;
  M(0, 1) = m.a2;
  M(0, 2) = m.a3;
  M(0, 3) = m.a4;
  M(1, 0) = m.b1;
  M(1, 1) = m.b2;
  M(1, 2) = m.b3;
  M(1, 3) = m.b4;
  M(2, 0) = m.c1;
  M(2, 1) = m.c2;
  M(2, 2) = m.c3;
  M(2, 3) = m.c4;
  M(3, 0) = m.d1;
  M(3, 1) = m.d2;
  M(3, 2) = m.d3;
  M(3, 3) = m.d4;
  return M;
}

TPoint3Df aiVecToMrpt(const aiVector3D& v) { return {v.x, v.y, v.z}; }

}  // anonymous namespace
#endif

void CAnimatedAssimpModel::extractSkeletonData()
{
  bones_.clear();
  boneNameToIndex_.clear();
  vertexBoneMap_.clear();
  animations_.clear();
  hasSkeleton_ = false;

#if MRPT_HAS_ASSIMP
  const aiScene* scene = getAssimpScene();
  if (!scene)
  {
    return;
  }

  // --- 1) Extract bones from all meshes --------------------------------
  for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
  {
    const aiMesh* mesh = scene->mMeshes[m];
    if (!mesh->HasBones())
    {
      continue;
    }

    for (unsigned int b = 0; b < mesh->mNumBones; ++b)
    {
      const aiBone* bone = mesh->mBones[b];
      const std::string boneName(bone->mName.data);

      // Get or create bone entry
      size_t boneIndex;
      auto it = boneNameToIndex_.find(boneName);
      if (it == boneNameToIndex_.end())
      {
        boneIndex = bones_.size();
        boneNameToIndex_[boneName] = boneIndex;

        Bone newBone;
        newBone.name = boneName;
        newBone.offsetMatrix = aiMatrixToMrpt(bone->mOffsetMatrix);
        newBone.localTransform = CMatrixDouble44::Identity();
        newBone.globalTransform = CMatrixDouble44::Identity();
        newBone.finalTransform = CMatrixDouble44::Identity();
        bones_.push_back(newBone);
      }
      else
      {
        boneIndex = it->second;
      }

      // Store per-vertex weights using the composite key
      for (unsigned int w = 0; w < bone->mNumWeights; ++w)
      {
        const aiVertexWeight& vw = bone->mWeights[w];
        const uint64_t key = vertexKey(m, vw.mVertexId);
        vertexBoneMap_[key].addBoneWeight(static_cast<int>(boneIndex), vw.mWeight);
      }
    }
  }

  if (bones_.empty())
  {
    return;
  }

  // --- 2) Build bone hierarchy from node tree --------------------------
  buildBoneHierarchy(scene->mRootNode, -1);

  // Store global inverse transform (root node)
  globalInverseTransform_ = aiMatrixToMrpt(scene->mRootNode->mTransformation);
  // Invert it:
  {
    CMatrixDouble44 tmp = globalInverseTransform_;
    globalInverseTransform_ = tmp.asEigen().inverse().eval();
  }

  // --- 3) Extract animations -------------------------------------------
  for (unsigned int a = 0; a < scene->mNumAnimations; ++a)
  {
    const aiAnimation* aiAnim = scene->mAnimations[a];

    Animation anim;
    anim.name = aiAnim->mName.data;
    anim.duration = aiAnim->mDuration;
    anim.ticksPerSecond = (aiAnim->mTicksPerSecond > 0.0) ? aiAnim->mTicksPerSecond : 25.0;

    for (unsigned int c = 0; c < aiAnim->mNumChannels; ++c)
    {
      const aiNodeAnim* ch = aiAnim->mChannels[c];

      BoneAnimation ba;
      ba.boneName = ch->mNodeName.data;

      auto itB = boneNameToIndex_.find(ba.boneName);
      if (itB != boneNameToIndex_.end())
      {
        ba.boneIndex = static_cast<int>(itB->second);
      }

      // Position keys
      ba.positionKeys.reserve(ch->mNumPositionKeys);
      for (unsigned int k = 0; k < ch->mNumPositionKeys; ++k)
      {
        VectorKey vk;
        vk.time = ch->mPositionKeys[k].mTime;
        vk.value = aiVecToMrpt(ch->mPositionKeys[k].mValue);
        ba.positionKeys.push_back(vk);
      }

      // Rotation keys
      ba.rotationKeys.reserve(ch->mNumRotationKeys);
      for (unsigned int k = 0; k < ch->mNumRotationKeys; ++k)
      {
        QuatKey qk;
        qk.time = ch->mRotationKeys[k].mTime;
        const auto& q = ch->mRotationKeys[k].mValue;
        qk.w = static_cast<float>(q.w);
        qk.x = static_cast<float>(q.x);
        qk.y = static_cast<float>(q.y);
        qk.z = static_cast<float>(q.z);
        ba.rotationKeys.push_back(qk);
      }

      // Scaling keys
      ba.scalingKeys.reserve(ch->mNumScalingKeys);
      for (unsigned int k = 0; k < ch->mNumScalingKeys; ++k)
      {
        VectorKey sk;
        sk.time = ch->mScalingKeys[k].mTime;
        sk.value = aiVecToMrpt(ch->mScalingKeys[k].mValue);
        ba.scalingKeys.push_back(sk);
      }

      anim.channels.push_back(std::move(ba));
    }

    animations_.push_back(std::move(anim));
  }

  hasSkeleton_ = true;

  // --- 4) Build the vertex-origin map (must match recursive_render order)
  captureBindPose();

#endif  // MRPT_HAS_ASSIMP
}

// ============================================================================
// Build bone hierarchy
// ============================================================================

void CAnimatedAssimpModel::buildBoneHierarchy(const aiNode* node, int parentBoneIdx)
{
#if MRPT_HAS_ASSIMP
  if (!node)
  {
    return;
  }

  const std::string nodeName(node->mName.data);
  int thisBoneIdx = parentBoneIdx;

  // If this node corresponds to a bone, set its parent
  auto it = boneNameToIndex_.find(nodeName);
  if (it != boneNameToIndex_.end())
  {
    thisBoneIdx = static_cast<int>(it->second);
    bones_[it->second].parentIndex = parentBoneIdx;
  }

  // Recurse children
  for (unsigned int i = 0; i < node->mNumChildren; ++i)
  {
    buildBoneHierarchy(node->mChildren[i], thisBoneIdx);
  }
#else
  (void)node;
  (void)parentBoneIdx;
#endif
}

void CAnimatedAssimpModel::captureBindPose()
{
#if MRPT_HAS_ASSIMP
  const aiScene* scene = getAssimpScene();
  if (!scene)
  {
    return;
  }

  meshBindPoses_.resize(scene->mNumMeshes);

  for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
  {
    const aiMesh* mesh = scene->mMeshes[m];
    auto& bp = meshBindPoses_[m];

    bp.positions.resize(mesh->mNumVertices);
    bp.normals.resize(mesh->mNumVertices);

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
    {
      const auto& sv = mesh->mVertices[v];
      bp.positions[v] = {sv.x, sv.y, sv.z};

      if (mesh->mNormals)
      {
        const auto& sn = mesh->mNormals[v];
        bp.normals[v] = {sn.x, sn.y, sn.z};
      }
      else
      {
        bp.normals[v] = {0.f, 0.f, 1.f};
      }
    }
  }
#endif
}

void CAnimatedAssimpModel::applySkinningToScene() const
{
#if MRPT_HAS_ASSIMP
  // const_cast is safe: we restore the original data in
  // restoreBindPoseToScene() immediately after the base class
  // regenerates its triangle buffers.
  aiScene* scene = const_cast<aiScene*>(getAssimpScene());
  if (!scene)
  {
    return;
  }

  for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
  {
    aiMesh* mesh = scene->mMeshes[m];
    if (!mesh->HasBones())
    {
      continue;
    }
    if (m >= meshBindPoses_.size())
    {
      continue;
    }

    const auto& bp = meshBindPoses_[m];
    if (bp.positions.size() != mesh->mNumVertices)
    {
      continue;  // safety — mesh/bind-pose size mismatch
    }

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
    {
      const uint64_t vk = vertexKey(m, v);

      auto it = vertexBoneMap_.find(vk);
      if (it == vertexBoneMap_.end())
      {
        continue;  // vertex has no bone weights
      }

      // Skin position (from the bind-pose, NOT from the current
      // mesh data which may already have been skinned in a
      // previous frame).
      const auto skinnedPos = skinVertex(bp.positions[v], vk);
      mesh->mVertices[v].x = skinnedPos.x;
      mesh->mVertices[v].y = skinnedPos.y;
      mesh->mVertices[v].z = skinnedPos.z;

      // Skin normal
      if (mesh->mNormals)
      {
        const auto skinnedNrm = skinNormal(bp.normals[v], vk);
        mesh->mNormals[v].x = skinnedNrm.x;
        mesh->mNormals[v].y = skinnedNrm.y;
        mesh->mNormals[v].z = skinnedNrm.z;
      }
    }
  }
#endif
}

void CAnimatedAssimpModel::restoreBindPoseToScene() const
{
#if MRPT_HAS_ASSIMP
  aiScene* scene = const_cast<aiScene*>(getAssimpScene());
  if (!scene)
  {
    return;
  }

  for (unsigned int m = 0; m < scene->mNumMeshes; ++m)
  {
    aiMesh* mesh = scene->mMeshes[m];
    if (!mesh->HasBones())
    {
      continue;
    }
    if (m >= meshBindPoses_.size())
    {
      continue;
    }

    const auto& bp = meshBindPoses_[m];
    if (bp.positions.size() != mesh->mNumVertices)
    {
      continue;
    }

    for (unsigned int v = 0; v < mesh->mNumVertices; ++v)
    {
      mesh->mVertices[v].x = bp.positions[v].x;
      mesh->mVertices[v].y = bp.positions[v].y;
      mesh->mVertices[v].z = bp.positions[v].z;

      if (mesh->mNormals)
      {
        mesh->mNormals[v].x = bp.normals[v].x;
        mesh->mNormals[v].y = bp.normals[v].y;
        mesh->mNormals[v].z = bp.normals[v].z;
      }
    }
  }
#endif
}

// ============================================================================
// Bone Transform Update
// ============================================================================

void CAnimatedAssimpModel::updateBoneTransforms()
{
  if (animations_.empty() || activeAnimation_ >= animations_.size())
  {
    return;
  }

  const auto& anim = animations_[activeAnimation_];
  const double ticksPerSec = (anim.ticksPerSecond > 0.0) ? anim.ticksPerSecond : 25.0;

  double animTime = currentTime_ * ticksPerSec;
  if (looping_ && anim.duration > 0.0)
  {
    animTime = std::fmod(animTime, anim.duration);
  }
  else if (anim.duration > 0.0)
  {
    animTime = std::min(animTime, anim.duration);
  }

  // Update each bone's local transform from animation channels
  for (const auto& channel : anim.channels)
  {
    if (channel.boneIndex < 0 || static_cast<size_t>(channel.boneIndex) >= bones_.size())
    {
      continue;
    }

    Bone& bone = bones_[static_cast<size_t>(channel.boneIndex)];

    // Skip if bone has manual override
    if (bone.hasOverride)
    {
      bone.localTransform = bone.overrideTransform;
      continue;
    }

    // Interpolate position, rotation, scale
    const auto pos = interpolatePosition(channel, animTime);
    float qw, qx, qy, qz;
    interpolateRotation(channel, animTime, qw, qx, qy, qz);
    const auto scl = interpolateScaling(channel, animTime);

    // Build local transform matrix: T * R * S
    bone.localTransform = buildTransformMatrix(pos, qw, qx, qy, qz, scl);
  }

  // Compute global transforms recursively from root bones
  for (size_t i = 0; i < bones_.size(); ++i)
  {
    if (bones_[i].parentIndex < 0)
    {
      computeGlobalTransforms(i);
    }
  }

  // Compute final transforms for skinning:
  //   finalTransform = globalInverse * boneGlobal * offset
  for (size_t i = 0; i < bones_.size(); ++i)
  {
    auto& b = bones_[i];
    // Using Eigen under the hood for the triple product:
    b.finalTransform.asEigen() =
        globalInverseTransform_.asEigen() * b.globalTransform.asEigen() * b.offsetMatrix.asEigen();
  }
}

void CAnimatedAssimpModel::computeGlobalTransforms(size_t boneIndex)
{
  Bone& bone = bones_[boneIndex];

  if (bone.parentIndex >= 0)
  {
    const Bone& par = bones_[static_cast<size_t>(bone.parentIndex)];
    bone.globalTransform.asEigen() = par.globalTransform.asEigen() * bone.localTransform.asEigen();
  }
  else
  {
    bone.globalTransform = bone.localTransform;
  }

  // Recursively update children
  for (size_t i = 0; i < bones_.size(); ++i)
  {
    if (bones_[i].parentIndex == static_cast<int>(boneIndex))
    {
      computeGlobalTransforms(i);
    }
  }
}

// ============================================================================
// Build Transform Matrix (T * R * S)
// ============================================================================

CMatrixDouble44 CAnimatedAssimpModel::buildTransformMatrix(
    const TPoint3Df& pos, float qw, float qx, float qy, float qz, const TPoint3Df& scale)
{
  // Normalize quaternion
  const float len = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
  if (len > 1e-7f)
  {
    const float invLen = 1.0f / len;
    qw *= invLen;
    qx *= invLen;
    qy *= invLen;
    qz *= invLen;
  }

  // Rotation matrix from quaternion
  const float xx = qx * qx;
  const float yy = qy * qy;
  const float zz = qz * qz;
  const float xy = qx * qy;
  const float xz = qx * qz;
  const float yz = qy * qz;
  const float wx = qw * qx;
  const float wy = qw * qy;
  const float wz = qw * qz;

  CMatrixDouble44 M;
  // Row 0
  M(0, 0) = static_cast<double>((1.0f - 2.0f * (yy + zz)) * scale.x);
  M(0, 1) = static_cast<double>((2.0f * (xy - wz)) * scale.y);
  M(0, 2) = static_cast<double>((2.0f * (xz + wy)) * scale.z);
  M(0, 3) = static_cast<double>(pos.x);

  // Row 1
  M(1, 0) = static_cast<double>((2.0f * (xy + wz)) * scale.x);
  M(1, 1) = static_cast<double>((1.0f - 2.0f * (xx + zz)) * scale.y);
  M(1, 2) = static_cast<double>((2.0f * (yz - wx)) * scale.z);
  M(1, 3) = static_cast<double>(pos.y);

  // Row 2
  M(2, 0) = static_cast<double>((2.0f * (xz - wy)) * scale.x);
  M(2, 1) = static_cast<double>((2.0f * (yz + wx)) * scale.y);
  M(2, 2) = static_cast<double>((1.0f - 2.0f * (xx + yy)) * scale.z);
  M(2, 3) = static_cast<double>(pos.z);

  // Row 3
  M(3, 0) = 0.0;
  M(3, 1) = 0.0;
  M(3, 2) = 0.0;
  M(3, 3) = 1.0;

  return M;
}

// ============================================================================
// Vertex Skinning
// ============================================================================

TPoint3Df CAnimatedAssimpModel::skinVertex(const TPoint3Df& vertex, uint64_t vkey) const
{
  auto it = vertexBoneMap_.find(vkey);
  if (it == vertexBoneMap_.end())
  {
    return vertex;  // not skinned
  }

  const auto& vbd = it->second;
  TPoint3Df result(0.f, 0.f, 0.f);
  float totalWeight = 0.f;

  for (int i = 0; i < VertexBoneData::MAX_BONES_PER_VERTEX; ++i)
  {
    if (vbd.boneIds[i] < 0)
    {
      continue;
    }

    const auto boneIdx = static_cast<size_t>(vbd.boneIds[i]);
    if (boneIdx >= bones_.size())
    {
      continue;
    }

    const float weight = vbd.weights[i];
    if (weight < 1e-6f)
    {
      continue;
    }

    const auto& M = bones_[boneIdx].finalTransform;

    const float x =
        static_cast<float>(M(0, 0) * vertex.x + M(0, 1) * vertex.y + M(0, 2) * vertex.z + M(0, 3));
    const float y =
        static_cast<float>(M(1, 0) * vertex.x + M(1, 1) * vertex.y + M(1, 2) * vertex.z + M(1, 3));
    const float z =
        static_cast<float>(M(2, 0) * vertex.x + M(2, 1) * vertex.y + M(2, 2) * vertex.z + M(2, 3));

    result.x += x * weight;
    result.y += y * weight;
    result.z += z * weight;
    totalWeight += weight;
  }

  // Blend remaining weight with un-transformed position
  if (totalWeight < 0.999f)
  {
    const float rem = 1.0f - totalWeight;
    result.x += vertex.x * rem;
    result.y += vertex.y * rem;
    result.z += vertex.z * rem;
  }

  return result;
}

TPoint3Df CAnimatedAssimpModel::skinNormal(const TPoint3Df& normal, uint64_t vkey) const
{
  auto it = vertexBoneMap_.find(vkey);
  if (it == vertexBoneMap_.end())
  {
    return normal;
  }

  const auto& vbd = it->second;
  TPoint3Df result(0.f, 0.f, 0.f);
  float totalWeight = 0.f;

  for (int i = 0; i < VertexBoneData::MAX_BONES_PER_VERTEX; ++i)
  {
    if (vbd.boneIds[i] < 0)
    {
      continue;
    }

    const auto boneIdx = static_cast<size_t>(vbd.boneIds[i]);
    if (boneIdx >= bones_.size())
    {
      continue;
    }

    const float weight = vbd.weights[i];
    if (weight < 1e-6f)
    {
      continue;
    }

    const auto& M = bones_[boneIdx].finalTransform;

    // Rotate only (no translation for normals)
    const float nx =
        static_cast<float>(M(0, 0) * normal.x + M(0, 1) * normal.y + M(0, 2) * normal.z);
    const float ny =
        static_cast<float>(M(1, 0) * normal.x + M(1, 1) * normal.y + M(1, 2) * normal.z);
    const float nz =
        static_cast<float>(M(2, 0) * normal.x + M(2, 1) * normal.y + M(2, 2) * normal.z);

    result.x += nx * weight;
    result.y += ny * weight;
    result.z += nz * weight;
    totalWeight += weight;
  }

  if (totalWeight < 0.999f)
  {
    const float rem = 1.0f - totalWeight;
    result.x += normal.x * rem;
    result.y += normal.y * rem;
    result.z += normal.z * rem;
  }

  // Re-normalize
  const float len = std::sqrt(result.x * result.x + result.y * result.y + result.z * result.z);
  if (len > 1e-6f)
  {
    result.x /= len;
    result.y /= len;
    result.z /= len;
  }

  return result;
}

// ============================================================================
// Override to apply CPU skinning
// ============================================================================

void CAnimatedAssimpModel::renderUpdateBuffers() const
{
  if (!hasSkeleton_)
  {
    // No skeleton — delegate to the base class.
    CAssimpModel::renderUpdateBuffers();
    return;
  }

  // --- 1. Write skinned positions into the aiScene mesh data ---------
  //     (modifies aiMesh::mVertices / mNormals via const_cast)
  applySkinningToScene();

  // --- 2. Regenerate ALL triangle buffers from the (now-skinned)
  //     mesh data.  onUpdateBuffers_all() handles textured-vs-
  //     non-textured routing, texture assignment, and voxel split
  //     correctly because the source data is already skinned.
  const_cast<CAnimatedAssimpModel&>(*this).onUpdateBuffers_all();

  // --- 3. Upload every CPU buffer to its GPU VBO ---------------------
  CAssimpModel::renderUpdateBuffers();

  // --- 4. Restore bind-pose positions so the next frame starts clean -
  restoreBindPoseToScene();
}
// ============================================================================
// Helper: Add bone weight to vertex
// ============================================================================

void CAnimatedAssimpModel::VertexBoneData::addBoneWeight(int boneId, float weight)
{
  // Find first empty slot
  for (int i = 0; i < MAX_BONES_PER_VERTEX; ++i)
  {
    if (boneIds[i] < 0)
    {
      boneIds[i] = boneId;
      weights[i] = weight;
      return;
    }
  }

  // All slots full — replace smallest if new weight is larger
  int minIdx = 0;
  float minWeight = weights[0];
  for (int i = 1; i < MAX_BONES_PER_VERTEX; ++i)
  {
    if (weights[i] < minWeight)
    {
      minWeight = weights[i];
      minIdx = i;
    }
  }
  if (weight > minWeight)
  {
    boneIds[minIdx] = boneId;
    weights[minIdx] = weight;
  }
}

// ============================================================================
// Interpolation: Position (linear)
// ============================================================================

TPoint3Df CAnimatedAssimpModel::interpolatePosition(
    const BoneAnimation& channel, double animTime) const
{
  if (channel.positionKeys.empty())
  {
    return {0.f, 0.f, 0.f};
  }
  if (channel.positionKeys.size() == 1)
  {
    return channel.positionKeys[0].value;
  }

  // Find surrounding keyframes
  size_t idx0 = 0;
  for (size_t i = 0; i + 1 < channel.positionKeys.size(); ++i)
  {
    if (animTime < channel.positionKeys[i + 1].time)
    {
      idx0 = i;
      break;
    }
    idx0 = i;
  }

  const size_t idx1 = idx0 + 1;
  if (idx1 >= channel.positionKeys.size())
  {
    return channel.positionKeys.back().value;
  }

  const auto& k0 = channel.positionKeys[idx0];
  const auto& k1 = channel.positionKeys[idx1];

  const double dt = k1.time - k0.time;
  const float t = (dt > 1e-6) ? static_cast<float>((animTime - k0.time) / dt) : 0.f;

  return {
      k0.value.x + t * (k1.value.x - k0.value.x), k0.value.y + t * (k1.value.y - k0.value.y),
      k0.value.z + t * (k1.value.z - k0.value.z)};
}

// ============================================================================
// Interpolation: Rotation (SLERP)
// ============================================================================

void CAnimatedAssimpModel::interpolateRotation(
    const BoneAnimation& channel, double animTime, float& qw, float& qx, float& qy, float& qz) const
{
  if (channel.rotationKeys.empty())
  {
    qw = 1.f;
    qx = qy = qz = 0.f;
    return;
  }
  if (channel.rotationKeys.size() == 1)
  {
    const auto& k = channel.rotationKeys[0];
    qw = k.w;
    qx = k.x;
    qy = k.y;
    qz = k.z;
    return;
  }

  size_t idx0 = 0;
  for (size_t i = 0; i + 1 < channel.rotationKeys.size(); ++i)
  {
    if (animTime < channel.rotationKeys[i + 1].time)
    {
      idx0 = i;
      break;
    }
    idx0 = i;
  }

  const size_t idx1 = idx0 + 1;
  if (idx1 >= channel.rotationKeys.size())
  {
    const auto& k = channel.rotationKeys.back();
    qw = k.w;
    qx = k.x;
    qy = k.y;
    qz = k.z;
    return;
  }

  const auto& k0 = channel.rotationKeys[idx0];
  const auto& k1 = channel.rotationKeys[idx1];

  const double dt = k1.time - k0.time;
  const float t = (dt > 1e-6) ? static_cast<float>((animTime - k0.time) / dt) : 0.f;

  // SLERP
  float dot = k0.w * k1.w + k0.x * k1.x + k0.y * k1.y + k0.z * k1.z;

  float w1 = k1.w, x1 = k1.x, y1 = k1.y, z1 = k1.z;
  if (dot < 0.f)
  {
    dot = -dot;
    w1 = -w1;
    x1 = -x1;
    y1 = -y1;
    z1 = -z1;
  }

  float s0, s1;
  if (dot > 0.9995f)
  {
    // Very close — linear interpolation
    s0 = 1.f - t;
    s1 = t;
  }
  else
  {
    const float theta = std::acos(std::clamp(dot, -1.0f, 1.0f));
    const float sinTheta = std::sin(theta);
    s0 = std::sin((1.f - t) * theta) / sinTheta;
    s1 = std::sin(t * theta) / sinTheta;
  }

  qw = s0 * k0.w + s1 * w1;
  qx = s0 * k0.x + s1 * x1;
  qy = s0 * k0.y + s1 * y1;
  qz = s0 * k0.z + s1 * z1;
}

// ============================================================================
// Interpolation: Scaling (linear)
// ============================================================================

TPoint3Df CAnimatedAssimpModel::interpolateScaling(
    const BoneAnimation& channel, double animTime) const
{
  if (channel.scalingKeys.empty())
  {
    return {1.f, 1.f, 1.f};
  }
  if (channel.scalingKeys.size() == 1)
  {
    return channel.scalingKeys[0].value;
  }

  size_t idx0 = 0;
  for (size_t i = 0; i + 1 < channel.scalingKeys.size(); ++i)
  {
    if (animTime < channel.scalingKeys[i + 1].time)
    {
      idx0 = i;
      break;
    }
    idx0 = i;
  }

  const size_t idx1 = idx0 + 1;
  if (idx1 >= channel.scalingKeys.size())
  {
    return channel.scalingKeys.back().value;
  }

  const auto& k0 = channel.scalingKeys[idx0];
  const auto& k1 = channel.scalingKeys[idx1];

  const double dt = k1.time - k0.time;
  const float t = (dt > 1e-6) ? static_cast<float>((animTime - k0.time) / dt) : 0.f;

  return {
      k0.value.x + t * (k1.value.x - k0.value.x), k0.value.y + t * (k1.value.y - k0.value.y),
      k0.value.z + t * (k1.value.z - k0.value.z)};
}

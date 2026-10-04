/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <mrpt/core/get_env.h>
#include <mrpt/core/lock_helper.h>
#include <mrpt/opengl/Texture.h>
#include <mrpt/opengl/opengl_api.h>

#include <iostream>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <tuple>
#include <vector>

using namespace mrpt::opengl;

const bool MRPT_OPENGL_VERBOSE = mrpt::get_env<bool>("MRPT_OPENGL_VERBOSE", false);

// Whether to profile memory allocations:
// #define TEXTUREOBJ_PROFILE_MEM_ALLOC

// Whether to use a memory pool for the texture buffer:
#define TEXTUREOBJ_USE_MEMPOOL

#ifdef TEXTUREOBJ_USE_MEMPOOL
#include <mrpt/system/CGenericMemoryPool.h>
#endif

void Texture::unloadTexture()
{
  m_tex.run_on_all(
      [](std::optional<texture_name_unit_t>& tnu)
      {
        if (!tnu)
        {
          return;
        }
        releaseTextureName(tnu.value().name);
        tnu.reset();
      });
}

namespace
{
/** What makes two texture uploads identical, so they can share one OpenGL
 * texture. */
struct SharedTextureKey
{
  const void* scope = nullptr;
  const uint8_t* rgbData = nullptr;
  const uint8_t* alphaData = nullptr;
  int width = 0;
  int height = 0;
  int channels = 0;
  size_t rowStride = 0;
  bool generateMipMaps = true;
  bool magnifyLinearFilter = true;
  bool enableTransparency = false;
  bool isColorData = true;
  Texture::Wrapping wrappingModeS = Texture::Wrapping::Repeat;
  Texture::Wrapping wrappingModeT = Texture::Wrapping::Repeat;

  bool operator<(const SharedTextureKey& o) const
  {
    return std::tie(
               scope, rgbData, alphaData, width, height, channels, rowStride, generateMipMaps,
               magnifyLinearFilter, enableTransparency, isColorData, wrappingModeS, wrappingModeT) <
           std::tie(
               o.scope, o.rgbData, o.alphaData, o.width, o.height, o.channels, o.rowStride,
               o.generateMipMaps, o.magnifyLinearFilter, o.enableTransparency, o.isColorData,
               o.wrappingModeS, o.wrappingModeT);
  }
};

SharedTextureKey makeSharedTextureKey(
    const mrpt::img::CImage& rgb, const mrpt::img::CImage* alpha, const Texture::Options& o)
{
  SharedTextureKey k;
  k.scope = o.shareScope;
  k.rgbData = rgb.ptrLine<uint8_t>(0);
  k.alphaData = alpha != nullptr ? alpha->ptrLine<uint8_t>(0) : nullptr;
  k.width = static_cast<int>(rgb.getWidth());
  k.height = static_cast<int>(rgb.getHeight());
  k.channels = static_cast<int>(rgb.channels());
  k.rowStride = rgb.getRowStride();
  k.generateMipMaps = o.generateMipMaps;
  k.magnifyLinearFilter = o.magnifyLinearFilter;
  k.enableTransparency = o.enableTransparency;
  k.isColorData = o.isColorData;
  k.wrappingModeS = o.wrappingModeS;
  k.wrappingModeT = o.wrappingModeT;
  return k;
}
}  // namespace

/** This class is a workaround to crashes and memory leaks caused by not
 * reserving and freeing opengl textures from the same thread.
 *
 * Textures uploaded from the same image data within the same share scope are
 * shared (see acquireSharedTexture()), so each texture name is reference
 * counted and only deleted in OpenGL once its last user releases it. */
class TextureResourceHandler
{
 public:
  static TextureResourceHandler& Instance()
  {
    static TextureResourceHandler o;
    return o;
  }

  /// Return textureName
  texture_name_t generateTextureID()
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    auto lck = mrpt::lockHelper(m_texturesMtx);

    processDestroyQueue();

    // Create one OpenGL texture
    GLuint textureID = 0;
    glGenTextures(1, &textureID);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    m_textureReservedFrom[textureID] = std::this_thread::get_id();
    m_textureRefCount[textureID] = 1;

    if (MRPT_OPENGL_VERBOSE)
    {
      std::cout << "[mrpt generateTextureID] textureName:" << textureID << "\n";
    }

    return textureID;
#else
    THROW_EXCEPTION("This function needs OpenGL");
#endif
  }

  /** If a texture was already uploaded with this key, adds one user to it and
   * returns its name. */
  std::optional<texture_name_t> acquireSharedTexture(const SharedTextureKey& key)
  {
#if (MRPT_HAS_OPENGL || MRPT_HAS_EGL)
    auto lck = mrpt::lockHelper(m_texturesMtx);

    auto it = m_sharedTextures.find(key);
    if (it != m_sharedTextures.end())
    {
      m_textureRefCount.at(it->second)++;
      return it->second;
    }
    return {};
#else
    return {};
#endif
  }

  /** Makes a texture just uploaded from these images available to later
   * users of the same key. The images (shallow copies) are kept alive, so
   * their pixel buffers cannot be reused by other images meanwhile. */
  void registerSharedTexture(
      texture_name_t name,
      const SharedTextureKey& key,
      const mrpt::img::CImage& rgb,
      const mrpt::img::CImage* alpha)
  {
#if (MRPT_HAS_OPENGL || MRPT_HAS_EGL)
    auto lck = mrpt::lockHelper(m_texturesMtx);

    SharedTextureEntry e;
    e.key = key;
    e.rgb = mrpt::img::CImage(rgb, mrpt::img::SHALLOW_COPY);
    if (alpha != nullptr)
    {
      e.alpha = mrpt::img::CImage(*alpha, mrpt::img::SHALLOW_COPY);
    }
    m_sharedTextures[key] = name;
    m_sharedTextureEntries[name] = std::move(e);
#endif
  }

  void releaseTextureID(unsigned int texName)
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    MRPT_START
    auto lck = mrpt::lockHelper(m_texturesMtx);

    if (MRPT_OPENGL_VERBOSE)
      std::cout << "[mrpt releaseTextureID] textureName: " << texName << "\n";

    // Whoever re-assigns this image from now on gets a fresh upload: callers
    // release a texture before re-uploading an image whose content changed.
    unregisterSharedTexture(texName);

    // Unknown names are ignored: this runs from destructors, so it must not throw.
    auto itCount = m_textureRefCount.find(texName);
    if (itCount == m_textureRefCount.end())
    {
      return;
    }
    if (--itCount->second > 0)
    {
      return;  // still in use by others
    }
    m_textureRefCount.erase(itCount);

    auto itFrom = m_textureReservedFrom.find(texName);
    m_destroyQueue[itFrom->second].push_back(texName);
    m_textureReservedFrom.erase(itFrom);

    processDestroyQueue();
    MRPT_END
#endif
  }

 private:
  TextureResourceHandler()
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    glGetIntegerv(GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS, &m_maxTextureUnits);
    if (MRPT_OPENGL_VERBOSE)
      std::cout << "[mrpt TextureResourceHandler] maxTextureUnits:" << m_maxTextureUnits << "\n";
#endif
  }

  void unregisterSharedTexture(texture_name_t name)
  {
    if (auto it = m_sharedTextureEntries.find(name); it != m_sharedTextureEntries.end())
    {
      m_sharedTextures.erase(it->second.key);
      m_sharedTextureEntries.erase(it);
    }
  }

  void processDestroyQueue()
  {
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
    if (auto itLst = m_destroyQueue.find(std::this_thread::get_id());
        itLst != m_destroyQueue.end() && !itLst->second.empty())
    {
      auto& lst = itLst->second;

      // Delete in OpenGL:
      glDeleteTextures(static_cast<GLsizei>(lst.size()), lst.data());
      CHECK_OPENGL_ERROR_IN_DEBUG();

      for (const auto id : lst)
      {
        unregisterSharedTexture(id);
      }

      if (MRPT_OPENGL_VERBOSE)
      {
        std::cout << "[mrpt processDestroyQueue] threadId=" << std::this_thread::get_id()
                  << " destroyed " << lst.size() << "\n";
      }
      lst.clear();
      m_destroyQueue.erase(itLst);
    }
    if (!m_destroyQueue.empty() && MRPT_OPENGL_VERBOSE)
    {
      std::cout << "[mrpt processDestroyQueue] threadId=" << std::this_thread::get_id()
                << ". Remaining at output: ";
      for (const auto& lst : m_destroyQueue)
        std::cout << "[" << lst.first << "]=" << lst.second.size() << " textures ";
      std::cout << "\n";
    }
#endif
  }

#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  struct SharedTextureEntry
  {
    SharedTextureKey key;
    mrpt::img::CImage rgb;
    mrpt::img::CImage alpha;
  };

  std::mutex m_texturesMtx;
  std::map<GLuint, std::thread::id> m_textureReservedFrom;
  std::map<GLuint, int> m_textureRefCount;  //!< Number of users of each texture name
  std::map<std::thread::id, std::vector<GLuint>> m_destroyQueue;
  std::map<SharedTextureKey, GLuint> m_sharedTextures;
  std::map<GLuint, SharedTextureEntry> m_sharedTextureEntries;
  GLint m_maxTextureUnits;
#endif
};

/// Returns: [texture name, texture unit]
texture_name_t mrpt::opengl::getNewTextureNumber(
    [[maybe_unused]] const uint8_t* optionalRgbDataForAssociation)
{
  return TextureResourceHandler::Instance().generateTextureID();
}

void mrpt::opengl::releaseTextureName(const texture_name_t& t)
{
  TextureResourceHandler::Instance().releaseTextureID(t);
}

bool Texture::initialized() const
{
  // already assigned an ID?
  return m_tex.get().has_value();
}

void Texture::assignImage2D(const mrpt::img::CImage& rgb, const Options& o, int textureUnit)
{
  try
  {
    internalAssignImage_2D(&rgb, nullptr, o, textureUnit);
  }
  catch (std::exception& e)
  {
    THROW_EXCEPTION_FMT("Error assigning texture: %s", e.what());
  }
}

void Texture::assignImage2D(
    const mrpt::img::CImage& rgb, const mrpt::img::CImage& alpha, const Options& o, int textureUnit)
{
  try
  {
    internalAssignImage_2D(&rgb, &alpha, o, textureUnit);
  }
  catch (std::exception& e)
  {
    THROW_EXCEPTION_FMT("Error assigning texture: %s", e.what());
  }
}

// Data types for memory pooling CRenderizableShaderTexturedTriangles:
#ifdef TEXTUREOBJ_USE_MEMPOOL

struct CRenderizableShaderTexturedTriangles_MemPoolParams
{
  /** size of the vector<unsigned char> */
  size_t len = 0;

  inline bool isSuitable(const CRenderizableShaderTexturedTriangles_MemPoolParams& req) const
  {
    return len == req.len;
  }
};
struct CRenderizableShaderTexturedTriangles_MemPoolData
{
  std::vector<uint8_t> data;
};

using TMyMemPool = mrpt::system::CGenericMemoryPool<
    CRenderizableShaderTexturedTriangles_MemPoolParams,
    CRenderizableShaderTexturedTriangles_MemPoolData>;
#endif

// Auxiliary function for loadTextureInOpenGL(): reserve memory and return
// 16byte aligned starting point within it:
namespace
{
unsigned char* reserveDataBuffer(size_t len, std::vector<uint8_t>& data)
{
#ifdef TEXTUREOBJ_USE_MEMPOOL
  TMyMemPool* pool = TMyMemPool::getInstance();
  if (pool != nullptr)
  {
    CRenderizableShaderTexturedTriangles_MemPoolParams mem_params;
    mem_params.len = len;

    CRenderizableShaderTexturedTriangles_MemPoolData* mem_block = pool->request_memory(mem_params);
    if (mem_block != nullptr)
    {
      // Recover the memory block via a swap:
      data.swap(mem_block->data);
      delete mem_block;
    }
  }
#endif
  data.resize(len);
  void* ptr = data.data();
  size_t space = len;
  return reinterpret_cast<unsigned char*>(std::align(16, 1 /*dummy size*/, ptr, space));
}
}  // namespace

void Texture::internalAssignImage_2D(
    const mrpt::img::CImage* in_rgb,
    const mrpt::img::CImage* in_alpha,
    const Options& o,
    int textureUnit)
{
#if (MRPT_HAS_OPENGL || MRPT_HAS_EGL)
  unsigned char* dataAligned = nullptr;
  std::vector<uint8_t> data;

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
  static mrpt::system::CTimeLogger tim;
#endif

  ASSERT_(in_rgb);

  in_rgb->forceLoad();  // just in case they are lazy-load imgs
  if (in_alpha != nullptr)
  {
    in_alpha->forceLoad();
  }

  // Drop the reference held from a previous assignment, if any, so each
  // reference is released exactly once:
  if (get().has_value())
  {
    releaseTextureName(get()->name);
    get().reset();
  }

  // Check if we already have this texture loaded in GPU and avoid creating
  // duplicated texture ID:
  const auto sharedKey = makeSharedTextureKey(*in_rgb, in_alpha, o);
  const auto existingTextureId =
      o.shareScope != nullptr ? TextureResourceHandler::Instance().acquireSharedTexture(sharedKey)
                              : std::optional<texture_name_t>();
  if (existingTextureId.has_value())
  {
    get() = existingTextureId.value();
    get()->unit = textureUnit;

    if (MRPT_OPENGL_VERBOSE)
    {
      std::cout << "[mrpt internalAssignImage_2D] Reusing existing textureName:" << get()->name
                << "\n";
    }

    return;
  }

  mrpt::img::CImage rgb;

  switch (in_rgb->getPixelDepth())
  {
    // Ideal case, nothing to do:
    case mrpt::img::PixelDepth::D8U:
      rgb = mrpt::img::CImage(*in_rgb, mrpt::img::SHALLOW_COPY);
      break;

    case mrpt::img::PixelDepth::D16U:
    {
      // Convert 16-bit to 8-bit by scaling (keeping the high byte)
      const auto nCh = in_rgb->isColor() ? mrpt::img::CH_RGB : mrpt::img::CH_GRAY;
      const int32_t w = in_rgb->getWidth();
      const int32_t h = in_rgb->getHeight();
      const int chCount = static_cast<int>(nCh);

      rgb.resize(w, h, nCh, mrpt::img::PixelDepth::D8U);

      for (int32_t y = 0; y < h; y++)
      {
        const auto* src = in_rgb->ptrLine<uint16_t>(y);
        auto* dst = rgb.ptrLine<uint8_t>(y);
        for (int32_t x = 0; x < w * chCount; x++)
        {
          dst[x] = static_cast<uint8_t>(src[x] >> 8);
        }
      }
    }
    break;

    default:
      THROW_EXCEPTION_FMT(
          "Unhandled pixel depth: PixelDepth=#%i", static_cast<int>(in_rgb->getPixelDepth()));
  };

  // Shallow copy of the images, for the case we need to downsample them
  // below:
  mrpt::img::CImage alpha;
  if (in_alpha != nullptr)
  {
    alpha = mrpt::img::CImage(*in_alpha, mrpt::img::SHALLOW_COPY);
  }

  // allocate texture names:
  get() = getNewTextureNumber();
  get()->unit = textureUnit;
  if (o.shareScope != nullptr)
  {
    TextureResourceHandler::Instance().registerSharedTexture(
        get()->name, sharedKey, *in_rgb, in_alpha);
  }

  // activate the texture unit first before binding texture
  bindAsTexture2D();

  // when texture area is small, linear interpolation:
  if (o.generateMipMaps)
  {
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }
  else
  {
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // when texture area is large:
  glTexParameterf(
      GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, o.magnifyLinearFilter ? GL_LINEAR : GL_NEAREST);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  const auto lmbdWrapMap = [](const Wrapping w)
  {
    // clang-format off
    switch (w)
    {
      case Wrapping::Repeat:           return GL_REPEAT;
      case Wrapping::MirroredRepeat:   return GL_MIRRORED_REPEAT;
      case Wrapping::ClampToEdge:      return GL_CLAMP_TO_EDGE;
      case Wrapping::ClapToBorder:     return GL_CLAMP_TO_BORDER;
      default:
        THROW_EXCEPTION_FMT("Invalid texture wrapping value: %i", static_cast<int>(w));
    };
    // clang-format on
  };

  // if wrap is true, the texture wraps over at the edges (repeat)
  //       ... false, the texture ends at the edges (clamp)
  glTexParameterf(
      GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, static_cast<GLfloat>(lmbdWrapMap(o.wrappingModeS)));
  CHECK_OPENGL_ERROR_IN_DEBUG();

  glTexParameterf(
      GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, static_cast<GLfloat>(lmbdWrapMap(o.wrappingModeT)));
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Ensure that the images do not overpass the maximum dimensions allowed
  // by OpenGL:
  // ------------------------------------------------------------------------------------
  GLint texSize = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &texSize);
  while (rgb.getHeight() > static_cast<unsigned int>(texSize) ||
         rgb.getWidth() > static_cast<unsigned int>(texSize))
  {
    static bool warningEmitted = false;
    if (!warningEmitted)
    {
      warningEmitted = true;
      std::cerr << "[mrpt::opengl::Texture] "
                   "**PERFORMACE WARNING**:\n"
                << " Downsampling texture image of size " << rgb.getWidth() << "x"
                << rgb.getHeight()
                << " since maximum allowed OpenGL texture size "
                   "(GL_MAX_TEXTURE_SIZE) is "
                << texSize << "\n";
    }

    rgb = rgb.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
    if (in_alpha != nullptr)
    {
      alpha = alpha.scaleHalf(mrpt::img::IMG_INTERP_LINEAR);
    }
  }

  const int width = rgb.getWidth();
  const int height = rgb.getHeight();

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
  {
    const std::string sSec = mrpt::format(
        "opengl_texture: load %ix%i %s %stransp", width, height, rgb->isColor() ? "RGB" : "BW",
        m_enableTransparency ? "" : "no ");
    tim.enter(sSec.c_str());
  }
#endif

  // GL_LUMINANCE and GL_LUMINANCE_ALPHA were removed in OpenGL 3.1
  // Convert grayscale images into color:
  if (!rgb.isColor())
  {
    rgb = rgb.colorImage();
  }

  // ----------------------------------------------
  // Color texture WITH alpha channel
  // ----------------------------------------------
  if (o.enableTransparency)
  {
    ASSERT_(!alpha.isColor());
    ASSERT_EQUAL_(alpha.getWidth(), rgb.getWidth());
    ASSERT_EQUAL_(alpha.getHeight(), rgb.getHeight());

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
    const std::string sSec =
        mrpt::format("opengl_texture_alloc %ix%i (color,trans)", width, height);
    tim.enter(sSec.c_str());
#endif

    dataAligned = reserveDataBuffer(height * width * 4 + 512, data);

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
    tim.leave(sSec.c_str());
#endif

    for (int y = 0; y < height; y++)
    {
      const auto* ptrSrcCol = rgb.ptrLine<uint8_t>(y);
      const auto* ptrSrcAlfa = alpha.ptrLine<uint8_t>(y);
      auto* ptr = dataAligned + static_cast<ptrdiff_t>(y * width * 4);

      for (int x = 0; x < width; x++)
      {
        *ptr++ = *ptrSrcCol++;
        *ptr++ = *ptrSrcCol++;
        *ptr++ = *ptrSrcCol++;
        *ptr++ = *ptrSrcAlfa++;
      }
    }

    // Prepare image data types:
    const GLenum img_type = GL_UNSIGNED_BYTE;
    // STB always outputs RGB/RGBA order; legacy OpenCV images may be BGR.
    // getChannelsOrder() returns "RGB" (3ch) or "RGBA" (4ch) for RGB-order data.
    const auto chOrder = rgb.getChannelsOrder();
    const bool is_RGB_order = (chOrder == "RGB" || chOrder == "RGBA");
    const GLenum img_format = (is_RGB_order ? GL_RGBA : GL_BGRA);

    // Send image data to OpenGL:
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ROW_LENGTH, width);

    /* sRGB internal: GPU decodes at sampling */
    const GLint internalFormat = o.isColorData ? GL_SRGB8_ALPHA8 : GL_RGBA8;

    glTexImage2D(
        GL_TEXTURE_2D, 0 /*level*/, internalFormat, width, height, 0 /*border*/, img_format,
        img_type, dataAligned);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset
    CHECK_OPENGL_ERROR_IN_DEBUG();

  }  // End of color texture WITH trans.
  else
  {
    // Color texture without transparency,
    // or with integrated RGBA alpha channel
    // --------------------------------------
    // Prepare image data types:
    const GLenum img_type = GL_UNSIGNED_BYTE;
    const int nBytesPerPixel = rgb.channels();
    // STB always outputs RGB/RGBA order; legacy OpenCV images may be BGR.
    // getChannelsOrder() returns "RGB" (3ch) or "RGBA" (4ch) for RGB-order data.
    const auto chOrder2D = rgb.getChannelsOrder();
    const bool is_RGB_order = (chOrder2D == "RGB" || chOrder2D == "RGBA");
    const GLenum img_format = [=]()
    {
      switch (nBytesPerPixel)
      {
        case 1:
          return GL_LUMINANCE;
        case 3:
          return (is_RGB_order ? GL_RGB : GL_BGR);
        case 4:
          return (is_RGB_order ? GL_RGBA : GL_BGRA);
        default:
          THROW_EXCEPTION("Invalid texture image channel count.");
      };
    }();

    // Send image data to OpenGL:
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glPixelStorei(GL_UNPACK_ROW_LENGTH, static_cast<GLint>(rgb.getRowStride() / nBytesPerPixel));
    CHECK_OPENGL_ERROR_IN_DEBUG();

    /* sRGB internal: GPU decodes at sampling */
    const GLint internalFormat = o.isColorData
                                     ? (nBytesPerPixel == 3 ? GL_SRGB8 : GL_SRGB8_ALPHA8)  // NOLINT
                                     : (nBytesPerPixel == 3 ? GL_RGB8 : GL_RGBA8);

    glTexImage2D(
        GL_TEXTURE_2D, 0 /*level*/, internalFormat, width, height, 0 /*border*/, img_format,
        img_type, rgb.ptrLine<uint8_t>(0));
    CHECK_OPENGL_ERROR_IN_DEBUG();

    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }  // End of color texture WITHOUT trans.

  if (o.generateMipMaps)
  {
    glGenerateMipmap(GL_TEXTURE_2D);
    CHECK_OPENGL_ERROR_IN_DEBUG();
  }

  // Was: m_texture_is_loaded = true;
  // Now this situation is represented by the optional m_glTexture having
  // a valid value.

#ifdef TEXTUREOBJ_PROFILE_MEM_ALLOC
  {
    const std::string sSec = mrpt::format(
        "opengl_texture: load %ix%i %s %stransp", width, height, rgb->isColor() ? "RGB" : "BW",
        m_enableTransparency ? "" : "no ");
    tim.leave(sSec.c_str());
  }
#endif

#ifdef TEXTUREOBJ_USE_MEMPOOL
  // Before freeing the buffer in "data", donate my memory to the pool:
  if (!data.empty())
  {
    TMyMemPool* pool = TMyMemPool::getInstance();
    if (pool != nullptr)
    {
      CRenderizableShaderTexturedTriangles_MemPoolParams mem_params;
      mem_params.len = data.size();

      auto* mem_block = new CRenderizableShaderTexturedTriangles_MemPoolData();
      data.swap(mem_block->data);

      pool->dump_to_pool(mem_params, mem_block);
    }
  }
#endif
#endif
}

void Texture::bindAsTexture2D()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + get()->unit);
  glBindTexture(GL_TEXTURE_2D, get()->name);
  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}

void Texture::bindAsCubeTexture()
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL
  glActiveTexture(GL_TEXTURE0 + get()->unit);
  glBindTexture(GL_TEXTURE_CUBE_MAP, get()->name);
  CHECK_OPENGL_ERROR_IN_DEBUG();
#endif
}

void Texture::assignCubeImages(
    const std::array<mrpt::img::CImage, 6>& imgs, [[maybe_unused]] int textureUnit)
{
#if MRPT_HAS_OPENGL || MRPT_HAS_EGL

  // just in case they are lazy-load imgs
  for (const auto& im : imgs)
  {
    im.forceLoad();
    ASSERT_(im.getPixelDepth() == mrpt::img::PixelDepth::D8U);
    ASSERT_(im.isColor());
  }

  // allocate texture "name" (ID):
  if (get().has_value())
  {
    releaseTextureName(get()->name);
    get().reset();
  }
  get() = getNewTextureNumber();

  // activate the texture unit first before binding texture
  bindAsCubeTexture();

  glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  // Special "wrap" for cube skybox textures:
  glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameterf(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
  CHECK_OPENGL_ERROR_IN_DEBUG();

  // Check max hardware-allowed texture size:
  {
    GLint maxTexSize;
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTexSize);
    ASSERT_LE_(imgs[0].getHeight(), static_cast<unsigned int>(maxTexSize));
    ASSERT_LE_(imgs[0].getWidth(), static_cast<unsigned int>(maxTexSize));
  }

  for (int face = 0; face < 6; face++)
  {
    const auto& rgb = imgs.at(face);

    const int width = rgb.getWidth();
    const int height = rgb.getHeight();

    // Format: color texture without transparency, or with integrated
    // RGBA alpha channel

    // Prepare image data types:
    const GLenum img_type = GL_UNSIGNED_BYTE;
    const int nBytesPerPixel = rgb.channels();
    // STB always outputs RGB/RGBA order; legacy OpenCV images may be BGR.
    // getChannelsOrder() returns "RGB" (3ch) or "RGBA" (4ch) for RGB-order data.
    const auto chOrderCube = rgb.getChannelsOrder();
    const bool is_RGB_order = (chOrderCube == "RGB" || chOrderCube == "RGBA");
    const GLenum img_format = [=]()
    {
      switch (nBytesPerPixel)
      {
        case 1:
          return GL_LUMINANCE;
        case 3:
          return (is_RGB_order ? GL_RGB : GL_BGR);
        case 4:
          return (is_RGB_order ? GL_RGBA : GL_BGRA);
      };
      THROW_EXCEPTION("Invalid texture image channel count.");
    }();

    // Send image data to OpenGL:
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glPixelStorei(GL_UNPACK_ROW_LENGTH, static_cast<GLint>(rgb.getRowStride() / nBytesPerPixel));
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glTexImage2D(
        GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0 /*level*/,
        nBytesPerPixel == 3 ? GL_SRGB8 : GL_SRGB8_ALPHA8 /* sRGB: GPU decodes at sampling */, width,
        height, 0 /*border*/, img_format, img_type, rgb.ptrLine<uint8_t>(0));
    CHECK_OPENGL_ERROR_IN_DEBUG();
    glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);  // Reset
    CHECK_OPENGL_ERROR_IN_DEBUG();

  }  // end for each "face"

#endif
}

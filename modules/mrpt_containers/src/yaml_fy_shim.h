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

/* Plain-C, POD-only wrapper around libfyaml's event-based parser.
 *
 * <libfyaml.h> must never be #include'd from a C++ translation unit: the
 * library's atomics shim (used internally for ID generation) has been the
 * source of repeated cross-compiler breakage (Clang/_Atomic on Windows,
 * Xcode 16 <atomic>, etc.) that only manifests when the header is parsed
 * as C++. This shim is the only place in MRPT that #includes <libfyaml.h>,
 * and it is always compiled as plain C, so none of that ever applies here.
 * No fy_* type crosses this boundary.
 */

#ifndef MRPT_CONTAINERS_YAML_FY_SHIM_H
#define MRPT_CONTAINERS_YAML_FY_SHIM_H

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

  typedef enum
  {
    MRPT_FY_EV_END = 0,  // stream/container exhausted
    MRPT_FY_EV_MAP_START,
    MRPT_FY_EV_MAP_END,
    MRPT_FY_EV_SEQ_START,
    MRPT_FY_EV_SEQ_END,
    MRPT_FY_EV_SCALAR
  } mrpt_fy_event_type_t;

  // All char* fields are heap-allocated by mrpt_fy_parser_next() (or NULL),
  // and must be released with mrpt_fy_event_release() once consumed.
  typedef struct
  {
    mrpt_fy_event_type_t type;

    char* text;  // MRPT_FY_EV_SCALAR only
    size_t text_len;

    char* anchor_text;  // MRPT_FY_EV_SCALAR only, debug logging, may be NULL

    char* comment_top;
    char* comment_right;
    char* comment_bottom;

    int has_mark;
    int line;
    int column;
    size_t input_pos;
  } mrpt_fy_event_t;

  typedef struct mrpt_fy_parser mrpt_fy_parser_t;

  // Returns NULL on failure (invalid YAML/JSON text or out of memory).
  mrpt_fy_parser_t* mrpt_fy_parser_create(const char* yaml_text, size_t len);
  void mrpt_fy_parser_destroy(mrpt_fy_parser_t* p);

  // Fills *out with the next simplified event. Stream/document
  // start-or-end and alias events are consumed internally and never
  // surfaced. Returns 0 on success, nonzero on an unexpected internal
  // parser event type.
  int mrpt_fy_parser_next(mrpt_fy_parser_t* p, mrpt_fy_event_t* out);

  // Frees any heap-allocated strings inside *ev. Idempotent.
  void mrpt_fy_event_release(mrpt_fy_event_t* ev);

#ifdef __cplusplus
}
#endif

#endif  // MRPT_CONTAINERS_YAML_FY_SHIM_H

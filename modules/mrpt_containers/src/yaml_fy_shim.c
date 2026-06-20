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

/* This file must be compiled as plain C (not C++). See yaml_fy_shim.h. */

#include "yaml_fy_shim.h"

#include <libfyaml.h>
#include <stdlib.h>
#include <string.h>

struct mrpt_fy_parser
{
  struct fy_parser* p;
};

static char* dup_text(const char* s, size_t len)
{
  char* ret;
  if (!s) return NULL;
  ret = (char*)malloc(len + 1);
  if (!ret) return NULL;
  memcpy(ret, s, len);
  ret[len] = '\0';
  return ret;
}

static char* dup_comment(struct fy_token* t, enum fy_comment_placement cp)
{
  const char* s;
  if (!t) return NULL;
#if MRPT_LIBFYAML_VERSION >= 0x000904  // 0.9.4
  s = fy_token_get_comment(t, cp);
#else
  char buf[2048];
  s = fy_token_get_comment(t, buf, sizeof(buf), cp);
#endif
  if (!s || s[0] == '\0') return NULL;
  return dup_text(s, strlen(s));
}

static char* dup_anchor_text(struct fy_token* anchor)
{
  size_t len = 0;
  const char* s;
  if (!anchor) return NULL;
  s = fy_token_get_text(anchor, &len);
  return dup_text(s, len);
}

static void fill_comments_and_mark(struct fy_token* tk, mrpt_fy_event_t* out)
{
  const struct fy_mark* mrk;

  if (!tk)
  {
    out->has_mark = 0;
    return;
  }

  out->comment_top = dup_comment(tk, fycp_top);
  out->comment_right = dup_comment(tk, fycp_right);
  out->comment_bottom = dup_comment(tk, fycp_bottom);

  mrk = fy_token_start_mark(tk);
  if (mrk)
  {
    out->has_mark = 1;
    out->line = mrk->line;
    out->column = mrk->column;
    out->input_pos = mrk->input_pos;
  }
  else
  {
    out->has_mark = 0;
  }
}

mrpt_fy_parser_t* mrpt_fy_parser_create(const char* yaml_text, size_t len)
{
  struct fy_parse_cfg cfg;
  struct fy_parser* p;
  mrpt_fy_parser_t* ret;

  memset(&cfg, 0, sizeof(cfg));
  cfg.search_path = "";
  cfg.diag = NULL;
  cfg.flags = FYPCF_PARSE_COMMENTS;

  p = fy_parser_create(&cfg);
  if (!p) return NULL;

  if (fy_parser_set_string(p, yaml_text, len) != 0)
  {
    fy_parser_destroy(p);
    return NULL;
  }

  ret = (mrpt_fy_parser_t*)malloc(sizeof(mrpt_fy_parser_t));
  if (!ret)
  {
    fy_parser_destroy(p);
    return NULL;
  }
  ret->p = p;
  return ret;
}

void mrpt_fy_parser_destroy(mrpt_fy_parser_t* p)
{
  if (!p) return;
  if (p->p) fy_parser_destroy(p->p);
  free(p);
}

int mrpt_fy_parser_next(mrpt_fy_parser_t* p, mrpt_fy_event_t* out)
{
  memset(out, 0, sizeof(*out));

  for (;;)
  {
    struct fy_event* ev = fy_parser_parse(p->p);
    if (!ev)
    {
      out->type = MRPT_FY_EV_END;
      return 0;
    }

    switch (ev->type)
    {
      case FYET_NONE:
      case FYET_STREAM_START:
      case FYET_STREAM_END:
      case FYET_DOCUMENT_START:
      case FYET_DOCUMENT_END:
      case FYET_ALIAS:
        fy_parser_event_free(p->p, ev);
        continue;

      case FYET_MAPPING_START:
        out->type = MRPT_FY_EV_MAP_START;
        fill_comments_and_mark(ev->mapping_start.mapping_start, out);
        fy_parser_event_free(p->p, ev);
        return 0;

      case FYET_MAPPING_END:
        out->type = MRPT_FY_EV_MAP_END;
        fy_parser_event_free(p->p, ev);
        return 0;

      case FYET_SEQUENCE_START:
        out->type = MRPT_FY_EV_SEQ_START;
        fill_comments_and_mark(ev->sequence_start.sequence_start, out);
        fy_parser_event_free(p->p, ev);
        return 0;

      case FYET_SEQUENCE_END:
        out->type = MRPT_FY_EV_SEQ_END;
        fy_parser_event_free(p->p, ev);
        return 0;

      case FYET_SCALAR:
      {
        size_t len = 0;
        const char* txt = fy_token_get_text(ev->scalar.value, &len);

        out->type = MRPT_FY_EV_SCALAR;
        out->text = dup_text(txt, len);
        out->text_len = txt ? len : 0;
        out->anchor_text = dup_anchor_text(ev->scalar.anchor);
        fill_comments_and_mark(ev->scalar.value, out);

        fy_parser_event_free(p->p, ev);
        return 0;
      }

      default:
        fy_parser_event_free(p->p, ev);
        return -1;
    }
  }
}

void mrpt_fy_event_release(mrpt_fy_event_t* ev)
{
  if (!ev) return;
  free(ev->text);
  ev->text = NULL;
  free(ev->anchor_text);
  ev->anchor_text = NULL;
  free(ev->comment_top);
  ev->comment_top = NULL;
  free(ev->comment_right);
  ev->comment_right = NULL;
  free(ev->comment_bottom);
  ev->comment_bottom = NULL;
}

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          https://www.mrpt.org/                            |
   |                                                                           |
   | Copyright (c) 2005-2024, Individual contributors, see AUTHORS file        |
   | See: https://www.mrpt.org/Authors - All rights reserved.                  |
   | Released under BSD License. See details in https://www.mrpt.org/License   |
   +---------------------------------------------------------------------------+
   */
#pragma once
#include <functional>

#include "mrpt/obs/CSensoryFrame.h"
#include "mrpt/poses/CPose3DPDF.h"

using UserAction = std::function<void()>;

/** Singleton for undo and redo user actions
 */

class CUndoManager
{
 public:
  static CUndoManager& instance()
  {
    static CUndoManager s;
    return s;
  }

  void addAction(UserAction undo, UserAction redo);
  UserAction undoAction();
  bool hasUndo() const;
  UserAction redoAction();
  bool hasRedo() const;

 private:
  CUndoManager() = default;
  ~CUndoManager() = default;
  CUndoManager(CUndoManager const&) = delete;
  CUndoManager& operator=(CUndoManager const&) = delete;

  std::vector<UserAction> m_undo;
  std::vector<UserAction> m_redo;
  int m_lastAction{-1};
};

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
   */
#include "CUndoManager.h"

void CUndoManager::addAction(UserAction undo, UserAction redo)
{
	ASSERT_(m_undo.size() == m_redo.size());
	if (m_undo.size() >= 10)
	{
		m_undo.erase(m_undo.begin());
		m_redo.erase(m_redo.begin());
	}

	m_undo.push_back(undo);
	m_redo.push_back(redo);
	m_lastAction = m_undo.size() - 1;
}

UserAction CUndoManager::redoAction()
{
	ASSERT_(!m_redo.empty());
	int redoSize = m_redo.size();
	ASSERT_(m_lastAction < redoSize);
	++m_lastAction;
	auto it = m_redo.begin() + m_lastAction;
	ASSERT_(it != m_redo.end());

	UserAction action = *it;
	return action;
}

bool CUndoManager::hasRedo() const
{
	bool f = !m_redo.empty();
	int redoLastIndex = m_redo.size() - 1;
	bool s = m_lastAction < redoLastIndex;
	return (f && s);
}

UserAction CUndoManager::undoAction()
{
	ASSERT_(!m_undo.empty());
	int undoSize = m_undo.size();
	ASSERT_(m_lastAction < undoSize);
	auto it = m_undo.begin() + m_lastAction;
	ASSERT_(it != m_undo.end());

	UserAction action = *it;
	--m_lastAction;
	return action;
}

bool CUndoManager::hasUndo() const
{
	return (m_lastAction != -1 && !m_undo.empty());
}

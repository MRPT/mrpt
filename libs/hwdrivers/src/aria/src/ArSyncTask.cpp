/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "ArExport.h"
#include "ariaOSDef.h"
#include "ariaUtil.h"
#include "ArSyncTask.h"
#include "ArLog.h"

/**
   New should never be called to create an ArSyncTask except to create the 
   root node.  Read the detailed documentation of the class for details.
*/
AREXPORT ArSyncTask::ArSyncTask(const char *name, ArFunctor *functor,
				ArTaskState::State *state, ArSyncTask *parent)
{
  myName = name;
  myStatePointer = state;
  myFunctor = functor;
  myParent = parent;
  myIsDeleting = false;
  setState(ArTaskState::INIT);
  if (myParent != NULL)
  {
    setWarningTimeCB(parent->getWarningTimeCB());
    setNoTimeWarningCB(parent->getNoTimeWarningCB());
  }
  else
  {
    setWarningTimeCB(NULL);
    setNoTimeWarningCB(NULL);
  }
}

/**
   If you delete the task it deletes everything in its list, so to 
   delete the whole tree just delete the top one... also note that if you
   delete a node, it will remove itself from its parents list.
*/
AREXPORT ArSyncTask::~ArSyncTask()
{
  myIsDeleting = true;
  if (myParent != NULL && !myParent->isDeleting())
    myParent->remove(this);
  
  ArUtil::deleteSetPairs(myMultiMap.begin(), myMultiMap.end());  
  myMultiMap.clear();
}

AREXPORT ArTaskState::State ArSyncTask::getState(void)
{
  if (myStatePointer != NULL)
    return *myStatePointer;
  else
    return myState;
}

AREXPORT void ArSyncTask::setState(ArTaskState::State state)
{
  if (myStatePointer != NULL)
    *myStatePointer = state;
  else
    myState = state;
}

AREXPORT std::string ArSyncTask::getName(void)
{
  return myName;
}


/**
   Finds a node below (or at) this level in the tree with the given name
   @param name The name of the child we are interested in finding
   @return The task, if found.  If not found, NULL.
*/
AREXPORT ArSyncTask *ArSyncTask::find(ArFunctor *functor)
{
  ArSyncTask *proc;
  std::multimap<int, ArSyncTask *>::iterator it;
  
  if (myFunctor == functor)
    return this;

  for (it = myMultiMap.begin(); it != myMultiMap.end(); ++it)
  {
    proc = (*it).second;
    if (proc->find(functor) != NULL)
      return proc;
  }
  return NULL;
  
}

/**
   Finds a node below (or at) this level in the tree with the given name
   @param name The name of the child we are interested in finding
   @return The task, if found.  If not found, NULL.
*/
AREXPORT ArSyncTask *ArSyncTask::find(const char *name)
{
  ArSyncTask *proc;
  std::multimap<int, ArSyncTask *>::iterator it;
  
  if (strcmp(myName.c_str(), name) == 0)
    return this;

  for (it = myMultiMap.begin(); it != myMultiMap.end(); ++it)
  {
    proc = (*it).second;
    if (proc->find(name) != NULL)
      return proc;
  }
  return NULL;
  
}

/**
   Finds a child of this node with the given name
   @param name The name of the child we are interested in finding
   @return The task, if found.  If not found, NULL.
*/
AREXPORT ArSyncTask *ArSyncTask::findNonRecursive(const char * name)
{
  ArSyncTask *proc;
  std::multimap<int, ArSyncTask *>::iterator it;
  
  for (it = myMultiMap.begin(); it != myMultiMap.end(); ++it)
  {
    proc = (*it).second;
    if (strcmp(proc->getName().c_str(), name) == 0)  
      return proc;
  }
  return NULL;
}

/**
   Finds a child of this node with the given functor
   @param functor the functor we are interested in finding
   @return The task, if found.  If not found, NULL.
*/
AREXPORT ArSyncTask *ArSyncTask::findNonRecursive(ArFunctor *functor)
{
  ArSyncTask *proc;
  std::multimap<int, ArSyncTask *>::iterator it;
  
  for (it = myMultiMap.begin(); it != myMultiMap.end(); ++it)
  {
    proc = (*it).second;
    if (proc->getFunctor() == functor)
      return proc;
  }
  return NULL;
}

/**
   Creates a new task with the given name and puts the task into its 
   own iternal list at the given position.  
   @param nameOfNew Name to give to the new task.
   @param position place in list to put the branch, things are run/printed in 
   the order of highest number to lowest number, no limit on numbers (other 
   than that it is an int).  ARIA uses 0 to 100 just as a convention.
*/
AREXPORT void ArSyncTask::addNewBranch(const char *nameOfNew, int position,
				       ArTaskState::State *state)
{
  ArSyncTask *proc = new ArSyncTask(nameOfNew, NULL, state, this);
  myMultiMap.insert(std::pair<int, ArSyncTask *>(position, proc));
}

/**
   Creates a new task with the given name and puts the task into its 
   own iternal list at the given position.  Sets the nodes functor so that
   it will call the functor when run is called.
   @param nameOfNew Name to give to the new task.
   @param position place in list to put the branch, things are run/printed in 
   the order of highest number to lowest number, no limit on numbers (other 
   than that it is an int).  ARIA uses 0 to 100 just as a convention.
   @param functor ArFunctor which contains the functor to invoke when run is 
   called.
*/
AREXPORT void ArSyncTask::addNewLeaf(const char *nameOfNew, int position, 
				     ArFunctor *functor, 
				     ArTaskState::State *state)
{
  ArSyncTask *proc = new ArSyncTask(nameOfNew, functor, state, this);
  myMultiMap.insert(std::pair<int, ArSyncTask *>(position, proc));
}

AREXPORT void ArSyncTask::remove(ArSyncTask *proc)
{
  std::multimap<int, ArSyncTask *>::iterator it;
  
  for (it = myMultiMap.begin(); it != myMultiMap.end(); it++)
  {
    if ((*it).second == proc)
    {
      myMultiMap.erase(it);
      return;
    }
  }
}

AREXPORT bool ArSyncTask::isDeleting(void)
{
  return myIsDeleting;
}

AREXPORT ArFunctor *ArSyncTask::getFunctor(void)
{
  return myFunctor;
}

/**
   If this node is a leaf it calls the functor for the node, if it is
   a branch it goes through all of the children in the order of
   highest position to lowest position and calls run on them.
**/
AREXPORT void ArSyncTask::run(void)
{
  std::multimap<int, ArSyncTask *>::reverse_iterator it;
  ArTaskState::State state;
  ArTime runTime;
  int took;  

  state = getState();
  switch (state) 
  {
  case ArTaskState::SUSPEND:
  case ArTaskState::SUCCESS:
  case ArTaskState::FAILURE:
    // The task isn't running so just return
    return;
  case ArTaskState::INIT:
  case ArTaskState::RESUME:
  case ArTaskState::ACTIVE:
  default:
    break;
  }
  
  runTime.setToNow();
  if (myFunctor != NULL)
    myFunctor->invoke();
  
  if (myNoTimeWarningCB != NULL && !myNoTimeWarningCB->invokeR() && 
      myFunctor != NULL && myWarningTimeCB != NULL &&
      (took = runTime.mSecSince()) > (signed int)myWarningTimeCB->invokeR())
    ArLog::log(ArLog::Normal, 
	       "Warning: Task '%s' took %d ms to run (longer than the %d warning time)",
	       myName.c_str(), took, (signed int)myWarningTimeCB->invokeR());
  
  
  for (it = myMultiMap.rbegin(); it != myMultiMap.rend(); it++)
    (*it).second->run();
}

/**
   This sets a functor which will be called to find the time on the
   task such that if it takes longer than this number of ms to run a
   warning message will be issued, sets this on the children too.
**/
AREXPORT void ArSyncTask::setWarningTimeCB(ArRetFunctor<unsigned int> *functor)
{
  std::multimap<int, ArSyncTask *>::reverse_iterator it;
  myWarningTimeCB = functor;
  for (it = myMultiMap.rbegin(); it != myMultiMap.rend(); it++)
    (*it).second->setWarningTimeCB(functor);
}

/**
   This gets a functor which will be called to find the time on the
   task such that if it takes longer than this number of ms to run a
   warning message will be issued, sets this on the children too.
**/
AREXPORT ArRetFunctor<unsigned int> *ArSyncTask::getWarningTimeCB(void)
{
  return myWarningTimeCB;
}

/**
   This sets a functor which will be called to see if we should warn
   this time through or not.
**/
AREXPORT void ArSyncTask::setNoTimeWarningCB(ArRetFunctor<bool> *functor)
{
  std::multimap<int, ArSyncTask *>::reverse_iterator it;
  myNoTimeWarningCB = functor;
  for (it = myMultiMap.rbegin(); it != myMultiMap.rend(); it++)
    (*it).second->setNoTimeWarningCB(functor);
}

/**
   This sets a functor which will be called to see if we should warn
   this time through or not.
**/
AREXPORT ArRetFunctor<bool> *ArSyncTask::getNoTimeWarningCB(void)
{
  return myNoTimeWarningCB;
}


/**
   Prints the node... the defaulted depth parameter controls how far over to 
   print the data (how many tabs)... it recurses down all its children.
*/
AREXPORT void ArSyncTask::log(int depth)
{
  int i;
  std::multimap<int, ArSyncTask *>::reverse_iterator it;
  std::string str = "";
  ArTaskState::State state;
  
  for (i = 0; i < depth; i++)
    str += "\t";
  str += myName;
  str += " (";
  state = getState();
  switch (state) 
  {
  case ArTaskState::INIT:
    str += "INIT, running)";
    break;
  case ArTaskState::RESUME:
    str += "RESUME, running)";
    break;
  case ArTaskState::ACTIVE:
    str += "ACTIVE, running)";
    break;
  case ArTaskState::SUSPEND:
    str += "SUSPEND, NOT running)";
    break;
  case ArTaskState::SUCCESS:
    str += "SUCCESS, NOT running)";
    break;
  case ArTaskState::FAILURE:
    str += "FAILURE, NOT running)";
    break;
  default:
    str += state;
    str += ", running)";
    break;
  }
  ArLog::log(ArLog::Terse, const_cast<char *>(str.c_str()));
  for (it = myMultiMap.rbegin(); it != myMultiMap.rend(); it++)
    (*it).second->log(depth + 1);
  
}


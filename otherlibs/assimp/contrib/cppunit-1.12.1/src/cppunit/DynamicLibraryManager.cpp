#include <cppunit/plugin/DynamicLibraryManager.h>

#if !defined(CPPUNIT_NO_TESTPLUGIN)
#include <cppunit/plugin/DynamicLibraryManagerException.h>

CPPUNIT_NS_BEGIN


DynamicLibraryManager::DynamicLibraryManager( const std::string &libraryFileName )
    : m_libraryHandle( nullptr )
    , m_libraryName( libraryFileName )
{
  loadLibrary( libraryFileName );
}


DynamicLibraryManager::~DynamicLibraryManager()
{
  releaseLibrary();
}


DynamicLibraryManager::Symbol 
DynamicLibraryManager::findSymbol( const std::string &symbol )
{
  try
  {
    Symbol symbolPointer = doFindSymbol( symbol );
    if ( symbolPointer != nullptr )
      return symbolPointer;
  }
  catch ( ... )
  {
  }

  throw DynamicLibraryManagerException( m_libraryName, 
                                        symbol,
                                        DynamicLibraryManagerException::symbolNotFound );
  return nullptr;    // keep compiler happy
}


void
DynamicLibraryManager::loadLibrary( const std::string &libraryName )
{
  try
  {
    releaseLibrary();
    m_libraryHandle = doLoadLibrary( libraryName );
    if ( m_libraryHandle != nullptr )
      return;
  }
  catch (...)
  {
  }

  throw DynamicLibraryManagerException( m_libraryName,
                                        getLastErrorDetail(),
                                        DynamicLibraryManagerException::loadingFailed );
}


void 
DynamicLibraryManager::releaseLibrary()
{
  if ( m_libraryHandle != nullptr )
  {
    doReleaseLibrary();
    m_libraryHandle = nullptr;
  }
}


CPPUNIT_NS_END


#endif // !defined(CPPUNIT_NO_TESTPLUGIN)

/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page porting_mrpt2 Porting code from MRPT 1.{3,4,5} to MRPT 2.*
*
* MRPT 2.0 includes several fundamental changes, most of them related to API clean ups
* and the introduction of C++14 as the minimum supported version of the language.
*
* Existing user applications may need to be adapted to continue compiling and working
* as usual after updating to MRPT 2.*:
*
* **Mandatory changes**
*  - Your project must use C++14. Using CMake this is easy by adding this right after your top-level `PROJECT`:
*     \code
*     CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
*     set (CMAKE_CXX_STANDARD 14)  # Require C++14
*     \endcode
*  - **Smart pointers** are now standard [`std::shared_ptr<>`](http://en.cppreference.com/w/cpp/memory/shared_ptr) instead of those based on `stlplus`. Required changes:
*     - `ptr.clear()`  --> `ptr.reset()`. Also, notice that the former `stlplus` semantics of `clear()` deleting **all** copies of the object, as hold by different smart pointers, is no longer maintained. There is no longer such a possibility, since the C++11 standard does not allow it to happen (and it makes sense in this way).
*     - `ptr.clear_unique()` --> `ptr.reset()`. (Read this note above)
*     - `ptr.make_unique()` does no longer exists, and does not make sense (read above).
*     - `ptr.pointer()` --> `ptr.get()`
*  - Smart pointers have been renamed from `CFooPtr` to the more standard `CFoo::Ptr`, with a new pointer-to-const version `CFoo::ConstPtr`.
*    - Note: To help with porting and maintaining existing code bases, MRPT 2.* comes by default with a CMake flag `MRPT_1X_BACKCOMPATIB_SMARTPTR_NAMES`, which declares aliases of smart pointers with the old name `CFooPtr`. It is recommended to switch it OFF when writing new code as well as port existing code since this feature will be removed in the future.
*  - You can keep using code like:
*    \code
*    CFoo::Ptr o = CFoo::Create();
*    \endcode
*    in MRPT 2.0 to create a smart pointer, but can also use `std::make_shared<CFoo>()`, or `mrpt::make_aligned_shared<CFoo>()` if the class must be memory-aligned (typically, if it contains Eigen matrices).
*    The arguments of `Create()` are now [perfectly-forwarded](http://en.cppreference.com/w/cpp/utility/forward) to the class ctor, so the parameter list must exactly match any of the available ctors.
*  - Smart pointer typecasting now is done via C++11 standard functions:
*     - Example: Old code
*        \code
*        CObservationPtr obs = getObsFromSomewhere();
*        CObservationGPSPtr gps = CObservationGPS(obs);
*        \endcode
*       becomes in MRPT 2.0:
*        \code
*        CObservation::Ptr obs = getObsFromSomewhere();
*        CObservationGPS::Ptr gps = std::dynamic_pointer_cast<CObservationGPS>(obs);
*        \endcode
*  - Threads, semaphores and mutexes are now based on C++11 standard library. Required changes:
*    - `mrpt::synch::CCriticalSection cs;` --> `std::mutex cs;`
*    - `mrpt::synch::CCriticalSectionLocker lock(&cs);` --> `std::lock_guard<std::mutex> lock(cs);`
*    - `mrpt::system::TThreadHandle h = mrpt::system::createThread(...);` --> `std::thread h = std::thread(...);`
*    - `mrpt::system::sleep(5);` --> `std::this_thread::sleep_for(5ms);`
*    - `mrpt::synch::CSemaphore sem; sem.waitForSignal(timeout); sem.release();` --> `std::promise<void> sem; auto fut = sem.get_future(); fut.wait_for(...); sem.set_value();`
*  - `mrpt::utils::CObject::duplicate()` has been removed, use the equivalent (redundant) `mrpt::utils::CObject::clone()`.
*  - CSerialPort, `mrpt::utils::net`, sockets: have been moved to its own new module \ref mrpt_comms_grp under namespace `mrpt::comms`.
*
* **Optional changes**
*   - Use the `Foo::ConstPtr` smart pointers when possible instead of its non-const counterpart.
*
*
*/

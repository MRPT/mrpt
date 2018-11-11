/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

/** \page porting_mrpt2 Porting code from MRPT 1.{3,4,5} to MRPT 2.*
 *
 * MRPT 2.0 includes several fundamental changes, most of them related to API
 * clean ups and the introduction of C++14 as the minimum supported version of
 * the language.
 *
 * Existing user applications may need to be adapted to continue compiling and
 * working as usual after updating to MRPT 2.*:
 *
 * **Mandatory changes**
 *  - Your project must use C++17. Using CMake this is now done automatically
 *  when linking your targets against MRPT imported targes.
 *  See: \ref mrpt_from_cmake.
 *  - **Smart pointers** are now standard
 * [`std::shared_ptr<>`](http://en.cppreference.com/w/cpp/memory/shared_ptr)
 * instead of those based on `stlplus`. Required changes:
 *     - `ptr.clear()`  --> `ptr.reset()`. Also, notice that the former
 * `stlplus` semantics of `clear()` deleting **all** copies of the object, as
 * hold by different smart pointers, is no longer maintained. There is no longer
 * such a possibility, since the C++11 standard does not allow it to happen (and
 * it makes sense in this way).
 *     - `ptr.clear_unique()` --> `ptr.reset()`. (Read this note above)
 *     - `ptr.make_unique()` does no longer exists, and does not make sense
 * (read above).
 *     - `ptr.pointer()` --> `ptr.get()`
 *  - Smart pointers have been renamed from `CFooPtr` to the more standard
 * `CFoo::Ptr`, with a new pointer-to-const version `CFoo::ConstPtr`.
 *    - Note: To help with porting and maintaining existing code bases, MRPT
 * >=1.5.4 offers MRPT2-like `CFoo::Ptr` smart pointers. Refer to changelog of
 * mrpt 1.5.4.
 *  - You can keep using code like:
 *    \code
 *    CFoo::Ptr o = CFoo::Create();
 *    \endcode
 *    in MRPT 2.0 to create a smart pointer, but can also use
 * `std::make_shared<CFoo>()`, or `mrpt::make_aligned_shared<CFoo>()` if the
 * class must be memory-aligned (typically, if it contains Eigen matrices). The
 * arguments of `Create()` are now
 * [perfectly-forwarded](http://en.cppreference.com/w/cpp/utility/forward) to
 * the class ctor, so the parameter list must exactly match any of the available
 * ctors.
 *  - Smart pointer typecasting now is done via C++11 standard functions:
 *     - Example: Old code
 *        \code
 *        CObservationPtr obs = getObsFromSomewhere();
 *        CObservationGPSPtr gps = CObservationGPS(obs);
 *        \endcode
 *       becomes pure C++14 in MRPT 2.0:
 *        \code
 *        CObservation::Ptr obs = getObsFromSomewhere();
 *        CObservationGPS::Ptr gps =
 * std::dynamic_pointer_cast<CObservationGPS>(obs); \endcode or, if you need to
 * keep your code compatible with MRPT >=1.5.4: \code CObservation::Ptr obs =
 * getObsFromSomewhere(); CObservationGPS::Ptr gps =
 * mrpt::ptr_cast<CObservationGPS>::from(obs); \endcode
 *  - Threads, semaphores and mutexes are now based on C++11 standard library.
 * Required changes:
 *    - `mrpt::synch::CCriticalSection cs;` --> `std::mutex cs;`
 *    - `mrpt::synch::CCriticalSectionLocker lock(&cs);` -->
 * `std::lock_guard<std::mutex> lock(cs);`
 *    - `mrpt::system::TThreadHandle h = mrpt::system::createThread(...);` -->
 * `std::thread h = std::thread(...);`
 *    - `mrpt::system::sleep(5);` --> `std::this_thread::sleep_for(5ms);`
 *    - `mrpt::synch::CSemaphore sem; sem.waitForSignal(timeout);
 * sem.release();` --> `std::promise<void> sem; auto fut = sem.get_future();
 * fut.wait_for(...); sem.set_value();`
 *    - Scheduler functions are now in a new header `<mrpt/system/scheduler.h>`,
 * not in the old `<mrpt/system/threads.h`:
 *      - `mrpt::system::changeCurrentProcessPriority()`
 *      - `mrpt::system::changeCurrentThreadPriority()`
 *  - `mrpt::utils::CObject::duplicate()` has been removed, use the equivalent
 * (redundant) `mrpt::utils::CObject::clone()`.
 *  - CSerialPort, `mrpt::utils::net`, sockets: have been moved to its own new
 * module \ref mrpt_comms_grp under namespace `mrpt::comms`.
 *  - Static variables have been dropped in favor of global getter/setter
 * functions. This allowed removing all DLL import/export macros for Windows
 * compilers. Important changes are:
 *    - `mrpt::math::randomGenerator` --> `mrpt::math::getRandomGenerator()`
 *    - `mrpt::global_settings` old static variables have been replaced by
 * getter/setter functions.
 *  - `ASSERT_*` macros must now be ended with a semicolon, e.g. `ASSERT_(a>0);`
 *  - Serialization: See tutorial of the new module \ref mrpt_serialization_grp
 *    - To serialize an object to/from a CStream, you must now use CArchive:
 *        \code
 *          CStreamXXXX f;  // Any mrpt::io::CStream type
 *          auto arch = mrpt::serialization::archiveFrom(f);
 *          arch << object;
 *          arch >> object;
 *        \endcode
 *    - The two methods `writeToStream()` and `readFromStream()` of old
 * `CSerializable` classes must be replaced by the three methods:
 * `serializeGetVersion()`, `serializeTo()`, and `serializeTo()`. See tutorials
 * in \ref mrpt_serialization_grp.
 *  - Implicit constructor to convert from mrpt::poses::CPose3D to
 * mrpt::math::TPose3D has been removed, due to the refactoring of mrpt::math
 * and mrpt::poses into separate libraries. To convert CPose3D -> TPose3D, use
 * the new method mrpt::poses::CPose3D::asTPose() \code mrpt::poses::CPose3D p1;
 *          mrpt::math::TPose3D p2 = p1;  // ERROR in mrpt 2.0 (built in
 * MRPT 1.*) mrpt::math::TPose3D p3 = p1.asTPose(); // OK for mrpt 2.0 \endcode
 *  - 16-bytes memory-aligned STL containers are now defined in separate
 * headers, one for each container type, and based on templatized `using`.
 * Example: \code
 *       // Old: MRPT 1.* code
 *       #include <mrpt/core/aligned_std_vector.h>
 *       mrpt::aligned_std_vector<Foo>  v;
 *
 *       // New: MRPT 2.* code
 *       #include <mrpt/core/aligned_std_vector.h>
 *       mrpt::aligned_std_vector<Foo> v;
 *       \endcode
 *  - mrpt::system::TTimeStamp is now a C++11-compatible std::chrono clock
 * time_point. All existing backwards-compatible functions to handle dates and
 * timestamps in MRPT remain, but C++11 chrono functions can be now also used
 * instead. mrpt::system::secondsToTimestamp() has been removed since it mixed
 * up a duration with time_point and may be prone to errors.
 *
 *
 * **Optional changes**
 *   - Use the `Foo::ConstPtr` smart pointers when possible instead of its
 * non-const counterpart.
 *
 *
 */

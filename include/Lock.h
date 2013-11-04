/**
 * @file Lock.h
 *
 * Small C++ wrapper classes for pthread mutexes
 *
 * @author Jan Sommer
 *  Created on: Apr 10, 2013
 *
 */

#ifndef LOCK_H
#define LOCK_H

#include <stdexcept>
#include <string.h>
#include <pthread.h>

namespace USU {

/*!
 \brief Wrapper class for pthread mutexes

 \ingroup threading
*/
class Lock
{
private:
    pthread_mutex_t mMutex; /*!< pthread mutex handle */

    Lock(const Lock& arg);/*!< Copy constructor made inaccessible by declaring it private */
    Lock& operator=(const Lock& rhs);/*!< Assignment constructor made inaccessible by declaring it private */

public:

    Lock(); /*!< Constructor: Creates the pthread-mutex */

    virtual ~Lock(); /*!< Destructor: Frees the pthread-mutex */

    void lock(); /*!< Locks the mutex */

    void unlock(); /*!< Unlocks the mutex */
};

inline
Lock::Lock()
{
    int ret;
    if ( (ret = pthread_mutex_init(&mMutex, NULL)) != 0)
    {
        throw std::runtime_error(strerror(errno));
    }
}

inline
Lock::~Lock()
{
    int ret;
    if ( (ret = pthread_mutex_destroy(&mMutex) ) != 0)
    {
        throw std::runtime_error(strerror(errno));
    }
}


inline
void Lock::lock()
{
    pthread_mutex_lock(&mMutex);
}

inline
void Lock::unlock()
{
    pthread_mutex_unlock(&mMutex);
}

///////////////////////////////=============================/////////////////////////

/*!
 \brief Provides a helper class for Scoped Mutexes

 Create this object by passing a reference to a Lock object.
 It will lock the mutex when created and unlock it when destroyed, i.e.
 when going out of scope at the end of the "}".
 Can make it more convenient than manual (un)locking.

 TODO: Test if it works correctly with a getter-method

  \ingroup threading

*/
class ScopedLock
{
private:
    Lock &mLock; /*!< Reference to the mutex which is locked */

    ScopedLock(const ScopedLock& thread); /*!< Copy constructor made inaccessible by declaring it private */
    ScopedLock& operator=(const ScopedLock& rhs); /*!< Assignment constructor made inaccessible by declaring it private */

public:
    /*!
     \brief Constructor: will lock the mutex

     \param lock Reference to the Lock it needs to hold
    */
    ScopedLock(Lock &lock);

    /*!
     \brief Destructor: will unlock the mutex
    */
    virtual ~ScopedLock();
};

inline
ScopedLock::ScopedLock(Lock &lock)
    :mLock(lock)
{
    mLock.lock();
}

inline
ScopedLock::~ScopedLock()
{
    mLock.unlock();
}

}

#endif // LOCK_H

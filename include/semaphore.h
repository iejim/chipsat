/**
 * @file semaphore.h
 *
 * Small wrapper class for semaphore
 *
 * @author Jan Sommer
 *  Created on: Apr 30, 2013
 *
 */

#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include <semaphore.h>
#include <system_error>

namespace USU
{


/*!
 \brief Wrapper class for semaphores

 \ingroup threading

*/
class Semaphore
{
private:

    sem_t mSem;  /*!< Internal semaphore handler */

    Semaphore(const Semaphore& arg);/*!< Copy constructor made inaccessible by declaring it private */
    Semaphore& operator=(const Semaphore& rhs);/*!< Assignment constructor made inaccessible by declaring it private */

public:

    Semaphore(); /*!< Constructor: Creates the pthread-Semaphore */

    virtual ~Semaphore(); /*!< Destructor: Frees the pthread-Semaphore */

    void post(); /*!< Increases the semaphore by 1 */

    /*!
     \brief Trys to get the semaphore, blocking

     Takes the semaphore by decreasing the counter by 1, will wait for the semaphore
     to be given if the counter = 0.

    */
    void wait();

    /*!
     \brief Trys to get the semaphore, non-blocking

     Takes the semaphore by decreasing the counter by 1, will return if the counter = 0.

     \return bool false if semaphore was empty, true if semaphore was successfully acquired
    */
    bool tryWait();
};

Semaphore::Semaphore()
{
    int ret = sem_init(&mSem, 0, 0);
    if(ret != 0)
    {
        throw std::system_error(ret, std::system_category());
    }
}

Semaphore::~Semaphore()
{
    int ret = sem_close(&mSem);
    if(ret != 0)
    {
        throw std::system_error(ret, std::system_category());
    }
}

inline
void Semaphore::post()
{
    sem_post(&mSem);
}

inline
void Semaphore::wait()
{
    sem_wait(&mSem);
}

inline
bool Semaphore::tryWait()
{
    return (!sem_trywait(&mSem) ? true : false);
}

}

#endif // SEMAPHORE_H

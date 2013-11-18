#ifndef _I2CBus_h
#define _I2CBus_h

#include <stdint.h>
#include "exceptions.h"


/*!
 \brief Wrapper class for I2C-bus communication


 \ingroup motor

*/
class I2CBus
{
public:
    /*!
    \brief Constructor

    Sets up the interface to the I2C-bus deviceName

    \param deviceName Name of the I2C-bus device
    */
    I2CBus(const char * deviceName);

    /*!
     \brief Destructor
    */
    ~I2CBus();

    /*!
     \brief Set the address of the I2C device the bus will read and write data to

     \param address 7-bit address (trailing 0)
    */
    void addressSet(uint8_t address);

    /*!
     \brief Write a byte to the register command

     \param command Register to write the byte to
     \param data Byte of data to write to the device set with \ref addressSet()
    */
    void writeByte(uint8_t command, uint8_t data);

    /*!
     \brief Write a byte without a specifying a register

     \param data Byte of data which will be written directly to the device set with \ref addressSet()
    */
    void writeByte(uint8_t data);

    /*!
     \brief Read a byte from the register command

     \param command Register to read from
     \return uint8_t Value of the register command
    */
    uint8_t readByte(uint8_t command);

    /*!
     \brief Read a byte directly without specifying a register

     Read a byte directly from the device set with \ref addressSet() without specifying a register.

     \return uint8_t Value of the read data byte
    */
    uint8_t readByte();

    /*!
     \brief Read a word (2 bytes) from the register command

     \param command Register to read the word from
     \return uint16_t Value of the register command
    */
    uint16_t readWord(uint8_t command);

    /*!
     \brief Read a word (2 bytes) directly without specifying a register

     Read a word (2 bytes) directly from the device set with \ref addressSet() without specifying a register

    \return uint16_t Value of the read data word
    */
    uint16_t readWord();

    /*!
     \brief Tries to read a byte from register command

     Difference to \ref readByte(uint8_t) is, that this function won't check if the reading was successful.
     Returns the value of the register if successful and -1 if the read failed.

     \param command
     \return int
    */
    int tryReadByte(uint8_t command);

    /*!
     \brief Read a block of data from the device starting at register command

     \param command Register to start reading from
     \param size Number of bytes to read
     \param data Allocated buffer with length of at least size
    */
    void readBlock(uint8_t command, uint8_t size, uint8_t * data);

private:
    int fd; /*!< File handler for the I2C device */
};

#endif

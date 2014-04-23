#include "include/i2cdev.h"


/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout) {
    uint8_t b;
    uint8_t count = readByte(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}

/*
void readBit(uint8_t slave_address, uint8_t reg_address, uint8_t bitNum, uint8_t *buf) {
   uint8_t set[] = {reg_address, 0};
   i2c_transmitinit( slave_address, 2, set );
   while(!i2c_transferred());
     
   // Create buffer to store result into:
   int BUF_SIZE = 9;
   uint8_t buffer[BUF_SIZE];
     
   i2c_receiveinit( slave_address, 2, buffer ); //need to receive two bytes, otherwise we get ERROR I2C: Arbitration lost
   while(!i2c_transferred()); /* Wait for transfer */
   // Get queried bit number from buffer and return result:
   &buf = 0 | buffer[bitNum]; // not sure if this is valid... (maybe + ?)
   //&buf = 0b0000000 buffer[bitNum];
*/

/** Read a single bit from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-15)
 * @param data Container for single bit value
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
/*
int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout) {
    uint16_t b;
    uint8_t count = readWord(devAddr, regAddr, &b, timeout);
    *data = b & (1 << bitNum);
    return count;
}
*/

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t count, b;
    if ((count = readByte(devAddr, regAddr, &b, timeout)) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        b &= mask;
        b >>= (bitStart - length + 1);
        *data = b;
    }
    return count;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data, uint16_t timeout)
{
    return readBytes(devAddr, regAddr, 1, data, timeout);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Status of read operation (true = success)
 */
/*
int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data, uint16_t timeout)
{
    return readWords(devAddr, regAddr, 1, data, timeout);
}
*/
/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return Number of bytes read (-1 indicates failure)
 */
int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout) 
{
#ifdef I2CDEV_SERIAL_DEBUG
    Serial.print("I2C (0x");
    Serial.print(devAddr, HEX);
    Serial.print(") reading ");
    Serial.print(length, DEC);
    Serial.print(" bytes from 0x");
    Serial.print(regAddr, HEX);
    Serial.print("...");
#endif

    int8_t count = 0;
    uint32_t t1 = millis();

    // Arduino v1.0.1+, Wire library
    // Adds official support for repeated start condition, yay!

    // I2C/TWI subsystem uses internal buffer that breaks with large data requests
    // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
    // smaller chunks instead of all at once
    for (uint8_t k = 0; k < length; k += min(length, BUFFER_LENGTH)) {
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
        Wire.endTransmission();
        Wire.beginTransmission(devAddr);
        Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));
        
        while (Wire.available() && (timeout == 0 || millis() - t1 < timeout)) {
            data[count++] = Wire.read();
            
#ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[count], HEX);
            if (count < length) {
                Serial.print(" ");
            }                
#endif
        }
        
        Wire.endTransmission();
    }

    // check for timeout
    if (timeout > 0 && millis() - t1 >= timeout && count < length) {
        count = -1; // timeout
    }
    
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif

    return count;
}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    readByte(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return writeByte(devAddr, regAddr, b);
}

/** write a single bit in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-15)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
/*
bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data) {
    uint16_t w;
    readWord(devAddr, regAddr, &w);
    w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
    return writeWord(devAddr, regAddr, w);
}
*/

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
bool writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t b;

    if (readByte(devAddr, regAddr, &b) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        b &= ~(mask); // zero all important bits in existing byte
        b |= data; // combine data with existing byte

        return writeByte(devAddr, regAddr, b);
    }

    return false;
}

/** Write multiple bits in a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-15)
 * @param length Number of bits to write (not more than 16)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
/*
bool I2Cdev::writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data)
{
    //              010 value to write
    // fedcba9876543210 bit numbers
    //    xxx           args: bitStart=12, length=3
    // 0001110000000000 mask byte
    // 1010111110010110 original value (sample)
    // 1010001110010110 original & ~mask
    // 1010101110010110 masked | value
    uint16_t w;

    if (readWord(devAddr, regAddr, &w) != 0) {
        uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
        data <<= (bitStart - length + 1); // shift data into correct position
        data &= mask; // zero all non-important bits in data
        w &= ~(mask); // zero all important bits in existing word
        w |= data; // combine data with existing word
        
        return writeWord(devAddr, regAddr, w);
    } 

    return false;
}
*/
/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    return writeBytes(devAddr, regAddr, 1, &data);
}

/** Write single word to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New word value to write
 * @return Status of operation (true = success)
 */
bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
    return writeWords(devAddr, regAddr, 1, &data);
}

/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
#ifdef I2CDEV_SERIAL_DEBUG
    Serial.print("I2C (0x");
    Serial.print(devAddr, HEX);
    Serial.print(") writing ");
    Serial.print(length, DEC);
    Serial.print(" bytes to 0x");
    Serial.print(regAddr, HEX);
    Serial.print("...");
#endif

    uint8_t status = 0;

    Wire.beginTransmission(devAddr);
    Wire.write((uint8_t) regAddr); // send address

    for (uint8_t i = 0; i < length; i++) {
        Wire.write((uint8_t) data[i]);

#ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(data[i], HEX);
        if (i + 1 < length) Serial.print(" ");
#endif
    }

    status = Wire.endTransmission();

#ifdef I2CDEV_SERIAL_DEBUG
    Serial.println(". Done.");
#endif

#ifdef __SAM3X8E__
    return status > 0;
#else
    return status == 0;
#endif
}

/** Write multiple words to a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of words to write
 * @param data Buffer to copy new data from
 * @return Status of operation (true = success)
 */
bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{
#ifdef I2CDEV_SERIAL_DEBUG
    Serial.print("I2C (0x");
    Serial.print(devAddr, HEX);
    Serial.print(") writing ");
    Serial.print(length, DEC);
    Serial.print(" words to 0x");
    Serial.print(regAddr, HEX);
    Serial.print("...");
#endif

    uint8_t status = 0;

    Wire.beginTransmission(devAddr);
    Wire.write(regAddr); // send address

    for (uint8_t i = 0; i < length * 2; i++) {
        Wire.write((uint8_t)(data[i++] >> 8)); // send MSB
        Wire.write((uint8_t)data[i]);          // send LSB

#ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(data[i], HEX);
        if (i + 1 < length) Serial.print(" ");
#endif
    }

    status = Wire.endTransmission();

#ifdef I2CDEV_SERIAL_DEBUG
    Serial.println(". Done.");
#endif

#ifdef __SAM3X8E__
    return status > 0;
#else
    return status == 0;
#endif
}

/** Default timeout value for read operations.
 * Set this to 0 to disable timeout detection.
 */
uint16_t readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
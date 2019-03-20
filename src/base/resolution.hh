/// MPINHO 2-mar-2019 BEGIN ///
#ifndef __BASE_RESOLUTION_HH__
#define __BASE_RESOLUTION_HH__

#include <inttypes.h>

/**
 * Returns the unsigned resolution, in bits, of an integer value. It is the
 * number of bits required to represent this value (without precision loss),
 * which can be recovered with zero bit extension.
 * The unsigned resolution is measured as the position of the most significant
 * 1 bit. It returns zero for val == 0.
 *
 * @param val: up to 64-bit integer value
 *
 * @return resolution in bits (0 to 64 unsigned)
 * @todo this function is exactly the same as base/bitfield.hh findMsbSet
 */
int unsignedIntResolution(uint64_t val);

/**
 * Returns the signed resolution, in bits, of an integer value. It ist the
 * number of bits required to represent that value (without precision loss),
 * which can be recovered through sign bit extension.
 * The signed resolution is measured as the position (starting in 1) of the
 * least significant leading 0 or 1 bit.
 *
 * @param val 64-bit integer value
 *
 * @return resolution in bits (1 to 64, unsigned)
 */
int signedIntResolution(uint64_t val);

/**
 * Returns the bit blocks needed to represent an integer value.  For a given
 * block size, calculates the minimal block count that keeps the resolution,
 * according to the signed int resolution definition in
 * @ref signedIntResolution.
 *
 * @param val 64-bit integer value
 * @param block size of the blocks in bits
 *
 * @return count of needed blocks
 */
int blockSIntResolution(uint64_t val, uint8_t block);

/**
 * Returns the base 2 log of the blocks needed for an integer value.  For a
 * given a block size, calculates the power of two that is enough to represent
 * that value keeping the resolution according to the signed int resolution
 * definition in @ref signedIntResolution.
 *
 * @param val 64-bit integer value
 * @param block size of the blocks in bits
 *
 * @return smallest power of two of blocks needed
 */
int logSIntResolution(uint64_t val, uint8_t block);

#endif // __BASE_RESOLUTION_HH__
/// MPINHO 2-mar-2019 END ///

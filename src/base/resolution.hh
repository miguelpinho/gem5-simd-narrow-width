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
 * Rounds up the precision value to the nearest multiple of a given block size.
 *
 * @param prc precision value
 * @param block size of the blocks in bit, which must be a power of two
 *
 * @return smallest multiple of block where the precision val fits
 */
int roundPrcBlock(int prc, int block);

/**
 * Rounds up the precision value to the nearest power of two starting in the
 * block size.
 *
 * @param prc precision value
 * @param block size of the blocks in bit, which must be a power of two
 *
 * @return smallest power of two of block where the precision val fits
 */
int roundPrcBlockLog(int prc, int block);

#endif // __BASE_RESOLUTION_HH__
/// MPINHO 2-mar-2019 END ///

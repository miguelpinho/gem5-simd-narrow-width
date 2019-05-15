/// MPINHO 2-mar-2019 BEGIN ///

#include "base/resolution.hh"
#include "arch/utility.hh"

int
unsignedIntResolution(uint64_t val)
{
    if (val == 0) return 0;

    int prc = 1; // in findMsbSet they don't have this

    if (val >> 32) { val += 32; val = val >> 32; }
    if (val >> 16) { prc += 16; val = val >> 16; }
    if (val >> 8) { prc += 8; val = val >> 8; }
    if (val >> 4) { prc += 4; val = val >> 4; }
    if (val >> 2) { prc += 2; val = val >> 2; }
    if (val >> 1) { prc++; }

    return prc;
}

int
signedIntResolution(uint64_t val)
{
    // the precision of a value is equal to its complement's
    if (val >> 63) val = ~val;

    int prc = 1;

    // fast result
    if (!val) return prc;

    // find the most significative 1 bit and add one
    prc++;
    if (val >> 32) { prc += 32; val = val >> 32; }
    if (val >> 16) { prc += 16; val = val >> 16; }
    if (val >> 8) { prc += 8; val = val >> 8; }
    if (val >> 4) { prc += 4; val = val >> 4; }
    if (val >> 2) { prc += 2; val = val >> 2; }
    if (val >> 1) { prc++; }

    return prc;
}

/// MPINHO 15-may-2019 BEGIN ///
int
roundPrcBlock(int prc, int block)
{
    assert(prc >= 0 && block >= 0);

    // Source: https://stackoverflow.com/questions/3407012/
    // Make sure block is a power of 2.
    assert(block && ((block & (block-1)) == 0));

    return (prc + block - 1) & -block;
}

// TODO: implement roundPrcBlockLog function

/// MPINHO 15-may-2019 END ///

/// MPINHO 2-mar-2019 END ///

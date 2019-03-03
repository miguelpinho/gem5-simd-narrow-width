/// MPINHO 2-mar-2019 BEGIN ///

#include "base/resolution.hh"

uint8_t
unsignedIntResolution(uint64_t val)
{
    if (val == 0) return 0;

    unsigned prc = 1;
    uint64_t aux = val;

    if (aux >> 32) { prc += 32; aux = aux >> 32; }
    if (aux >> 16) { prc += 16; aux = aux >> 16; }
    if (aux >> 8) { prc += 8; aux = aux >> 8; }
    if (aux >> 4) { prc += 4; aux = aux >> 4; }
    if (aux >> 2) { prc += 2; aux = aux >> 2; }
    if (aux >> 1) { prc++; }

    return prc;
}

uint8_t
signedIntResolution(uint64_t val)
{
    uint64_t aux = val;

    // the precision of a value is equal to its complement's
    if (aux >> 63) aux = ~aux;

    unsigned prc = 1;

    // fast result
    if (!aux) return prc;

    // find the most significative 1 bit and add one
    prc++;
    if (aux >> 32) { prc += 32; aux = aux >> 32; }
    if (aux >> 16) { prc += 16; aux = aux >> 16; }
    if (aux >> 8) { prc += 8; aux = aux >> 8; }
    if (aux >> 4) { prc += 4; aux = aux >> 4; }
    if (aux >> 2) { prc += 2; aux = aux >> 2; }
    if (aux >> 1) { prc++; }

    return prc;
}

uint8_t
blockSIntResolution(uint64_t val, uint8_t block)
{
    unsigned prc = signedIntResolution(val);

    return (prc + block - 1) / block;
}

uint8_t
logSIntResolution(uint64_t val, uint8_t block)
{
    unsigned aux = blockSIntResolution(val, block);

    unsigned log = 0;

    while (aux > (1 << log)) log++;

    return log + 1;
}

/// MPINHO 2-mar-2019 END ///

/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_PACKING_CRITERIA_HH__
#define __CPU_O3_PACKING_CRITERIA_HH__

#include <algorithm>
#include <functional>

#include "arch/utility.hh"
#include "cpu/o3/width_code.hh"

bool simplePacking(VecWidthCode mask1, VecWidthCode mask2) {
    // Simple: each lane must have enough space seperately.
    assert(mask1.vectorSize() == mask2.vectorSize());

    // If the width of the instructions does not match, packing fails.
    if (mask1.elemBits() != mask2.elemBits())
        return false;

    // Evaluate lane by lane. If any lane has width overflow, packing fails.
    int eBits = mask1.elemBits();
    // Prepared for the case where an instruction uses only a portion of the
    // lane. This happens in ARM Neon for example.
    int nElem = std::min(mask1.numElem(), mask2.numElem());
    for (int i = 0; i < nElem; i++) {
        if (mask1.get(i) + mask2.get(i) > eBits) {
            return false;
        }
    }

    return true;
}

bool optimalPacking(VecWidthCode mask1, VecWidthCode mask2) {
    // Optimal: count number of set bits in mask.
    assert(mask1.vectorSize() == mask2.vectorSize());

    // Try to pack as much as possible, even if unfeasible.
    return (mask1.count() + mask2.count()) <= VecWidthCode::vectorSize();
}

#endif // __CPU_O3_PACKING_CRITERIA_HH__

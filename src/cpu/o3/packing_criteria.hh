/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_PACKING_CRITERIA_HH__
#define __CPU_O3_PACKING_CRITERIA_HH__

#include <algorithm>
#include <functional>

#include "arch/utility.hh"
#include "cpu/o3/width_code.hh"

using PackingCriteria = std::function<bool(VecWidthCode, VecWidthCode)>;

bool simplePacking(VecWidthCode mask1, VecWidthCode mask2);
bool optimalPacking(VecWidthCode mask1, VecWidthCode mask2);

#endif // __CPU_O3_PACKING_CRITERIA_HH__

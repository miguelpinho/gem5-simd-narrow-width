/// MPINHO 23-jul-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_INFO_HH_
#define __CPU_O3_WIDTH_INFO_HH_

#include <string>
#include <utility>

#include "arch/utility.hh"
#include "cpu/o3/packing_criteria.hh"
#include "cpu/o3/width_code.hh"
#include "enums/WidthClass.hh"

/*
 * Type for the width information associated with an instruction.
 *
 * Used to verify the width used by the instruction and to check if two
 * instructions can be fused.
 */
class WidthInfo {
    protected:
        WidthClass width_class;
        VecWidthCode width_mask;

    public:
        WidthInfo();
        WidthInfo(WidthClass _width_class);
        WidthInfo(WidthClass _width_class, VecWidthCode _width_mask);

        WidthClass getWidthClass();
        bool isFuseType();
        bool canFuse(WidthInfo &b, PackingCriteria packingCriteria);
};

#endif // __CPU_O3_WIDTH_INFO_HH_
/// MPINHO 23-jul-2019 END ///

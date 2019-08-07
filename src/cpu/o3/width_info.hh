/// MPINHO 23-jul-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_INFO_HH_
#define __CPU_O3_WIDTH_INFO_HH_

#include <cstdint>
#include <string>
#include <utility>

#include "arch/utility.hh"
#include "cpu/o3/packing_criteria.hh"
#include "cpu/o3/width_code.hh"
#include "enums/VecElemSize.hh"
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
        VecElemSize elem_size;

    public:
        WidthInfo();
        WidthInfo(WidthClass _width_class);
        WidthInfo(WidthClass _width_class, VecWidthCode _width_mask,
                  uint8_t _size);

        WidthClass getWidthClass() { return width_class; }
        VecElemSize getElemSize() { return elem_size; }
        bool isFuseType();
        bool canFuse(WidthInfo &b, PackingCriteria packingCriteria);
        std::string to_string();
};

#endif // __CPU_O3_WIDTH_INFO_HH_
/// MPINHO 23-jul-2019 END ///

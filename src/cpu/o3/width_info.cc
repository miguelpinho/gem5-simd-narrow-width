/// MPINHO 23-jul-2019 BEGIN ///

#include "cpu/o3/width_info.hh"

WidthInfo::WidthInfo(WidthClass _width_class)
    : width_class(_width_class)
{}

WidthInfo::WidthInfo(WidthClass _width_class,
                     VecWidthCode _width_mask)
    : width_class(_width_class), width_mask(_width_mask)
{}

bool
WidthInfo::isFuseType()
{
    if (width_class == WidthClass::SimdPackingAdd) return true;
    if (width_class == WidthClass::SimdPackingMult) return true;

    return false;
}

bool
WidthInfo::canFuse(WidthInfo &b, PackingCriteria packingCriteria)
{
    if (width_class == WidthClass::NoInfo ||
        width_class == WidthClass::SimdNoPacking ||
        b.width_class == WidthClass::NoInfo ||
        b.width_class == WidthClass::SimdNoPacking) {
        return false;
    }

    if (width_class != b.width_class) {
        return false;
    }

    return packingCriteria(width_mask, b.width_mask);
}

/// MPINHO 23-jul-2019 END ///

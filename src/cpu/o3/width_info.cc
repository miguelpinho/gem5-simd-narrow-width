/// MPINHO 23-jul-2019 BEGIN ///

#include "cpu/o3/width_info.hh"

WidthInfo::WidthInfo()
    : width_class(WidthClass::NoInfo),
      elem_size(VecElemSize::Unknown)
{}

WidthInfo::WidthInfo(WidthClass _width_class)
    : width_class(_width_class),
      elem_size(VecElemSize::Unknown)
{}

WidthInfo::WidthInfo(WidthClass _width_class,
                     VecWidthCode _width_mask,
                     uint8_t _size)
    : width_class(_width_class), width_mask(_width_mask)
{
    switch (_size) {
        case 0:
            elem_size = VecElemSize::Bit8;
            break;

        case 1:
            elem_size = VecElemSize::Bit16;
            break;

        case 2:
            elem_size = VecElemSize::Bit32;
            break;

        case 3:
            elem_size = VecElemSize::Bit64;
            break;

        default:
            panic("Invalid vector elem size: %d.", _size);
            break;
    }
}

bool
WidthInfo::isFuseType()
{
    if (width_class == WidthClass::SimdPackingAdd) return true;
    if (width_class == WidthClass::SimdPackingMult) return true;

    return false;
}

bool
WidthInfo::matchType(WidthInfo &b)
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

    return true;
}

bool
WidthInfo::canFuse(WidthInfo &b, PackingCriteria packingCriteria)
{
    if (!matchType(b))
        return false;

    return packingCriteria(width_mask, b.width_mask);
}

std::string
WidthInfo::to_string()
{
    std::stringstream ss;

    ss << WidthClassStrings[static_cast<int>(width_class)];

    if (width_class != WidthClass::NoInfo) {
        ss << "::";
        ss << width_mask.to_string();
    }

    return ss.str();
}

/// MPINHO 23-jul-2019 END ///

/// MPINHO 23-jul-2019 BEGIN ///

#include "cpu/o3/width_info.hh"

WidthInfo::WidthInfo(WidthClass _width_class)
    : width_class(_width_class)
{}

WidthInfo::WidthInfo(WidthClass _width_class,
                     VecWidthCode _width_mask)
    : width_class(_width_class), width_mask(_width_mask)
{}

/// MPINHO 23-jul-2019 END ///
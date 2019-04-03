
#include "cpu/o3/width_board.hh"

#include "config/the_isa.hh"

PrecisionBoard::PrecisionBoard(const std::string &_my_name,
                               unsigned _numPhysicalVecRegs)
    : _name(_my_name),
      vecRegBoard(_numPhysicalVecRegs),
      numPhysicalVecRegs(_numPhysicalVecRegs)
{
}
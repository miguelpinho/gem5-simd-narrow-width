#ifndef __CPU_O3_WIDTH_BOARD_HH__
#define __CPU_O3_WIDTH_BOARD_HH__

#include <iostream>
#include <utility>
#include <vector>

#include "base/trace.hh"
#include "config/the_isa.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/width_code.hh"
#include "debug/WidthBoard.hh"

/**
 * Implements a auxiliary structure to store the width of the
 * physical registers of a certain class. The register indexing is
 * relative, as the register class is implied by the instance.
 *
 * @todo make template, to allow different types of definitions of
 * width and/or types to be supplied; handle width miss for vector
 * registers; by value to by reference?
 */
class PrecisionBoard
{
  private:
    const std::string _name;

    /** Vector registers. */
    std::vector<PrecStruct> vecRegBoard;

    /**
     * Number of physical vector registers
     */
    unsigned numPhysicalVecRegs;

  public:
    PrecisionBoard(const std::string &_my_name,
                   unsigned _numPhysicalRegs);

    ~PrecisionBoard() {}

    std::string name() const { return _name; };

    VecWidthCode getWidthVecReg(PhysRegIdPtr phys_reg) const
    {
        assert(phys_reg->isVectorPhysReg());

        DPRINTF(WidthBoard, "Getting width of reg %i (%s)\n",
                phys_reg->index(), phys_reg->className());


        if (regPrecBoard[phys_reg->flatIndex()].isNone()) {
            DPRINTF(PrecBoard, "Reg %i (%s) had no assinged width\n",
                    phys_reg->index(), phys_reg->className());
        }

        return vecRegBoard[phys_reg->index()];
    }

    void setWidthVecReg(PhysRegIdPtr phys_reg, VecWidthCode) {
        assert(phys_reg->flatIndex() < numPhysRegs);

        DPRINTF(PrecBoard, "Setting precision of reg %i (%s)\n",
                phys_reg->index(), phys_reg->className());

        // zero reg can never change precision
        if (phys_reg->isZeroReg()) {
            DPRINTF(PrecBoard, "Reg %i (%s) is a zero register\n",
                    phys_reg->index(), phys_reg->className());

            return;
        }

        regPrecBoard[phys_reg->flatIndex()].setByVal(val);
    }

    void clearWidthVecReg(PhysRegIdPtr phys_reg) {
        assert(phys_reg->flatIndex() < numPhysRegs);

        DPRINTF(PrecBoard, "Clearing precision of reg %i (%s)\n",
                phys_reg->index(), phys_reg->className());

        // zero reg always has the same precision
        if (phys_reg->isZeroReg()) {
            DPRINTF(PrecBoard, "Reg %i (%s) is a zero register\n",
                    phys_reg->index(), phys_reg->className());
            return;
        }

        regPrecBoard[phys_reg->flatIndex()].clear();
    }

};

#endif // __CPU_O3_WIDTH_BOARD_HH__

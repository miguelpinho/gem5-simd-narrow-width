/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_DECODER_HH__
#define __CPU_O3_WIDTH_DECODER_HH__

#include <algorithm>
#include <array>
#include <iostream>
#include <utility>
#include <vector>

#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/o3/width_code.hh"
#include "cpu/op_class.hh"
#include "debug/WidthDecoder.hh"
#include "enums/PackingClass.hh"
#include "enums/WidthDefinition.hh"
#include "enums/WidthPackingPolicy.hh"

struct DerivO3CPUParams;

template <class Impl>
class InstructionQueue;

/**
 * cpu structure for evaluate that evaluates the width required by an
 * operation.
 *
 * @todo better name; make template, to allow reimplementation for different
 * architectures; create width mask type (class).
 */
template <class Impl>
class WidthDecoder
{
  public:
    // Typedefs from the Impl.
    // typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;

    // // Register types.
    // using VecRegContainer = TheISA::VecRegContainer;
    // using VecElem = TheISA::VecElem;
    // static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;

    // TODO: Specific for ARMv8. Generalize for other architectures.
    static const size_t SizeVecRegister = 128;

    /** Empty constructor. */
    WidthDecoder();

    /** Constructs a width decoder with given parameters. */
    WidthDecoder(DerivO3CPUParams *params);

    /** Destructs the width decoder. */
    ~WidthDecoder();

    /** Returns the name of the width decoder. */
    std::string name() const { return _name; };

    /** Initializes with decoder with parameters. */
    void init(DerivO3CPUParams *params);

    /** Registers statistics. */
    void regStats();

    /** Sets the pointer to the IQ. */
    void setIQ(InstructionQueue<Impl> *iq_ptr);

    /** Return width maks of a vector instruction. */
    VecWidthCode vecInstWidthMask(DynInstPtr &inst);

    /** Return width maks of one of the vector source registers. */
    VecWidthCode
    vecSrcRegWidthMask(DynInstPtr &inst, int src, unsigned eSize,
                       unsigned nElem);

    /** Returns true if vector instruction can be fused. */
    bool canFuseVecInst(DynInstPtr &inst1, DynInstPtr &inst2);

    /** Returns true if vector instruction is of type that can be fused. */
    bool isFuseVecType(DynInstPtr &inst);

  protected:
    std::string _name;

    /** Pointer to the Instruction Queue. */
    InstructionQueue<Impl> *iqPtr;

    /// MPINHO 08-may-2019 BEGIN ///
    ///////////////////
    // Parameters
    ///////////////////
    /** Definition for operand and operation width/resolution. */
    WidthDefinition widthDef;

    /** Block size for width/resolution considerations. */
    unsigned blockSize;

    /** Policy for packing operations. */
    WidthPackingPolicy packingPolicy;
    /// MPINHO 08-may-2019 END ///

    /// MPINHO 13-may-2019 BEGIN ///
    /** Packing class of each operation. */
    std::array<PackingClass, Enums::Num_OpClass> packingClassMap;

    /** Create packing class map */
    void initPackingClass();

    /////////////////////////
    // Packing Criteria
    /////////////////////////
    /** Optimal packing criteria. */
    bool optimalPacking(VecWidthCode mask1, VecWidthCode mask2);

    /// MPINHO 13-may-2019 END ///
};

#endif // __CPU_O3_WIDTH_DECODER_BOARD_HH__
/// MPINHO 12-mar-2019 END ///

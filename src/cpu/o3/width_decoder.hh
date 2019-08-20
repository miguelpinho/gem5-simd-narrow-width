/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_DECODER_HH__
#define __CPU_O3_WIDTH_DECODER_HH__

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "arch/types.hh"
#include "base/statistics.hh"
#include "base/types.hh"
#include "cpu/func_unit_width.hh"
#include "cpu/o3/packing_criteria.hh"
#include "cpu/o3/width_code.hh"
#include "cpu/o3/width_info.hh"
#include "cpu/op_class.hh"
#include "debug/WidthDecoder.hh"
#include "enums/WidthClass.hh"
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
    typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;

    // Typedefs for the ISA.
    typedef TheISA::ExtMachInst ExtMachInst;

    // // Register types.
    // using VecRegContainer = TheISA::VecRegContainer;
    // using VecElem = TheISA::VecElem;
    // static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;

    // TODO: Specific for ARMv8. Generalize for other architectures.
    static const size_t SizeVecRegister = 128;

    /** Empty constructor. */
    WidthDecoder();

    /** Constructs a width decoder with given parameters. */
    WidthDecoder(O3CPU *cpu_ptr, DerivO3CPUParams *params);

    /** Destructs the width decoder. */
    ~WidthDecoder();

    /** Returns the name of the width decoder. */
    std::string name() const { return _name; };

    /** Initializes with decoder with parameters. */
    void init(DerivO3CPUParams *params);

    /** Registers statistics. */
    void regStats();

    /** Sets the pointer to the CPU. */
    void setCPU(O3CPU *cpu_ptr);

    /** Sets the pointer to the IQ. */
    void setIQ(InstructionQueue<Impl> *iq_ptr);

    /** Return width mask of a vector instruction. */
    VecWidthCode vecInstWidthMask(const DynInstPtr &inst);

    /** Return width maks of one of the vector source registers. */
    VecWidthCode
    vecSrcRegWidthMask(const DynInstPtr &inst, uint8_t q, uint8_t size,
                       uint8_t op);

    /** Returns the width information for a given instruction. */
    void addWidthInfo(const DynInstPtr &inst);

    /** Returns true if vector instruction is of type that can be fused. */
    bool isFuseType(const DynInstPtr &inst);

    /** Returns true if the two instructions are of compatible fuse types. */
    bool matchFuseType(const DynInstPtr &inst1, const DynInstPtr &inst2);

    /** Returns true if the two instructions are compatible for fuse. */
    bool canFuseInst(const DynInstPtr &inst1, const DynInstPtr &inst2);

    VecWidthCode widthOp1VectorRegl(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1);
    VecWidthCode widthOp1VectorAcross(const DynInstPtr &inst,
                                      uint8_t q, uint8_t size,
                                      uint8_t op1);
    VecWidthCode widthOp2VectorRegl(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2);
    VecWidthCode widthOp2VectorPair(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2);
    VecWidthCode widthOpAcrossVector(const DynInstPtr &inst,
                                     uint8_t q, uint8_t size,
                                     uint8_t op);

  protected:
    std::string _name;

    /** Pointer to the CPU. */
    O3CPU *cpu;

    /** Pointer to the Instruction Queue. */
    InstructionQueue<Impl> *iqPtr;

    template <int Size, typename Elem>
    VecWidthCode
    getWidthVecReg(const DynInstPtr &inst, int nElem, int nBits,
                   uint8_t op);

    /// MPINHO 08-may-2019 BEGIN ///
    ///////////////////
    // Parameters
    ///////////////////
    /** Definition for operand and operation width/resolution. */
    WidthDefinition widthDef;
    std::function<int(uint64_t)> prcFunc;

    /** Block size for width/resolution considerations. */
    unsigned blockSize;
    std::function<int(int, int)> roundFunc;
    std::function<int(uint64_t)> roundedPrcFunc;

    /** Policy for packing operations. */
    WidthPackingPolicy packingPolicy;
    /// MPINHO 08-may-2019 END ///
    /// MPINHO 13-may-2019 BEGIN ///

    /////////////////////////
    // Packing Criteria
    /////////////////////////
    /** Chosen packing condition. */
    PackingCriteria packingCriteria;
    /// MPINHO 13-may-2019 END ///

    /** Decode instruction width. */
    WidthInfo decode(const DynInstPtr &inst);
    /** Decode Neon 3Same instruction width. */
    WidthInfo decodeNeon3Same(const DynInstPtr &inst);
    /** Decode Neon 2RegMisc instruction width. */
    WidthInfo decodeNeon2RegMisc(const DynInstPtr &inst);
    /** Decode Neon AcrossLanes instruction width. */
    WidthInfo decodeNeonAcrossLanes(const DynInstPtr &inst);
};

#endif // __CPU_O3_WIDTH_DECODER_BOARD_HH__
/// MPINHO 12-mar-2019 END ///

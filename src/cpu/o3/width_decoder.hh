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
#include "enums/VecElemSize.hh"
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

    /** Return width maks of one of the widened vector source registers. */
    VecWidthCode
    vecSrcRegWidthMaskWide(const DynInstPtr &inst, uint8_t q, uint8_t size,
                           uint8_t op);

    /** Return width maks of one of the indexed vector source registers. */
    VecWidthCode
    vecSrcRegWidthMaskIndex(const DynInstPtr &inst, uint8_t size,
                            uint8_t op, uint8_t idx);

    /** Return width maks of one of the indexed vector source registers. */
    VecWidthCode
    vecSrcRegWidthMaskBroadcast(const DynInstPtr &inst, uint8_t q,
                                uint8_t size, uint8_t op,
                                uint8_t idx);

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
    VecWidthCode widthOp1VectorLong(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1);
    VecWidthCode widthOp1VectorPairLong(const DynInstPtr &inst,
                                        uint8_t q, uint8_t size,
                                        uint8_t op1);
    VecWidthCode widthOp1VectorAcross(const DynInstPtr &inst,
                                      uint8_t q, uint8_t size,
                                      uint8_t op1);
    VecWidthCode widthOp1VectorIndex(const DynInstPtr &inst,
                                     uint8_t size,
                                     uint8_t op1, uint8_t idx);
    VecWidthCode widthOp1VectorBroadcast(const DynInstPtr &inst,
                                         uint8_t q, uint8_t size,
                                         uint8_t op1, uint8_t idx);
    VecWidthCode widthOp2VectorRegl(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2);
    VecWidthCode widthOp2VectorPair(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2);
    VecWidthCode widthOp2VectorMix(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2,
                                    int idx, int stride);
    VecWidthCode widthOp2VectorJoin(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2,
                                    bool lower);
    VecWidthCode widthOp2VectorLong(const DynInstPtr &inst,
                                    uint8_t q, uint8_t size,
                                    uint8_t op1, uint8_t op2);
    VecWidthCode widthOp2VectorWide(const DynInstPtr &inst,
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
    getWidthVecReg(const DynInstPtr &inst, int nElem,
                   int nBits, uint8_t op);
    template <int Size, typename Elem>
    VecWidthCode
    getWidthVecRegIndex(const DynInstPtr &inst,
                        int nBits, uint8_t op, uint8_t idx);
    template <int Size, typename Elem>
    VecWidthCode
    getWidthVecRegBroadcast(const DynInstPtr &inst, int nElem,
                            int nBits, uint8_t op, uint8_t idx);
    template <int Size, typename Elem>
    VecWidthCode
    getWidthVecRegWiden(const DynInstPtr &inst, int nElem, int nBits,
                        uint8_t op, bool low);

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
    /** Decode Neon 3Same instruction width. */
    WidthInfo decodeNeon3Diff(const DynInstPtr &inst);
    /** Decode Neon 2RegMisc instruction width. */
    WidthInfo decodeNeon2RegMisc(const DynInstPtr &inst);
    /** Decode Neon AcrossLanes instruction width. */
    WidthInfo decodeNeonAcrossLanes(const DynInstPtr &inst);
    /** Decode Neon ShiftByImm instruction width. */
    WidthInfo decodeNeonShiftByImm(const DynInstPtr &inst);
    /** Decode Neon Copy instruction width. */
    WidthInfo decodeNeonCopy(const DynInstPtr &inst);
    /** Decode Neon Ext instruction width. */
    WidthInfo decodeNeonExt(const DynInstPtr &inst);
    /** Decode Neon ZipUzpTrn instruction width. */
    WidthInfo decodeNeonZipUzpTrn(const DynInstPtr &inst);
    /** Decode Neon TblTbx instruction width. */
    WidthInfo decodeNeonTblTbx(const DynInstPtr &inst);

    /////////////////////////
    // Consts
    /////////////////////////
    static constexpr int Num_VecElemSize =
        static_cast<int>(VecElemSize::Num_VecElemSize);
    static constexpr int Num_WidthClass =
        static_cast<int>(WidthClass::Num_WidthClass);
    static const std::array<const int, Num_VecElemSize> Bits_VecElemSize;
    static const std::array<const VecElemSize, 4> SizeToVecElemSize;

    /////////////////////////
    // Stats
    /////////////////////////
    /** Stat for width of vector operand elements, by vector element size. */
    std::array<Stats::Distribution, Num_VecElemSize>
      statVectorOpElemWidthBySize;
    /** Stat for total vector operand width, by vector element size. */
    std::array<Stats::Distribution, Num_VecElemSize>
      statVectorOpTotalWidthBySize;
    /** Stat for width of vector inst elements, by vector element size. */
    std::array<Stats::Distribution, Num_VecElemSize>
      statVectorInstElemWidthBySize;
    /** Stat for total vector inst width, by vector element size. */
    std::array<Stats::Distribution, Num_VecElemSize>
      statVectorInstTotalWidthBySize;

    /** Stat for total vector inst width, by width class. */
    Stats::VectorDistribution statVectorInstTotalWidthByClass;

    /** Sample width distribution for vector operands. */
    void sampleVecOp(VecWidthCode &mask, uint8_t size);
    /** Sample width distribution for vector insts. */
    void sampleVecInst(VecWidthCode &mask, uint8_t size);
};

template<class Impl>
const std::array<const int, WidthDecoder<Impl>::Num_VecElemSize>
  WidthDecoder<Impl>::Bits_VecElemSize = {0, 8, 16, 32, 64};

template<class Impl>
const std::array<const VecElemSize, 4>
  WidthDecoder<Impl>::SizeToVecElemSize = {
    VecElemSize::Bit8,
    VecElemSize::Bit16,
    VecElemSize::Bit32,
    VecElemSize::Bit64
  };
#endif // __CPU_O3_WIDTH_DECODER_BOARD_HH__
/// MPINHO 12-mar-2019 END ///

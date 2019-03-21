/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_DECODER_HH__
#define __CPU_O3_WIDTH_DECODER_HH__

#include <algorithm>
#include <bitset>
#include <iostream>
#include <utility>
#include <vector>

// #include "arch/isa_traits.hh"
// #include "config/the_isa.hh"
#include "base/statistics.hh"
#include "base/types.hh"

// #include "cpu/o3/isa_specific.hh"
#include "cpu/op_class.hh"
#include "debug/WidthDecoder.hh"

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
  protected:
    std::string _name;

  public:
    // Typedefs from the Impl.
    // typedef typename Impl::O3CPU O3CPU;
    typedef typename Impl::DynInstPtr DynInstPtr;

    // // Register types.
    // using VecRegContainer = TheISA::VecRegContainer;
    // using VecElem = TheISA::VecElem;
    // static constexpr auto NumVecElemPerVecReg = TheISA::NumVecElemPerVecReg;

    // Number of bits per resolution flag.
    // TODO: change to be defined as parameter?
    static const size_t ResolGranularity = 3; // 8-bit granularity

    // TODO: Specific for ARMv8. Generalize for other architectures.
    static const size_t SizeVecRegister = 128;

    // Number of bits of the resolution masks
    static constexpr size_t NumVecResolBits = (128) >> ResolGranularity;

    /* TODO: Class for vector width information encoding. */
    // class VecWidthCode
    // {
    //   private:
    //     int eBits;
    //     int nElem;
    //     int codeResol;
    //     std::vector<int> code;

    //   public:
    //     VecWidthCode(int _eSize)
    //     {
    //       assert(_eSize >= 0 && _eSize <= 3);

    //       eBits = 1 << (3 + _eSize);
    //       nElem = SizeVecRegister >> (3 + _eSize);
    //       codeResol = eBits >> ResolGranularity;
    //       code = std::vector<int>(nElem);
    //     }

    //     VecWidthCode(VecWidthCode &a, VecWidthCode &b)
    //     {
    //       assert(a->eBits() == b->eBits());

    //       eBits = a->eBits;
    //       nElem = a->nElem;
    //       codeResol = a->codeResol;
    //       code = std::vector<int>(nElem);

    //       for (int i = 0; i <= nElem; i++) {
    //         code[i] = std::max(a->code[i], b->code[i]);
    //       }
    //     }

    //     ~VecWidthCode() {}

    //     void
    //     set(int pos, int val) {
    //       assert(pos >= 0 && pos <= nElem);

    //       code[pos] = val;
    //     }

    //     void
    //     get(int pos) {
    //       assert(pos >= 0 && pos <= nElem);

    //       return (code[pos]);
    //     }
    // };

    /** Empty constructor. */
    WidthDecoder();

    /** Constructs a width decoder with given parameters. */
    // WidthDecoder(DerivO3CPUParams *params);

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
    std::bitset<NumVecResolBits> vecInstWidthMask(DynInstPtr &inst);

    /** Return width maks of one of the vector source registers. */
    std::bitset<NumVecResolBits>
    vecSrcRegWidthMask(DynInstPtr &inst, int src, unsigned eSize,
                       unsigned nElem);

    /** Returns true if vector instruction can be fused. */
    bool canFuseVecInst(DynInstPtr &inst1, DynInstPtr &inst2);

    /** Returns true if vector instruction is of type that can be fused. */
    bool isFuseVecType(DynInstPtr &inst);

  private:
    /** Pointer to the Instruction Queue. */
    InstructionQueue<Impl> *iqPtr;

};

#endif // __CPU_O3_WIDTH_DECODER_BOARD_HH__
/// MPINHO 12-mar-2019 END ///

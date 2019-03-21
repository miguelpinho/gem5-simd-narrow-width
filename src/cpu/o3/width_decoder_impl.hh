/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_DECODER_IMPL_HH__
#define __CPU_O3_WIDTH_DECODER_IMPL_HH__

#include "arch/generic/vec_reg.hh"
#include "base/logging.hh"
#include "base/resolution.hh"
#include "base/trace.hh"
#include "cpu/o3/inst_queue.hh"
#include "cpu/o3/width_decoder.hh"
#include "debug/WidthDecoder.hh"
#include "enums/OpClass.hh"
#include "params/DerivO3CPU.hh"

template <class Impl>
WidthDecoder<Impl>::WidthDecoder()
    : iqPtr(NULL)
{
}

// template <class Impl>
// WidthDecoder<Impl>::WidthDecoder(DerivO3CPUParams *params)
//     : _name(params->name() + ".widthdecoder")
// {
//     // DPRINTF(Widthdecoder, "Creating WidthDecoder object.\n");
// }

template <class Impl>
void
WidthDecoder<Impl>::setIQ(InstructionQueue<Impl> *iq_ptr)
{
    iqPtr = iq_ptr;
}

template <class Impl>
WidthDecoder<Impl>::~WidthDecoder() {}

template <class Impl>
void
WidthDecoder<Impl>::init(DerivO3CPUParams *params)
{
     DPRINTF(WidthDecoder, "Creating WidthDecoder object.\n");

    _name = csprintf("%s.widthDecoder", params->name);
}

template <class Impl>
std::bitset<WidthDecoder<Impl>::NumVecResolBits>
WidthDecoder<Impl>::vecInstWidthMask(DynInstPtr &inst)
{
    unsigned eSize = inst->staticInst->vecElemSize();
    unsigned nElem = inst->staticInst->vecNumElem();

    // uses the architecture vec width for the mask size
    std::bitset<NumVecResolBits> mask;

    if (inst->opClass() == Enums::SimdMult ||
        inst->opClass() == Enums::SimdMultAcc) {
        // multiplicand source registers
        int srcVn = 2, srcVm = 3;

        std::bitset<NumVecResolBits> maskVn =
            vecSrcRegWidthMask(inst, srcVn, eSize, nElem);
        std::bitset<NumVecResolBits> maskVm =
            vecSrcRegWidthMask(inst, srcVm, eSize, nElem);

        // default: operation width is the max of operands
        // TODO: should mult width be that of the operands?
        mask = maskVn|maskVm;
    }

    DPRINTF(WidthDecoder, "Vector inst mask is %s (eSize=%i).\n",
            mask.to_string(),
            eSize);

    return mask;
}

/**
 * @todo Change to use resol granularity.
 */
template <class Impl>
std::bitset<WidthDecoder<Impl>::NumVecResolBits>
WidthDecoder<Impl>::vecSrcRegWidthMask(DynInstPtr &inst, int src,
                                       unsigned eSize, unsigned nElem)
{
    // uses the architecture vec width for the mask size
    std::bitset<NumVecResolBits> mask;

    // TODO: check if eSize and nElem match architecture

    if (eSize == 0)
    {
        // 8-bit

        mask.set();
    }
    else if (eSize == 1)
    {
        // 16-bit
        // Specific for ARMv8

        const VecRegT<int16_t, 8, true> &vsrc16 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++)
        {
            int rsl = signedIntResolution((int64_t) vsrc16[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, vsrc16[i], rsl);

            int idx = (16 >> ResolGranularity) * i;
            if (rsl > 8)
            {
                mask.set(idx + 1);
            }
            mask.set(idx);
        }
    }
    else if (eSize == 2)
    {
        // 32-bit
        // Specific for ARMv8

        const VecRegT<int32_t, 4, true> &vsrc32 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++)
        {
            int rsl = signedIntResolution((int64_t) vsrc32[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, vsrc32[i], rsl);

            int idx = (32 >> ResolGranularity) * i; // TODO: use shift
            if (rsl > 24) { mask.set(idx + 3); }
            if (rsl > 16) { mask.set(idx + 2); }
            if (rsl >  8) { mask.set(idx + 1); }
            mask.set(idx);
        }
    }
    else if (eSize == 3)
    {
        // 64-bit
        const VecRegT<int64_t, 2, true> &vsrc64 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++)
        {
            int rsl = signedIntResolution(vsrc64[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, vsrc64[i], rsl);

            int idx = (64 >> ResolGranularity) * i; // TODO: use shift
            if (rsl > 56) { mask.set(idx + 7); }
            if (rsl > 48) { mask.set(idx + 6); }
            if (rsl > 40) { mask.set(idx + 5); }
            if (rsl > 32) { mask.set(idx + 4); }
            if (rsl > 24) { mask.set(idx + 3); }
            if (rsl > 16) { mask.set(idx + 2); }
            if (rsl >  8) { mask.set(idx + 1); }
            mask.set(idx);
        }
    }
    else
    {
        panic("Unknown eSize %d.", eSize);
    }

    DPRINTF(WidthDecoder, "Source operand %d mask is %s (eSize=%i).\n",
            src,
            mask.to_string(),
            eSize);

    return mask;
}

template <class Impl>
bool
WidthDecoder<Impl>::canFuseVecInst(DynInstPtr &inst1, DynInstPtr &inst2)
{
    if (!inst1->isVector() || !inst2->isVector()) {
        panic("Trying to fuse non-vector operations.");
    }

    // Can only pack instructions of certain type combinations
    // For now only pack SimdMult/SimdMultAcc
    if (!isFuseVecType(inst1) || !isFuseVecType(inst2)) {
        return false;
    }

    std::bitset<NumVecResolBits> mask1 = vecInstWidthMask(inst1);
    std::bitset<NumVecResolBits> mask2 = vecInstWidthMask(inst2);

    DPRINTF(WidthDecoder, "Trying to fuse %s and %s.\n",
            mask1.to_string(),
            mask2.to_string());

    // Optimal: count number of set bits in mask.
    // Try to pack as much as possible, even if unfeasible.
    return (mask1.count() + mask2.count()) <= NumVecResolBits;
}

template <class Impl>
bool
WidthDecoder<Impl>::isFuseVecType(DynInstPtr &inst)
{
    if (!inst->isVector()) {
        return false;
    }

    return (inst->opClass() == Enums::SimdMult ||
            inst->opClass() == Enums::SimdMultAcc);
}

template <class Impl>
void
WidthDecoder<Impl>::regStats()
{}

#endif // __CPU_O3_WIDTH_DECODER_IMPL_HH__
/// MPINHO 12-mar-2019 END ///

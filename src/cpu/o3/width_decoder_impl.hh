/// MPINHO 12-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_DECODER_IMPL_HH__
#define __CPU_O3_WIDTH_DECODER_IMPL_HH__

#include "arch/arm/generated/decoder.hh" /// MPINHO 17-jul-2019 END ///
#include "arch/generic/vec_reg.hh"
#include "arch/utility.hh"
#include "base/bitfield.hh"
#include "base/logging.hh"
#include "base/resolution.hh"
#include "base/trace.hh"
#include "cpu/o3/comm.hh"
#include "cpu/o3/inst_queue.hh"
#include "cpu/o3/packing_criteria.hh"
#include "cpu/o3/width_decoder.hh"
#include "debug/WidthDecoder.hh"
#include "enums/OpClass.hh"
#include "params/DerivO3CPU.hh"

template <class Impl>
WidthDecoder<Impl>::WidthDecoder()
    : iqPtr(NULL)
{}

/// MPINHO 11-may-2019 BEGIN ///
template <class Impl>
WidthDecoder<Impl>::WidthDecoder(DerivO3CPUParams *params)
    : _name(params->name + ".widthdecoder"),
      iqPtr(NULL),
      widthDef(params->widthDefinition),
      blockSize(params->widthBlockSize),
      packingPolicy(params->widthPackingPolicy)
{
    DPRINTF(WidthDecoder, "Creating WidthDecoder object.\n");

    /** Parameters */
    widthDef = params->widthDefinition;
    blockSize = params->widthBlockSize;
    packingPolicy = params->widthPackingPolicy;

    // blockSize must be a power of 2.
    if (!(blockSize && ((blockSize & (blockSize-1)) == 0))) {
        fatal("Block size (%u) must be a power of 2.",
              blockSize);
    }

    // Set width definition function.
    switch (widthDef) {
        case WidthDefinition::Unsigned :
            prcFunc = unsignedIntResolution;
            break;

        case WidthDefinition::Signed :
            prcFunc = signedIntResolution;
            break;

        default:
            panic("\"%s\" unimplemented width definition.",
                WidthDefinitionStrings[static_cast<int>(widthDef)]);
            break;
    }

    // Set width rounding function based on the block size.
    roundFunc = roundPrcBlock;
    roundedPrcFunc =
        std::bind(roundFunc, std::bind(prcFunc, std::placeholders::_1),
                  blockSize);

    initPackingClass();
    // Set packing policy function.
    switch (packingPolicy) {
        case WidthPackingPolicy::Disabled :
            packingCriteria =
                [] (VecWidthCode a, VecWidthCode b) { return false; };
            break;

        case WidthPackingPolicy::Simple :
            packingCriteria = simplePacking;
            break;

        case WidthPackingPolicy::Optimal :
            packingCriteria = optimalPacking;
            break;

        default:
            panic("\"%s\" packing criteria is not implemented.",
                  WidthPackingPolicyStrings[static_cast<int>(packingPolicy)]);
            break;
    }

    DPRINTF(WidthDecoder, "\tWidth definition: %s.\n",
            WidthDefinitionStrings[static_cast<int>(widthDef)]);
    DPRINTF(WidthDecoder, "\tBlock size: %u (bits)).\n", blockSize);
    DPRINTF(WidthDecoder, "\tPacking policy: %s.\n",
            WidthPackingPolicyStrings[static_cast<int>(packingPolicy)]);
}
/// MPINHO 11-may-2019 END ///

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

    /// MPINHO 08-may-2019 BEGIN ///
    /** Parameters */
    widthDef = params->widthDefinition;
    blockSize = params->widthBlockSize;
    packingPolicy = params->widthPackingPolicy;

    // Set width definition function.
    switch (widthDef) {
    case WidthDefinition::Unsigned :
        prcFunc = unsignedIntResolution;
        break;

    case WidthDefinition::Signed :
        prcFunc = signedIntResolution;
        break;

    default:
        panic("\"%s\" unimplemented width definition.",
              WidthDefinitionStrings[static_cast<int>(widthDef)]);
        break;
    }

    // Set width rounding function based on the block size.
    roundFunc = roundPrcBlock;
    roundedPrcFunc =
        std::bind(roundFunc, std::bind(prcFunc, std::placeholders::_1),
                  blockSize);

    initPackingClass();
    // Set packing policy function.
    switch (packingPolicy) {
        case WidthPackingPolicy::Disabled :
            packingCriteria =
                [] (VecWidthCode a, VecWidthCode b) { return false; };
            break;

        case WidthPackingPolicy::Simple :
            packingCriteria = simplePacking;
            break;

        case WidthPackingPolicy::Optimal :
            packingCriteria = optimalPacking;
            break;

        default:
            panic("\"%s\" packing criteria is not implemented.",
                  WidthPackingPolicyStrings[static_cast<int>(packingPolicy)]);
            break;
    }

    DPRINTF(WidthDecoder, "\tWidth definition: %s.\n",
            WidthDefinitionStrings[static_cast<int>(widthDef)]);
    DPRINTF(WidthDecoder, "\tBlock size: %u (bits)).\n", blockSize);
    DPRINTF(WidthDecoder, "\tPacking policy: %s.\n",
            WidthPackingPolicyStrings[static_cast<int>(packingPolicy)]);
    /// MPINHO 08-may-2019 END ///
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::vecInstWidthMask(DynInstPtr &inst)
{
    unsigned eSize = inst->staticInst->vecElemSize();
    unsigned nElem = inst->staticInst->vecNumElem();

    return VecWidthCode(nElem, 8 << eSize, 8 << eSize);

#if 0
    // uses the architecture vec width for the mask size
    VecWidthCode mask;

    PackingClass pkClass = packingClassMap[inst->opClass()];
    // TODO: revise this for special instructions, like immediate and
    // instructions with different element size.
    if (pkClass == PackingClass::PackingSimdMult) {
        // Integer simd multiplication.
        // FIXME: check if this is correct for all multiplication
        // instructions.

        // Multiplicand source registers
        int srcVn = 2, srcVm = 3;

        VecWidthCode maskVn =
            vecSrcRegWidthMask(inst, srcVn, eSize, nElem);
        VecWidthCode maskVm =
            vecSrcRegWidthMask(inst, srcVm, eSize, nElem);

        // default: operation width is the max of operands
        // TODO: should mult width be that of the operands?
        mask = maskVn|maskVm;
    }
#endif
}

/**
 * @todo Change to use resol granularity.
 */
template <class Impl>
VecWidthCode
WidthDecoder<Impl>::vecSrcRegWidthMask(DynInstPtr &inst, int src,
                                       unsigned eSize, unsigned nElem)
{
    // uses the architecture vec width for the mask size
    VecWidthCode mask;

    // TODO: check if eSize and nElem match architecture

    if (eSize == 0) {
        // 16x8-bit
        // Specific for ARMv8 NEON
        mask = VecWidthCode(16, 8);

        const VecRegT<uint8_t, 16, true> &vsrc8 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++) {
            int rsl = roundedPrcFunc((uint64_t) vsrc8[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, (int8_t) vsrc8[i], rsl);

            mask.set(i, rsl);
        }
    } else if (eSize == 1) {
        // 8x16-bit
        // Specific for ARMv8 NEON
        mask = VecWidthCode(8, 16);

        const VecRegT<uint16_t, 8, true> &vsrc16 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++) {
            int rsl = roundedPrcFunc((uint64_t) vsrc16[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, (int16_t) vsrc16[i], rsl);

            mask.set(i, rsl);
        }
    } else if (eSize == 2) {
        // 4x32-bit
        // Specific for ARMv8 NEON
        mask = VecWidthCode(4, 32);

        const VecRegT<uint32_t, 4, true> &vsrc32 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++)
        {
            int rsl = roundedPrcFunc((uint64_t) vsrc32[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, (int32_t) vsrc32[i], rsl);

            mask.set(i, rsl);
        }
    } else if (eSize == 3) {
        // 2x64-bit
        // Specific for ARMv8 NEON
        mask = VecWidthCode(2, 64);

        const VecRegT<uint64_t, 2, true> &vsrc64 =
            inst->readVecRegOperand(inst->staticInst.get(), src);

        for (size_t i = 0; i < nElem; i++)
        {
            int rsl = roundedPrcFunc(vsrc64[i]);

            DPRINTF(WidthDecoder, "    Vec Lane %i: val=%d, rsl=%d\n",
                    i, (int64_t) vsrc64[i], rsl);

            mask.set(i, rsl);
        }
    } else {
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
WidthDecoder<Impl>::isFuseVecType(DynInstPtr &inst)
{
    decode(inst);

#if 0
    if (!inst->isVector()) {
        return false;
    }

    return packingClassMap[inst->opClass()] != PackingClass::NoPacking;
#endif
    return false;
}

template <class Impl>
bool
WidthDecoder<Impl>::canFuseVecInst(DynInstPtr &inst1, DynInstPtr &inst2)
{
#if 0
    if (!inst1->isVector() || !inst2->isVector()) {
        panic("Trying to fuse non-vector operations.");
    }

    // Can only pack instructions of certain type combinations.
    if (packingClassMap[inst1->opClass()] == PackingClass::NoPacking ||
        packingClassMap[inst2->opClass()] == PackingClass::NoPacking) {
        // One of the instructions is not of a packing type.
        return false;
    } else if (packingClassMap[inst1->opClass()] !=
               packingClassMap[inst2->opClass()]) {
        // The instructions' packing class does not match.
        return false;
    }

    VecWidthCode mask1 = vecInstWidthMask(inst1);
    VecWidthCode mask2 = vecInstWidthMask(inst2);

    DPRINTF(WidthDecoder, "Trying to fuse %s (%s) and %s (%s).\n",
            mask1.to_string(),
            Enums::OpClassStrings[inst1->opClass()],
            mask2.to_string(),
            Enums::OpClassStrings[inst2->opClass()]);

    // TODO: Use chosen packing.
    return optimalPacking(mask1, mask2);
#endif

    return false;
}

template <class Impl>
void
WidthDecoder<Impl>::decode(DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    if (ILLEGALEXEC == 0x0 && DECODERFAULT == 0x0 && THUMB == 0x0) {
        if (AARCH64 == 0x1) {
            if (bits(machInst, 27, 25) == 0x7) {
                // bit 27:25=111 -> AdvSimd
                if (bits(machInst, 28) == 0) {
                    if (bits(machInst, 31) == 0) {
                        // AdvSimd Vector inst.
                        DPRINTF(WidthDecoder, "AdvSimd Vector"
                                " inst decoded: %s.\n",
                                inst->staticInst->disassemble(
                                    inst->instAddr()));

                        if (bits(machInst, 24) == 1) {
                            if (bits(machInst, 10) == 0) {
                                // Neon IndexedElem.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector IndexedElem"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                            } else if (bits(machInst, 23) == 1) {
                                // Nop.
                            } else {
                                if (bits(machInst, 22, 19)) {
                                    // Neon ShiftByImm.
                                    DPRINTF(WidthDecoder,
                                            "Neon Vector ShiftByImm"
                                            " inst decoded: %s.\n",
                                            inst->staticInst->disassemble(
                                                inst->instAddr()));
                                } else {
                                    // Neon NeonModImm.
                                    DPRINTF(WidthDecoder,
                                            "Neon Vector ModImm"
                                            " inst decoded: %s.\n",
                                            inst->staticInst->disassemble(
                                                inst->instAddr()));
                                }
                            }
                        } else if (bits(machInst, 21) == 1) {
                            if (bits(machInst, 10) == 1) {
                                // Neon 3Same.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector 3Same"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                decode3Same(inst);
                            } else if (bits(machInst, 11) == 0) {
                                // Neon 3Diff.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector 3Diff"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                            } else if (bits(machInst, 20, 17) == 0x0) {
                                // Neon 2RegMisc.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector 2RegMisc"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                            } else if (bits(machInst, 20, 17) == 0x8) {
                                // Neon AcrossLanes.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector AcrossLanes"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                            }
                        } else if (bits(machInst, 24) ||
                                bits(machInst, 21) ||
                                bits(machInst, 15)) {
                            // Nop.
                        } else if (bits(machInst, 10) == 1) {
                            if (!bits(machInst, 23, 22)) {
                                // Neon Copy.
                                DPRINTF(WidthDecoder,
                                        "Neon Vector Copy"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                            }
                        } else if (bits(machInst, 29) == 1) {
                            // Neon Ext.
                            DPRINTF(WidthDecoder,
                                    "Neon Ext inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                        } else if (bits(machInst, 11) == 1) {
                            // Neon ZipUzpTrn.
                            DPRINTF(WidthDecoder,
                                    "Neon Vector ZipUzpTrn"
                                    " inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                        } else if (bits(machInst, 23, 22) == 0x0) {
                            // NeonTblTbx.
                            DPRINTF(WidthDecoder,
                                    "Neon Vector TblTbx"
                                    " inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                        }
                    }
                } else if (bits(machInst, 31) == 0) {
                    // AdvSimd Scalar inst.

                    DPRINTF(WidthDecoder,
                            "AdvSimd Scalar inst decoded: %s.\n",
                            inst->staticInst->disassemble(
                                inst->instAddr()));
                } else {
                    // Other AdvSimd inst.
                    DPRINTF(WidthDecoder, "Other AdvSimd"
                            " inst decoded: %s.\n",
                            inst->staticInst->disassemble(
                                inst->instAddr()));
                }
            }

            return;
        }
    }

    // DPRINTF(WidthDecoder, "Non AARCH64 inst decoded: %s.\n",
    //         inst->staticInst->disassemble(inst->instAddr()));
}

template <class Impl>
void
WidthDecoder<Impl>::decode3Same(DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t size = bits(machInst, 23, 22);
    uint8_t opcode = bits(machInst, 15, 11);

    // IntRegIndex vd = (IntRegIndex) (uint8_t) bits(machInst, 4, 0);
    // IntRegIndex vn = (IntRegIndex) (uint8_t) bits(machInst, 9, 5);
    // IntRegIndex vm = (IntRegIndex) (uint8_t) bits(machInst, 20, 16);

    // uint8_t size_q = (size << 1) | q;
    // uint8_t sz_q = size_q & 0x3;

    switch (opcode) {
        case 0x12:
            // MlsDX, MlsQX, MlaDX, MlaQX
            DPRINTF(WidthDecoder,
                    "Neon MLA inst decoded: %s. Size: %d, Q: %d.\n",
                    inst->staticInst->disassemble(inst->instAddr()),
                    size, q);
        case 0x13:
            if (!u) {
                // MulDX, MulQX
                DPRINTF(WidthDecoder,
                        "Neon MUL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
            }
            break;
    }
}

template <class Impl>
void
WidthDecoder<Impl>::regStats()
{}

template <class Impl>
void
WidthDecoder<Impl>::initPackingClass()
{
    packingClassMap.fill(PackingClass::NoPacking);

    // SimdAlu generic classes
    // packingClassMap[Enums::SimdAlu] = PackingClass::PackingSimdAlu;
    // packingClassMap[Enums::SimdCmp] = PackingClass::PackingSimdAlu;
    // packingClassMap[Enums::SimdCvt] = PackingClass::PackingSimdAlu;
    // packingClassMap[Enums::SimdMisc] = PackingClass::PackingSimdAlu;
    // packingClassMap[Enums::SimdShift] = PackingClass::PackingSimdAlu;
    // packingClassMap[Enums::SimdShiftAcc] = PackingClass::PackingSimdAlu;

    // SimdAdd classes
    // packingClassMap[Enums::SimdAdd] = PackingClass::PackingSimdAdd;
    // packingClassMap[Enums::SimdAddAcc] = PackingClass::PackingSimdAdd;

    // SimdMult classes
    packingClassMap[Enums::SimdMult] = PackingClass::PackingSimdMult;
    packingClassMap[Enums::SimdMultAcc] = PackingClass::PackingSimdMult;
}
#endif // __CPU_O3_WIDTH_DECODER_IMPL_HH__
/// MPINHO 12-mar-2019 END ///

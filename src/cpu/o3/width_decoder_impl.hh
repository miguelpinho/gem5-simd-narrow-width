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
#include "cpu/o3/width_decoder.hh"
#include "cpu/reg_class.hh"
#include "debug/WidthDecoder.hh"
#include "debug/WidthDecoderDecode.hh"
#include "debug/WidthDecoderWidth.hh"
#include "enums/OpClass.hh"
#include "params/DerivO3CPU.hh"

template <class Impl>
WidthDecoder<Impl>::WidthDecoder()
    : iqPtr(NULL)
{}

/// MPINHO 11-may-2019 BEGIN ///
template <class Impl>
WidthDecoder<Impl>::WidthDecoder(O3CPU *cpu_ptr, DerivO3CPUParams *params)
    : _name(params->name + ".widthdecoder"),
      cpu(cpu_ptr),
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
WidthDecoder<Impl>::setCPU(O3CPU *cpu_ptr)
{
    cpu = cpu_ptr;
}

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
void
WidthDecoder<Impl>::regStats()
{
    using namespace Stats;

    for (int i = 0; i < Num_VecElemSize; i++) {
        statVectorOpElemWidthBySize[i]
            .init(0, Bits_VecElemSize[i], 1)
            .name(name() + ".statVectorOpElemWidthBySize_" +
                  VecElemSizeStrings[i])
            .desc("Width of vector operand elements, by vector element size.")
            .prereq(statVectorOpElemWidthBySize[i])
            ;
        statVectorOpTotalWidthBySize[i]
            .init(0, VecSizeBits, 1)
            .name(name() + ".statVectorOpTotalWidthBySize" +
                  VecElemSizeStrings[i])
            .desc("Total width of vector operand elements,"
                  " by vector element size.")
            .prereq(statVectorOpTotalWidthBySize[i])
            ;
        statVectorInstElemWidthBySize[i]
            .init(0, Bits_VecElemSize[i], 1)
            .name(name() + ".statVectorInstElemWidthBySize" +
                  VecElemSizeStrings[i])
            .desc("Width of vector inst elements, by vector element size.")
            .prereq(statVectorInstElemWidthBySize[i])
            ;
        statVectorInstTotalWidthBySize[i]
            .init(0, VecSizeBits, 1)
            .name(name() + ".statVectorInstTotalWidthBySize" +
                  VecElemSizeStrings[i])
            .desc("Total width of vector inst,"
                  " by vector element size.")
            .prereq(statVectorInstTotalWidthBySize[i])
            ;
    }

    statVectorInstTotalWidthByClass
        .init(Num_WidthClass, 0, VecSizeBits, 8)
        .name(name() + ".statVectorInstTotalWidthByClass")
        .desc("Total width of vector isnt,"
              " by width class.")
        ;
    for (int i = 0; i < Num_WidthClass; i++) {
        statVectorInstTotalWidthByClass.subname(i, WidthClassStrings[i]);
    }
}

/**
 * @todo Change to use resol granularity.
 */
template <class Impl>
VecWidthCode
WidthDecoder<Impl>::vecSrcRegWidthMask(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op)
{
    VecWidthCode mask;

    if (size == 0) {
        // 16x8-bit or (8x8-bit)
        mask = getWidthVecReg<16, int8_t>(inst, (q ? 16 : 8), 8, op);
    } else if (size == 1) {
        // 8x16-bit or (4x16-bit)
        mask = getWidthVecReg<8, int16_t>(inst, (q ? 8 : 4), 16, op);
    } else if (size == 2) {
        // 4x32-bit or (2x32-bit)
        mask = getWidthVecReg<4, int32_t>(inst, (q ? 4 : 2), 32, op);
    } else if (size == 3) {
        // 2x64-bit (or 1x64-bit)
        mask = getWidthVecReg<2, int64_t>(inst, (q ? 2 : 1), 64, op);
    } else {
        panic("Unknown eSize %d.", size);
    }

    DPRINTF(WidthDecoderWidth, "Source operand %d mask is %s (eSize=%i).\n",
            op,
            mask.to_string(),
            size);

    return mask;
}

/**
 * @todo Change to use resol granularity.
 */
template <class Impl>
VecWidthCode
WidthDecoder<Impl>::vecSrcRegWidthMaskWide(const DynInstPtr &inst,
                                           uint8_t q, uint8_t size,
                                           uint8_t op)
{
    VecWidthCode mask;

    bool low = q ? true : false;

    if (size == 1) {
        // low 8x8-bit (q = 0) or high 8x8-bit (q = 1)
        mask = getWidthVecRegWiden<16, int8_t>(inst, 8, 16, op, low);
    } else if (size == 2) {
        // low 4x16-bit (q = 0) or high 4x16-bit (q = 1)
        mask = getWidthVecRegWiden<8, int16_t>(inst, 4, 32, op, low);
    } else if (size == 3) {
        // low 2x32-bit (q = 0) or high 2x32-bit (q = 1)
        mask = getWidthVecRegWiden<4, int32_t>(inst, 2, 64, op, low);
    } else {
        panic("Unknown eSize %d.", size);
    }

    DPRINTF(WidthDecoderWidth, "Source operand %d mask is %s (eSize=%i).\n",
            op,
            mask.to_string(),
            size);

    return mask;
}

template <class Impl>
template <int Size, typename Elem>
VecWidthCode
WidthDecoder<Impl>::getWidthVecReg(const DynInstPtr &inst, int nElem,
                                   int nBits, uint8_t op)
{
    assert(nElem <= Size);

    VecWidthCode mask(Size, nBits);

    // FIXME: this count as an invalid access to the register, in terms of
    // stats?? Create proxy access function?
    const VecRegT<Elem, Size, true> &vsrc =
        inst->readVecRegOperand(inst->staticInst.get(), op);

    for (size_t i = 0; i < nElem; i++)
    {
        int rsl = roundedPrcFunc((uint64_t) (int64_t) vsrc[i]);

        DPRINTF(WidthDecoderWidth, "    Vec Lane %i: val=%d, rsl=%d\n",
                i, (int) vsrc[i], rsl);

        assert(rsl <= nBits);

        mask.set(i, rsl);
    }

    return mask;
}

template <class Impl>
template <int Size, typename Elem>
VecWidthCode
WidthDecoder<Impl>::getWidthVecRegWiden(const DynInstPtr &inst, int nElem,
                                        int nBits, uint8_t op, bool low)
{
    VecWidthCode mask(nElem, nBits);

    int bias = low ? 0 : nElem;

    // FIXME: this count as an invalid access to the register, in terms of
    // stats?? Create proxy access function?
    const VecRegT<Elem, Size, true> &vsrc =
        inst->readVecRegOperand(inst->staticInst.get(), op);

    for (size_t i = 0; i < nElem; i++)
    {
        int rsl = roundedPrcFunc((uint64_t) (int64_t) vsrc[bias + i]);

        DPRINTF(WidthDecoderWidth, "    Vec Lane %i: val=%d, rsl=%d\n",
                i, (int) vsrc[i], rsl);

        assert(rsl <= nBits);

        mask.set(i, rsl);
    }

    return mask;
}

template <class Impl>
void
WidthDecoder<Impl>::addWidthInfo(const DynInstPtr &inst)
{
    WidthInfo width = decode(inst);
    inst->setWidth(width);

    statVectorInstTotalWidthByClass[(int) inst->getWidthClass()]
        .sample(inst->getWidthVal());

    DPRINTF(WidthDecoder, "Instruction \"%s\" was assigned"
            " width information: \"%s\".\n",
            inst->staticInst->disassemble(
                inst->instAddr()),
            inst->getWidth().to_string());
}

template <class Impl>
bool
WidthDecoder<Impl>::isFuseType(const DynInstPtr &inst)
{
    return inst->getWidth().isFuseType();
}

template <class Impl>
bool
WidthDecoder<Impl>::matchFuseType(const DynInstPtr &inst1,
                                  const DynInstPtr &inst2)
{
    WidthInfo inst1_width = inst1->getWidth();
    WidthInfo inst2_width = inst2->getWidth();

    DPRINTF(WidthDecoder, "Checking if \"%s\" and \"%s\" match.\n",
            inst1->staticInst->disassemble(inst1->instAddr()),
            inst2->staticInst->disassemble(inst2->instAddr()));

    // TODO: Use chosen packing.
    return inst1_width.matchType(inst2_width);
}

template <class Impl>
bool
WidthDecoder<Impl>::canFuseInst(const DynInstPtr &inst1,
                                const DynInstPtr &inst2)
{
    WidthInfo inst1_width = inst1->getWidth();
    WidthInfo inst2_width = inst2->getWidth();

    DPRINTF(WidthDecoder, "Trying to fuse \"%s\" and \"%s\".\n",
            inst1->staticInst->disassemble(inst1->instAddr()),
            inst2->staticInst->disassemble(inst2->instAddr()));

    // TODO: Use chosen packing.
    return inst1_width.canFuse(inst2_width, optimalPacking);
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decode(const DynInstPtr &inst)
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
                        DPRINTF(WidthDecoderDecode,
                                "AdvSimd Vector inst decoded: %s.\n",
                                inst->staticInst->disassemble(
                                    inst->instAddr()));

                        if (bits(machInst, 24) == 1) {
                            if (bits(machInst, 10) == 0) {
                                // Neon IndexedElem.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector IndexedElem"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return(WidthInfo(WidthClass::SimdNoInfo));
                            } else if (bits(machInst, 23) == 1) {
                                // Nop.
                                return(WidthInfo(WidthClass::NoInfo));
                            } else {
                                if (bits(machInst, 22, 19)) {
                                    // Neon ShiftByImm.
                                    DPRINTF(WidthDecoderDecode,
                                            "Neon Vector ShiftByImm"
                                            " inst decoded: %s.\n",
                                            inst->staticInst->disassemble(
                                                inst->instAddr()));
                                    return decodeNeonShiftByImm(inst);
                                } else {
                                    // Neon NeonModImm.
                                    DPRINTF(WidthDecoderDecode,
                                            "Neon Vector ModImm"
                                            " inst decoded: %s.\n",
                                            inst->staticInst->disassemble(
                                                inst->instAddr()));
                                    return(WidthInfo(WidthClass::SimdNoInfo));
                                }
                            }
                        } else if (bits(machInst, 21) == 1) {
                            if (bits(machInst, 10) == 1) {
                                // Neon 3Same.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector 3Same"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return decodeNeon3Same(inst);
                            } else if (bits(machInst, 11) == 0) {
                                // Neon 3Diff.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector 3Diff"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return decodeNeon3Diff(inst);
                            } else if (bits(machInst, 20, 17) == 0x0) {
                                // Neon 2RegMisc.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector 2RegMisc"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return decodeNeon2RegMisc(inst);
                            } else if (bits(machInst, 20, 17) == 0x8) {
                                // Neon AcrossLanes.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector AcrossLanes"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return decodeNeonAcrossLanes(inst);
                            }
                        } else if (bits(machInst, 24) ||
                                bits(machInst, 21) ||
                                bits(machInst, 15)) {
                            // Nop.
                        } else if (bits(machInst, 10) == 1) {
                            if (!bits(machInst, 23, 22)) {
                                // Neon Copy.
                                DPRINTF(WidthDecoderDecode,
                                        "Neon Vector Copy"
                                        " inst decoded: %s.\n",
                                        inst->staticInst->disassemble(
                                            inst->instAddr()));
                                return(WidthInfo(WidthClass::SimdNoInfo));
                            }
                        } else if (bits(machInst, 29) == 1) {
                            // Neon Ext.
                            DPRINTF(WidthDecoderDecode,
                                    "Neon Ext inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                            return(WidthInfo(WidthClass::SimdNoInfo));
                        } else if (bits(machInst, 11) == 1) {
                            // Neon ZipUzpTrn.
                            DPRINTF(WidthDecoderDecode,
                                    "Neon Vector ZipUzpTrn"
                                    " inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                            return(WidthInfo(WidthClass::SimdNoInfo));
                        } else if (bits(machInst, 23, 22) == 0x0) {
                            // NeonTblTbx.
                            DPRINTF(WidthDecoderDecode,
                                    "Neon Vector TblTbx"
                                    " inst decoded: %s.\n",
                                    inst->staticInst->disassemble(
                                        inst->instAddr()));
                            return(WidthInfo(WidthClass::SimdNoInfo));
                        }
                    }
                } else if (bits(machInst, 31) == 0) {
                    // AdvSimd Scalar inst.

                    DPRINTF(WidthDecoderDecode,
                            "AdvSimd Scalar inst decoded: %s.\n",
                            inst->staticInst->disassemble(
                                inst->instAddr()));
                    return(WidthInfo(WidthClass::SimdNoInfo));
                } else {
                    // Other AdvSimd inst.
                    DPRINTF(WidthDecoderDecode,
                            "Other AdvSimd inst decoded: %s.\n",
                            inst->staticInst->disassemble(
                                inst->instAddr()));
                    return(WidthInfo());
                }
            }

            return(WidthInfo());
        }
    }

    DPRINTF(WidthDecoderDecode,
            "Non AARCH64 inst decoded: %s.\n",
            inst->staticInst->disassemble(inst->instAddr()));
    return(WidthInfo());
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeon3Same(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t size = bits(machInst, 23, 22);
    uint8_t opcode = bits(machInst, 15, 11);

    uint8_t size_q = (size << 1) | q;

    switch (opcode) {
        case 0x00:
            if (size != 0x3) {
                // UhaddDX, UhaddQX, ShaddDX, ShaddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon HADD inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x01:
            if (size_q != 0x6) {
                // UqaddDX, UqaddQX, SqaddDX, SqaddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon QADD inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp2VectorRegl(inst, q, size, 2, 3),
                                size));
            }
            break;
        case 0x02:
            if (size != 0x3) {
                // UrhaddDX, UrhaddQX, SrhaddDX, SrhaddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon RHADD inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x04:
            if (size != 0x3) {
                // UhsubDX, UhsubQX, ShsubDX, ShsubQX
                DPRINTF(WidthDecoderDecode,
                        "Neon HSUB inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp2VectorRegl(inst, q, size, 2, 3),
                                size));
            }
            break;
        case 0x05:
            if (size_q != 0x6) {
                // UqsubDX, UqsubQX, SqsubDX, SqsubQX
                DPRINTF(WidthDecoderDecode,
                        "Neon QSUB inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x06:
            if (size_q != 0x6) {
                // CmhiDX, CmhiQX, CmgtDX, CmgtQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "CMHI" : "CMGT",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x07:
            if (size_q != 0x6) {
                // CmhsDX, CmhsQX, CmgeDX, CmgeQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "CMHS" : "CMGE",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x0c:
            if (size != 0x3) {
                // UmaxDX, UmaxQX, SmaxDX, SmaxQX
                DPRINTF(WidthDecoderDecode,
                        "Neon MAX inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x0d:
            if (size != 0x3) {
                // UminDX, UminQX, SminDX, SminQX
                DPRINTF(WidthDecoderDecode,
                        "Neon MIN inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x0e:
            if (size != 0x3) {
                // UabdDX, UabdQX, SabdDX, SabdQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ABA inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x0f:
            if (size != 0x3) {
                // UabaDX, UabaQX, SabaDX, SabaQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ABA inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x10:
            if (size_q != 0x6) {
                // SubDX, SubQX, AddDX, AddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "SUB" : "ADD",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x11:
            if (size_q != 0x6) {
                if (u) {
                    // CmeqDX, CmeqQX
                    DPRINTF(WidthDecoderDecode,
                            "Neon CMEQ inst decoded: %s. Size: %d, Q: %d.\n",
                            inst->staticInst->disassemble(inst->instAddr()),
                            size, q);
                    return(WidthInfo(WidthClass::SimdPackingAlu,
                                     widthOp2VectorRegl(inst, q, size, 2, 3),
                                     size));
                } else {
                    // CmtstDX, CmtstQX
                    DPRINTF(WidthDecoderDecode,
                            "Neon CMTST inst decoded: %s. Size: %d, Q: %d.\n",
                            inst->staticInst->disassemble(inst->instAddr()),
                            size, q);
                    return(WidthInfo(WidthClass::SimdPackingAlu,
                                     widthOp2VectorRegl(inst, q, size, 2, 3),
                                     size));
                }
            }
            break;
        case 0x12:
            // MlsDX, MlsQX, MlaDX, MlaQX
            if (size != 0x3) {
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "MLS" : "MLA",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x13:
            if (!u) {
                // MulDX, MulQX
                DPRINTF(WidthDecoderDecode,
                        "Neon MUL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x14:
            if (size != 0x3) {
                // UmaxpDX, UmaxpQX, SmaxpDX, SmaxpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon MAXP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorPair(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x15:
            if (size != 0x3) {
                // UminpDX, UminpQX, SminpDX, SminpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon MINP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorPair(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x16:
            if (size != 0x3 && size != 0x0) {
                // SqrdmulhQX, SqrdmulhDX, SqdmulhQX, SqdmulhDX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "SQRDMULH" : "SQDMULH",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorRegl(inst, q, size, 2, 3),
                                 size));
            }
            break;
        case 0x17:
            if (u || size_q == 0x6) {
                return(WidthInfo());
            } else {
                // AddpDX, AddpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorPair(inst, q, size, 2, 3),
                                 size));
            }
            break;
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeon3Diff(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t size = bits(machInst, 23, 22);
    uint8_t opcode = bits(machInst, 15, 12);

    switch (opcode) {
        case 0x0:
            if (size != 0x3) {
                // UaddlX, Uaddl2X, SaddlX, Saddl2X
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x1:
            if (size != 0x3) {
                // UaddwX, Uaddw2X, SaddwX, Saddw2X
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDW inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorWide(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x2:
            if (size != 0x3) {
                // UsublX, Usubl2X, SsublX, Ssubl2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SUBL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x3:
            if (size != 0x3) {
                // UsubwX, Usubw2X, SsubwX, Ssubw2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SUBW inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorWide(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x4:
            if (size != 0x3) {
                // RaddhnX, Raddhn2X, AddhnX, Addhn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDHN inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x5:
            if (size != 0x3) {
                // UabalX, Uabal2X, SabalX, Sabal2X
                DPRINTF(WidthDecoderDecode,
                        "Neon ABAL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x6:
            if (size != 0x3) {
                // RsubhnX, Rsubhn2X, SubhnX, Subhn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SUBHN inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorRegl(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x7:
            if (size != 0x3) {
                // UabdlX, Uabdl2X, SabdlX, Sabdl2X
                DPRINTF(WidthDecoderDecode,
                        "Neon ABDL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x8:
            if (size != 0x3) {
                // UmlalX, Umlal2X, SmlalX, Smlal2X
                DPRINTF(WidthDecoderDecode,
                        "Neon MLAL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0x9:
            if (!(u) && size != 0x0 && size != 0x3) {
                // SqdmlalX, Sqdmlal2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SQDMLAL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0xa:
            if (size != 0x3) {
                // UmlslX, Umlsl2X, SmlslX, Smlsl2X
                DPRINTF(WidthDecoderDecode,
                        "Neon MLSL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0xb:
            if (!(u) && size != 0x0 && size != 0x3) {
                // SqdmlslX, Sqdmlsl2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SQDMLSL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0xc:
            if (size != 0x3) {
                // UmullX, Umull2X, SmullX, Smull2X
                DPRINTF(WidthDecoderDecode,
                        "Neon MULL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
        case 0xd:
            if (!(u) && size != 0x0 && size != 0x3) {
                // SqdmullX, Sqdmull2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SQDMULL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingMult,
                                 widthOp2VectorLong(inst, q, size+1, 2, 3),
                                 size+1));
            }
            break;
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeon2RegMisc(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t size = bits(machInst, 23, 22);
    uint8_t opcode = bits(machInst, 16, 12);

    uint8_t size_q = (size << 1) | q;
    uint8_t switchVal = opcode | ((u ? 1 : 0) << 5);

    switch (switchVal) {
        case 0x02:
            if (size != 0x3) {
                // SaddlpDX, SaddlpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDLP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorPairLong(inst, q, size, 2),
                                 size+1));
            }
            break;
        case 0x03:
            if (size_q != 0x6) {
                // SuqaddDX, SuqaddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon SUQADD inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp2VectorRegl(inst, q, size, 2, 3),
                                size));
            }
            break;
        case 0x04:
            if (size != 0x3) {
                // ClsDX, ClsQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CLS inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x06:
            if (size != 0x3) {
                // SadalpDX, SadalpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADALP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorPairLong(inst, q, size, 2),
                                 size+1));
            }
            break;
        case 0x07:
            if (size_q != 0x6) {
                // SqabsDX, SqabsQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ABS inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x08:
            if (size_q != 0x6) {
                // CmgtZeroDX, CmgtZeroQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CMGT(zero) inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x09:
            if (size_q != 0x6) {
                // CmeqZeroDX, CmeqZeroQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CMEQ(zero) inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x0a:
            if (size_q != 0x6) {
                // CmltZeroDX, CmltZeroQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CMLT(zero) inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x0b:
            if (size_q != 0x6) {
                // AbsDX, AbsQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ABS inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x1d:
            if (size_q != 0x2 && size < 0x2) {
                uint8_t sizeCvt = (size & 0x1) ? 3 : 2;

                // ScvtfDX, ScvtfQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CVT inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        sizeCvt, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, sizeCvt, 2),
                                 sizeCvt));
            }
            break;
        case 0x22:
            if (size != 0x3) {
                // UaddlpDX, UaddlpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDLP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorPairLong(inst, q, size, 2),
                                 size+1));
            }
            break;
        case 0x23:
            if (size_q != 0x6) {
                // UsqaddDX, UsqaddQX
                DPRINTF(WidthDecoderDecode,
                        "Neon USQADD inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp2VectorRegl(inst, q, size, 2, 3),
                                size));
            }
            break;
        case 0x24:
            if (size != 0x3) {
                // ClzDX, ClzQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CLZ inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x26:
            if (size != 0x3) {
                // UadalpDX, UadalpQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADALP inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorPairLong(inst, q, size, 2),
                                 size+1));
            }
            break;
        case 0x27:
            if (size_q != 0x6) {
                // SqnegDX, SqnegQX
                DPRINTF(WidthDecoderDecode,
                        "Neon SQNEG inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x28:
            if (size_q != 0x6) {
                // CmgeZeroDX, CmgeZeroQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CMGE(zero) inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x29:
            if (size_q != 0x6) {
                // CmleZeroDX, CmleZeroQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CMLE(zero) inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x2b:
            if (size_q != 0x6) {
                // NegDX, NegQX
                DPRINTF(WidthDecoderDecode,
                        "Neon NEG inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x3d:
            if (size_q != 0x2 && size < 0x2) {
                uint8_t sizeCvt = (size & 0x1) ? 3 : 2;

                // UcvtfDX, UcvtfQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CVT inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        sizeCvt, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, sizeCvt, 2),
                                 sizeCvt));
            }
            break;
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeonAcrossLanes(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t size = bits(machInst, 23, 22);
    uint8_t opcode = bits(machInst, 16, 12);

    uint8_t size_q = (size << 1) | q;
    uint8_t switchVal = opcode | ((u ? 1 : 0) << 5);

    switch (switchVal) {
        case 0x0a:
            if (size_q != 0x4 && size != 0x3) {
                // SmaxvDX, SmaxvQX
                DPRINTF(WidthDecoderDecode,
                        "Neon SMAX inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorAcross(inst, q, size, 2),
                                 size));
            }
            break;
        case 0x1a:
            if (size_q != 0x4 && size != 0x3) {
                // SminvDX, SminvQX
                DPRINTF(WidthDecoderDecode,
                        "Neon SMIN inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorAcross(inst, q, size, 2),
                                 size));
            }
            break;
        case 0x1b:
            if (size_q != 0x4 && size != 0x3) {
                // AddvDX, AddvQX
                DPRINTF(WidthDecoderDecode,
                        "Neon ADDV inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorAcross(inst, q, size, 2),
                                 size));
            }
            break;
        case 0x2a:
            if (size_q != 0x4 && size != 0x3) {
                // UmaxvDX, UmaxvQX
                DPRINTF(WidthDecoderDecode,
                        "Neon UMAX inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorAcross(inst, q, size, 2),
                                 size));
            }
            break;
        case 0x3a:
            if (size_q != 0x4 && size != 0x3) {
                // UminvDX, UminvQX
                DPRINTF(WidthDecoderDecode,
                        "Neon UMIN inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorAcross(inst, q, size, 2),
                                 size));
            }
            break;
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeonShiftByImm(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t u = bits(machInst, 29);
    uint8_t immh = bits(machInst, 22, 19);
    uint8_t opcode = bits(machInst, 15, 11);

    uint8_t immh3 = bits(machInst, 22);
    uint8_t immh3_q = (immh3 << 1) | q;
    uint8_t op_u = (bits(machInst, 12) << 1) | u;
    uint8_t size = findMsbSet(immh);

    switch (opcode) {
        case 0x00:
            if (immh3_q != 0x2) {
                // UshrDX, UshrQX, SshrDX, SshrQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "USHR" : "SSHR",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x02:
            if (immh3_q != 0x2) {
                // UsraDX, UsraQX, SsraDX, SsraQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "USRA" : "SSRA",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x04:
            if (immh3_q != 0x2) {
                // UrshrDX, UrshrQX, SrshrDX, SrshrQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "URSHR" : "SRSHR",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x06:
            if (immh3_q != 0x2) {
                // UrsraDX, UrsraQX, SrsraDX, SrsraQX
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "URSRA" : "SRSRA",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                 widthOp1VectorRegl(inst, q, size, 2), size));
            }
            break;
        case 0x0a:
            if (immh3_q != 0x2) {
                if (!u) {
                    // ShlDX, ShlQX
                    DPRINTF(WidthDecoderDecode,
                            "Neon SHL inst decoded: %s. Size: %d, Q: %d.\n",
                            inst->staticInst->disassemble(inst->instAddr()),
                            size, q);
                    return(WidthInfo(WidthClass::SimdPackingAlu,
                                    widthOp1VectorRegl(inst, q, size, 2),
                                                       size));
                }
            }
            break;
        case 0x0c:
            if (u && !(immh3_q == 0x2 || op_u == 0x0)) {
                // SqshluDX, SqshluQX
                DPRINTF(WidthDecoderDecode,
                        "Neon SQSHLU inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size, 2),
                                                    size));
            }
            break;
        case 0x0e:
            if (!(immh3_q == 0x2 || op_u == 0x0)) {
                // UqshlDX, UqshlQX, SqshlDX, SqshlQX
                DPRINTF(WidthDecoderDecode,
                        "Neon QSHL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size, 2),
                                                    size));
            }
            break;
        case 0x10:
            if (!(immh3)) {
                // SqshrunX, Sqshrun2X, ShrnX, Shrn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "SQSHRUN" : "SHRN",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size+1, 2),
                                                    size+1));
            }
            break;
        case 0x11:
            if (!(immh3)) {
                // SqrshrunX, Sqeshrun2X, RshrnX, Rshrn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "RSQSHRUN" : "RSHRN",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size+1, 2),
                                                    size+1));
            }
            break;
        case 0x12:
            if (!(immh3)) {
                // UqshrnX, Uqshrn2X, SqshrnX, Sqshrn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "UQSHRN" : "SQSHRN",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size+1, 2),
                                                    size+1));
            }
            break;
        case 0x13:
            if (!(immh3)) {
                // UqrshrnX, Uqrshrn2X, SqrshrnX, Sqrshrn2X
                DPRINTF(WidthDecoderDecode,
                        "Neon %s inst decoded: %s. Size: %d, Q: %d.\n",
                        (u) ? "UQRSHRN" : "SQRSHRN",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, size+1, 2),
                                                    size+1));
            }
            break;
        case 0x14:
            if (!(immh3)) {
                // UshllX, Ushll2X, SshllX, Sshll2X
                DPRINTF(WidthDecoderDecode,
                        "Neon SHLL inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        size+1, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorLong(inst, q, size+1, 2),
                                                    size+1));
            }
            break;
        case 0x1c:
            if (!(immh < 0x4 || immh3_q == 0x2)) {
                uint8_t sizeCvt = (size & 0x1) ? 3 : 2;

                // UcvtfDX, UcvtfQX, ScvtfDX, ScvtfQX
                DPRINTF(WidthDecoderDecode,
                        "Neon CVT inst decoded: %s. Size: %d, Q: %d.\n",
                        inst->staticInst->disassemble(inst->instAddr()),
                        sizeCvt, q);
                return(WidthInfo(WidthClass::SimdPackingAlu,
                                widthOp1VectorRegl(inst, q, sizeCvt, 2),
                                                    sizeCvt));
            }
            break;
        // TODO: Remaining shift insts.
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
WidthInfo
WidthDecoder<Impl>::decodeNeonTblTbx(const DynInstPtr &inst)
{
    using namespace ArmISAInst;

    ArmISA::ExtMachInst machInst = inst->staticInst->machInst;

    uint8_t q = bits(machInst, 30);
    uint8_t switchVal = bits(machInst, 14, 12);

    switch (switchVal) {
        case 0x0:
            DPRINTF(WidthDecoderDecode,
                    "TBL (1 reg) inst decoded: %s. Q: %d.\n",
                    inst->staticInst->disassemble(inst->instAddr()),
                    q);
            return(WidthInfo(WidthClass::SimdPackingAlu,
                             widthOp1VectorRegl(inst, q, 1, 4), 1));
            break;
        case 0x2:
            DPRINTF(WidthDecoderDecode,
                    "TBL (2 reg) inst decoded: %s. Q: %d.\n",
                    inst->staticInst->disassemble(inst->instAddr()),
                    q);
            return(WidthInfo(WidthClass::SimdPackingAlu,
                             widthOp2VectorRegl(inst, q, 1, 4, 5), 1));
            break;
    }

    return(WidthInfo(WidthClass::SimdNoInfo));
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp1VectorRegl(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1)
{
    VecWidthCode maskOp1;

    maskOp1 = vecSrcRegWidthMask(inst, q, size, op1);
    sampleVecOp(maskOp1, size);

    sampleVecInst(maskOp1, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 1 vector operand (regular)"
            " has width mask %s (eSize=%i).\n",
            maskOp1.to_string(),
            size);
    return maskOp1;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp1VectorLong(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1)
{
    VecWidthCode maskOp1;

    maskOp1 = vecSrcRegWidthMaskWide(inst, q, size, op1);
    sampleVecOp(maskOp1, size);

    sampleVecInst(maskOp1, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 1 vector operand (long)"
            " has width mask %s (eSize=%i).\n",
            maskOp1.to_string(),
            size);
    return maskOp1;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp1VectorPairLong(const DynInstPtr &inst,
                                           uint8_t q, uint8_t size,
                                           uint8_t op1)
{
    VecWidthCode maskOp1, maskRes;

    maskOp1 = vecSrcRegWidthMaskWide(inst, q, size, op1);
    sampleVecOp(maskOp1, size);

    maskRes = maskOp1.generate1OpPairLong();
    sampleVecInst(maskOp1, size+1);
    DPRINTF(WidthDecoderWidth, "Instruction with 1 vector operand "
            " (pairwise long) has width mask %s (eSize=%i).\n",
            maskOp1.to_string(),
            size+1);
    return maskRes;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp1VectorAcross(const DynInstPtr &inst,
                                         uint8_t q, uint8_t size,
                                         uint8_t op1)
{
    VecWidthCode maskOp1, maskRes;

    maskOp1 = vecSrcRegWidthMask(inst, q, size, op1);
    sampleVecOp(maskOp1, size);

    maskRes = maskOp1.generate1OpAcross();
    sampleVecInst(maskRes, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 1 vector operand (across)"
            " has width mask %s (eSize=%i).\n",
            maskOp1.to_string(),
            size);
    return maskRes;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp2VectorRegl(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1, uint8_t op2)
{
    VecWidthCode maskOp1, maskOp2, maskRes;

    maskOp1 = vecSrcRegWidthMask(inst, q, size, op1);
    maskOp2 = vecSrcRegWidthMask(inst, q, size, op2);
    sampleVecOp(maskOp1, size);
    sampleVecOp(maskOp2, size);

    maskRes = maskOp1.combine2OpRegl(maskOp2);
    sampleVecInst(maskRes, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 2 vectors operands (regular)"
            " has width mask %s (eSize=%i).\n",
            maskRes.to_string(),
            size);
    return maskRes;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp2VectorPair(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1, uint8_t op2)
{
    VecWidthCode maskOp1, maskOp2, maskRes;

    maskOp1 = vecSrcRegWidthMask(inst, q, size, op1);
    maskOp2 = vecSrcRegWidthMask(inst, q, size, op2);
    sampleVecOp(maskOp1, size);
    sampleVecOp(maskOp2, size);

    maskRes = maskOp1.combine2OpPair(maskOp2);
    sampleVecInst(maskRes, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 2 vectors operands (pairwise)"
            " has width mask %s (eSize=%i).\n",
            maskRes.to_string(),
            size);
    return maskRes;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp2VectorLong(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1, uint8_t op2)
{
    VecWidthCode maskOp1, maskOp2, maskRes;

    maskOp1 = vecSrcRegWidthMaskWide(inst, q, size, op1);
    maskOp2 = vecSrcRegWidthMaskWide(inst, q, size, op2);
    sampleVecOp(maskOp1, size);
    sampleVecOp(maskOp2, size);

    maskRes = maskOp1.combine2OpRegl(maskOp2);
    sampleVecInst(maskRes, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 2 vectors operands (long)"
            " has width mask %s (eSize=%i).\n",
            maskRes.to_string(),
            size);
    return maskRes;
}

template <class Impl>
VecWidthCode
WidthDecoder<Impl>::widthOp2VectorWide(const DynInstPtr &inst,
                                       uint8_t q, uint8_t size,
                                       uint8_t op1, uint8_t op2)
{
    VecWidthCode maskOp1, maskOp2, maskRes;

    maskOp1 = vecSrcRegWidthMask(inst, 1, size, op1);
    maskOp2 = vecSrcRegWidthMaskWide(inst, q, size, op2);
    sampleVecOp(maskOp1, size);
    sampleVecOp(maskOp2, size);

    maskRes = maskOp1.combine2OpRegl(maskOp2);
    sampleVecInst(maskRes, size);
    DPRINTF(WidthDecoderWidth, "Instruction with 2 vectors operands (wide)"
            " has width mask %s (eSize=%i).\n",
            maskRes.to_string(),
            size);
    return maskRes;
}

template <class Impl>
void
WidthDecoder<Impl>::sampleVecOp(VecWidthCode &mask, uint8_t size)
{
    VecElemSize eSize = SizeToVecElemSize[size];

    for (int i = 0; i < mask.numElem(); i++) {
        statVectorOpElemWidthBySize[(int) eSize].sample(mask.get(i));
    }

    statVectorOpTotalWidthBySize[(int) eSize].sample(mask.totalWidth());
}

template <class Impl>
void
WidthDecoder<Impl>::sampleVecInst(VecWidthCode &mask, uint8_t size)
{
    VecElemSize eSize = SizeToVecElemSize[size];

    for (int i = 0; i < mask.numElem(); i++) {
        statVectorInstElemWidthBySize[(int) eSize].sample(mask.get(i));
    }

    statVectorInstTotalWidthBySize[(int) eSize].sample(mask.totalWidth());
}

#endif // __CPU_O3_WIDTH_DECODER_IMPL_HH__
/// MPINHO 12-mar-2019 END ///

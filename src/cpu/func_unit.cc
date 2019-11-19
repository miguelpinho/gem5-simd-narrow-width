/*
 * Copyright (c) 2002-2006 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Steve Raasch
 */

#include "cpu/func_unit.hh"

#include <sstream>

#include "base/logging.hh"

using namespace std;


////////////////////////////////////////////////////////////////////////////
//
//  The funciton unit
//
FuncUnit::FuncUnit()
{
    opLatencies.fill(0);
    pipelined.fill(false);
    capabilityList.reset();
    /// MPINHO 22-aug-2019 BEGIN ///
    issueCap = 0;
    widthCap = 0;
    availIssueCap = 0;
    availWidthCap = 0;
    simd = false;
    floatp = false;
    /// MPINHO 22-aug-2019 END ///
}


//  Copy constructor
FuncUnit::FuncUnit(const FuncUnit &fu)
{

    for (int i = 0; i < Num_OpClasses; ++i) {
        opLatencies[i] = fu.opLatencies[i];
        pipelined[i] = fu.pipelined[i];
    }

    capabilityList = fu.capabilityList;
    /// MPINHO 22-aug-2019 BEGIN ///
    issueCap = fu.issueCap;
    availIssueCap = fu.issueCap;
    widthCap = fu.widthCap;
    availWidthCap = fu.widthCap;
    simd = fu.simd;
    floatp = fu.floatp;
    /// MPINHO 22-aug-2019 END ///
}


void
FuncUnit::addCapability(OpClass cap, unsigned oplat, bool pipeline)
{
    if (oplat == 0)
        panic("FuncUnit:  you don't really want a zero-cycle latency do you?");

    capabilityList.set(cap);

    opLatencies[cap] = oplat;
    pipelined[cap] = pipeline;
}

/// MPINHO 22-aug-2019 BEGIN ///
void
FuncUnit::setIssueCap(unsigned _issueCap)
{
    issueCap = _issueCap;
    availIssueCap = _issueCap;
}

void
FuncUnit::setWidthCap(unsigned _widthCap)
{
    widthCap = _widthCap;
    availWidthCap = _widthCap;
}

void
FuncUnit::setSimd(bool _simd)
{
    simd = _simd;
}

void
FuncUnit::setFP(bool _floatp)
{
    floatp = _floatp;
}

bool
FuncUnit::isSimd()
{
    return simd;
}

bool
FuncUnit::isFP()
{
    return floatp;
}
/// MPINHO 22-aug-2019 END ///

bool
FuncUnit::provides(OpClass capability)
{
    return capabilityList[capability];
}

bitset<Num_OpClasses>
FuncUnit::capabilities()
{
    return capabilityList;
}

unsigned &
FuncUnit::opLatency(OpClass cap)
{
    return opLatencies[cap];
}

bool
FuncUnit::isPipelined(OpClass capability)
{
    return pipelined[capability];
}

/// MPINHO 22-aug-2019 BEGIN ///
unsigned FuncUnit::getIssueCap()
{
    return availIssueCap;
}

unsigned FuncUnit::getUsedIssueCap()
{
    return issueCap - availIssueCap;
}

void FuncUnit::useIssueCap()
{
    assert(availIssueCap > 0);
    --availIssueCap;
}

void FuncUnit::resetIssueCap()
{
    availIssueCap = issueCap;
}

unsigned FuncUnit::getWidthCap()
{
    return availWidthCap;
}

unsigned FuncUnit::getUsedWidthCap()
{
    return widthCap - availWidthCap;
}

void FuncUnit::useWidthCap(unsigned width)
{
    assert(width <= availWidthCap);
    availWidthCap -= width;
}

void FuncUnit::resetWidthCap()
{
    availWidthCap = widthCap;
}
/// MPINHO 22-aug-2019 END ///

////////////////////////////////////////////////////////////////////////////
//
//  The SimObjects we use to get the FU information into the simulator
//
////////////////////////////////////////////////////////////////////////////

//
//  We use 2 objects to specify this data in the INI file:
//    (1) OpDesc - Describes the operation class & latencies
//                   (multiple OpDesc objects can refer to the same
//                   operation classes)
//    (2) FUDesc - Describes the operations available in the unit &
//                   the number of these units
//
//


//
//  The operation-class description object
//
OpDesc *
OpDescParams::create()
{
    return new OpDesc(this);
}

//
//  The FuDesc object
//
FUDesc *
FUDescParams::create()
{
    return new FUDesc(this);
}

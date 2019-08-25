/*
 * Copyright (c) 2012-2013 ARM Limited
 * All rights reserved
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Authors: Kevin Lim
 */

#include "cpu/o3/fu_pool.hh"

#include <sstream>

#include "cpu/func_unit.hh"

using namespace std;

////////////////////////////////////////////////////////////////////////////
//
//  A pool of function units
//

inline void
FUPool::FUIdxQueue::addFU(int fu_idx)
{
    funcUnitsIdx.push_back(fu_idx);
    ++size;
}

inline int
FUPool::FUIdxQueue::getFU()
{
    int retval = funcUnitsIdx[idx++];

    if (idx == size)
        idx = 0;

    return retval;
}

FUPool::~FUPool()
{
    fuListIterator i = funcUnits.begin();
    fuListIterator end = funcUnits.end();
    for (; i != end; ++i)
        delete *i;
}


// Constructor
FUPool::FUPool(const Params *p)
    : SimObject(p)
{
    numFU = 0;
    numSimdFU = 0;
    maxIssueCap = 1;
    maxWidthCap = 0;

    funcUnits.clear();

    maxOpLatencies.fill(Cycles(0));
    pipelined.fill(true);

    //
    //  Iterate through the list of FUDescData structures
    //
    const vector<FUDesc *> &paramList =  p->FUList;
    for (FUDDiterator i = paramList.begin(); i != paramList.end(); ++i) {

        //
        //  Don't bother with this if we're not going to create any FU's
        //
        if ((*i)->number) {
            //
            //  Create the FuncUnit object from this structure
            //   - add the capabilities listed in the FU's operation
            //     description
            //
            //  We create the first unit, then duplicate it as needed
            //
            FuncUnit *fu = new FuncUnit;

            OPDDiterator j = (*i)->opDescList.begin();
            OPDDiterator end = (*i)->opDescList.end();
            for (; j != end; ++j) {
                // indicate that this pool has this capability
                capabilityList.set((*j)->opClass);

                // Add each of the FU's that will have this capability to the
                // appropriate queue.
                for (int k = 0; k < (*i)->number; ++k)
                    fuPerCapList[(*j)->opClass].addFU(numFU + k);

                // indicate that this FU has the capability
                fu->addCapability((*j)->opClass, (*j)->opLat, (*j)->pipelined);

                if ((*j)->opLat > maxOpLatencies[(*j)->opClass])
                    maxOpLatencies[(*j)->opClass] = (*j)->opLat;

                if (!(*j)->pipelined)
                    pipelined[(*j)->opClass] = false;
            }

            /// MPINHO 22-aug-2019 BEGIN ///
            int newIssueCap = (*i)->fuseCap + 1;
            int newWidthCap = (*i)->widthCap;
            fu->setIssueCap(newIssueCap);
            fu->setWidthCap(newWidthCap);
            if (newIssueCap > maxIssueCap) maxIssueCap = newIssueCap;
            if (newWidthCap > maxWidthCap) maxWidthCap = newWidthCap;
            fu->setSimd((*i)->simd);
            /// MPINHO 22-aug-2019 END ///

            numFU++;
            if ((*i)->simd)
                numSimdFU++;

            //  Add the appropriate number of copies of this FU to the list
            fu->name = (*i)->name() + "(0)";
            funcUnits.push_back(fu);

            for (int c = 1; c < (*i)->number; ++c) {
                ostringstream s;
                numFU++;
                if ((*i)->simd)
                    numSimdFU++;
                FuncUnit *fu2 = new FuncUnit(*fu);

                s << (*i)->name() << "(" << c << ")";
                fu2->name = s.str();
                funcUnits.push_back(fu2);
            }
        }
    }

    unitBusy.resize(numFU);

    for (int i = 0; i < numFU; i++) {
        unitBusy[i] = false;
    }
}

/// MPINHO 23-aug-2019 BEGIN ///
void
FUPool::regStats()
{
    statSimdFUUsed
        .init(0, numSimdFU, 1)
        .name(name() + ".simd_fu_used")
        .desc("dist of the number of Simd FU used")
        ;
    statSimdIssueUsed
        .init(numSimdFU, 0, maxIssueCap, 1)
        .name(name() + ".issue_used_simd_fu")
        .desc("dist of insts issued for each Simd FU")
        ;
    statSimdWidthUsed
        .init(numFU, 0, maxWidthCap, 8)
        .name(name() + ".width_used_fu")
        .desc("dist of width used by each Simd FU")
        ;
    for (int i = 0; i < numSimdFU; i++) {
        std::stringstream ss;
        ss << "(" << i << ")";
        statSimdIssueUsed.subname(i, ss.str());
        statSimdWidthUsed.subname(i, ss.str());
    }
}
/// MPINHO 23-aug-2019 END ///

int
FUPool::getUnit(OpClass capability)
{
    //  If this pool doesn't have the specified capability,
    //  return this information to the caller
    if (!capabilityList[capability])
        return -2;

    int fu_idx = fuPerCapList[capability].getFU();
    int start_idx = fu_idx;

    // Iterate through the circular queue if needed, stopping if we've reached
    // the first element again.
    while (unitBusy[fu_idx]) {
        fu_idx = fuPerCapList[capability].getFU();
        if (fu_idx == start_idx) {
            // No FU available
            return -1;
        }
    }

    assert(fu_idx < numFU);

    unitBusy[fu_idx] = true;

    return fu_idx;
}

void
FUPool::freeUnitNextCycle(int fu_idx)
{
    assert(unitBusy[fu_idx]);
    unitsToBeFreed.push_back(fu_idx);
}

void
FUPool::processFreeUnits()
{
    while (!unitsToBeFreed.empty()) {
        int fu_idx = unitsToBeFreed.back();
        unitsToBeFreed.pop_back();

        assert(unitBusy[fu_idx]);

        unitBusy[fu_idx] = false;
    }
}

void
FUPool::dump()
{
    cout << "Function Unit Pool (" << name() << ")\n";
    cout << "======================================\n";
    cout << "Free List:\n";

    for (int i = 0; i < numFU; ++i) {
        if (unitBusy[i]) {
            continue;
        }

        cout << "  [" << i << "] : ";

        cout << funcUnits[i]->name << " ";

        cout << "\n";
    }

    cout << "======================================\n";
    cout << "Busy List:\n";
    for (int i = 0; i < numFU; ++i) {
        if (!unitBusy[i]) {
            continue;
        }

        cout << "  [" << i << "] : ";

        cout << funcUnits[i]->name << " ";

        cout << "\n";
    }
}

/// MPINHO 13-aug-2019 BEGIN ///
std::string
FUPool::getFUName(int fu_idx)
{
    return funcUnits[fu_idx]->name;
}

unsigned
FUPool::getFUIssueCap(int fu_idx)
{
    return funcUnits[fu_idx]->getIssueCap();
}

void
FUPool::useFUIssueCap(int fu_idx)
{
    funcUnits[fu_idx]->useIssueCap();
}

unsigned
FUPool::getFUWidthCap(int fu_idx)
{
    return funcUnits[fu_idx]->getWidthCap();
}

void
FUPool::useFUWidthCap(int fu_idx,
                      unsigned width)
{
    funcUnits[fu_idx]->useWidthCap(width);
}

void
FUPool::resetFUCaps()
{
    for (int i = 0; i < numFU; ++i) {
        if (!unitBusy[i]) {
            funcUnits[i]->resetIssueCap();
            funcUnits[i]->resetWidthCap();
        }
    }
}

bool
FUPool::hasCapability(int fu_idx, OpClass capability)
{
    return funcUnits[fu_idx]->provides(capability);
}

void
FUPool::updateStats()
{
    int usedSimd = 0;
    // Record stats for used Simd FUs.
    for (int i = 0; i < numFU; ++i) {
        if (funcUnits[i]->isSimd() &&
            funcUnits[i]->getUsedIssueCap() > 0) {
            statSimdIssueUsed[usedSimd]
                .sample(funcUnits[i]->getUsedIssueCap());
            statSimdWidthUsed[usedSimd]
                .sample(funcUnits[i]->getUsedWidthCap());

            DPRINTF(FU, "SimdFU(*%d*). Issued: *%d*. Width: *%d*.\n",
                    usedSimd,
                    funcUnits[i]->getUsedIssueCap(),
                    funcUnits[i]->getUsedWidthCap());

            ++usedSimd;
        }
    }

    statSimdFUUsed.sample(usedSimd);
    DPRINTF(FU, "TotalSimdFU used simd FU used: *%d*.\n",
            usedSimd);

    // Record remaining Simd FUs as unused.
    for (int i = usedSimd; i < numSimdFU; ++i) {
        statSimdIssueUsed[i].sample(0);
        statSimdWidthUsed[i].sample(0);
    }
}
/// MPINHO 13-aug-2019 END ///

bool
FUPool::isDrained() const
{
    bool is_drained = true;
    for (int i = 0; i < numFU; i++)
        is_drained = is_drained && !unitBusy[i];

    return is_drained;
}

//

////////////////////////////////////////////////////////////////////////////
//
//  The SimObjects we use to get the FU information into the simulator
//
////////////////////////////////////////////////////////////////////////////

//
//    FUPool - Contails a list of FUDesc objects to make available
//

//
//  The FuPool object
//
FUPool *
FUPoolParams::create()
{
    return new FUPool(this);
}

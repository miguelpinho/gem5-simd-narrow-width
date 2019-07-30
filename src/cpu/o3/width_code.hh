/// MPINHO 21-mar-2019 BEGIN ///
#ifndef __CPU_O3_WIDTH_CODE_HH__
#define __CPU_O3_WIDTH_CODE_HH__

#include <algorithm>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "arch/utility.hh"

/*
 * Class for vector width information encoding.
 *
 * Used to verify if instructions can be merged and register usage.
 */
class VecWidthCode
{
    private:
        static constexpr auto VecRegSizeBits = 128;

        int eBits;
        int nElem;
        std::vector<int> code;

    public:
        VecWidthCode();

        VecWidthCode(int _nElem, int _eBits);

        VecWidthCode(int _nElem, int _eBits, int val);

        ~VecWidthCode() {}

        static int vectorSize() { return static_cast<int>(VecRegSizeBits); }
        int elemBits() { return eBits; }
        int numElem() { return nElem; }

        void
        set(int pos, int val) {
            if (pos < 0 || pos > nElem) {
                panic("Invalid code position: %d", pos);
            }
            if (val < 0 || val > eBits) {
                panic("Invalid code val: %d (size: %d, pos: %d).",
                    val, eBits, pos);
            }

            code[pos] = val;
        }

        int
        get(int pos) {
            if (pos < 0 || pos > nElem) {
                panic("Invalid code position: %d", pos);
            }

            return (code[pos]);
        }

        bool
        match(const VecWidthCode& b) {
            return eBits == b.eBits && nElem == b.nElem;
        }

        int count();

        std::string to_string();

        VecWidthCode combine2OpRegl(const VecWidthCode& b);
        VecWidthCode combine2OpPair(const VecWidthCode& b);
};
#endif // __CPU_O3_WIDTH_CODE_HH__

/// MPINHO 23-mar-2019 END ///
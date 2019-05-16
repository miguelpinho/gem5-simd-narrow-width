/// MPINHO 21-mar-2019 BEGIN ///
#ifndef __WIDTH_CODE_HH__
#define __WIDTH_CODE_HH__

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
        static constexpr auto VecRegSizeBits =
            TheISA::VecRegSizeBytes * 4;

        int eBits;
        int nElem;
        std::vector<int> code;

    public:
        VecWidthCode();

        VecWidthCode(int _nElem, int _eBits);

        VecWidthCode(int _nElem, int _eBits, int val);

        ~VecWidthCode() {}

        static int vectorSize() { return VecRegSizeBits; }
        int elemBits() { return eBits; }
        int numElem() { return nElem; }

        void
        set(int pos, int val) {
            assert(pos >= 0 && pos < nElem);
            assert(val >= 0 && val <= eBits);

            code[pos] = val;
        }

        int
        get(int pos) {
            assert(pos >= 0 && pos < nElem);

            return (code[pos]);
        }

        bool
        match(const VecWidthCode& b) {
            return eBits == b.eBits && nElem == b.nElem;
        }

        int count();

        std::string to_string();

        VecWidthCode operator|(const VecWidthCode& b);
};

#endif // __WIDTH_CODE_HH__

/// MPINHO 23-mar-2019 END ///

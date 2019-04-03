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
        int eBits;
        int nElem;
        std::vector<int> code;

    public:
        VecWidthCode()
        {}

        VecWidthCode(int _nElem, int _eBits)
            : eBits(_eBits),
              nElem(_nElem)
        {
            code = std::vector<int>(nElem);
        }

        ~VecWidthCode() {}

        void
        set(int pos, int val) {
            assert(pos < nElem);
            assert(val <= eBits);

            code[pos] = val;
        }

        int
        get(int pos) {
            assert(pos < nElem);

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

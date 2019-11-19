/// MPINHO 24-mar-2019 BEGIN ///

#include <algorithm>
#include <numeric>

#include "cpu/o3/width_code.hh"
#include "config/the_isa.hh"

VecWidthCode::VecWidthCode()
{}

VecWidthCode::VecWidthCode(int _nElem, int _eBits)
    : eBits(_eBits),
      nElem(_nElem)
{
    if (_nElem * _eBits > VecSizeBits) {
        panic("Vector code is too large (%dx%d-bits).",
              _nElem, _eBits);
    }

    code = std::vector<int>(nElem);
}

VecWidthCode::VecWidthCode(int _nElem, int _eBits, int val)
    : VecWidthCode(_nElem, _eBits)
{
    if (val < 0 || val > eBits) {
        panic("Invalid code init val: %d (size: %d).",
              val, _eBits);
    }

    std::fill(code.begin(), code.end(), val);
}

int
VecWidthCode::totalWidth()
{
    // TODO: use std::accumullate().
    int ret = 0;
    for (int i = 0; i < nElem; i++) {
        ret += code[i];
    }
    return ret;
}

int
VecWidthCode::maxWidth()
{
    // TODO: use std::max_element().
    int ret = 0;
    for (int i = 0; i < nElem; i++) {
        if (code[i] > ret)
            ret = code[i];
    }
    return ret;
}

std::string
VecWidthCode::to_string()
{
    std::stringstream ss;

    for (int i = 0; i < nElem; i++) {
        ss << code[i] << ":";
    }
    ss << ":" << nElem << "x" << eBits << "-bit";

    return ss.str();
}

VecWidthCode
VecWidthCode::generate1OpPairLong()
{
    assert((nElem & 1) == 0);

    VecWidthCode res(nElem >> 1, eBits << 1);

    int pos = 0;
    for (int i = 0; i < nElem; i += 2) {
        res.code[pos++] = std::max(code[i], code[i+1]);
    }
    return res;
}

VecWidthCode
VecWidthCode::generate1OpAcross()
{
    return VecWidthCode(nElem, eBits, maxWidth());
}

VecWidthCode
VecWidthCode::combine2OpRegl(const VecWidthCode &b)
{
    assert(match(b));

    VecWidthCode res(nElem, eBits);

    for (int i = 0; i < nElem; i++) {
        res.code[i] = std::max(code[i], b.code[i]);
    }
    return res;
}

VecWidthCode
VecWidthCode::combine2OpPair(const VecWidthCode &b)
{
    assert(match(b));

    VecWidthCode res(nElem, eBits);

    int pos = 0;
    for (int i = 0; i < nElem; i += 2) {
        res.code[pos++] = std::max(code[i], code[i+1]);
    }
    for (int i = 0; i < nElem; i += 2) {
        res.code[pos++] = std::max(b.code[i], b.code[i+1]);
    }
    return res;
}

/// MPINHO 24-mar-2019 END ///

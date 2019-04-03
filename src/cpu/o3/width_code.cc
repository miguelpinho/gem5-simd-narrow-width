/// MPINHO 24-mar-2019 BEGIN ///

#include "cpu/o3/width_code.hh"

int
VecWidthCode::count()
{
    int ret = 0;
    for (int i = 0; i < nElem; i++) {
        ret += code[i];
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
VecWidthCode::operator|(const VecWidthCode &b)
{
    assert(match(b));

    VecWidthCode res(nElem, eBits);
    for (int i = 0; i < nElem; i++)
    {
        res.code[i] = std::max(code[i], b.code[i]);
    }
    return res;
}

/// MPINHO 24-mar-2019 END ///

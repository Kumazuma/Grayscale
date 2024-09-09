#pragma once

#include <wx/wx.h>
#include <wx/wxprec.h>
#include <Windows.h>
#include <wx/filepicker.h>
#include <wx/spinctrl.h>
#include <wx/dcbuffer.h>
#if defined(_M_AMD64)
#include <mmintrin.h>
#include <immintrin.h>
#elif defined(_M_ARM64)
#include <arm64_neon.h>
#include "sse2neon.h"
#define _MM_MK_INSERTPS_NDX(srcField, dstField, zeroMask) \
        (((srcField)<<6) | ((dstField)<<4) | (zeroMask))
#endif

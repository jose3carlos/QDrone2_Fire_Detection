#pragma once
// SDK uses GNU annotations for unused/format/always_inline in portable code.
#if defined(_MSC_VER) && !defined(__clang__)
#define __attribute__(x)
#ifndef __restrict__
#define __restrict__ __restrict
#endif
#endif

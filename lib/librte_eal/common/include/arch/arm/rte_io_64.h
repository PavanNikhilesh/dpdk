/*
 *   BSD LICENSE
 *
 *   Copyright (C) Cavium networks Ltd. 2016.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *     * Neither the name of Cavium networks nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _RTE_IO_ARM64_H_
#define _RTE_IO_ARM64_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RTE_OVERRIDE_IO_H

#include "generic/rte_io.h"
#include "rte_atomic_64.h"

static inline __attribute__((always_inline)) uint8_t
rte_read8_relaxed(const volatile void *addr)
{
	uint8_t val;

	asm volatile(
		    "ldrb %w[val], [%x[addr]]"
		    : [val] "=r" (val)
		    : [addr] "r" (addr));
	return val;
}

static inline __attribute__((always_inline)) uint16_t
rte_read16_relaxed(const volatile void *addr)
{
	uint16_t val;

	asm volatile(
		    "ldrh %w[val], [%x[addr]]"
		    : [val] "=r" (val)
		    : [addr] "r" (addr));
	return val;
}

static inline __attribute__((always_inline)) uint32_t
rte_read32_relaxed(const volatile void *addr)
{
	uint32_t val;

	asm volatile(
		    "ldr %w[val], [%x[addr]]"
		    : [val] "=r" (val)
		    : [addr] "r" (addr));
	return val;
}

static inline __attribute__((always_inline)) uint64_t
rte_read64_relaxed(const volatile void *addr)
{
	uint64_t val;

	asm volatile(
		    "ldr %x[val], [%x[addr]]"
		    : [val] "=r" (val)
		    : [addr] "r" (addr));
	return val;
}

static inline __attribute__((always_inline)) void
rte_write8_relaxed(uint8_t val, volatile void *addr)
{
	asm volatile(
		    "strb %w[val], [%x[addr]]"
		    :
		    : [val] "r" (val), [addr] "r" (addr));
}

static inline __attribute__((always_inline)) void
rte_write16_relaxed(uint16_t val, volatile void *addr)
{
	asm volatile(
		    "strh %w[val], [%x[addr]]"
		    :
		    : [val] "r" (val), [addr] "r" (addr));
}

static inline __attribute__((always_inline)) void
rte_write32_relaxed(uint32_t val, volatile void *addr)
{
	asm volatile(
		    "str %w[val], [%x[addr]]"
		    :
		    : [val] "r" (val), [addr] "r" (addr));
}

static inline __attribute__((always_inline)) void
rte_write64_relaxed(uint64_t val, volatile void *addr)
{
	asm volatile(
		    "str %x[val], [%x[addr]]"
		    :
		    : [val] "r" (val), [addr] "r" (addr));
}

#define rte_read8(addr) \
	({ uint8_t __v = rte_read8_relaxed(addr); rte_io_rmb(); __v; })

#define rte_read16(addr) \
	({ uint16_t __v = rte_read16_relaxed(addr); rte_io_rmb(); __v; })

#define rte_read32(addr) \
	({ uint32_t __v = rte_read32_relaxed(addr); rte_io_rmb(); __v; })

#define rte_read64(addr) \
	({ uint64_t __v = rte_read64_relaxed(addr); rte_io_rmb(); __v; })

#define rte_write8(value, addr) \
	({ rte_io_wmb(); rte_write8_relaxed(value, addr); })

#define rte_write16(value, addr) \
	({ rte_io_wmb(); rte_write16_relaxed(value, addr); })

#define rte_write32(value, addr) \
	({ rte_io_wmb(); rte_write32_relaxed(value, addr); })

#define rte_write64(value, addr) \
	({ rte_io_wmb(); rte_write64_relaxed(value, addr); })

#ifdef __cplusplus
}
#endif

#endif /* _RTE_IO_ARM64_H_ */

/*******************************************************************************
 *
 * The Clear BSD License
 *
 * Copyright (c) 2016, snickerbockers
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 ******************************************************************************/

#include <cstring>

#include "BaseException.h"

#include "Memory.h"

Memory::Memory(bool allow_unaligned) {
    mem = new uint8_t[MEM_SZ];
    memset(mem, 0, sizeof(uint8_t) * MEM_SZ);
    init_hex_sprites();

    this->allow_unaligned = true;
}

Memory::~Memory() {
    delete[] mem;
}

uint8_t Memory::read8(unsigned idx) {
    if (idx >= MEM_SZ)
        throw MemBoundsError(idx);
    return mem[idx];
}

void Memory::write8(unsigned idx, uint8_t val) {
    if (idx >= MEM_SZ)
        throw MemBoundsError(idx);
    mem[idx] = val;
}

uint16_t Memory::read16(unsigned idx) {
    if (idx >= (MEM_SZ - 1))
        throw MemBoundsError(idx);
    if ((idx & 1) && !allow_unaligned)
        throw MemAlignError(idx);

    return (mem[idx] << 8) | mem[idx + 1];
}

void Memory::write16(unsigned idx, uint16_t val) {
    if (idx >= (MEM_SZ - 1))
        throw MemBoundsError(idx);
    if ((idx & 1) && !allow_unaligned)
        throw MemAlignError(idx);

    mem[idx] = (val & 0xff00) >> 8;
    mem[idx + 1] = val & 0xff;
}

static const uint8_t hex_sprites[16][5] = {
    {
        0xf0, 0x90, 0x90, 0x90, 0xf0
    },
    {
        0x20, 0x60, 0x20, 0x20, 0x70
    },
    {
        0xf0, 0x10, 0xf0, 0x80, 0xf0
    },
    {
        0xf0, 0x10, 0xf0, 0x10, 0xf0
    },
    {
        0x90, 0x90, 0xf0, 0x10, 0x10
    },
    {
        0xf0, 0x80, 0xf0, 0x10, 0xf0
    },
    {
        0xf0, 0x80, 0xf0, 0x90, 0xf0
    },
    {
        0xf0, 0x10, 0x20, 0x40, 0x40
    },
    {
        0xf0, 0x90, 0xf0, 0x90, 0xf0
    },
    {
        0xf0, 0x90, 0xf0, 0x10, 0xf0
    },
    {
        0xf0, 0x90, 0xf0, 0x90, 0x90
    },
    {
        0xe0, 0x90, 0xe0, 0x90, 0xe0
    },
    {
        0xf0, 0x80, 0x80, 0x80, 0xf0
    },
    {
        0xe0, 0x90, 0x90, 0x90, 0xe0
    },
    {
        0xf0, 0x80, 0xf0, 0x80, 0xf0
    },
    {
        0xf0, 0x80, 0xf0, 0x80, 0x80
    }
};

void Memory::init_hex_sprites(void) {
    unsigned addr = HEX_SPRITE_START;
    for (unsigned hex_val = 0; hex_val < 16; hex_val++)
        for (unsigned row = 0; row < 5; row++)
            write8(addr++, hex_sprites[hex_val][row]);
}

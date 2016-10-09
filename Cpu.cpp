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
#include <arpa/inet.h> // for endian functions

#include "BaseException.h"

#include "Cpu.h"

Cpu::Cpu(Memory *mem) {
    memset(v, 0, sizeof(v));
    tim = snd = 0;
    pc = ROM_START_ADDR;

    this->mem = mem;
}

void Cpu::int_tim(void) {
    if (tim)
        tim--;
    if (snd)
        snd--;
}

void Cpu::next_inst(void) {
    inst_t inst;

    inst = mem->read16(pc);
    int nibbles[] = {
        get_nibble_from_inst(inst, 0),
        get_nibble_from_inst(inst, 1),
        get_nibble_from_inst(inst, 2),
        get_nibble_from_inst(inst, 3)
    };

    if (nibbles[3] == 0) {
        if (nibbles[2] == 0) {
            if (nibbles[1] == 0xE) {
                if (nibbles[0] == 0)
                    throw UnimplementedInstructionError("CLS");
                    // return std::string("CLS"); // 00E0
                else if (nibbles[0] == 0xE)
                    throw UnimplementedInstructionError("RET");
                    // return std::string("RET"); // 00EE
            }
        }
        // 0NNN
        throw UnimplementedInstructionError("SYS");
        // return std::string("SYS ") + nibble_to_hex(nibbles[2]) +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 1) {
        // 1NNN
        throw UnimplementedInstructionError("JP");
        // return std::string("JP ") + nibble_to_hex(nibbles[2]) +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 2) {
        // 2NNN
        throw UnimplementedInstructionError("CALL");
        // return std::string("CALL ") + nibble_to_hex(nibbles[2]) +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 3) {
        // 3xkk
        throw UnimplementedInstructionError("SE");
        // return std::string("SE V") + nibble_to_hex(nibbles[2]) + ", " +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 4) {
        // 4xkk
        throw UnimplementedInstructionError("SNE");
        // return std::string("SNE V") + nibble_to_hex(nibbles[2]) + ", " +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 5) {
        if (nibbles[0] == 0) {
            // 5xy0
            throw UnimplementedInstructionError("SE");
            // return std::string("SE V") + nibble_to_hex(nibbles[2]) + ", V" +
            //                    nibble_to_hex(nibbles[1]);
        } else {
            throw BadOpcodeError();
        }
    } else if (nibbles[3] == 6) {
        // 6xkk
        throw UnimplementedInstructionError("LD");
        // return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", " +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 7) {
        // 7xkk
        throw UnimplementedInstructionError("ADD");
        // return std::string("ADD V") + nibble_to_hex(nibbles[2]) + ", " +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 8) {
        if (nibbles[0] == 0) {
            // 8xy0
            throw UnimplementedInstructionError("LD");
            // return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 1) {
            // 8xy1
            throw UnimplementedInstructionError("OR");
            // return std::string("OR V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 2) {
            // 8xy2
            throw UnimplementedInstructionError("AND");
            // return std::string("AND V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 3) {
            // 8xy3
            throw UnimplementedInstructionError("XOR");
            // return std::string("XOR V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 4) {
            // 8xy4
            throw UnimplementedInstructionError("ADD");
            // return std::string("ADD V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 5) {
            // 8xy5
            throw UnimplementedInstructionError("SUB");
            // return std::string("SUB V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 6) {
            // 8xy6
            throw UnimplementedInstructionError("SHR");
            // return std::string("SHR V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[0] == 7) {
            // 8xy7
            throw UnimplementedInstructionError("SUBN");
            // return std::string("SUBN V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 0xe) {
            // 8xye
            throw UnimplementedInstructionError("SHL");
            // return std::string("SHL V") + nibble_to_hex(nibbles[2]);
        }

        throw BadOpcodeError();
    } else if (nibbles[3] == 9) {
        // 9xy0
        if (nibbles[0] == 0) {
            throw UnimplementedInstructionError("SNE");
            // return std::string("SNE V") + nibble_to_hex(nibbles[2]) + ", V" +
            //     nibble_to_hex(nibbles[1]);
        }

        throw BadOpcodeError();
    } else if (nibbles[3] == 0xa) {
        // Annn
        throw UnimplementedInstructionError("LD");
        // return std::string("LD I, ") + nibble_to_hex(nibbles[2]) +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xb) {
        // Bnnn
        throw UnimplementedInstructionError("JP");
        // return std::string("JP V0, ") + nibble_to_hex(nibbles[2]) +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xc) {
        // Cxkk
        throw UnimplementedInstructionError("RND");
        // return std::string("RND V") + nibble_to_hex(nibbles[2]) + ", " +
        //     nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xd) {
        // Dxyn
        throw UnimplementedInstructionError("DRW");
        // return std::string("DRW V") + nibble_to_hex(nibbles[2]) + ", V" +
        //     nibble_to_hex(nibbles[1]) + ", " + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xe) {
        if (nibbles[1] == 0x9 && nibbles[0] == 0xe) {
            // Ex9E
            throw UnimplementedInstructionError("SKP");
            // return std::string("SKP V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] = 0xA && nibbles[0] == 0x1) {
            // EXA1
            throw UnimplementedInstructionError("SKNP");
            // return std::string("SKNP V") + nibble_to_hex(nibbles[2]);
        }
        throw BadOpcodeError();
    } else if (nibbles[3] == 0xf) {
        if (nibbles[1] == 0x0 && nibbles[0] == 0x7) {
            // Fx07
            throw UnimplementedInstructionError("LD");
            // return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", DT";
        } else if (nibbles[1] == 0x0 && nibbles[0] == 0xa) {
            // Fx0A
            throw UnimplementedInstructionError("LD");
            // return std::string("LD V"), + nibble_to_hex(nibbles[2]) + ", K";
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x5) {
            // Fx15
            throw UnimplementedInstructionError("LD");
            // return std::string("LD ST, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x8) {
            // Fx18
            throw UnimplementedInstructionError("LD");
            // return std::string("LD ST, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0xe) {
            // Fx1E
            throw UnimplementedInstructionError("ADD");
            // return std::string("ADD I, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x2 && nibbles[0] == 0x9) {
            // Fx29
            throw UnimplementedInstructionError("LD");
            // return std::string("LD F, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x3 && nibbles[0] == 0x3) {
            // Fx33
            throw UnimplementedInstructionError("LD");
            // return std::string("LD B, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x5 && nibbles[0] == 0x5) {
            // Fx55
            throw UnimplementedInstructionError("LD");
            // return std::string("LD [I], V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x6 && nibbles[0] == 0x5) {
            // Fx65
            throw UnimplementedInstructionError("LD");
            // return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", [I]";
        }

        throw BadOpcodeError();
    }

    throw BadOpcodeError();
}

unsigned Cpu::get_nibble_from_inst(inst_t inst, unsigned idx)
{
    /*
     * Chip8 programs are stored in big-endian format.
     * The format used by this function is:
     * 3210
     * where the left-side (3) is the most-significant nibble and the
     * right-side (0) is the least-significant nibble.
     */
    switch (idx)
    {
    case 0:
        return ntohs(inst) & 0xf;
    case 1:
        return (ntohs(inst) & 0xf0) >> 4;
    case 2:
        return (ntohs(inst) & 0xf00) >> 8;
    case 3:
        return (ntohs(inst) & 0xf000) >> 12;
    }

    throw InvalidParamError("Invalid parameter sent to get_nibble_from_isnt");
}

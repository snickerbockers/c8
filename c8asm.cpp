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

/*
 * This site was used as a reference during the writing of this source file:
 * http://devernay.free.fr/hacks/chip8/C8TECH10.HTM
 */

// for endian-dependency functions
#include <arpa/inet.h>

#include <stdint.h>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <exception>

typedef uint16_t inst_t;

char nibble_to_hex(unsigned nibble) {
    static char const tbl[] = {
        '0', '1', '2', '3',
        '4', '5', '6', '7',
        '8', '9', 'a', 'b',
        'c', 'd', 'e', 'f'
    };

    return tbl[nibble & 0xf];
}

unsigned get_nibble_from_inst(inst_t inst, unsigned idx)
{
    /*
     * Chip8 programs are stored in big-endian format.
     * This function respects that, so the format used by this function is:
     * 3210
     * where the left-side (3) is the most-significant nibble and the
     * right-side (0) is the least-significant nibble.
     *
     * 
     */
    switch (idx)
    {
    case 0:
        return inst & 0xf;
    case 1:
        return (inst & 0xf0) >> 4;
    case 2:
        return (inst & 0xf00) >> 8;
    case 3:
        return (inst & 0xf000) >> 12;
    }

    // TODO: use a real exception class
    throw std::string("Invalid parameter sent to get_nibble_from_isnt");
}

std::string decode_instruction(inst_t inst)
{
    unsigned nibbles[] = {
        get_nibble_from_inst(inst, 0),
        get_nibble_from_inst(inst, 1),
        get_nibble_from_inst(inst, 2),
        get_nibble_from_inst(inst, 3)
    };

    if (nibbles[3] == 0) {
        if (nibbles[2] == 0) {
            if (nibbles[1] == 0xE) {
                if (nibbles[0] == 0) {
                    return std::string("CLS"); // 00E0
                } else if (nibbles[0] == 0xE) {
                    return std::string("RET"); // 00EE
                }
            }
        }
        // 0NNN - this will never be implemented
        return std::string("SYS ") + nibble_to_hex(nibbles[2]) +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 1) {
        // 1NNN
        return std::string("JP ") + nibble_to_hex(nibbles[2]) +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 2) {
        // 2NNN
        return std::string("CALL ") + nibble_to_hex(nibbles[2]) +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 3) {
        // 3xkk
        return std::string("SE V") + nibble_to_hex(nibbles[2]) + ", " +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 4) {
        // 4xkk
        return std::string("SNE V") + nibble_to_hex(nibbles[2]) + ", " +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 5) {
        if (nibbles[0] == 0) {
            // 5xy0
            return std::string("SE V") + nibble_to_hex(nibbles[2]) + ", V" +
                               nibble_to_hex(nibbles[1]);
        } else {
            return "";
        }
    } else if (nibbles[3] == 6) {
        // 6xkk
        return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", " +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 7) {
        // 7xkk
        return std::string("ADD V") + nibble_to_hex(nibbles[2]) + ", " +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 8) {
        if (nibbles[0] == 0) {
            // 8xy0
            return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 1) {
            // 8xy1
            return std::string("OR V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 2) {
            // 8xy2
            return std::string("AND V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 3) {
            // 8xy3
            return std::string("XOR V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 4) {
            // 8xy4
            return std::string("ADD V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 5) {
            // 8xy5
            return std::string("SUB V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 6) {
            // 8xy6
            return std::string("SHR V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[0] == 7) {
            // 8xy7
            return std::string("SUBN V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        } else if (nibbles[0] == 0xe) {
            // 8xye
            return std::string("SHL V") + nibble_to_hex(nibbles[2]);
        }

        return "";
    } else if (nibbles[3] == 9) {
        // 9xy0
        if (nibbles[0] == 0) {
            return std::string("SNE V") + nibble_to_hex(nibbles[2]) + ", V" +
                nibble_to_hex(nibbles[1]);
        }

        return "";
    } else if (nibbles[3] == 0xa) {
        // Annn
        return std::string("LD I, ") + nibble_to_hex(nibbles[2]) +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xb) {
        // Bnnn
        return std::string("JP V0, ") + nibble_to_hex(nibbles[2]) +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xc) {
        // Cxkk
        return std::string("RND V") + nibble_to_hex(nibbles[2]) + ", " +
            nibble_to_hex(nibbles[1]) + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xd) {
        // Dxyn
        return std::string("DRW V") + nibble_to_hex(nibbles[2]) + ", V" +
            nibble_to_hex(nibbles[1]) + ", " + nibble_to_hex(nibbles[0]);
    } else if (nibbles[3] == 0xe) {
        if (nibbles[1] == 0x9 && nibbles[0] == 0xe) {
            // Ex9E
            return std::string("SKP V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0xA && nibbles[0] == 0x1) {
            // EXA1
            return std::string("SKNP V") + nibble_to_hex(nibbles[2]);
        }
        return "";
    } else if (nibbles[3] == 0xf) {
        if (nibbles[1] == 0x0 && nibbles[0] == 0x7) {
            // Fx07
            return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", DT";
        } else if (nibbles[1] == 0x0 && nibbles[0] == 0xa) {
            // Fx0A
            return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", K";
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x5) {
            // Fx15
            return std::string("LD DT, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x8) {
            // Fx18
            return std::string("LD ST, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0xe) {
            // Fx1E
            return std::string("ADD I, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x2 && nibbles[0] == 0x9) {
            // Fx29
            return std::string("LD F, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x3 && nibbles[0] == 0x3) {
            // Fx33
            return std::string("LD B, V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x5 && nibbles[0] == 0x5) {
            // Fx55
            return std::string("LD [I], V") + nibble_to_hex(nibbles[2]);
        } else if (nibbles[1] == 0x6 && nibbles[0] == 0x5) {
            // Fx65
            return std::string("LD V") + nibble_to_hex(nibbles[2]) + ", [I]";
        }

        return "";
    }

    return "";
}

void read_instruction(std::fstream *stream)
{
    inst_t inst;
    static unsigned offset = 0x200;
    uint8_t inst_hi, inst_lo;

    stream->read((char*)&inst_hi, sizeof(inst_hi));
    stream->read((char*)&inst_lo, sizeof(inst_lo));
    inst = unsigned(inst_hi) << 8 | inst_lo;
    std::cout << std::hex << offset << ": " << decode_instruction(inst) << "\t;; " <<
        std::setfill('0') << std::setw(4) << std::hex << unsigned(inst) << std::endl;
    offset += 2;
}

int main(int argc, char **argv)
{
    std::fstream stream(argv[1], std::fstream::in | std::fstream::binary);
    //stream.seekg(512);

    while (stream.good())
        read_instruction(&stream);

    return 0;
}

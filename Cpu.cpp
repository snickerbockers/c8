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

#include <arpa/inet.h>
#include <cstring>
#include <cstdlib>

#include "BaseException.h"

#include "Cpu.h"

Cpu::Cpu(Memory *mem, Screen *screen, Keyboard *kbd) {
    memset(v, 0, sizeof(v));
    memset(stack, 0, sizeof(stack));
    tim = snd = 0;
    sp = 0;
    pc = ROM_START_ADDR;
    reg_i = 0;
    key_irq_active = false;

    this->mem = mem;
    this->screen = screen;
    this->kbd = kbd;
}

void Cpu::int_tim(void) {
    if (tim)
        tim--;
    if (snd)
        snd--;
}

void Cpu::int_key(int which_key) {
    if (key_irq_active) {
        key_irq = true;
        key_irq_which = which_key;
    }
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
                if (nibbles[0] == 0) {
                    //00E0
                    inst_cls();
                    return;
                } else if (nibbles[0] == 0xE) {
                    //00EE
                    inst_ret();
                    return;
                }
            }
        }
        // 0NNN - this will never be implemented
        throw UnimplementedInstructionError("SYS");
    } else if (nibbles[3] == 1) {
        // 1NNN
        inst_jp(get_addr_from_inst(inst));
        return;
    } else if (nibbles[3] == 2) {
        // 2NNN
        inst_call(get_addr_from_inst(inst));
        return;
    } else if (nibbles[3] == 3) {
        // 3xkk
        inst_se_reg_val(nibbles[2], get_low_byte_from_inst(inst));
        return;
    } else if (nibbles[3] == 4) {
        // 4xkk
        inst_sne_reg_val(nibbles[2], get_low_byte_from_inst(inst));
        return;
    } else if (nibbles[3] == 5) {
        if (nibbles[0] == 0) {
            // 5xy0
            inst_se_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else {
            throw BadOpcodeError();
        }
    } else if (nibbles[3] == 6) {
        // 6xkk
        inst_ld_reg_val(nibbles[2], get_low_byte_from_inst(inst));
        return;
    } else if (nibbles[3] == 7) {
        // 7xkk
        inst_add_reg_val(nibbles[2], get_low_byte_from_inst(inst));
        return;
    } else if (nibbles[3] == 8) {
        if (nibbles[0] == 0) {
            // 8xy0
            inst_ld_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 1) {
            // 8xy1
            inst_or_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 2) {
            // 8xy2
            inst_and_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 3) {
            // 8xy3
            inst_xor_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 4) {
            // 8xy4
            inst_add_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 5) {
            // 8xy5
            inst_sub_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 6) {
            // 8xy6
            inst_shr_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 7) {
            // 8xy7
            inst_subn_reg_reg(nibbles[2], nibbles[1]);
            return;
        } else if (nibbles[0] == 0xe) {
            // 8xye
            inst_shl_reg_reg(nibbles[2], nibbles[1]);
            return;
        }

        throw BadOpcodeError();
    } else if (nibbles[3] == 9) {
        // 9xy0
        if (nibbles[0] == 0) {
            inst_sne_reg_reg(nibbles[2], nibbles[1]);
            return;
        }

        throw BadOpcodeError();
    } else if (nibbles[3] == 0xa) {
        // Annn
        inst_ld_i(get_addr_from_inst(inst));
        return;
    } else if (nibbles[3] == 0xb) {
        // Bnnn
        inst_jp_offset(get_addr_from_inst(inst));
        return;
    } else if (nibbles[3] == 0xc) {
        // Cxkk
        inst_rnd(nibbles[2], get_low_byte_from_inst(inst));
        return;
    } else if (nibbles[3] == 0xd) {
        // Dxyn
        inst_drw(nibbles[2], nibbles[1], nibbles[0]);
        return;
    } else if (nibbles[3] == 0xe) {
        if (nibbles[1] == 0x9 && nibbles[0] == 0xe) {
            // Ex9E
            inst_skp_key(nibbles[2]);
            return;
        } else if (nibbles[1] = 0xA && nibbles[0] == 0x1) {
            // EXA1
            inst_sknp_key(nibbles[2]);
            return;
        }
        throw BadOpcodeError();
    } else if (nibbles[3] == 0xf) {
        if (nibbles[1] == 0x0 && nibbles[0] == 0x7) {
            // Fx07
            inst_ld_reg_tim(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x0 && nibbles[0] == 0xa) {
            // Fx0A
            inst_ld_key(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x5) {
            // Fx15
            inst_ld_tim_reg(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0x8) {
            // Fx18
            inst_ld_snd_reg(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x1 && nibbles[0] == 0xe) {
            // Fx1E
            inst_add_i_reg(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x2 && nibbles[0] == 0x9) {
            // Fx29
            inst_ld_i_hex(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x3 && nibbles[0] == 0x3) {
            // Fx33
            inst_ld_bcd(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x5 && nibbles[0] == 0x5) {
            // Fx55
            inst_ld_push_regs(nibbles[2]);
            return;
        } else if (nibbles[1] == 0x6 && nibbles[0] == 0x5) {
            // Fx65
            inst_ld_pop_regs(nibbles[2]);
            return;
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
        return inst & 0xf;
    case 1:
        return (inst & 0xf0) >> 4;
    case 2:
        return (inst & 0xf00) >> 8;
    case 3:
        return (inst & 0xf000) >> 12;
    }

    throw InvalidParamError("Invalid parameter sent to get_nibble_from_isnt");
}

unsigned Cpu::get_addr_from_inst(inst_t inst) {
    return ntohs(inst & 0xfff);
}

unsigned Cpu::get_low_byte_from_inst(inst_t inst) {
    return inst & 0xff;
}

// CLS - clear screen
void Cpu::inst_cls(void) {
    screen->clear();
    pc += 2;
}

// RET - return from a subroutine
void Cpu::inst_ret(void) {
    if (sp == 0)
        throw StackUnderflowError();

    pc = stack[sp--];
}

// JP - jump to address
void Cpu::inst_jp(unsigned where_to) {
    pc = where_to;
}

// CALL - call subroutine
void Cpu::inst_call(unsigned where_to) {
    if (sp >= STACK_SZ - 1)
        throw StackOverflowError();

    /*
     * XXX stack[0] will never get used because the call instruction increments
     * the stack pointer before setting its value to the pc.  This seems weird,
     * but I do believe this is how it is supposed to behave based on what I've
     * read at Cowgod's site
     */
    stack[++sp] = pc;
    pc = where_to;
}

//SE - Skip if Equal
void Cpu::inst_se_reg_val(unsigned reg_no, unsigned val) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    if (v[reg_no] == val)
        pc += 4;
    else
        pc += 2;
}

//SNE - Skip if Not Equal
void Cpu::inst_sne_reg_val(unsigned reg_no, unsigned val) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    if (v[reg_no] != val)
        pc += 4;
    else
        pc += 2;
}

//SE - Skip if Equal
void Cpu::inst_se_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    if (v[reg1] == v[reg2])
        pc += 4;
    else
        pc += 2;
}

// LD - Load value
void Cpu::inst_ld_reg_val(unsigned reg_no, unsigned val) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = val;
    pc += 2;
}

// ADD - add value into register
void Cpu::inst_add_reg_val(unsigned reg_no, unsigned val) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    // XXX Strangely enough, Cowgod's site doesn't say anything about setting
    //     VF for this variant of the add instruction

    v[reg_no] += val;
    pc += 2;
}

// LD - Load value
void Cpu::inst_ld_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] = v[reg2];
    pc += 2;
}

// OR - Bitwise-OR values
void Cpu::inst_or_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] |= v[reg2];
    pc += 2;
}

// AND - Bitwise-AND values
void Cpu::inst_and_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] &= v[reg2];
    pc += 2;
}

// XOR - Bitwise-XOR values
void Cpu::inst_xor_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] ^= v[reg2];
    pc += 2;
}

// ADD - Add two values
void Cpu::inst_add_reg_reg(unsigned reg1, unsigned reg2) {
    unsigned tmp;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    tmp = v[reg1] + v[reg2];
    if (tmp > 0xff)
        v[0xf] = 1;

    v[reg1] = tmp;
    pc += 2;
}

// SUB - Subtract two values
void Cpu::inst_sub_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[0xf] = v[reg1] > v[reg2];

    v[reg1] -= v[reg2];
    pc += 2;
}

// SHR - shift right
void Cpu::inst_shr_reg_reg(unsigned reg1, unsigned reg2) {
    // The instruction takes two parameters even though it only uses one.
    // This function also takes two parameters to reflect that.
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[0xf] = v[reg1] & 1;

    // XXX I'm assuming this is meant to be logical and not arithmatic
    v[reg1] >>= 1;
    pc += 2;
}

// SUBN - it's like SUB but backwards
void Cpu::inst_subn_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[0xf] = v[reg2] > v[reg1];
    v[reg1] = v[reg2] - v[reg1];
    pc += 2;
}

// SHL - shift left
void Cpu::inst_shl_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[0xf] = v[reg1] & 0x80;
    v[reg1] <<= 1;
    pc += 2;
}

//SNE - Skip if Not Equal
void Cpu::inst_sne_reg_reg(unsigned reg1, unsigned reg2) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    if (v[reg1] != v[reg2])
        pc += 4;
    else
        pc += 2;
}

// LD - load value into I
void Cpu::inst_ld_i(unsigned addr) {
    reg_i = addr;
    pc += 2;
}

// JP - jump to v0 + some offset
void Cpu::inst_jp_offset(unsigned addr) {
    pc += v[0] + addr;
}

// RND - random number generation
void Cpu::inst_rnd(unsigned reg_no, unsigned val) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = rand() & 0xff & val;
    pc += 2;
}

// DRW - draw a sprite on the screen
void Cpu::inst_drw(unsigned reg1, unsigned reg2, unsigned n_bytes) {
    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    int new_vf = 0;
    int pix_x = v[reg1];
    int pix_y = v[reg2];

    for (unsigned row = 0; row < n_bytes; row++) {
        int src_row = mem->read8(reg_i + row);
        for (unsigned col = 0; col < 8; col++) {
            int orig_pix = screen->get_pixel(col + pix_x, row + pix_y);
            int src_pix = src_row & (1 << col);
            int new_pix = orig_pix != src_pix;
            if (!new_pix && orig_pix) {
                new_vf = 1;
            }
            screen->set_pixel(col + pix_x, row + pix_y, new_pix);
        }
    }

    v[0xf] = new_vf;
    pc += 2;
}

// SKP - skip if the given key is pressed
void Cpu::inst_skp_key(unsigned reg_no) {
    int key;

    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    key = v[reg_no];

    if (kbd->get_key_state(key))
        pc += 4;
    else
        pc += 2;
}

// SKNP - skip if the given key is not pressed
void Cpu::inst_sknp_key(unsigned reg_no) {
    int key;

    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    key = v[reg_no];

    if (!kbd->get_key_state(key))
        pc += 4;
    else
        pc += 2;
}

// LD - load value from timer register
void Cpu::inst_ld_reg_tim(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = tim;

    pc += 2;
}

// LD - wait for key press then load key into register
void Cpu::inst_ld_key(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    if (key_irq_active && key_irq) {
        key_irq_active = key_irq = false;
        v[reg_no] = key_irq_which;
        pc += 2;
    } else {
        key_irq_active = true;
    }
}

// LD - load general-purpose register into tim
void Cpu::inst_ld_tim_reg(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    tim = v[reg_no];

    pc += 2;
}

// LD - load general-purpose register into snd
void Cpu::inst_ld_snd_reg(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    snd = v[reg_no];

    pc += 2;
}

// ADD - add general-purpose register into I
void Cpu::inst_add_i_reg(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    reg_i += v[reg_no];

    pc += 2;
}

// LD - load hex-value speicifed by register into I
void Cpu::inst_ld_i_hex(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    reg_i = Memory::HEX_SPRITE_START + v[reg_no] * 5;

    pc += 2;
}

// LD - convert register into 3-digit bcd number with
//      digits at [I], [I+1], [I+2]
void Cpu::inst_ld_bcd(unsigned reg_no) {
    unsigned val = 0;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    val = v[reg_no];
    mem->write8(reg_i, val / 100);
    val %= 100;
    mem->write8(reg_i + 1, val / 10);
    val %= 10;
    mem->write8(reg_i + 2, val);

    pc += 2;
}

// LD - copy all regs up to and including the specified
//      register to memory starting at [I]
void Cpu::inst_ld_push_regs(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    for (unsigned idx = 0; idx <= reg_no; idx++) {
        mem->write8(reg_i + idx, v[reg_no]);
    }

    pc += 2;
}

// LD - copy into all regs up to and including the specified
//      register from memory starting at [I]
void Cpu::inst_ld_pop_regs(unsigned reg_no) {
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    for (unsigned idx = 0; idx <= reg_no; idx++) {
        v[reg_no] = mem->read8(reg_i + idx);
    }

    pc += 2;
}

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
#include <cstdlib>
#include <iostream>

#include "BaseException.h"
#include "Speaker.h"

#include "Cpu.h"

Cpu::Cpu(Memory *mem, Screen *screen, Keyboard *kbd, Speaker *speaker) {
    memset(v, 0, sizeof(v));
    memset(stack, 0, sizeof(stack));
    tim = snd = 0;
    sp = 0;
    pc = ROM_START_ADDR;
    reg_i = 0;
    key_irq_active = false;
    key_irq = false;

    this->mem = mem;
    this->screen = screen;
    this->kbd = kbd;
    this->speaker = speaker;

    set_breakpoint(-1);
}

void Cpu::int_tim(void) {
    if (tim)
        tim--;
    if (snd)
        snd--;
    if (!snd)
        speaker->stop();
}

void Cpu::int_key(int which_key) {
    if (key_irq_active) {
        key_irq = true;
        key_irq_which = which_key;
    }
}

void Cpu::set_breakpoint(int bp) {
    this->breakpoint = bp;
}

int Cpu::get_breakpoint() const {
    return breakpoint;
}

unsigned Cpu::get_pc() const {
    return pc;
}

void Cpu::print_regs() const {
    for (int idx = 0; idx < REG_COUNT; idx++)
        printf("v%d == %02x\n", idx, v[idx]);

    printf("sp == %04x\n", sp);
    printf("tim == %02x\n", tim);
    printf("snd == %02x\n", snd);
}

const struct Cpu::opcode Cpu::opcodes[] = {
    { "00E0", "CLS",  &Cpu::inst_cls,          ARG_NONE },
    { "00EE", "RET",  &Cpu::inst_ret,          ARG_NONE },
    { "0nnn", "SYS",  &Cpu::inst_sys,          ARG_ADDR },
    { "1nnn", "JP",   &Cpu::inst_jp,           ARG_ADDR },
    { "2nnn", "CALL", &Cpu::inst_call,         ARG_ADDR },
    { "3xkk", "SE",   &Cpu::inst_se_reg_val,   ARG_REG_VAL },
    { "4xkk", "SNE",  &Cpu::inst_sne_reg_val,  ARG_REG_VAL },
    { "5xy0", "SE",   &Cpu::inst_se_reg_reg,   ARG_REG_REG },
    { "6xkk", "LD",   &Cpu::inst_ld_reg_val,   ARG_REG_VAL },
    { "7xkk", "ADD",  &Cpu::inst_add_reg_val,  ARG_REG_VAL },
    { "8xy0", "LD",   &Cpu::inst_ld_reg_reg,   ARG_REG_REG },
    { "8xy1", "OR",   &Cpu::inst_or_reg_reg,   ARG_REG_REG },
    { "8xy2", "AND",  &Cpu::inst_and_reg_reg,  ARG_REG_REG },
    { "8xy3", "XOR",  &Cpu::inst_xor_reg_reg,  ARG_REG_REG },
    { "8xy4", "ADD",  &Cpu::inst_add_reg_reg,  ARG_REG_REG },
    { "8xy5", "SUB",  &Cpu::inst_sub_reg_reg,  ARG_REG_REG },
    { "8xy6", "SHR",  &Cpu::inst_shr_reg_reg,  ARG_REG_REG },
    { "8xy7", "SUBN", &Cpu::inst_subn_reg_reg, ARG_REG_REG },
    { "8xyE", "SHL",  &Cpu::inst_shl_reg_reg,  ARG_REG_REG },
    { "9xy0", "SNE",  &Cpu::inst_sne_reg_reg,  ARG_REG_REG },
    { "Annn", "LD",   &Cpu::inst_ld_i,         ARG_ADDR },
    { "Bnnn", "JP",   &Cpu::inst_jp_offset,    ARG_ADDR },
    { "Cxkk", "RND",  &Cpu::inst_rnd,          ARG_REG_VAL },
    { "Dxyn", "DRW",  &Cpu::inst_drw,          ARG_DRW },
    { "Ex9E", "SKP",  &Cpu::inst_skp_key,      ARG_REG },
    { "ExA1", "SKNP", &Cpu::inst_sknp_key,     ARG_REG },
    { "Fx07", "LD",   &Cpu::inst_ld_reg_tim,   ARG_REG },
    { "Fx0A", "LD",   &Cpu::inst_ld_key,       ARG_REG },
    { "Fx15", "LD",   &Cpu::inst_ld_tim_reg,   ARG_REG },
    { "Fx18", "LD",   &Cpu::inst_ld_snd_reg,   ARG_REG },
    { "Fx1E", "ADD",  &Cpu::inst_add_i_reg,    ARG_REG },
    { "Fx29", "LD",   &Cpu::inst_ld_i_hex,     ARG_REG },
    { "Fx33", "LD",   &Cpu::inst_ld_bcd,       ARG_REG },
    { "Fx55", "LD",   &Cpu::inst_ld_push_regs, ARG_REG },
    { "Fx65", "LD",   &Cpu::inst_ld_pop_regs,  ARG_REG },
    { NULL, NULL, NULL }
};

struct Cpu::opcode const *Cpu::decode_inst(Cpu::inst_t inst) const {
    struct opcode const *curs = opcodes;

    unsigned nibbles[] = {
        get_nibble_from_inst(inst, 0),
        get_nibble_from_inst(inst, 1),
        get_nibble_from_inst(inst, 2),
        get_nibble_from_inst(inst, 3)
    };

    while (curs->pattern) {
        int idx;
        for (idx = 0; idx < 4; idx++) {
            char c = curs->pattern[idx];
            unsigned val;

            if (c >= '0' && c <= '9')
                val = c - '0';
            else if (c >= 'A' && c <= 'F')
                val = c - 'A' + 0xa;
            else
                continue;

            if (val != nibbles[3 - idx])
                break;
        }

        if (idx == 4) {
            return curs;
        }
        curs++;
    }

    return NULL;
}

bool Cpu::next_inst(void) {
    inst_t inst;
    union inst_args args;
    struct opcode const *opcode;

    if (pc == breakpoint)
        return true;

    inst = mem->read16(pc);
    unsigned nibbles[] = {
        get_nibble_from_inst(inst, 0),
        get_nibble_from_inst(inst, 1),
        get_nibble_from_inst(inst, 2),
        get_nibble_from_inst(inst, 3)
    };

    opcode = decode_inst(inst);
    if (!opcode)
        throw BadOpcodeError();

    // std::cout << std::hex << inst << ": " << opcode->pattern << std::endl;

    switch (opcode->arg_type) {
    case ARG_NONE:
        break;
    case ARG_REG_VAL:
        args.reg_val.reg_no = nibbles[2];
        args.reg_val.val = get_low_byte_from_inst(inst);
        break;
    case ARG_REG_REG:
        args.reg_reg.reg1 = nibbles[2];
        args.reg_reg.reg2 = nibbles[1];
        break;
    case ARG_REG:
        args.reg.reg_no = nibbles[2];
        break;
    case ARG_ADDR:
        args.addr.addr = get_addr_from_inst(inst);
        break;
    case ARG_DRW:
        args.drw.reg1 = nibbles[2];
        args.drw.reg2 = nibbles[1];
        args.drw.n_bytes = nibbles[0];
        break;
    default:
        throw BadOpcodeError(); // should never happen
    }

    (this->*opcode->func)(&args);

    return false;
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
    return inst & 0xfff;
}

unsigned Cpu::get_low_byte_from_inst(inst_t inst) {
    return inst & 0xff;
}

// CLS - clear screen
void Cpu::inst_cls(union inst_args const *args) {
    screen->clear();
    pc += 2;
}

// RET - return from a subroutine
void Cpu::inst_ret(union inst_args const *args) {
    if (sp == 0)
        throw StackUnderflowError();

    pc = stack[sp--];
}

// SYS - execute native machine code (unimplemented)
void Cpu::inst_sys(union inst_args const *args) {
    throw UnimplementedInstructionError("SYS");
}

// JP - jump to address
void Cpu::inst_jp(union inst_args const *args) {
    unsigned addr = args->addr.addr;
    pc = addr;
}

// CALL - call subroutine
void Cpu::inst_call(union inst_args const *args) {
    unsigned addr = args->addr.addr;

    if (sp >= STACK_SZ - 1)
        throw StackOverflowError();

    /*
     * XXX stack[0] will never get used because the call instruction increments
     * the stack pointer before setting its value to the pc.  This seems weird,
     * but I do believe this is how it is supposed to behave based on what I've
     * read at Cowgod's site
     */
    stack[++sp] = pc + 2;
    pc = addr;
}

//SE - Skip if Equal
void Cpu::inst_se_reg_val(union inst_args const *args) {
    unsigned reg_no = args->reg_val.reg_no;
    unsigned val = args->reg_val.val;

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
void Cpu::inst_sne_reg_val(union inst_args const *args) {
    unsigned reg_no = args->reg_val.reg_no;
    unsigned val = args->reg_val.val;

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
void Cpu::inst_se_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_ld_reg_val(union inst_args const *args) {
    unsigned reg_no = args->reg_val.reg_no;
    unsigned val = args->reg_val.val;

    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = val;
    pc += 2;
}

// ADD - add value into register
void Cpu::inst_add_reg_val(union inst_args const *args) {
    unsigned reg_no = args->reg_val.reg_no;
    unsigned val = args->reg_val.val;

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
void Cpu::inst_ld_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] = v[reg2];
    pc += 2;
}

// OR - Bitwise-OR values
void Cpu::inst_or_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] |= v[reg2];
    pc += 2;
}

// AND - Bitwise-AND values
void Cpu::inst_and_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] &= v[reg2];
    pc += 2;
}

// XOR - Bitwise-XOR values
void Cpu::inst_xor_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg1] ^= v[reg2];
    pc += 2;
}

// ADD - Add two values
void Cpu::inst_add_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;
    unsigned tmp;

    if (reg1 >= REG_COUNT || reg2 >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    tmp = v[reg1] + v[reg2];
    v[0xf] = (tmp > 0xff);

    v[reg1] = tmp;
    pc += 2;
}

// SUB - Subtract two values
void Cpu::inst_sub_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_shr_reg_reg(union inst_args const *args) {
    // The instruction takes two parameters even though it only uses one.
    // This function also takes two parameters to reflect that.
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_subn_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_shl_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_sne_reg_reg(union inst_args const *args) {
    unsigned reg1 = args->reg_reg.reg1;
    unsigned reg2 = args->reg_reg.reg2;

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
void Cpu::inst_ld_i(union inst_args const *args) {
    unsigned addr = args->addr.addr;

    reg_i = addr;
    pc += 2;
}

// JP - jump to v0 + some offset
void Cpu::inst_jp_offset(union inst_args const *args) {
    unsigned addr = args->addr.addr;

    pc += v[0] + addr;
}

// RND - random number generation
void Cpu::inst_rnd(union inst_args const *args) {
    unsigned reg_no = args->reg_val.reg_no;
    unsigned val = args->reg_val.val;

    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = rand() & 0xff & val;
    pc += 2;
}

// DRW - draw a sprite on the screen
void Cpu::inst_drw(union inst_args const *args) {
    unsigned reg1 = args->drw.reg1;
    unsigned reg2 = args->drw.reg2;
    unsigned n_bytes = args->drw.n_bytes;

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
            int src_pix = src_row & (0x80 >> col) ? 1 : 0;
            int new_pix = (orig_pix != src_pix);

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
void Cpu::inst_skp_key(union inst_args const *args) {
    int key;
    unsigned reg_no = args->reg.reg_no;

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
void Cpu::inst_sknp_key(union inst_args const *args) {
    int key;
    unsigned reg_no = args->reg.reg_no;

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
void Cpu::inst_ld_reg_tim(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    v[reg_no] = tim;

    pc += 2;
}

// LD - wait for key press then load key into register
void Cpu::inst_ld_key(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
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
void Cpu::inst_ld_tim_reg(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    tim = v[reg_no];

    pc += 2;
}

// LD - load general-purpose register into snd
void Cpu::inst_ld_snd_reg(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    snd = v[reg_no];
    if (snd)
        speaker->start();
    else
        speaker->stop();

    pc += 2;
}

// ADD - add general-purpose register into I
void Cpu::inst_add_i_reg(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    reg_i += v[reg_no];

    pc += 2;
}

// LD - load hex-value speicifed by register into I
void Cpu::inst_ld_i_hex(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
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
void Cpu::inst_ld_bcd(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
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
void Cpu::inst_ld_push_regs(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    for (unsigned idx = 0; idx <= reg_no; idx++) {
        mem->write8(reg_i + idx, v[idx]);
    }

    pc += 2;
}

// LD - copy into all regs up to and including the specified
//      register from memory starting at [I]
void Cpu::inst_ld_pop_regs(union inst_args const *args) {
    unsigned reg_no = args->reg.reg_no;
    if (reg_no >= REG_COUNT) {
        // this should be impossible because the registers are read from 4-bit
        // values, but I check anyways.
        throw InvalidRegisterError();
    }

    for (unsigned idx = 0; idx <= reg_no; idx++) {
        v[idx] = mem->read8(reg_i + idx);
    }

    pc += 2;
}

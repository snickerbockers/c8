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

#ifndef CPU_H_
#define CPU_H_

#include <stdint.h>

#include "Memory.h"
#include "Screen.h"
#include "Keyboard.h"

class Cpu {
public:
    const static int ROM_START_ADDR = 0x200;
    static const int REG_COUNT = 16;
    static const int STACK_SZ = 16;

    Cpu(Memory *mem, Screen *screen, Keyboard *kbd);

    /*
     * Timer interrupts.  These don't go to the software, but they do signal to
     * the CPU to decrement tim and snd.
     */
    void int_tim(void);

    // Key interrupt.  Called whenever a key is pressed for inst_ld_key's sake.
    void int_key(int which_key);

    // returns true if we hit the breakpoint
    bool next_inst(void);

    // set to < 0 to disable
    void set_breakpoint(int bp);
    int get_breakpoint() const;

    unsigned get_pc() const;

private:
    typedef uint16_t inst_t;

    Memory *mem;
    Screen *screen;
    Keyboard *kbd;

    uint8_t v[REG_COUNT];     // general-purpose registers
    uint16_t reg_i;           // the address register
    uint8_t tim, snd;         // timer and sound registers
    int pc;                   // program counter
    uint16_t stack[STACK_SZ]; // the stack
    unsigned sp;              // stack pointer

    int breakpoint;           // if >= 0, this pauses execution for debugging

    bool key_irq_active;
    bool key_irq;
    int key_irq_which;

    static unsigned get_nibble_from_inst(inst_t inst, unsigned idx);
    static unsigned get_addr_from_inst(inst_t inst);
    static unsigned get_low_byte_from_inst(inst_t inst);

    void inst_cls(void);
    void inst_ret(void);
    void inst_jp(unsigned where_to);
    void inst_call(unsigned where_to);
    void inst_se_reg_val(unsigned reg_no, unsigned val);
    void inst_sne_reg_val(unsigned reg_no, unsigned val);
    void inst_se_reg_reg(unsigned reg1, unsigned reg2);
    void inst_ld_reg_val(unsigned reg_no, unsigned val);
    void inst_add_reg_val(unsigned reg_no, unsigned val);
    void inst_ld_reg_reg(unsigned reg1, unsigned reg2);
    void inst_or_reg_reg(unsigned reg1, unsigned reg2);
    void inst_and_reg_reg(unsigned reg1, unsigned reg2);
    void inst_xor_reg_reg(unsigned reg1, unsigned reg2);
    void inst_add_reg_reg(unsigned reg1, unsigned reg2);
    void inst_sub_reg_reg(unsigned reg1, unsigned reg2);
    void inst_shr_reg_reg(unsigned reg1, unsigned reg2);
    void inst_subn_reg_reg(unsigned reg1, unsigned reg2);
    void inst_shl_reg_reg(unsigned reg1, unsigned reg2);
    void inst_sne_reg_reg(unsigned reg1, unsigned reg2);
    void inst_ld_i(unsigned addr);
    void inst_jp_offset(unsigned addr);
    void inst_rnd(unsigned reg_no, unsigned val);
    void inst_drw(unsigned reg1, unsigned reg2, unsigned n_bytes);
    void inst_skp_key(unsigned reg_no);
    void inst_sknp_key(unsigned reg_no);
    void inst_ld_reg_tim(unsigned reg_no);
    void inst_ld_key(unsigned reg_no);
    void inst_ld_tim_reg(unsigned reg_no);
    void inst_ld_snd_reg(unsigned reg_no);
    void inst_add_i_reg(unsigned reg_no);
    void inst_ld_i_hex(unsigned reg_no);
    void inst_ld_bcd(unsigned reg_no);
    void inst_ld_push_regs(unsigned reg_no);
    void inst_ld_pop_regs(unsigned reg_no);
};

#endif

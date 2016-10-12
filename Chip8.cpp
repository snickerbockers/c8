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

#include <stdint.h>

#include <iostream>
#include <fstream>
#include <iterator>

#include "Chip8.h"

Chip8::Chip8(bool allow_unaligned) : mem(allow_unaligned), screen(),
                                     cpu(&mem, &screen, &kbd),
                                     kbd(this) {
    mpi = 1000 / DEFAULT_IPS;

    screen.set_bg_color(Screen::pack_color(0x45, 0x19, 0x10, 0xff));
    screen.set_fg_color(Screen::pack_color(0x8c, 0x89, 0x83, 0xff));

    screen.clear();
}

void Chip8::main_loop() {
    SDL_Event event;
    int is_running = 1;
    unsigned cur_ticks, prev_ticks, delta_ticks;
    unsigned last_int_tim_ticks;
    unsigned accum_ticks = 0;

    last_int_tim_ticks = cur_ticks = prev_ticks = SDL_GetTicks();

    while (is_running) {
        prev_ticks = cur_ticks;
        cur_ticks = SDL_GetTicks();
        delta_ticks = cur_ticks - prev_ticks;
        accum_ticks += delta_ticks;

        if ((cur_ticks - last_int_tim_ticks) >= (1000.0 / 60.0)) {
            last_int_tim_ticks = cur_ticks;
            cpu.int_tim();
        }

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                is_running = 0;
            else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
                kbd.handle_key_event(&event.key);
        }

        while (accum_ticks >= mpi) {
            if (cpu.next_inst()) {
                char choice;
                do {
                    cpu.set_breakpoint(-1);

                    std::cout << std::hex << cpu.get_pc() << std::endl;
                    std::cout << "db> ";

                    std::cin >> choice;

                    if (choice == 'n')
                        cpu.next_inst();
                } while (choice != 'c');

                /*
                 * Don't let breakpoints accumulate too much frame time.
                 * This works by nuking it down so that delta_ticks will almost
                 * certainly be zero next frame, but there are bound to be
                 * inaccuracies when you suspend execution like this.  Some sort
                 * of virtual timing system may be possible, but it's not worth
                 * the effort on a CHIP-8 emulator's debugger.
                 */
                cur_ticks = SDL_GetTicks();
            }

            accum_ticks -= mpi;
        }

        screen.flip();
    }
}

void Chip8::load_rom(char const *path)
{
    unsigned addr = Cpu::ROM_START_ADDR;
    unsigned bytes_read = 0;

    SDL_RWops *fp = SDL_RWFromFile(path, "rb");
    while (bytes_read < SDL_RWsize(fp)) {
        mem.write8(addr++, SDL_ReadU8(fp));
        bytes_read++;
    }
    SDL_RWclose(fp);

    std::cout << bytes_read << " bytes read" << std::endl;
}

void Chip8::int_key(int which_key) {
    cpu.int_key(which_key);
}

void Chip8::set_breakpoint(int bp) {
    cpu.set_breakpoint(bp);
}

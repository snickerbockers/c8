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

#include <SDL2/SDL.h>

#include <cstring>

#include "BaseException.h"
#include "Chip8.h"

#include "Keyboard.h"

Keyboard::Keyboard(Chip8 *c8) {
    this->c8 = c8;

    key_states = new bool[SDL_NUM_SCANCODES];
    memset(key_states, 0, sizeof(bool) * SDL_NUM_SCANCODES);

    bind_key(0, SDL_SCANCODE_0);
    bind_key(1, SDL_SCANCODE_1);
    bind_key(2, SDL_SCANCODE_2);
    bind_key(3, SDL_SCANCODE_3);
    bind_key(4, SDL_SCANCODE_4);
    bind_key(5, SDL_SCANCODE_5);
    bind_key(6, SDL_SCANCODE_6);
    bind_key(7, SDL_SCANCODE_7);
    bind_key(8, SDL_SCANCODE_8);
    bind_key(9, SDL_SCANCODE_9);
    bind_key(10, SDL_SCANCODE_A);
    bind_key(11, SDL_SCANCODE_B);
    bind_key(12, SDL_SCANCODE_C);
    bind_key(13, SDL_SCANCODE_D);
    bind_key(14, SDL_SCANCODE_E);
    bind_key(15, SDL_SCANCODE_F);
}

Keyboard::~Keyboard() {
    delete[] key_states;
}

void Keyboard::bind_key(int key, SDL_Scancode bind) {
    if (key < 0 || key >= N_KEYS) {
        throw InvalidParamError("invalid key index");
    }

    binds[key] = bind;
}

bool Keyboard::get_key_state(int key) const {
    if (key < 0 || key >= N_KEYS) {
        throw InvalidParamError("invalid key index");
    }

    return key_states[binds[key]];
}

void Keyboard::handle_key_event(SDL_KeyboardEvent const *event) {
    SDL_Scancode scancode = event->keysym.scancode;

    if (scancode < 0 || scancode >= SDL_NUM_SCANCODES)
        throw InvalidParamError("invalid key index"); // should be inpossible

    key_states[scancode] = (event->state == SDL_PRESSED);

    if (event->state)
        for (int key_no = 0; key_no < N_KEYS; key_no++)
            if (binds[key_no] == scancode)
                c8->int_key(key_no);
}

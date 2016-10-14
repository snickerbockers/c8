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

#include <unistd.h>
#include <iostream>
#include <exception>
#include <cstdlib>
#include <sys/time.h>

#include "Chip8.h"

int main(int argc, char **argv) {
    char optchar;
    int bp = -1;
    bool allow_unaligned = false;
    bool mute = false;

    while ((optchar = getopt(argc, argv, "b:um")) != -1) {
        if (optchar == 'b') {
            bp = atoi(optarg);
        } else if (optchar == 'u') {
            allow_unaligned = true;
        } else if (optchar == 'm') {
            mute = true;
        } else {
            std::cerr << "Usage: " << argv[0] << " rom_path" << std::endl;
            return 1;
        }
    }
    argc -= optind;
    argv += optind;

    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO);

    if (argc != 1) {
        std::cerr << "Usage: " << argv[0] << " rom_path" << std::endl;
        return 1;
    }

    // seed libc's random number generator
    struct timeval tv;
    gettimeofday(&tv, NULL);
    srand(tv.tv_sec);

    Chip8 c8(allow_unaligned, mute);

    try {
        c8.load_rom(argv[0]);
        c8.set_breakpoint(bp);
        c8.main_loop();
    } catch (std::exception err) {
        std::cerr << err.what() << std::endl;
        return 1;
    }

    SDL_Quit();

    return 0;
}

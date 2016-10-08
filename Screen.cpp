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

#include <string.h>

#include "BaseException.h"
#include "Screen.h"

Screen::Screen() {
    SDL_CreateWindowAndRenderer(WIDTH * 10, HEIGHT * 10, 0, &win, &ren);
    screen_tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
                                   SDL_TEXTUREACCESS_STREAMING, WIDTH, HEIGHT);
    if (!screen_tex)
        throw InitError("Failed to init SDL");

    memset(frame_buffer, 0, sizeof(frame_buffer));

    set_bg_color(pack_color(0, 0, 0, 0xff));
    set_fg_color(pack_color(0xff, 0xff, 0xff, 0xff));

    clear();
}

void Screen::flip() {
    SDL_UpdateTexture(screen_tex, NULL, frame_buffer, WIDTH * sizeof(color_t));
    SDL_RenderClear(ren);
    SDL_RenderCopy(ren, screen_tex, NULL, NULL);
    SDL_RenderPresent(ren);
}

void Screen::set_pixel(int pixel_x, int pixel_y, int set) {
    frame_buffer[pixel_y * WIDTH + pixel_x] = pixel_colors[set];
}

Screen::color_t Screen::pack_color(color_t red, color_t green,
                                   color_t blue, color_t alpha) {
    return ((red & 0xff) << RED_SHIFT) | ((green & 0xff) << GREEN_SHIFT) |
        ((blue & 0xff) << BLUE_SHIFT) | ((alpha & 0xff) << ALPHA_SHIFT);
}

void Screen::set_bg_color(Screen::color_t bg) {
    pixel_colors[0] = bg;
}

void Screen::set_fg_color(Screen::color_t fg) {
    pixel_colors[1] = fg;
}

void Screen::clear() {
    unsigned row, col;
    for (row = 0; row < HEIGHT; row++)
        for (col = 0; col < WIDTH; col++)
            set_pixel(col, row, 0);
}

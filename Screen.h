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

class Screen {
public:
    typedef uint32_t color_t;
    
    static const unsigned WIDTH = 64;
    static const unsigned HEIGHT = 32;

    static const unsigned RED_SHIFT = 16;
    static const unsigned GREEN_SHIFT = 8;
    static const unsigned BLUE_SHIFT = 0;
    static const unsigned ALPHA_SHIFT = 24;

    static const color_t RED_MASK = 0xff << RED_SHIFT;
    static const color_t GREEN_MASK = 0xff << GREEN_SHIFT;
    static const color_t BLUE_MASK = 0xff << BLUE_SHIFT;
    static const color_t ALPHA_MASK = 0xff << ALPHA_SHIFT;

    Screen();

    void flip();

    void set_pixel(int pixel_x, int pixel_y, int set);

    static color_t pack_color(color_t red, color_t green,
                              color_t blue, color_t alpha);

    // make sure to call clear after evert set_fg_color or set_bg_color()
    void set_fg_color(color_t fg);
    void set_bg_color(color_t bg);

    void clear();
private:
    SDL_Window *win;
    SDL_Renderer *ren;
    SDL_Texture *screen_tex;
    color_t frame_buffer[WIDTH * HEIGHT];

    // pixel_colors[0] is background color, pixel_colors[1] is foreground color
    color_t pixel_colors[2];
};

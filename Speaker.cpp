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

#include <SDL2/SDL_audio.h>

#include <cmath>
#include <climits>

#include "Speaker.h"

Speaker::Speaker(unsigned freq, double vol_scale) {
    this->freq = freq;
    this->sample_no = 0;
    this->vol_scale = vol_scale;

    SDL_AudioSpec spec, actual_spec;

    spec.freq = SAMP_FREQ;
    spec.format = AUDIO_S16SYS;
    spec.channels = 1;
    spec.silence = 0;
    spec.samples = SAMP_FREQ;
    spec.padding = 0;
    spec.size = 0;
    spec.callback = static_audio_callback;
    spec.userdata = this;

    SDL_OpenAudio(&spec, &actual_spec);
}

void Speaker::static_audio_callback(void *userdata, Uint8 *stream, int len) {
    ((Speaker*)userdata)->audio_callback(stream, len);
}

void Speaker::audio_callback(Uint8 *stream, int len) {
    Sint16 *out = (Sint16*)stream;

    while (len >= 2) {
        *out++ = (Sint16)(sin(2.0 * M_PI * freq *
                              double(sample_no++) / SAMP_FREQ) *
                          vol_scale * SHRT_MAX);

        len -= 2;
    }
}

void Speaker::start(void) {
    SDL_PauseAudio(0);
}

void Speaker::stop(void) {
    SDL_PauseAudio(1);
}

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

#ifndef BASEEXCEPTION_H_
#define BASEEXCEPTION_H_

#include <string>
#include <exception>

class InitError : std::exception {
public:
    InitError(char const *desc) {
        this->desc = desc;
    }

    char const *what() {
        return desc;
    }
private:
    char const *desc;
};

class MemBoundsError : std::exception {
public:
    MemBoundsError(unsigned addr) {
        this->addr = addr;
    }

    char const* what() {
        // TODO: IDK how to put the addr in the what() output without
        //       making an allocation that may throw another exception
        return "Memory access error (bad address)";
    }
private:
    unsigned addr;
};

class MemAlignError : std::exception {
public:
    MemAlignError(unsigned addr) {
        this->addr = addr;
    }
    char const* what() {
        // TODO: IDK how to put the addr in the what() output without
        //       making an allocation that may throw another exception
        return "Memory access error (unaligned 16-bit read or write)";
    }
private:
    unsigned addr;
};

class InvalidParamError : std::exception {
public:
    InvalidParamError(char const *desc) {
        this->desc = desc;
    }

    char const *what() {
        return desc;
    }
private:
    char const *desc;
};

class UnimplementedInstructionError : std::exception {
public:
    UnimplementedInstructionError(char const *inst_name) {
        this->inst_name = inst_name;
    }

    char const *what() {
        return inst_name;
    }
private:
    char const *inst_name;
};

class BadOpcodeError : std::exception {
public:
    char const *what() {
        return "Bad opcode";
    }
};

class StackUnderflowError : std::exception {
public:
    char const *what() {
        return "Stack underflow";
    }
};

class StackOverflowError : std::exception {
public:
    char const *what() {
        return "Stack overflow";
    }
};

class InvalidRegisterError : std::exception {
public:
    char const *what() {
        return "Invalid register";
    }
};

#endif

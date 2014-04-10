/*
Copyright (C) 2013 Ben Dyer

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef EXPORTS_H_
#define EXPORTS_H_

#include "log.h"

enum fcs_log_open_mode_t {
    FCS_MODE_READ = 'r',
    FCS_MODE_WRITE = 'w',
    FCS_MODE_APPEND = 'a',
    FCS_MODE_CLOSED = 'c'
};

void fcs_exports_init(void);

struct fcs_log_t *fcs_exports_log_open(enum fcs_log_type_t type,
enum fcs_log_open_mode_t mode);
struct fcs_log_t *fcs_exports_log_close(struct fcs_log_t *log);

#endif

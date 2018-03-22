## Makefile.am --- Automake file for the rs-rainflow library.

# Copyright (C) 2015 Ralph Schleicher

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in
#      the documentation and/or other materials provided with the
#      distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

## Code:

noinst_LTLIBRARIES = libex.la
noinst_PROGRAMS = ex-basic ex-rainflow ex-reservoir ex-infinite \
ex-read1 ex-read1-short ex-read2 ex-read2-short ex-rainflow-matrix

LDADD = libex.la

libex_la_SOURCES = ex.h ex.c
libex_la_LIBADD = ../librs-rainflow.la

TESTS = t-infinite.sh t-rainflow.sh t-reservoir.sh
AM_TESTS_ENVIRONMENT = srcdir='$(srcdir)'

EXTRA_DIST = $(TESTS) \
sig1.csv sig1-double.bin sig1-float.bin \
rf1-merge.csv rf1-sort.csv rv1-sort.csv

check_PROGRAMS = $(noinst_PROGRAMS)

## Makefile.am ends here

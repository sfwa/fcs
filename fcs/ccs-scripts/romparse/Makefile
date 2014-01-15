#*
#*
#* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
#*
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*    Redistributions of source code must retain the above copyright
#*    notice, this list of conditions and the following disclaimer.
#*
#*    Redistributions in binary form must reproduce the above copyright
#*    notice, this list of conditions and the following disclaimer in the
#*    documentation and/or other materials provided with the
#*    distribution.
#*
#*    Neither the name of Texas Instruments Incorporated nor the names of
#*    its contributors may be used to endorse or promote products derived
#*    from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#*


ifdef BISONSKEL
 BISONDEFS=-S $(BISONSKEL)
endif

TARGET=c66x
I2C_BUS_ADDR=0


all: gen_cdefdep romparse

romparse: cdefdep rparse.tab.o lex.yy.o romparse.c
	gcc -DIBL_CFG_I2C_MAP_TABLE_DATA_BUS_ADDR=$(I2C_BUS_ADDR) -D$(TARGET) -o romparse -g romparse.c rparse.tab.o lex.yy.o -I.

rparse.tab.c: rparse.y
	bison -d $(BISONDEFS) rparse.y

rparse.tab.o: rparse.tab.c
	gcc -c rparse.tab.c

lex.yy.c: rparse.flex
	flex rparse.flex

lex.yy.o: lex.yy.c
	gcc -c lex.yy.c

clean:
	rm -f romparse rparse.tab.o lex.yy.o cdefdep cdefdep.tmp

gen_cdefdep:
	@echo Checking command line dependencies
	@echo $(TARGET) > cdefdep.tmp
	@sh -c 'if diff -q cdefdep.tmp cdefdep ; then echo same ; else cp cdefdep.tmp cdefdep ; fi '







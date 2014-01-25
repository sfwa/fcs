all: fcs.debug.dat fcs.release.dat

b2ccs:
	$(CC) -o b2ccs b2ccs.c

bconvert64x:
	$(CC) -o bconvert64x bconvert64x.c

b2i2c:
	$(CC) -o b2i2c b2i2c.c

bootbuild:
	$(CC) -o bootbuild bootbuild.c

ccs2bin:
	$(CC) -o ccs2bin ccs2bin.c

# Run bconvert64x on the output to ensure that sections with sizes that are
# not multiples of 32 are loaded correctly
fcs.debug.bc64: bconvert64x
	./bconvert64x -le ../Debug/fcs.btbl fcs.debug.bc64

fcs.release.bc64: bconvert64x
	./bconvert64x -le ../Release/fcs.btbl fcs.release.bc64

# Convert the 32-byte-aligned boot table image to I2C format (which is also
# used for SPI).
fcs.debug.i2c: b2i2c fcs.debug.bc64
	./b2i2c fcs.debug.bc64 fcs.debug.i2c

fcs.release.i2c: b2i2c fcs.release.bc64
	./b2i2c fcs.release.bc64 fcs.release.i2c

# Convert the I2C image to CCS format for romparse
fcs.debug.ccs: b2ccs fcs.debug.i2c
	./b2ccs fcs.debug.i2c fcs.debug.ccs

fcs.release.ccs: b2ccs fcs.release.i2c
	./b2ccs fcs.release.i2c fcs.release.ccs

# bootbuild adds some extra boot configuration data and writes the resulting
# file out to i2crom.ccs in the current directory
fcs.debug.rom: bootbuild fcs.debug.ccs
	./bootbuild spi-Debug.map
	mv i2crom.ccs fcs.debug.rom

fcs.release.rom: bootbuild fcs.release.ccs
	./bootbuild spi-Release.map
	mv i2crom.ccs fcs.release.rom

# ccs2bin does the same, but maybe better?
fcs.debug.bin: ccs2bin fcs.debug.rom
	./ccs2bin -swap fcs.debug.rom fcs.debug.bin

fcs.release.bin: ccs2bin fcs.release.rom
	./ccs2bin -swap fcs.release.rom fcs.release.bin

clean:
	rm -f *.o bootbuild b2ccs b2i2c ccs2bin *.bc64 *.btbl *.i2c *.ccs *.rom *.dat *.bin

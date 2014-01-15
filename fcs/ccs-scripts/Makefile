all: fcs.debug.dat fcs.release.dat

b2ccs:
	$(CC) -o b2ccs b2ccs.c

bconvert64x:
	$(CC) -o bconvert64x bconvert64x.c

b2i2c:
	$(CC) -o b2i2c b2i2c.c

byteswapccs:
	$(CC) -o byteswapccs byteswapccs.c

bootbuild:
	$(CC) -o bootbuild bootbuild.c

# Run bconvert54x on the output to ensure that sections with sizes that are
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

# byteswapccs converts the image to big-endian for the RBL
fcs.debug.dat: byteswapccs fcs.debug.rom
	./byteswapccs fcs.debug.rom fcs.debug.dat

fcs.release.dat: byteswapccs fcs.release.rom
	./byteswapccs fcs.release.rom fcs.release.dat

clean:
	rm -f *.o bootbuild b2ccs b2i2c byteswapccs *.btbl *.i2c *.ccs *.rom *.dat

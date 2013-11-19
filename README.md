# fcs

Flight control system firmware for the SFWA CPU board DSP (TMS320C6657)

## Building

Requires `cmake` version 2.8.7 or higher.

Create a build directory outside the source tree, then use cmake to generate
the makefile.

**Note: currently only tests can be built**

`mkdir fcs_build`

`cd fcs_build`

`cmake /path/to/fcs/test`

Now, build the library using the `make` command.


## Testing

The `googletest` library is used for unit testing.

After creating the build directory, `make check` will automatically download
and build googletest, build the unit tests and then run them.

To build the unit tests without running them, use `make unittest`. The unit
tests can then manually be run (with more detailed reporting) by running
`./unittest` in the build directory.

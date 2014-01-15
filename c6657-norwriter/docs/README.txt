NOR Writer Utility

NOR Writer is a simple utility to program a CCS format image/data file to the NOR flash.

Steps to program the NOR:

1. Be sure to set the boot mode dip switch to no boot/EMIF16 boot mode on the EVM.

2. Copy the binary file to writer\nor\evmc66xxl\bin directory, and rename it to app.bin.

3. Change the file_name and start_addr in writer\nor\evmc66xxl\bin\norwriter_input.txt if necessary. 
   By default the NOR writer will load app.bin to DSP memory and write the data to NOR device start byte address 0, 
   the start_addr should always be set to the start byte addess of a sector.

4. Open CCSv5 and launch the evmc66xx emulator target configuration and connect to core 0.

5. Load the program writer\nor\evmc66xxl\bin\norwriter_evm66xxl.out to CCS, be sure evmc66xxl.gel is used in CCS 
   and DDR is intialized.

6. Open the Memory view (in CCSv5, view->Memory Browser), and view the memory address 0x80000000.

7. Load app.bin to 0x80000000:
     * In CCSv5, right click mouse in memory window, select "load memory".
     * Browse and select writer\nor\evmc66xxl\bin\app.bin (raw data format), click "next"
     * Set the Start Address to "0x80000000", Type-size to 32-bits, leave swap unchecked, click "finish"

8. After the binary file is loaded into the memory, run the program (in CCSv5, press F8), it will start to program the 
   NOR.

9. When programming is completed, the console will print "NOR programming completed successfully", if there
   is any error, the console will show the error message.

Steps to re-build norwriter:

1. Uses CCS to build norwriter:
   * Import the norwriter CCS project from writer\nor\evmc66xxl directory (in CCSv5, Project->Import Existing CCS/
   CCE Eclipse Projects).
   * Clean and build the norwriter project.
   * After the project build is completed, norwriter_evm66xxl.out and norwriter_evm66xxl.map will be generated under 
     writer\nor\evmc66xxl\bin directory.

2. Uses Makefile to build norwriter:
   NOTE FOR BUILDING ON WINDOWS ENVIRONMENT: For building on windows environment GNU utilities like
   "make" would be required. The following build procedure should also work on MINGW-MSYS Bourne shell.

    Before starting the build following environment setup has to be done 
    1) variable C_DIR should be set to the top directory of the Code Generation tools e.g.
       Linux bash shell: 
          export C_DIR=/opt/TI/TI_CGT_C6000_7.2.1/
       MSYS bash shell: 
          export C_DIR='"C:/Program Files/Texas Instruments/ccsv5/tools/compiler/c6000"'
    2) Code Generation tool binaries should be in the path e.g.
       Linux bash shell: 
          export PATH=/opt/TI/TI_CGT_C6000_7.2.1/bin:$PATH
       MSYS bash shell: 
          export PATH=$PATH:/c/Program\ Files/Texas\ Instruments/ccsv5/tools/compiler/c6000/bin/
    3) variable PFORM_LIB_DIR should be set the directory of the Platform Library root, e.g.
       Linux bash shell: 
          export PFORM_LIB_DIR=pdk_C66xx_1_0_0_10/packages/ti/platform
       MSYS bash shell: 
          export PFORM_LIB_DIR='"C:/Program Files/Texas Instruments/pdk_C66xx_1_0_0_10/packages/ti/platform"'

    The makefile for building the norwriter is in the directory "tools/writer/nor/evmc66xxl"
    Following are the steps to build norwriter, e.g.:
    cd tools/writer/nor/evmc66xxl
    make DEVICE=<device number>
        supported device numbers are 
            C6678
            C6670
            C6657

Please refer to BIOS MCSDK 2.0 User's Guide (http://processors.wiki.ti.com/index.php/BIOS_MCSDK_2.0_User_Guide) for more details.
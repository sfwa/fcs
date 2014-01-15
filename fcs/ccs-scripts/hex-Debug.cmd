../Debug/fcs.out
--ascii
--boot
--entry_point _c_int00

ROMS
{
    SPI: org = 0x00000400, length = 0x00100000, memwidth = 32, romwidth = 32
    files = { ../Debug/fcs.btbl }
}

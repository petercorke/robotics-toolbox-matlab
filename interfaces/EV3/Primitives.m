%Primitives Primitives enumeration for the brick
%
% Notes::
% - Primitives can be found in the EV3 documentation and source code
% (bytecodes.h 1584-1606)

classdef Primitives < uint8
    enumeration
        pshort (0) % PRIMPAR_SHORT = 0x00
        plong (128) % PRIMPAR_LONG = 0x80
        
        pconst(0) % PRIMPAR_CONST = 0x00
        pvariabel(64) % PRIMPAR_VARIABEL = 0x40
        plocal(0) % PRIMPAR_LOCAL = 0x00
        pglobal(32) % PRIMPAR_GLOBAL = 0x20
        phandle(16) % PRIMPAR_HANDLE = 0x10
        paddr(8) % PRIMPAR_ADDR = 0x08
        
        pindex(31) % PRIMPAR_INDEX = 0x1F
        pconst_sign(32) % PRIMPAR_CONST_SIGN = 0x20
        pvalue (63) % PRIMPAR_VALUE = 0x3F
        
        pbytes(7) % PRIMPAR_BYTES = 0x07
        
        pstring_old (0) % PRIMPAR_STIRNG_OLD = 0
        p1_byte (1) % PRIMPAR_1_BYTE = 1
        p2_byte (2) % PRIMPAR_2_BYTE = 2
        p4_byte (3) % PRIMPAR_4_BYTE = 3
        pstring (4) % PRIMPAR_STRING = 4
        
        plabel (32) % PRIMPAR_LABEL = 0x20
    end
end

# About the tests files

dc-arm7wrestler by snickerbockers: https://github.com/snickerbockers/dc-arm7wrestler (itself a adaptation of https://github.com/arisotura/arm7wrestler for the Dreamcast).
I compiled the arm7 portion and included it here as armwrestler-dc.bin for convenience (and CI), but it is not very useful outside of a full DC system.
So I'll attempt to split it up and try the tests one by one before integrating the arm core into the full DC emulator. This is what the armwrestler_dc_test_#.s files are: 
extracts of armwrestler_dc meant to run on the arm emulator without any external communication. 

All other .bin files are generated from .s assembly files in asm/.

@echo off
IF EXIST "srec_cat.exe" (
srec_cat SIDKick.ino.TEENSY41.hex -Intel -crop 0x60000000 0x60630000 -offset -0x60000000  -o SIDKick.ino.bin -Binary
SIDKickLauncher
srec_cat SIDKick_Launcher.ino.bin -binary -o SIDKick_Launcher.hex -Intel
ECHO you can use 'SIDKick_Launcher.hex' with the Teensy loader
del SIDKick.ino.bin SIDKick_Launcher.ino.bin
) else (
    ECHO srec_cat.exe not found! Get srecord from https://sourceforge.net/projects/srecord/files/srecord-win32/
)

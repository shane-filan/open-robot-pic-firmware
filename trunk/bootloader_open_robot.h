///////////////////////////////////////////////////////////////////////////
////                       bootloader_open_robot.h                     ////
////                                                                   ////
////  This include file must be included by any Open-Robot             ////
////  application that is loaded.                                      ////
///////////////////////////////////////////////////////////////////////////

#define LOADER_END 0x4FF
#define LOADER_SIZE 0x3FF

#ifndef _bootloader

#build(reset=LOADER_END+2, interrupt=LOADER_END+9)

#org 0, LOADER_END {}

#endif

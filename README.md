# fdc765

This code is a C port of the freeware `fdc765.dll` x86 assembly source code. The original author kindly provided the source code, but he has nothing to do with this port. All enquires about the code in this repository should be directed to me.

## Build

A Visual Studio project that can target both x86 and x64 is included. A `Makefile` is also include, and should be able to build a shared library for Linux, and a DLL for Windows in a MSYS2 prompt. For the later, x86 or x64 will be used depending on the prompt used.

## Usage

Just include `fdc765.h` in your code, and link against the shared object or Windows import library. You can also just drop `fdc765.c` into your code base and compile it along with your project.

## Translation to C

The original source code kept part of the emulation state as code addresses that were jumped to at the required times. While this is fine in assembly, this makes the code hard to port to C. The objective of this port was to make a C replacement that could be used in places where the original x86 DLL wouldn't work. Porting to higher level constructs was NOT one of the objectives. Most of the translation was done with regexes.

So, to minimize adding bugs to the code and making sure everything works as the original DLL, it was translated as directly from assembly to C as possible. This resulted in `goto`s and a huge `switch` to emulate `jmp eax` instructions. A lot of macros were used to simulate Intel flags. Look at the code at your own risk.

## License

MIT, enjoy!

Windows port
============

This is an ongoing effort to port libindi, indiserver and the basic drivers
to Windows. Refer to the main README file for more background details.

Building Environment/Dependencies
---------------------------------

The port requires the following:
 - MinGW (chosen because the author already has to use it for Stellarium)
   http://www.mingw.org/
 - zlib, for example the GnuWin port
   http://gnuwin32.sourceforge.net/packages/zlib.htm
 - cfitsio - you can try compiling it from source, but the pre-compiled
   Windows package seems to work, or at least to compile without complaints
   http://heasarc.gsfc.nasa.gov/fitsio/
 - CMake for Windows
   http://www.cmake.org/

After intalling/unpacking the libraries, you'll probably need to add the paths
to MinGW's include/, lib/ and bin/ subdirectories to the PATH environmental
variable. The same applies for the unpacked directory of the cfitsio library.
(You can save some effort by installing zlib in the MinGW directories.)

In the future, some kind of USB library will also be necessary. (At the moment,
the USB dependency in the CMakeLists.txt file has been disabled for Windows.)
You can skip also the CFITSIO dependency by creative modification of the files.

Building
--------

As this port is a work in progress, some of the make targets won't compile.
You need to explicitly pass to make the target names of the components you
wish to build.

The author uses Qt Creator as a developer environment, as his other work
uses Qt. Qt Creator supports CMake projects and MinGW nicely. You can pick some
other Windows IDE that supports MinGW, such as CodeBlocks. Basic alternatives
are using MSYS (minimal Unix system, companion to the MinGW project) or the bare
Windows command prompt.

The commands necessary for building are almost identical to those on Linux.
Just make sure to uses the proper -G option for CMake:
 - if you are using MSYS:
     cmake -G "MSYS Makefiles" ..
 - if you are using the Windows command prompt:
     cmake -G "MinGW Makefiles" ..

CMake can also generate project files for CodeBlocks, Eclipse and Visual Studio:
http://www.cmake.org/cmake/help/v2.8.9/cmake.html#section_Generators

Progress
--------

Or what works so far:
 - indiserver can accept connections from clients and start driver sub-processes
 - the tutorial_* drivers seem to work

Authors
-------

In addition to the authors of the INDI code, the Windows port so far has
been developed by Bogdan Marinov.
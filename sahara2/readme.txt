This directory contains a makefile that builds the static library libsahara.a.
To do this, it copies the needed c source files from the sahara2 kernel driver
directory (LINUX2.6/linux/driver/mxc/security/sahara2).

To build the static library, type 'make'.  This assumes that CROSS_COMPILE
is properly defined and that the kernel is located at ../../../linux.  So
to build the sahara2 static lib do the following: 

> cvs checkout LINUX2.6/linux
> cvs checkout LINUX2.6/misc
> cd LINUX2.6/misc/lib/sahara2
> make


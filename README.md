#Rail Marine Simulator

This program is a personal train simulator I've been working on since 2007.
I use it mostly as a straight train simulator, but it does have some
car float and car ferry features.  It can use Microsoft Train Simulator (MSTS)
routes and rolling stock, but it does not attempt do duplicate MSTS behavoir
or support all MSTS content.  There is only limited support for MSTS
activity, wagon and engine files.

The simulator is written in C++ and uses Open Scene Graph (OSG) for rendering.
The current version runs on Ubuntu 20.04.  An earlier version ran on OSX.
Compiling on Windows has never been attempted.

MIT License

##Compiling

To compile cd to src directory and run make.  Dependencies include: OSG,
osgEarth, openal, opengl, openthreads, proj4, plib, python, zlib and
microhttpd.

##Usage

bin/rmsim *startupFile* *[options]*

The startup file is a utf8 text file containing line oriented commands.
The options are text string defined in the startup file.  See the examples
directory for sample startup files.

# C++ Shared-Memory Implementation for Linux

`shmem.hpp` is a header-only library that allows to easily use *shared memory* with C++ applications on Linux. It is based on the System V implementation of interprocess communication (IPC).

This library only provides basic functionality without any guarantee of synchronization among processes that share resources via `shmem`. You can use POSIX semaphores (`semaphore.h`) for signaling and handling critical sections.

## Install

This library is made of a single header file (i.e., `shmem.hpp`), hence there is no build required. Anyway, to keep it updated across several projects that may rely on it, we can use `cmake` to install it under `/usr/local/include`, so that `gcc` or any other compiler can easily locate it. To install it, we simply need to configure once with `cmake .` and then, after every change to the header file, we can install it with:

    sudo make install

This will effectively just copy the header file to `/usr/local/include` for us. 

## Usage

To use it in a project, just

    #include <shmem.hpp>

in a C++ source or header file. Being in the default include path, the compiler should automatically find and include it in our source code.
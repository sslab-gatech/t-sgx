T-SGX
==========================

Environment & Prerequisites
---------------------------
- Intel Skylake CPU
- Ubuntu 16.04 LTS
- Intel SGX SDK & Driver
- cmake


Build
--------------------------
- Build tsgx-llvm backend
~~~~~{.sh}
$ mkdir build-llvm
$ cd build-llvm
$ cmake -G "Unix Makefiles" ../llvm-tsgx
$ make
~~~~~

- Build front-end pass
~~~~~{.sh}
$ cd function-{name|wrapper}-pass
$ mkdir build
$ cd build
$ LLVM_DIR=/path/to/llvmBuild/share/llvm/cmake cmake ..
  (e.g., $ LLVM_DIR=../../build-llvm/share/llvm/cmake cmake ..)
$ make
~~~~~

Usage
-------------------------
- See test/tsgx-test as a example
~~~~~{.sh}
$ cd test/tsgx-test
$ make
$ ./App
~~~~~

Note
------------------------
- Because of the lazy page allocation feature in Linux,
  we make a workaround by using tsgx_init function (see lib/).
- Please manually adjust the code size in tsgx_init accordingly.

Authors
------------------------
- Ming-Wei Shih <mingwei.shih@gatech.edu>
- Sangho Lee <sangho@gatech.edu>
- Taesoo Kim <taesoo@gatech.edu>
- Marcus Peinado

#!/usr/bin/env bash

set -ex
make -C /lib/modules/`uname -r`/build M=`pwd` EXTRA_CFLAGS="-DDEBUG" $*

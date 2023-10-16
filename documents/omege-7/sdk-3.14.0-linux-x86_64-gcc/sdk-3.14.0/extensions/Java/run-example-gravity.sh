#! /bin/bash

#  Copyright Copyright (C) 2001-2021 Force Dimension, Switzerland.
#  All Rights Reserved.
#
#  drd 3.14.0
#  Force Dimension SDK
#
#  THIS FILE CAN NOT BE COPIED AND/OR DISTRIBUTED WITHOUT EXPRESS
#  PERMISSION FROM FORCE DIMENSION.

if [ ! -f "../../lib/release/lin-x86_64-gcc/libdhd.so.3" ]; then
    ln -rs ../../lib/release/lin-x86_64-gcc/libdhd.so.3.14.0 ../../lib/release/lin-x86_64-gcc/libdhd.so.3
fi

export LD_LIBRARY_PATH="../../lib/release/lin-x86_64-gcc"
export LD_PRELOAD="../../lib/release/lin-x86_64-gcc/libdhd.so.3"
java -cp classes:classes/com/forcedimension/examples -Djava.library.path=jni/bin/release/lin-x86_64-gcc gravity

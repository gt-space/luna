#!/bin/bash

if [ "$1" == "musl" ]; then
    scp target/armv7-unknown-linux-musleabihf/debug/tel-sw debian@$2.local:~/
elif [ "$1" == "gnu" ]; then
    scp target/armv7-unknown-linux-gnueabihf/debug/tel-sw debian@$2.local:~/
fi
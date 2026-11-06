#!/bin/bash
pkill -f "[s]tream.py"
pkill -f "[t]errain.py"
sleep 1
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
exec python3 /home/ariyan/stream.py

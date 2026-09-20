#!/usr/bin/env python3
"""Compatibility alias; all implementation now lives inside ELF-OS/control."""
import sys
from go2 import main

if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0] == 'image':
        args[0] = 'observe'
    raise SystemExit(main(args))

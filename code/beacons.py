#!/usr/bin/env python3

import sys
import unhandled_exit
unhandled_exit.activate() # exit main thread if any threads crash.
from classes.modem import Modem

# Get command line arguments
mode = len(sys.argv) > 1 and sys.argv[1] or None
args = len(sys.argv) > 2 and sys.argv[2:] or None

# Run beacon code
b = Modem(mode=mode, args=args)
b.run(b.mode)

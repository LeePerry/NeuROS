# Copyright (c) 2024 Lee Perry

import os
import signal
import subprocess
import time

def process_for(cmd, seconds, func):
    try:
        timeout = time.time() + seconds
        p = subprocess.Popen(cmd,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        while time.time() < timeout:
            line = p.stdout.readline()
            if not line: break
            func(str(line, "utf-8"))
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    finally:
        os.system("stty sane")

# Copyright (c) 2023 Lee Perry

import os
import subprocess
import time

def process_for(cmd, seconds, func):
    try:
        timeout = time.time() + seconds
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        while time.time() < timeout:
            line = p.stdout.readline()
            if not line: break
            func(str(line, "utf-8"))
        p.kill()
    finally:
        os.system("stty sane")

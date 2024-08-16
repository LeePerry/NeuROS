# Copyright (c) 2024 Lee Perry

import os
import re
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

def process_for_simulated_duration(cmd, seconds, func):
    try:
        regex = re.compile(".* Simulated time: (\d*\.?\d+)")
        p = subprocess.Popen(cmd,
                             stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT,
                             preexec_fn=os.setsid)
        while True:
            line = p.stdout.readline()
            if not line: break
            line = str(line, "utf-8")
            capture = regex.search(line)
            if capture is not None and capture.group(1) is not None:
                if float(capture.group(1)) > seconds:
                    break
            func(line)
        os.killpg(os.getpgid(p.pid), signal.SIGTERM)
    finally:
        os.system("stty sane")
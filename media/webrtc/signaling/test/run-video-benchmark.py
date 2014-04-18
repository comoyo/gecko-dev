#!/usr/bin/env python
# import matplotlib.pyplot as plt
import sys
import os
import re
import subprocess
import argparse

duration = 30
frame_rates = [60]
#resolutions = [[176, 144], [320, 240], [640, 480], [1280, 720]]
#frame_rates = [5]
resolutions = [[176, 144], [352, 288]]

backlog_thresh = 20
verbose = 1

def debug(msg):
    if verbose > 0:
        sys.stderr.write("%s\n"%msg)

def trace(msg):
    if verbose > 1:
        sys.stderr.write("%s\n"%msg)

def die(msg):
    sys.stderr.write("ERROR: %s"%msg);
    sys.exit(1)

class Video(object):
    def __init__(self, file):
        f = open(file, "r")
        if f is None:
            die("File %s not found"%file)
        l = f.readline()
        m = re.match("YUV4MPEG2 W(\d+) H(\d+)", l)
        if m is None:
            die("Bad file format")

        self.file_ = file
        self.width_ = int(m.group(1))
        self.height_ = int(m.group(2))
        self.pushed_ = {}
        debug("Native resolution is %dx%d"%(self.width_, self.height_));

    def make_file_name(self, width, height):
        if ((width == self.width_) and (height == self.height_)):
            return self.file_
        else:
            return "%s-%d-%d"%(self.file_, width, height)

    def make_file(self, width, height):
        if (width > self.width_):
            die("Can't resize file upward")
        if (height > self.height_):
            die("Can't resize file upward")

        filename = self.make_file_name(width, height)
        if os.path.exists(filename):
            return filename

        # Now resize the file
        debug("Need to resize file to %dx%d"%(width, height))
        cmd = "yuvscaler -O SIZE_%dx%d < %s > %s"%(width, height, self.file_, filename)
        subprocess.check_call(cmd, shell=True, stdout=None, stderr=None)
        return filename

    def push_file(self, filename):
        debug("Pushing file %s"%filename)
        if filename in self.pushed_:
            return

        subprocess.check_call(["adb", "push", filename, "/data"])
        self.pushed_[filename] = True

    def ensure_file(self, width, height):
        filename = self.make_file(width, height)
        self.push_file(filename)
        return filename

    def run_benchmark(self, width, height, rate):
        debug("Running benchmark for %dx%d @ %d"%(width, height, rate))
        filename = self.ensure_file(width, height)
        cmd = 'adb shell "LD_LIBRARY_PATH=/system/b2g /data/video_benchmark -i /data/%s -R -r %d 2>/dev/null"'%(filename, rate)
        debug("Running %s"%cmd)
        sp = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=None)

        results = 0.0
        for l in sp.stdout:
            if re.match("Running",l) != None:
                continue
            if re.match("Finished",l) != None:
                break
            if re.match("fps", l) != None:
                ll = l.split("\t")
                results = float(ll[0][5:-1])

        return results

    def delete_file(self, filename):
        subprocess.check_call(["adb", "shell", "rm", "/data/%s"%filename])

    def delete_resolution(self, width, height):
        self.delete_file(self.make_file_name(width, height))

def run_benchmark(v):
    output = ["Resolution@Rate\tFPS"]
    out = dict()

    for res in resolutions:
        nRes = 'x'.join(map(str,res))
        out[nRes] = (list(), list(), list(),list())
        for rate in frame_rates:
            out[nRes][0].append(rate)
            result = v.run_benchmark(res[0], res[1], rate)
            print( result)
            out[nRes][1].append(result)
            debug("%dx%d@%d fps %f"%
                  (res[0], res[1], rate, result))
            output.append("%dx%d@%d \t%f"%
                          (res[0], res[1], rate, result))

    # Clean up
    v.delete_resolution(res[0], res[1])

    return output

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run video benchmarks')
    parser.add_argument('-f', '--file', dest = 'file', default='benchmark-stream.yuv')
    args = parser.parse_args();

    v = Video(args.file)
    output = run_benchmark(v)
    for o in output:
        print o

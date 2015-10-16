import sys
import os

cmd = "D:\\Lifan\\pbrt-v2\\bin\\pbrt"
args = " --outfile lightcuts/cornell_lightcuts.pfm"
args += " sibenik-igi.pbrt"
print cmd + args
os.system(cmd + args)

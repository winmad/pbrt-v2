import sys
import os

cmd = "D:\\Lifan\\pbrt-v2\\bin\\pbrt"
args = " --outfile lightcuts/cornell_lightcuts.pfm"
args += " lightcuts/cornell-igi.pbrt"
print cmd + args
os.system(cmd + args)

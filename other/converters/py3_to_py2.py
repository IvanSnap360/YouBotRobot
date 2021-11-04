import sys
import os
import glob
import os
import time
# print(os.path.dirname(os.path.abspath(__file__)))
print("\033[01m\033[04m\033[36mConverting PythonRosNodes for Python 2 \033[0m")
src_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = src_dir.split("/")[1:-2]
# print(src_dir)
src_dir_str = str()
for e in src_dir:
    src_dir_str += "/"+e

# print(src_dir_str)
src_dir = src_dir_str+"/src"
dirs = os.listdir(src_dir)



for dir in dirs:
    file_dir = glob.glob("{}/{}/src/*.py".format(src_dir, dir))
    if len(file_dir) != 0: 
        print("\033[01m\033[35mConverting--> \033[0m{}\033[0m".format(file_dir))
    for f in file_dir:
        file = open(f, "r")
        lines = file.readlines()
        file.close()
        lines[0] = "#! /usr/bin/env python2 \n"
        file = open(f, "w")
        file.writelines(lines)
        file.close()
        time.sleep(0.01)
        print("\033[01m\033[32mDone\033[0m")
print("\033[01m\033[32m############### DONE ####################\033[0m" )
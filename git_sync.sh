#!/bin/sh
echo "SYNCHRONIZATION WITH GIT $(basename `git rev-parse --show-toplevel`)"
echo "###############DOWNLOAD##################" >> git_log.log
date >> git_log.log
git fetch >> git_log.log
git pull >> git_log.log
echo "############### DONE ####################" >> git_log.log
echo "############### DONE ####################" 
echo ""

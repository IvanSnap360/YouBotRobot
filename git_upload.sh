#!/bin/sh
echo "UPLOAD TO GIT $(basename `git rev-parse --show-toplevel`)"
echo "###############UPLOAD####################" >> git_log.log
date >> git_log.log
git add -A >> git_log.log
git commit -m "SYNS_FROM_ROBOT_PC" >> git_log.log
git push >> git_log.log
echo "############### DONE ####################" >> git_log.log
echo "############### DONE ####################" 


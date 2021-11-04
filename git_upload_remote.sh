#!/bin/sh
printf "###############UPLOAD#################### \n" >> git_log.log
printf "UPLOAD TO GIT $(basename `git rev-parse --show-toplevel`)\n"
date >> git_log.log
git add -A >> git_log.log
git commit -m "SYNS_FROM_ROBOT_PC" >> git_log.log
git push >> git_log.log
printf "############### DONE ####################\n" >> git_log.log
printf "############### DONE ####################\n" 



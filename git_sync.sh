#!/bin/sh
printf "SYNCHRONIZATION WITH GIT $(basename `git rev-parse --show-toplevel`) \n"
printf "###############DOWNLOAD##################\n" >> git_log.log
date >> git_log.log
git fetch >> git_log.log
git pull >> git_log.log
printf "############### DONE ####################\n" >> git_log.log
printf "############### DONE ####################\n" 
printf "\n">> git_log.log
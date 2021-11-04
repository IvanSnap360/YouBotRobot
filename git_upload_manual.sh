#!/bin/sh
read -p "Enter Commit text: " x
printf "###############UPLOAD#################### \n" >> git_log.log
printf "UPLOAD TO GIT $(basename `git rev-parse --show-toplevel`)\n"
date >> git_log.log
git add -A >> git_log.log
git commit -m "{$x}" >> git_log.log
git push >> git_log.log
printf "############### DONE ####################\n" >> git_log.log
printf "############### DONE ####################\n" 



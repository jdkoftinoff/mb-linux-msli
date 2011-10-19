#! /bin/bash -e

git submodule foreach --recursive git fetch origin
git submodule foreach --recursive git merge origin/master


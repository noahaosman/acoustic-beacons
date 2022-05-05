#!/usr/bin/env bash

local_dir="/Users/jasmine/main/jnash/code/acoustic-beacons"
remote_dir="/home/pi/nav"
host=$1
rsync -ra --partial-dir=/tmp --files-from=deploy_list.txt "$local_dir" "pi@$host:$remote_dir"

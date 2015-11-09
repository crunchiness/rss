#!/bin/sh

tar -czvf upload.tgz robot/ toddler.py

HOST='elimia.inf.ed.ac.uk'
USER='student'
PASSWORD='password'
TODDLER='toddler.py'

ftp -n ${HOST} <<END_SCRIPT
quote USER ${USER}
quote PASS ${PASSWORD}
put upload.tgz
quit
END_SCRIPT

rm upload.tgz

/usr/bin/expect <<EOD
spawn ssh -oStrictHostKeyChecking=no -oCheckHostIP=no "student@elimia.inf.ed.ac.uk" "rm -rf robot && rm -f toddler.py && tar -xzvf upload.tgz && rm upload.tgz && pwd"
expect "password"
send "password\r"
expect "/home/student"
EOD

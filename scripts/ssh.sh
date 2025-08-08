#!/usr/bin/expect

set timeout 20
set ip "10.21.41.1"      
set username "user"      
set password "'"      

# Start the SSH connection
spawn ssh $username@$ip

# Handle the SSH password prompt
expect "password:"
send "$password\r"

# Optionally, you can send any further commands after logging in
# For example, to list files after logging in
expect "$ "
send "ls -l\r"

# Interact allows the script to continue with an interactive shell
expect "$ "
interact

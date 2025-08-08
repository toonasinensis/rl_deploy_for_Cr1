#!/usr/bin/expect
#exit

# set ip "192.168.2.1"
# set username "user"
# set passwd "123456"

# set ip "172.16.15.217"
# set username "ysc"
# set passwd "'"

# set ip "172.16.15.63"
# set username "ysc"
# set passwd "'"

# set ip "172.16.14.80"
set ip "10.21.41.1"
set username "user"
set passwd "'"

set send_policy "0"
set policy_name "policy_wheel_k1.5-2_v4.17-5history.pt"

if { "$send_policy" == "1" } {
  puts "send policy $policy_name to remote"
  spawn scp ./policy/$policy_name  $username@$ip:/home/$username/rl_deploy_wt/policy/$policy_name
  expect {
    "密码："
          {
            send "$passwd\n"
          }
    "pass"
          {
            send "$passwd\n"
          }
    "yes/no"
          {
            sleep 5
            send_user "send yes"
            send "yes\n"
          }
    eof
      {
          sleep 5
          send_user "eof\n"
      }
  }
  set timeout 3000
  send "exit\r"
  expect eof
} else {
  puts "don't send policy to remote"
}

# /usr/bin/expect<<EOF

spawn scp build/rl_deploy  $username@$ip:/home/$username/rl_deploy_wt/bin/rl_deploy
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
send "exit\r"
expect eof


spawn scp -r config  $username@$ip:/home/$username/rl_deploy_wt/config
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
send "exit\r"
expect eof



spawn scp policy $username@$ip:/home/$username/rl_deploy_wt/policy
expect {
  "密码："
        {
          send "$passwd\n"
        }
   "pass"
        {
          send "$passwd\n"
        }
   "yes/no"
        {
          sleep 5
          send_user "send yes"
          send "yes\n"
        }
   eof
    {
        sleep 5
        send_user "eof\n"
    }
}
set timeout 3000
send "exit\r"
expect eof

spawn ssh $username@$ip
expect {
    "password: " {
        send "$passwd\n"
    }
    "pass" {
        send "$passwd\n"
    }
    "yes/no" {
        send "yes\n"
        exp_continue
    }
    eof {
        sleep 5
        send_user "eof\n"
    }
}




set timeout 20
set ip "10.21.41.1"      
set username "user"      
set password "'"      

# Start the SSH connection
spawn ssh $username@$ip

# Handle the SSH password prompt
expect "password:"
send "$password\r"
interact



# spawn scp -r third_party  $username@$ip:/home/$username/rl_deploy_wt/third_party
# expect {
#   "密码："
#         {
#           send "$passwd\n"
#         }
#    "pass"
#         {
#           send "$passwd\n"
#         }
#    "yes/no"
#         {
#           sleep 5
#           send_user "send yes"
#           send "yes\n"
#         }
#    eof
#     {
#         sleep 5
#         send_user "eof\n"
#     }
# }
# set timeout 3000
# send "exit\r"
# expect eof
# # EOF

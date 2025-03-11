if master ip 192.168.3.1
   host1 ip 192.168.3.2
   host2 ip 192.168.3.3

step 1
master 
ssh-keygen

host1 
ssh-keygen

host2 
ssh-keygen

step 2
master 
ssh-copy-id -i ubuntu@192.168.3.1

host1 
ssh-copy-id -i ubuntu@192.168.3.1

host2 
ssh-copy-id -i ubuntu@192.168.3.1

step 3
master 
scp -r ~/.ssh/authorized_keys jetauto@192.168.3.2:~/.ssh

master 
scp -r ~/.ssh/authorized_keys jetauto@192.168.3.3:~/.ssh

step 4
master
vim ~/.ssh/config

Host robot1
    HostName 192.168.3.2
    User jetauto
Host robot2
    HostName 192.168.3.3
    User jetauto

sudo date -s "2022/12/12 12:12:21"

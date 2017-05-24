#!/usr/bin/expect -f
spawn scp -r code02i/ in4342-06@ce-eslab.ewi.tudelft.nl:/data/home/in4342-06/assignment02/vlab/trials/
expect "in4342-06@ce-eslab.ewi.tudelft.nl's password:"
send "roma123\r"
interact


#!/bin/bash

# /usr/local/sbin/ethercatctl

if [ $# == 0 ];then
    sudo ethercatctl restart
else
    sudo ethercatctl stop
fi

# 读
# sudo ethercat upload -m [master id] -p [slave id] -t uint16 0x6072 0
# sudo ethercat upload -m [master id] -p [slave id] -t uint16 0x6073 0
# 写
# sudo ethercat download -m [master id] -p [slave id] -t uint16 0x6072 0 [新值]
# sudo ethercat download -m [master id] -p [slave id] -t uint16 0x6073 0 [新值]


# sudo ethercat download -m 1 -p 1 -t uint16 0x20fc 0x2 1


#master 0
#index 1-6---left_leg
#index 7-12---right_leg
#index 14-16---waist
#index 17-18---head
#master 1
#index 1-7---left_arm
#index 8---left hand
#index 9-15---right_arm
#index 16---right hand
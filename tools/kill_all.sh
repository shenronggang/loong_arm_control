#!/bin/bash

if [ $# == 0 ];then
    echo 需要参数！
    echo " d: driver_task"
    echo " i: interface_task"
    echo " a: control_task_arm"
    echo " p: python脚本"
    exit
elif [ $1 == 'd' ];then
    sudo killall driver_task
elif [ $1 == 'i' ];then
    sudo killall interface_task
elif [ $1 == 'a' ];then
    sudo killall control_task_arm
elif [ $1 == 'p' ];then
    killall python3
elif [ $1 == 'all' ];then
    sudo killall driver_task
    sudo killall interface_task
    sudo killall control_task_arm
    killall python3
fi
#!/bin/bash
source $HOME/.bashrc
source /home/caluckal/catkin_ws/devel/setup.bash
echo $1
echo $2
echo $3
base_name=$1
kill_term=$2
command_1=$3
shift
shift
shift
args=$@
quit() {
    echo "Quitting"
    echo $kill_term
    rosnode kill $kill_term
    # ps -ef | grep "$cmd $arg" | grep -v grep | awk '{print $2}' | xargs -r kill -9
    # echo "KILL STRAY PROCESS USING: 'ps -ef | grep '<process>' | grep -v grep | awk '{print \$2}' | xargs -r kill -9'"
    exit 0
}

trap quit INT
trap quit TERM


a=1
b=1
counter=1
echo "CODE WILL WAIT FOR 10 iterations to get baseline"
sleep 3
while true
do
    # free -m | awk 'NR==2{printf "Memory Usage: %s/%sMB (%.2f%%)\n", $3,$2,$3*100/$2 }'
    # df -h | awk '$NF=="/"{printf "Disk Usage: %d/%dGB (%s)\n", $3,$2,$5}'
    # top -bn1 | grep load | awk '{printf "CPU Load: %.2f\n", $(NF-2)}'
    # lscpu | grep "MHz"
    if [ $counter -lt 10 ]
        then
            echo "ITR:$counter"
    fi
    mem_usage=`free -m | awk 'NR==2{printf "%.2f", $3*100/$2 }'`
    echo $mem_usage
    cpu_mhz=`lscpu | grep "CPU MHz" | awk '{printf $3}'`
    echo $cpu_mhz
    cpu_util=`top -bn1 | grep '%Cpu(s)' | awk '{printf "%.2f",$2}'`
    echo $cpu_util
    sleep 0.5
    echo "$mem_usage,$cpu_mhz,$cpu_util" >> $base_name.csv
    # clear
    if [ $counter -gt 10 ]
        then
            if [ $a -eq $b ]
                then
                    a=0
                    printf "___________\n" >> $base_name.csv
                    # python3 ~/Desktop/test.py &
                    $command_1 $args &
            fi
    else
        counter=$((counter+1))
    fi
done


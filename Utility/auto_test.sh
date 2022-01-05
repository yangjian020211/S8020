#!/bin/bash

tty_x="ttyUSB0"
case_x="parameter"

rm -rf ${case_x}.log
sudo gnome-terminal -e "minicom -C ${case_x}.log"  

sudo stty -F /dev/${tty_x} ispeed 115200 ospeed 115200 cs8
cat ${case_x} | while read line
do
    if [[ ${line:0:1} !=  '#' ]]
    then
        cycles=`echo ${line} | cut -d " " -f 1`
        delay=`echo ${line} | cut -d " " -f 2`
        cmd=`echo ${line} | cut -d " " -f 3-`
        while (("$cycles">0))
        do
           echo -e "${cmd}\r" > /dev/${tty_x}
           echo ${cmd}
           sleep ${delay}
           let cycles=cycles-1
        done
    fi
done


grep -n "auto_test>" ${case_x}.log > tmp

rm -rf tmp2 
cat tmp | while read line
do
   echo ${line##*auto_test>} >> tmp2
done


echo -e "\tauto_test report " `date "+%Y-%m-%d %H:%M:%S"` "\n\n\n\n" > tmp 
sed '/-------------------- END --------------------/a\\n\n' tmp2 >> tmp
sed -e '/-------------------- PAR --------------------/d;/-------------------- CUSTOM --------------------/d' tmp > ${case_x}.report 

passed=$(grep -o 'PASSED' ${case_x}.report | wc -l)
failure=$(grep -o 'FAILURE' ${case_x}.report | wc -l)
total=`expr ${passed} + ${failure}`

sed -i "3iTOTAL:${total}\tPASSED:${passed}\tFAILURE:${failure}" ${case_x}.report

rm -rf tmp*

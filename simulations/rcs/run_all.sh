#!/bin/bash

cleanup() {
	local exit_status=$?
	if [[ $exit_status != 0 ]]; then
		if [[ -f ${config}.bk ]]; then
			mv ${config}.bk ${config}
		fi
	fi
}

trap cleanup EXIT

set -e

# data_dir directory collect the processed simulation log file, only retains the coin transaction related log
data_dir=${1}
if [[ -z "$data_dir" ]]; then
	echo "Input the path to save results as the first parameter.\n"
	exit
fi

if [[ ! -d ${data_dir} ]]; then
	mkdir -p ${data_dir}
fi

### scheme Options ###
# Scheme2 Scheme3 Scheme2_Pi Scheme3_Pi

### numCpuCores Options ###
# 1 2 3 4 ...

### map Options ###
# intersection_500 intersection_1000 intersection_1500 intersection_max
# beijing_200 beijing_500 beijing_1000 beijing_1500 beijing_2000
# paris_100 paris_300 paris_500 paris_700 paris_1000

### settings format ###
# map scheme numCpuCores
# example: "intersection_500 Scheme2 1"
settings=("intersection_500 Scheme2 1")

config="omnetpp.ini"
logfile="simulation.log"

# endrecord file records where the simulation ends and the number of successful and failed cases
endrecord="endrecord.log"

cp ${config} ${config}.bk

for setting in "${settings[@]}"
do
	paras=($setting)
	map=${paras[0]}
	scheme=${paras[1]}
	numCpuCore=${paras[2]}
	echo "Experiment ${scheme} ${map} GeneralApp ${numCpuCore}"
	sed -i "s/maps\/intersection/maps\/${map}/g" ${config}
	sed -i "s/\*\*\.numCpuCores = 1/\*\*\.numCpuCores = ${numCpuCore}/g" ${config}
	opp_run -r 0 -m -u Cmdenv -c "${scheme}_GeneralApp" -n ..:../../src:../../../inet-3.6.6/src:../../../inet-3.6.6/examples:../../../inet-3.6.6/tutorials:../../../inet-3.6.6/showcases:../../../veins/examples/veins:../../../veins/src/veins:../../../veins/subprojects/veins_inet3/src/veins_inet:../../../veins/subprojects/veins_inet3/examples/veins_inet --image-path=../../images:../../../inet-3.6.6/images:../../../veins/images:../../../veins/subprojects/veins_inet3/images -l ../../src/lte -l ../../../inet-3.6.6/src/INET -l ../../../veins/src/veins -l ../../../veins/subprojects/veins_inet3/src/veins_inet ${config}|| {
	echo "Experiment ${scheme} ${map} GeneralApp ${numCpuCore} finished at: $(tail -n 1 ${logfile})" >> ${endrecord}
	}
	sed -i -n "/\[WARN\]/p" ${logfile} 
	echo "Succeeded transactions: $(cat ${logfile} | grep -c "succeed" )" >> ${endrecord}
	echo "Failed transactions: $(cat ${logfile} | grep -c "fail" )" >> ${endrecord}
	mv ${logfile} ${data_dir}/cv2x_${scheme}_${map}_GeneralApp_${numCpuCore}.log
	cp ${config}.bk ${config}
done

rm ${config}.bk

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

data_dir=${1}
if [[ -z "$data_dir" ]]; then
	echo "Input the path to save results as the first parameter.\n"
	exit
fi

if [[ ! -d ${data_dir} ]]; then
	mkdir -p ${data_dir}
fi

# Scheme2 Scheme3
# Scheme2_Pi Scheme3_Pi
schemes="Scheme2" 
# 1 2 3 4
numCpuCores="2 3"
# intersection_500 intersection_1000 intersection_1500 intersection_max
# beijing_200 beijing_500 beijing_1000 beijing_1500 beijing_2000
# paris_100 paris_300 paris_500 paris_700 paris_1000
maps="beijing_500"
# PerceptionApp AuctionApp GeneralApp
apps="GeneralApp"

config="omnetpp.ini"
logfile="simulation.log"
endrecord="endrecord.log"
cp ${config} ${config}.bk

for map in $maps
do
	for scheme in $schemes
	do
		for app in $apps
		do
			for numCpuCore in $numCpuCores 
			do
				echo "Experiment ${scheme} ${map} ${app} ${numCpuCore}"
				sed -i "s/maps\/beijing_200/maps\/${map}/g" ${config}
				sed -i "s/\*\*\.numCpuCores = 1/\*\*\.numCpuCores = ${numCpuCore}/g" ${config}
				# opp_run -r 0 -m -u Cmdenv -c "${scheme}_${app}" -n .:../../src/veins --image-path=../../images -l ../../src/veins ${config}
				opp_run -r 0 -m -u Cmdenv -c "${scheme}_${app}" -n ..:../../src:../../../inet-3.6.6/src:../../../inet-3.6.6/examples:../../../inet-3.6.6/tutorials:../../../inet-3.6.6/showcases:../../../veins/examples/veins:../../../veins/src/veins:../../../veins/subprojects/veins_inet3/src/veins_inet:../../../veins/subprojects/veins_inet3/examples/veins_inet --image-path=../../images:../../../inet-3.6.6/images:../../../veins/images:../../../veins/subprojects/veins_inet3/images -l ../../src/lte -l ../../../inet-3.6.6/src/INET -l ../../../veins/src/veins -l ../../../veins/subprojects/veins_inet3/src/veins_inet ${config}|| {
				# I found all lte simulate will ends with error (but data is logged correctly), strange...
				echo "Experiment ${scheme} ${map} ${app} ${numCpuCore} finished at: $(tail -n 1 ${logfile})" >> ${endrecord}
				}
				sed -i -n "/\[WARN\]/p" ${logfile} 
				echo "Succeeded transactions: $(cat ${logfile} | grep -c "succeed" )" >> ${endrecord}
				echo "Failed transactions: $(cat ${logfile} | grep -c "fail" )" >> ${endrecord}
				mv ${logfile} ${data_dir}/cv2x_${scheme}_${map}_${app}_${numCpuCore}.log
				cp ${config}.bk ${config}
			done
		done
	done
done

rm ${config}.bk

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

schemes="Scheme3 Scheme2"
numCpuCores="1 2 3 4"
maps="intersection_500 intersection_1000 intersection_1500 intersection_max"
apps="PerceptionApp AuctionApp"

config="omnetpp.ini"
logfile="simulation.log"
cp ${config} ${config}.bk

for scheme in $schemes
do
	for map in $maps
	do
		for app in $apps
		do
			for numCpuCore in $numCpuCores
			do
				echo "Experiment ${scheme} ${map} ${app} ${numCpuCore}"
				sed -i "s/maps\/intersection/maps\/${map}/g" ${config}
				sed -i "s/rsu\[\*\]\.appl\.numCpuCores = 1/rsu[*].appl.numCpuCores = ${numCpuCore}/g" ${config}
				opp_run -r 0 -m -u Cmdenv -c "${scheme}_${app}" -n .:../../src/veins --image-path=../../images -l ../../src/veins ${config}
				sed -i -n "/\[WARN\]/p" ${logfile}
				echo "Succeeded transactions: "
				cat ${logfile} | grep -c "succeed"
				mv ${logfile} ${data_dir}/dsrc_${scheme}_${map}_${app}_${numCpuCore}.log
				cp ${config}.bk ${config}
			done
		done
	done
done

rm ${config}.bk

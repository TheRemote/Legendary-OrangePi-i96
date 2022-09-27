#!/bin/sh

Usage(){
	printf "\033[32mUsage: "
	printf "\t$0 [-h] [-f]... [-s <frequencies> <cpu number>]\n
Arguments:
  -h		Print Help (this message)
  -f          	CPU0~3 supported frequencies
  -s            Set the maximum frequency of CPU0~3
			e.g:  -s 1008000 1 (Set the maximum frequency of cpu1 to 1008000)
  -c            View the current frequency of cpu0~3
  -t            View the current CPU temperature
  -l            List CPU ID
  -i            Viewing cpu statistics
\033[0m"
}

while getopts ":hfFs:ctli" varname
do
	case $varname in
		h)
		Usage
		exit
		;;

		f)
		echo "CPU0~3 supported frequencies: "
		echo "  "
		echo "CPU[0]: "
		cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies
		echo "CPU[1]: "
		cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_available_frequencies
		echo "CPU[2]: "
		cat /sys/devices/system/cpu/cpu2/cpufreq/scaling_available_frequencies
		echo "CPU[3]: "
		cat /sys/devices/system/cpu/cpu3/cpufreq/scaling_available_frequencies
		exit
		;;

		s)
		echo "Set the maximum frequency of CPU$3: $OPTARG"
		echo $OPTARG > /sys/devices/system/cpu/cpu$3/cpufreq/scaling_max_freq
		exit
		;;

		c)
		echo "The current frequency of cpu0~3: "
		cat /sys/devices/system/cpu/cpu[0123]/cpufreq/cpuinfo_cur_freq
		exit
		;;

		t)
		echo "The current CPU temperature: "
		cat /sys/class/thermal/thermal_zone0/temp
		exit
		;;

		l)
		echo "List CPU ID: "
		cat /sys/class/sunxi_info/sys_info | grep "sunxi_chipid"
		exit
		;;

		i)
		echo "Viewing cpu statistics: "
		lscpu
		exit
		;;

		*)
		echo "\033[31mUnknow Option, Please use 'cpu_sh -h' for more commands.\033[0m"
		exit 1
		;;
	esac
done

Usage

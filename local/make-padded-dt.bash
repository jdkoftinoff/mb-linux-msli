#!/bin/bash

if [ ! -f "${1}" ]
    then
    echo "Input file is not specified"
    exit 1
fi

if [ "${2}" == "" ]
    then
    echo "Output file is not specified"
    exit 1
fi


linecount=`wc -l "${1}"|cut -f1 -d' '`
linenum=`grep -n bootargs "${1}"|cut -f1 -d:`

if [ "$linenum" == "" ]
    then
    echo "No bootargs in the drvice tree"
    exit 1
fi

((bottomlines=linecount-linenum+1))

outfiledir=`dirname "${2}"`
outfile=`mktemp "${outfiledir}/device-tree.XXXXXXXX"`

counter=1;

while true ; do
    
    (
	head -n $linenum "${1}"
	echo -n "pad_"
	for ((i=0;i<counter;i++))
	  do
	  echo -n "0"
	done
	echo "=\"_pad\";"
	tail -n $bottomlines "${1}"
	
	)|../dtc/dtc -f -o "${outfile}" -O dtb 2> /dev/null
    
    filelen=`stat -c %s "${outfile}"`
    ((recordlen=filelen+8))
    ((blocks=recordlen/256))
    ((wholeblockslen=blocks*256))
    if [ $recordlen == $wholeblockslen ]
	then
	mv "${outfile}" "${2}"
	exit 0;
    fi
    
    ((counter++))
    
    if ((counter>1024))
	then
	echo "Can't pad this device tree"
	rm "${outfile}"
	exit 1
    fi
done

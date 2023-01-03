#!/bin/bash
cd /home/lluis/omnetpp-5.2.1/samples/aflora-v0.8.5/simulations


if [[ $# -eq 0 ]]; then
     echo "2 parameters are needed: config runssimtimelist.txt "
     echo "Execute "
     exit 1
fi

echo "Processing"

declare -a RUNS
declare -a SIMTIME
#NEDDIR=../src:.:../../inet/src:../../inet/examples:../../inet/tutorials:../../inet/showcases
OPPPARM='-n ../src:.:../../inet/src:../../inet/examples:../../inet/tutorials:../../inet/showcases --image-path=../../inet/images -l ../../inet/src/INET'
RESULTDIR=/home/lluis/vbshare/LoRaWANPacketEnergyConsumption/results/resultsX/

if [[ ! -d $RESULTDIR ]]; then
    mkdir $RESULTDIR
fi
#RUNS=(0-17,36-53 18-23,54-59 24-26,60-62 27-29,63-65 30-32,66-68 33-35,69-71)
#SIMTIME=(1d      10d         18d         32d         50d         65d)
#SIMTIME=(86400 432000 1036800 2592000)

while IFS=" " read -r f1 f2
do
  #printf 'read: %s, simtime: %s\n' $f1 $f2
  if [ -n $f1 ]; then
     RUNS+=($f1)
     SIMTIME+=( $( [ ! -z "${f2##*[!0-9]*}" ] && echo ${f2}d ||  echo ${f2} ) )
  fi
done <${2:-/dev/stdin}

for ((i = 0; i < ${#RUNS[@]}; i++)); do
#     echo "Array content: " ${RUNS[i]}, ${SIMTIME[i]}d
     printf 'Run: %s, %s with %s  simulated time\n' $1 "${RUNS[i]}" "${SIMTIME[i]}"
#     read -p "pause... press Enter to resume"
     r1=${RUNS[i]%-*}
     r2=${RUNS[i]#*-}
     for ri in $(seq $r1 $r2); do
       # echo "$1 ${ri} ${SIMTIME[i]}"
       dbg --args ../src/aflora -m -u Cmdenv -c $1 -r ${ri} --result-dir=$RESULTDIR --sim-time-limit=${SIMTIME[i]} $OPPPARM  LoRaNetAnalysis.ini
     done
done

# new
# ../src/aflora -u Cmdenv -c S24m -r 0 -n ../src:../simulations:../../inet/examples:../../inet/src:../../inet/tutorials  -l ../../inet/src/INET -f LoRaNetAnalysis.ini

# old
#../src/flora -m -u Cmdenv -c S23b -r 0 --sim-time-limit=1d -n $NEDDIR --image-path=../../inet/images -l ../../inet/src/INET LoRaNetAnalysis.ini

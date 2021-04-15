
# Obstacle coverage
if true; then
#hermesFlag=1
#for obsNumber in 12 24 36 48 60 72 84 96 108
#do
#        for i in $(seq 0 30)
#        do
#                echo "ii: $i " >>  2019-2-19-obstacle-coverage-${obsNumber}.txt
#                for clientRS in $(seq 1 100)
#                do
#                        ./waf --run "scratch/Hermes++ --i=${i} --clientRS=${clientRS} --obsNumber=${obsNumber} --hermesFlag=${hermesFlag}" 2>> 2019-2-19-obstacle-coverage-${obsNumber}.txt
#                done
#        done
#done

# hermesFlag=0
i=1
# for obsNumber in 10 20 30 #10 20 30 #22 #6 12 17 22 28 34 40 46 52 58
# do
#         for i in $(seq 1 3)
#         do
                for ii in $(seq 1 $i)
                do
                        echo "ii: $ii " >>  2021-4-6-TGad-Functional-RORC.txt
                        for clientRS in $(seq 1 20)
                        do
                                ./waf --run "scratch/TGad_Functional_eval_living_room --ii=${ii} --clientRS=${clientRS}" 2>> 2021-4-6-TGad-Functional-RORC.txt
                        done
                done
        # done
# done
fi



# Multi-client
if false; then
obsNumber=24
hermesFlag=0
for clientNo in 1 3 6 9 12 15 18 21 24 27
do
    for i in $(seq 1 6)
    do
        for ii in $(seq 1 $i)
        do
                echo "ii: $ii " >>  2020-06-17-fixed-AP-${i}-client-${clientNo}.txt
                for clientRS in $(seq 1 30)
                do
                        ./waf --run "scratch/MultiAP_opt --i=${i} --ii=${ii} --clientRS=${clientRS} --hermesFlag=${hermesFlag} --clientNo=${clientNo} --obsNumber=${obsNumber}" 2>> 2020-06-17-fixed-AP-${i}-client-${clientNo}.txt
                done
        done
    done
done
fi



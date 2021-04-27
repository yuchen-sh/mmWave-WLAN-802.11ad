
if true; then

# numClientRS=2
# for obsNumber in 22 #6 12 17 22 28 34 40 46 52 58
# do
       # for i in $(seq 1)
        # do
                # for ii in $(seq 1 $i)
                # do
                        echo "Double-room Simulation " >>  2021-03-20-Multi-room-Multi-user.txt
                       # for clientRS in $(seq 1)
                       # do
                                ./waf --run "scratch/MultiAP_doubleRoom_withSmallOpen_simpleCase" 2>> 2021-03-20-Multi-room-Multi-user.txt
                        # done
                # done
        # done
# done
fi


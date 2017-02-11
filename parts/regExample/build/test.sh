#!/bin/bash 
for i in `seq 0.1 0.2 1`; do
    for j in `seq 0.2 0.2 1`; do
        for k in `seq 0.1 0.2 1`; do
            for l in `seq 0.1 0.2 1`; do
                printf "\n"
                ./regExample bla1.pcd bla2.pcd $i $j $k $l;
                printf "\nnext iteration \n\n\n\n\n"
            done
        done
    done
done

#!/bin/bash

iter=$1

if [ -z "$iter" ] #if iterations are not specified
then
    iter=5 #default number of iterations
fi

/home/root/powercycle.sh
./helloDSPgpp helloDSP.out $iter


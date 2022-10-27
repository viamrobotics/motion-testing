#!/bin/bash

TIME=20       # seconds
PLANNER=0     # 0 = RRT*, 1 = Informed RRT*

ROOT_DIR="<YOUR OMPL-EVALUATION DIRECTORY PATH>"  # Set this to whatever the absolute path of ompl-evaluation is
OMPL_RRT_RESULTS_DIR=$ROOT_DIR"results/ompl-RRTstar"
OMPL_INF_RRT_RESULTS_DIR=$ROOT_DIR"results/ompl-InfRRTstar"

PLANEVAL_EXEC="./build/planning_evaluator"

SCENES=("scene1" "scene2" "scene3" "scene4" "scene5" "scene6" "scene7" "scene8" "scene9")

for SCENE in ${SCENES[@]}
do
  echo
  echo "-------------------- Starting test iterations for [ $SCENE ] --------------------"

  for k in {1..10}
  do
    PLAN_FILE="$SCENE"".csv"
    PLAN_FILE_W_ITER="$SCENE"_"$k.csv"
    STATS_FILE="$SCENE"_"results.csv"
    STATS_FILE_W_ITER="$SCENE"_"$k"_"results.csv"

    cd $ROOT_DIR
    $PLANEVAL_EXEC $SCENE $TIME $PLANNER

    if [ $PLANNER -eq 0 ]; then
      cd $OMPL_RRT_RESULTS_DIR
    elif [ $PLANNER -eq 1 ]; then
      cd $OMPL_INF_RRT_RESULTS_DIR
    fi

    if [ -f $PLAN_FILE ]; then
      mv $PLAN_FILE $PLAN_FILE_W_ITER
    fi
    if [ -f $STATS_FILE ]; then
      mv $STATS_FILE $STATS_FILE_W_ITER
    fi
  done
done

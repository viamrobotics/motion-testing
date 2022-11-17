#!/bin/bash

TIME=20         # seconds
PLANNER=0       # 0 = RRT*, 1 = Informed RRT*, 2 = BIT*, 3 = ABIT*
K_MAX=100       # maximum number of tests to execute

ROOT_DIR="<YOUR OMPL-EVALUATION DIRECTORY HERE>"      # Set this to whatever the absolute path of ompl-evaluation is
PLANEVAL_EXEC=$ROOT_DIR"/build/planning_evaluator"     # Just goes right to the compiled C++ executable in the build dir

SCENES=("scene1" "scene2" "scene3" "scene4" "scene5" "scene6" "scene7" "scene8" "scene9" "scene10" "scene11" "scene12")

# Main execution block
echo "------------- Starting batch evaluation for OMPL / RDK integration -------------"
for SCENE in ${SCENES[@]}
do
  TITLE_OVERRIDE=()
  for ((k=1; k<=K_MAX; k++))
  do
    TITLE_OVERRIDE+=("$SCENE"_"$k")
  done

  echo "--------------- Starting [ $K_MAX ] test iterations for [ $SCENE ] ----------------"

  parallel -j 10 --link $PLANEVAL_EXEC ::: $SCENE ::: $TIME ::: $PLANNER ::: ${TITLE_OVERRIDE[@]}
done

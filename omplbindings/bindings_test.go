package main

import (
	"context"
	"fmt"
	"time"
	"testing"
	"math/rand"
	"os"
	"encoding/csv"
	"strconv"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"github.com/edaniels/golog"
	//~ "github.com/viamrobotics/visualization"
	//~ commonpb "go.viam.com/api/common/v1"
)

type plannerConstructor func(frame referenceframe.Frame, nCPU int, logger golog.Logger) (motionplan.MotionPlanner, error)
type seededPlannerConstructor func(frame referenceframe.Frame, nCPU int, seed *rand.Rand, logger golog.Logger) (motionplan.MotionPlanner, error)

//~ func TestPlanners(t *testing.T) {
	//~ planners := []plannerConstructor{
		//~ motionplan.NewRRTStarConnectMotionPlanner,
		//~ motionplan.NewCBiRRTMotionPlanner,
	//~ }
	
	//~ sceneName := "scene4"
	//~ for _, planner := range planners {
		
		//~ Init(sceneName)
		//~ cfg := scene
		//~ mp, err := planner(cfg.RobotFrame, 4, logger)
		//~ test.That(t, err, test.ShouldBeNil)
		//~ path, err := mp.Plan(context.Background(), spatialmath.PoseToProtobuf(cfg.Goal), cfg.Start, scenePlanOpts)
		//~ test.That(t, err, test.ShouldBeNil)
		//~ visualization.VisualizePlan(scene.RobotFrame, path, scene.WorldState)
		//~ fmt.Println(path)
	//~ }
//~ }

func plannerRun(t *testing.T, plannerFunc seededPlannerConstructor, plannerName string) {
	
	for sceneName, _ := range allScenes {
		for i := 1; i <= 10; i++{
			fmt.Println(sceneName)
			Init(sceneName)
			cfg := scene
			mp, err := plannerFunc(cfg.RobotFrame, 4, rand.New(rand.NewSource(int64(i))), logger)
			test.That(t, err, test.ShouldBeNil)
			start := time.Now()
			path, err := mp.Plan(context.Background(), spatialmath.PoseToProtobuf(cfg.Goal), cfg.Start, scenePlanOpts)
			test.That(t, err, test.ShouldBeNil)
			took := time.Since(start)
			
			//~ visualization.VisualizePlan(scene.RobotFrame, path, scene.WorldState)
			
			fmt.Println("took", took)
			//~ fmt.Println(path)
			f, err := os.Create("results/" + plannerName + "/" + sceneName + "_" + strconv.Itoa(i) + ".csv")
			test.That(t, err, test.ShouldBeNil)
			w := csv.NewWriter(f)
			for _, step := range path {
				stepStr := make([]string, 0, len(path))
				for _, joint := range step {
					stepStr = append(stepStr, fmt.Sprintf("%f", joint.Value))
				}
				w.Write(stepStr)
			}
			w.Flush()
			f.Close()
		}
	}
}

func TestCBiRRT(t *testing.T) {
	plannerRun(t, motionplan.NewCBiRRTMotionPlannerWithSeed, "cbirrt")
}

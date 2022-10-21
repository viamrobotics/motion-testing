package main

import (
	"context"
	"fmt"
	"testing"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"github.com/edaniels/golog"
	//~ commonpb "go.viam.com/api/common/v1"
)

type plannerConstructor func(frame referenceframe.Frame, nCPU int, logger golog.Logger) (motionplan.MotionPlanner, error)

func TestPlanners(t *testing.T) {
	planners := []plannerConstructor{
		//~ motionplan.NewRRTStarConnectMotionPlanner,
		motionplan.NewCBiRRTMotionPlanner,
	}
	
	sceneName := "scene4"
	for _, planner := range planners {
		
		Init(sceneName)
		cfg := scene
		mp, err := planner(cfg.RobotFrame, 4, logger)
		test.That(t, err, test.ShouldBeNil)
		path, err := mp.Plan(context.Background(), spatialmath.PoseToProtobuf(cfg.Goal), cfg.Start, scenePlanOpts)
		test.That(t, err, test.ShouldBeNil)
		fmt.Println(path)
	}
}

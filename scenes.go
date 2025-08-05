package main

import (
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
)

var logger = logging.NewLogger("motion-testing")

type sceneFunc func() (*armplanning.PlanRequest, error)

var allScenes = map[int]sceneFunc{
	// arm scenes
	1: armScene1,
	2: armScene2,
	3: armScene3,
	4: armScene4,
	5: armScene5,
	6: armScene6,
	7: armScene7,
	8: armScene8,
	9: armScene9,

	// base scenes
	// 13: baseScene1,
	// 14: baseScene2,
	// 15: baseScene3,
	// 16: baseScene4,
	// 17: baseScene5,
	// 18: baseScene6,
}

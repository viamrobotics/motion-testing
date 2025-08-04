package main

import (
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
)

var logger = logging.NewLogger("motion-testing")

type sceneFunc func() (*armplanning.PlanRequest, error)

var allScenes = map[int]sceneFunc{
	// arm scenes
	1:  scene1,
	2:  scene2,
	3:  scene3,
	4:  scene4,
	5:  scene5,
	6:  scene6,
	7:  scene7,
	8:  scene8,
	9:  scene9,
	10: scene10,
	11: scene11,
	12: scene12,

	// base scenes
	// 13: scene13,
	// 14: scene14,
	// 15: scene15,
	// 16: scene16,
	// 17: scene17,
	// 18: scene18,
}

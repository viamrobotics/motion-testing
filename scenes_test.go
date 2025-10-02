package motiontesting

import (
	"flag"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/test"
)

var nameFlag = flag.String("name", "", "name of test to run")

func TestDefault(t *testing.T) {
	logger := logging.NewTestLogger(t)

	name := *nameFlag
	if name == "" {
		name = "default"
	}
	test.That(t, RunScenes(name, armplanning.NewBasicPlannerOptions(), logger), test.ShouldBeNil)
}

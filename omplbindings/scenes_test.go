package main

import (
	"context"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"os"
	"strconv"
	"testing"
	"time"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

const numTests = 2
const timeout = 5.0 // seconds

var nameFlag = flag.String("name", "", "name of test to run")

func TestDefault(t *testing.T) {
	name := *nameFlag
	if name == "" {
		name = "default"
	}
	err := runScenes(t, name, map[string]interface{}{})
	test.That(t, err, test.ShouldBeNil)
}

func TestCBiRRT(t *testing.T) {
	name := *nameFlag
	if name == "" {
		name = "cbirrt"
	}
	err := runScenes(t, name, map[string]interface{}{
		"motion_profile": motionplan.FreeMotionProfile,
		"planning_alg":   "cbirrt",
	})
	test.That(t, err, test.ShouldBeNil)
}

func TestRRTStar(t *testing.T) {
	name := *nameFlag
	if name == "" {
		name = "rrt-star"
	}
	err := runScenes(t, name, map[string]interface{}{
		"planning_alg": "rrtstar",
	})
	test.That(t, err, test.ShouldBeNil)
}

func runScenes(t *testing.T, name string, options map[string]interface{}) error {
	outputFolder := "../results/" + name + "/"
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			return err
		}
	}

	for scene := range allScenes {
		if err := Init(scene); err != nil {
			return err
		}
		for i := 1; i <= numTests; i++ {
			testName := scene + "_" + strconv.Itoa(i)
			options["rseed"] = i
			options["timeout"] = timeout
			if err := runPlanner(outputFolder+testName, options); err != nil {
				return err
			}
		}
	}
	return nil
}

func runPlanner(fileName string, options map[string]interface{}) error {
	start := time.Now()

	// run planning query
	startMap := referenceframe.StartPositions(sceneFS)
	startMap[testArmFrame] = scene.Start
	planMap, err := motionplan.PlanMotion(
		context.Background(),
		logger,
		referenceframe.NewPoseInFrame("world", scene.Goal),
		sceneFS.Frame(testArmFrame),
		startMap,
		sceneFS,
		scene.WorldState,
		options,
	)

	// parse output
	success := "true"
	if err != nil {
		success = "false"
	}

	took := time.Since(start)

	// write stats file
	statsFile, err := os.Create(fileName + "_stats.txt")
	if err != nil {
		return err
	}
	defer statsFile.Close()

	w := csv.NewWriter(statsFile)
	defer w.Flush()
	w.Write([]string{success, fmt.Sprintf("%f", float64(took)/float64(time.Second))})

	// write solution to file
	csvFile, err := os.Create(fileName + ".csv")
	if err != nil {
		return err
	}
	defer csvFile.Close()

	if success == "true" {
		path, err := motionplan.FrameStepsFromRobotPath(testArmFrame, planMap)
		if err != nil {
			return err
		}
		w = csv.NewWriter(csvFile)
		defer w.Flush()
		for _, step := range path {
			stepStr := make([]string, 0, len(path))
			for _, joint := range step {
				stepStr = append(stepStr, fmt.Sprintf("%f", joint.Value))
			}
			w.Write(stepStr)
		}
	}
	return nil
}

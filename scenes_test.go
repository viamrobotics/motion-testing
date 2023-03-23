package main

import (
	"context"
	"encoding/csv"
	"errors"
	"flag"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
	"time"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/test"
)

const numTests = 10
const timeout = 5.0 // seconds

var nameFlag = flag.String("name", "", "name of test to run")

var resultsDirectory = "results"

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
	outputFolder := filepath.Join(resultsDirectory, name)
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		// TODO(rb): potentially create a temp directory to be storing these files
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			return err
		}
	}

	for sceneNum := range allScenes {
		if err := initScene(sceneNum); err != nil {
			return err
		}
		for i := 1; i <= numTests; i++ {
			options["rseed"] = i
			options["timeout"] = timeout
			if err := runPlanner(filepath.Join(outputFolder, "scene"+strconv.Itoa(sceneNum)+"_"+strconv.Itoa(i)), options); err != nil {
				return err
			}
		}
	}

	// Create SHA-containing file for this execution in the output folder
	err := generateHashFile(outputFolder)
	if err != nil {
		return err
	}

	return nil
}

func runPlanner(fileName string, options map[string]interface{}) error {
	start := time.Now()

	// run planning query
	startMap := referenceframe.StartPositions(scene.FrameSystem)
	startMap[scene.FrameToPlan] = scene.Start
	planMap, err := motionplan.PlanMotion(
		context.Background(),
		logger,
		referenceframe.NewPoseInFrame("world", scene.Goal),
		scene.FrameSystem.Frame(scene.FrameToPlan),
		startMap,
		scene.FrameSystem,
		scene.WorldState,
		nil,
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
		path, err := motionplan.FrameStepsFromRobotPath(scene.FrameToPlan, planMap)
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

func generateHashFile(folder string) error {
	hashFile, err := os.Create(filepath.Join(folder, "hash"))
	if err != nil {
		return err
	}
	defer hashFile.Close()

	//nolint:gosec
	cmd := exec.Command("git", "rev-parse", "HEAD")
	out, err := cmd.CombinedOutput()
	if err != nil {
		if len(out) != 0 {
			return fmt.Errorf("error running git rev-parse HEAD")
		}
		return err
	}

	hash := strings.TrimSpace(string(out))
	_, err = hashFile.WriteString(hash)
	if err != nil {
		return err
	}

	return nil
}

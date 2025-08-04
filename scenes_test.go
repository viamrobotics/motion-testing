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

	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/test"
)

const numTests = 1
const timeout = 5.0 // seconds

var nameFlag = flag.String("name", "", "name of test to run")

var resultsDirectory = "results"

func TestDefault(t *testing.T) {
	name := *nameFlag
	if name == "" {
		name = "default"
	}
	defaultOptions := armplanning.NewBasicPlannerOptions()
	defaultOptions.Timeout = timeout
	test.That(t, runScenes(name, defaultOptions), test.ShouldBeNil)
}

func runScenes(name string, options *armplanning.PlannerOptions) error {
	outputFolder := filepath.Join(resultsDirectory, name)
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		// TODO(rb): potentially create a temp directory to be storing these files
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			return err
		}
	}

	for sceneNum, scene := range allScenes {
		for i := 1; i <= numTests; i++ {
			req, err := scene()
			if err != nil {
				return err
			}
			req.PlannerOptions = options
			req.PlannerOptions.RandomSeed = i
			if err := runPlanner(filepath.Join(outputFolder, "scene"+strconv.Itoa(sceneNum)+"_"+strconv.Itoa(i)), req); err != nil {
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

// TODO: these options need to be integrated into the planner options
func runPlanner(fileName string, req *armplanning.PlanRequest) error {
	start := time.Now()

	// run planning query
	plan, err := armplanning.PlanMotion(context.Background(), logger, req)

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
	fName := "test_base"
	if _, ok := req.StartState.Configuration()[fName]; !ok {
		fName = "arm"
	}

	if success == "true" {
		path, err := plan.Trajectory().GetFrameInputs(fName)
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

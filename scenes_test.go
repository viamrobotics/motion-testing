package main

import (
	"context"
	"encoding/csv"
	"encoding/json"
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

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/test"
)

const numTests = 10

var nameFlag = flag.String("name", "", "name of test to run")

var resultsDirectory = "results"

func TestDefault(t *testing.T) {
	name := *nameFlag
	if name == "" {
		name = "default"
	}
	test.That(t, runScenes(t, name, armplanning.NewBasicPlannerOptions()), test.ShouldBeNil)
}

func runScenes(t *testing.T, name string, options *armplanning.PlannerOptions) error {
	outputFolder := filepath.Join(resultsDirectory, name)
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		// TODO(rb): potentially create a temp directory to be storing these files
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			return err
		}
	}

	for sceneNum, scene := range allScenes {
		t.Run(fmt.Sprintf("scene%d", sceneNum), func(t *testing.T) {
			logger := logging.NewTestLogger(t)
			for i := 1; i <= numTests; i++ {
				req, err := scene(logger)
				test.That(t, err, test.ShouldBeNil)

				req.PlannerOptions = options
				req.PlannerOptions.RandomSeed = i
				runPlanner(filepath.Join(outputFolder, "scene"+strconv.Itoa(sceneNum)+"_"+strconv.Itoa(i)), req, logger)
				test.That(t, err, test.ShouldBeNil)
			}
		})
	}

	// Create SHA-containing file for this execution in the output folder
	err := generateHashFile(outputFolder)
	if err != nil {
		return err
	}

	return nil
}

func writePlanRequest(filePrefix string, req *armplanning.PlanRequest) error {
	fn := filePrefix + "_request.json"

	data, err := json.MarshalIndent(req, "", "  ")
	if err != nil {
		return err
	}
	file, err := os.OpenFile(filepath.Clean(fn), os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0o600)
	if err != nil {
		return err
	}
	defer file.Close()
	_, err = file.Write(data)
	if err != nil {
		return err
	}
	return nil
}

func runPlanner(fileName string, req *armplanning.PlanRequest, logger logging.Logger) error {

	err := writePlanRequest(fileName, req)
	if err != nil {
		return err
	}

	start := time.Now()

	// run planning query
	plan, _, err := armplanning.PlanMotion(context.Background(), logger, req)

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

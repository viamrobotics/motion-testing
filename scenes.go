// Package motiontesting runs repeatable motion planning scenes and scores the resulting
// trajectories so that changes to Viam's motion planning code can be benchmarked against
// each other.
package motiontesting

import (
	"context"
	"encoding/csv"
	"errors"
	"fmt"
	"maps"
	"os"
	"os/exec"
	"path/filepath"
	"slices"
	"strconv"
	"strings"
	"time"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
)

const resultsDirectory = "results"

type sceneFunc func(logger logging.Logger) (*armplanning.PlanRequest, error)

var allScenes = map[int]sceneFunc{
	// arm scenes
	1:  armScene1,
	2:  armScene2,
	3:  armScene3,
	4:  armScene4,
	5:  armScene5,
	6:  armScene6,
	7:  armScene7,
	8:  armScene8,
	9:  armScene9,
	10: armSceneFile("data/sanding1.json"),
	11: armSceneFile("data/sanding-stroke1.json"),
	12: armSceneFile("data/salad1.json"),
}

var numTests = len(allScenes)

// sceneNumbers returns the scene ids in ascending order. Scenes must always be planned in the same
// order: armplanning keeps process-global state (the roadmap harvests waypoints from every
// successful plan), so a scene's result depends on what was planned before it, and Go randomises
// map iteration per process.
func sceneNumbers() []int {
	return slices.Sorted(maps.Keys(allScenes))
}

// RunScenes plans every scene numTests times with the given planner options, writing the
// request, trajectory and timing files for each run into results/<name>.
func RunScenes(name string, options *armplanning.PlannerOptions, logger logging.Logger) error {
	sceneLogger := logger.Sublogger("RunScenes logger")
	sceneLogger.SetLevel(logging.INFO)

	outputFolder := filepath.Join(resultsDirectory, name)
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		// TODO(rb): potentially create a temp directory to be storing these files
		err := os.MkdirAll(outputFolder, 0o750)
		if err != nil {
			return err
		}
	}

	for _, sceneNum := range sceneNumbers() {
		scene := allScenes[sceneNum]
		logger.Warnf("starting sceneNum: %d", sceneNum)
		logger := sceneLogger.Sublogger(fmt.Sprintf("scene_%d", sceneNum))
		req, err := scene(logger)
		if err != nil {
			return fmt.Errorf("scene failed for sceneNum: %d : %w", sceneNum, err)
		}

		// Run once to load caches; this plan's outcome is deliberately discarded.
		_, _, _ = armplanning.PlanMotion(context.Background(), logger, req)

		for i := 1; i <= numTests; i++ {
			logger.Infof("sceneNum: %d iteration: %d", sceneNum, i)

			req.PlannerOptions = options
			req.PlannerOptions.RandomSeed = i
			err = runPlanner(filepath.Join(outputFolder, "scene"+strconv.Itoa(sceneNum)+"_"+strconv.Itoa(i)), req, logger)
			if err != nil {
				return fmt.Errorf("runPlanner failed for sceneNum: %d : %w", sceneNum, err)
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

func runPlanner(fileName string, req *armplanning.PlanRequest, logger logging.Logger) error {
	err := req.WriteToFile(fileName + "_request.json")
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
	if err := writeCSV(fileName+"_stats.txt", [][]string{{success, fmt.Sprintf("%f", took.Seconds())}}); err != nil {
		return err
	}

	fName := ""

	for _, n := range []string{"test_base", "arm", "ur20-modular"} {
		_, ok := req.StartState.Configuration()[n]
		if ok {
			fName = n
			break
		}
	}

	if fName == "" {
		return fmt.Errorf("can't figure out what to move")
	}

	var solution [][]string
	if success == "true" {
		path, err := plan.Trajectory().GetFrameInputs(fName)
		if err != nil {
			return err
		}
		for _, step := range path {
			stepStr := make([]string, 0, len(step))
			for _, joint := range step {
				stepStr = append(stepStr, fmt.Sprintf("%f", joint))
			}
			solution = append(solution, stepStr)
		}
	}

	// write solution to file
	return writeCSV(fileName+".csv", solution)
}

// writeCSV writes records to fileName, surfacing flush and close errors so that a truncated
// results file cannot be silently scored later.
func writeCSV(fileName string, records [][]string) (err error) {
	f, err := os.Create(fileName)
	if err != nil {
		return err
	}
	defer func() {
		if closeErr := f.Close(); err == nil {
			err = closeErr
		}
	}()

	w := csv.NewWriter(f)
	return w.WriteAll(records)
}

func generateHashFile(folder string) error {
	cmd := exec.CommandContext(context.Background(), "git", "rev-parse", "HEAD")
	out, err := cmd.CombinedOutput()
	if err != nil {
		if len(out) != 0 {
			return fmt.Errorf("error running git rev-parse HEAD: %s", out)
		}
		return err
	}

	hash := strings.TrimSpace(string(out))
	return os.WriteFile(filepath.Join(folder, "hash"), []byte(hash), 0o600)
}

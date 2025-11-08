package motiontesting

import (
	"context"
	"encoding/csv"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
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
	11: armSceneFile("data/sanding-esha1.json"),

	// base scenes
	// 13: baseScene1,
	// 14: baseScene2,
	// 15: baseScene3,
	// 16: baseScene4,
	// 17: baseScene5,
	// 18: baseScene6,
}

var numTests = len(allScenes)

func RunScenes(name string, options *armplanning.PlannerOptions, logger logging.Logger) error {
	outputFolder := filepath.Join(resultsDirectory, name)
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		// TODO(rb): potentially create a temp directory to be storing these files
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			return err
		}
	}

	for sceneNum, scene := range allScenes {
		req, err := scene(logger)
		if err != nil {
			return fmt.Errorf("scene failed for sceneNum: %d : %w", sceneNum, err)
		}

		armplanning.PlanMotion(context.Background(), logger, req) // run once to load caches

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
	statsFile, err := os.Create(fileName + "_stats.txt")
	if err != nil {
		return err
	}
	defer statsFile.Close()

	w := csv.NewWriter(statsFile)
	defer w.Flush()
	w.Write([]string{success, fmt.Sprintf("%f", took.Seconds())})

	// write solution to file
	csvFile, err := os.Create(fileName + ".csv")
	if err != nil {
		return err
	}
	defer csvFile.Close()

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
				stepStr = append(stepStr, fmt.Sprintf("%f", joint))
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

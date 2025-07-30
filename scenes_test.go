package scenes

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
const timeout = 5.0 // seconds

var nameFlag = flag.String("name", "", "name of test to run")

var resultsDirectory = "results"

func TestPlanRequestRoundTrip(t *testing.T) {
	pr, err := scene5(context.Background(), logging.NewLogger("test-nick"))
	test.That(t, err, test.ShouldBeNil)
	prBytes, err := json.Marshal(pr)
	test.That(t, err, test.ShouldBeNil)
	var pr2 armplanning.PlanRequest
	test.That(t, json.Unmarshal(prBytes, &pr2), test.ShouldBeNil)
	test.That(t, pr.BoundingRegions, test.ShouldResemble, pr2.BoundingRegions)
	test.That(t, pr.Constraints, test.ShouldResemble, pr2.Constraints)
	test.That(t, pr.Goals, test.ShouldResemble, pr2.Goals)
	test.That(t, pr.PlannerOptions, test.ShouldResemble, pr2.PlannerOptions)
	test.That(t, pr.StartState, test.ShouldResemble, pr2.StartState)
	test.That(t, pr.WorldState, test.ShouldResemble, pr2.WorldState)
	test.That(t, pr.FrameSystem, test.ShouldResemble, pr2.FrameSystem)
}

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
		"motion_profile": armplanning.FreeMotionProfile,
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
		for i := 1; i <= numTests; i++ {
			if err := initScene(sceneNum); err != nil {
				return err
			}
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

// TODO: these options need to be integrated into the planner options
func runPlanner(fileName string, options map[string]interface{}) error {
	start := time.Now()
	jsonBytes, err := json.Marshal(scene)
	if err != nil {
		return err
	}
	if err := os.WriteFile(fileName+"_plan.json", jsonBytes, 0o666); err != nil {
		return err
	}

	// run planning query
	plan, err := armplanning.PlanMotion(context.Background(), logger, scene)

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
	if _, ok := scene.StartState.Configuration()[fName]; !ok {
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

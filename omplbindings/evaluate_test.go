package main

import (
	"encoding/csv"
	"flag"
	"fmt"
	"math"
	"os"
	"path"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"github.com/montanaflynn/stats"
	"go.viam.com/test"
)

type testResult struct {
	name  string
	score map[string]*testScore
}

type testScore struct {
	successes    int
	qualities    stats.Float64Data
	performances stats.Float64Data
	runs         int
}

var baselineFlag = flag.String("baselineDir", "cbirrt", "name of test to use as a baseline")
var modifiedFlag = flag.String("modifiedDir", "rrt-star", "name of test to compare to the baseline")

func TestScores(t *testing.T) {
	baseline, err := scoreFolder(*baselineFlag)
	test.That(t, err, test.ShouldBeNil)
	modification, err := scoreFolder(*modifiedFlag)
	test.That(t, err, test.ShouldBeNil)

	// compare folders with results
	test.That(t, compareResults(baseline, modification), test.ShouldBeNil)
}

func scoreFolder(folder string) (*testResult, error) {
	fullPath := filepath.Join(resultsDirectory, folder)
	fileInfo, err := os.Stat(fullPath)
	if err != nil || !fileInfo.IsDir() {
		return nil, fmt.Errorf("could not open folder: %s", fullPath)
	}

	runs, err := os.ReadDir(fullPath)
	if err != nil {
		return nil, err
	}

	f, err := os.Create(filepath.Join(fullPath, "results.csv"))
	if err != nil {
		return nil, err
	}
	defer f.Close()

	w := csv.NewWriter(f)
	defer w.Flush()
	w.Write([]string{"scene", "seed", "success", "time", "total_score", "joint_score", "line_score", "orient_score"})

	results := &testResult{
		name:  folder,
		score: make(map[string]*testScore, 0),
	}
	for _, run := range runs {
		if path.Ext(run.Name()) == ".txt" {
			fmt.Println(run.Name())
			res := strings.Split(run.Name(), "_")
			sceneNum := res[0]
			seed := res[1]

			bytes, err := os.ReadFile(filepath.Join(fullPath, run.Name()))
			if err != nil {
				return nil, err
			}

			rundata := string(bytes)
			res = strings.Split(rundata, ",")
			pass := res[0]

			time, err := strconv.ParseFloat(strings.TrimSpace(res[1]), 64)
			if err != nil {
				return nil, err
			}

			// Parse whether was successful or not
			if pass == "1" {
				pass = "true"
				time /= 1e9
			}
			if pass == "0" {
				pass = "false"
				time /= 1e9
			}

			// Only process paths that are valid
			score, ok := results.score[sceneNum]
			if !ok {
				score = &testScore{
					qualities:    make(stats.Float64Data, 0),
					performances: make(stats.Float64Data, 0),
				}
			}
			score.runs += 1
			if pass == "true" {
				data, err := readSolutionFromCSV(filepath.Join(fullPath, sceneNum+"_"+seed+".csv"))
				if err != nil {
					return nil, err
				}

				jScore, tScore, oScore, err := evaluateSolution(data, sceneNum)
				if err != nil {
					return nil, err
				}
				totalScore := jScore + tScore + oScore

				w.Write([]string{
					sceneNum,
					seed,
					pass,
					fmt.Sprintf("%f", time),
					fmt.Sprintf("%f", totalScore),
					fmt.Sprintf("%f", jScore),
					fmt.Sprintf("%f", tScore),
					fmt.Sprintf("%f", oScore),
				})

				score.successes += 1
				score.qualities = append(score.qualities, totalScore)
				score.performances = append(score.performances, time)
			} else {
				w.Write([]string{
					sceneNum,
					seed,
					pass,
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
				})
			}
			results.score[sceneNum] = score
		}
	}
	return results, nil
}

func compareResults(baseline, modification *testResult) error {
	f, err := os.Create(filepath.Join(resultsDirectory, "comparison.md"))
	if err != nil {
		return err
	}
	defer f.Close()

	var builder strings.Builder

	builder.WriteString(fmt.Sprintf("##Availability\n| Scene | %s | %s | Percent Difference | \n", baseline.name, modification.name))
	for name := range allScenes {
		builder.WriteString(availabilityEntry(name, baseline.score[name].successes, modification.score[name].successes))
	}

	builder.WriteString(fmt.Sprintf("##Quality\n| Scene | %s | %s | Percent Difference | \n", baseline.name, modification.name))
	for name := range allScenes {
		builder.WriteString(qualityEntry(name, baseline.score[name].qualities, modification.score[name].qualities))
	}

	f.WriteString(builder.String())
	return nil
}

func readSolutionFromCSV(filepath string) ([][]float64, error) {
	csvfile, err := os.Open(filepath)

	if err != nil {
		return nil, err
	}
	defer csvfile.Close()

	reader := csv.NewReader(csvfile)
	fields, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	solution := [][]float64{}
	for _, waypoint := range fields {
		step := make([]float64, 0, len(waypoint))
		for _, pos := range waypoint {
			posF, err := strconv.ParseFloat(pos, 64)
			if err != nil {
				return nil, err
			}
			step = append(step, posF)
		}
		solution = append(solution, step)
	}

	return solution, nil
}

func availabilityEntry(entry string, initial, final int) string {
	return fmt.Sprintf("| %s | %d | %d | %.0f%% | \n", entry, initial, final, 100.0*(float64(final-initial))/float64(initial))
}

func qualityEntry(entry string, initial, final stats.Float64Data) string {
	initialMean, initialStdDev := normalDistribution(initial)
	finalMean, finalStdDev := normalDistribution(final)

	// compare two distributions directly using Z-test
	zScore := (initialMean - finalMean) / math.Sqrt(initialStdDev*initialStdDev+finalStdDev*finalStdDev)
	return fmt.Sprintf("| %s | %f\u00B1%f | %f\u00B1%f | %f | \n", entry, initialMean, initialStdDev, finalMean, finalStdDev, zScore)
}

func normalDistribution(data stats.Float64Data) (float64, float64) {
	mean, err := data.Mean()
	if err != nil {
		return math.NaN(), math.NaN()
	}
	stdDev, err := data.StandardDeviation()
	if err != nil {
		return math.NaN(), math.NaN()
	}
	return mean, stdDev
}

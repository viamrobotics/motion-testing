package main

import (
	"encoding/csv"
	"errors"
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
	"gonum.org/v1/gonum/stat/distuv"
)

type testResult struct {
	name  string
	score map[int]*testScore
	sha1  string
}

type testScore struct {
	successes    float64
	qualities    stats.Float64Data
	performances stats.Float64Data
}

const nilFolder = ""

// flags used to define folders insider results folder to compare
var baselineFlag = flag.String("baselineDir", nilFolder, "name of test to use as a baseline")
var modifiedFlag = flag.String("modifiedDir", nilFolder, "name of test to compare to the baseline")

// these variables represent the lower and higher bounds (exclusive) for unacceptable and acceptable values respectively
var percentImprovementHealthThresholds = [2]float64{0, 0}
var probabilityImprovementHealthThresholds = [2]float64{16, 84} // numbers derive from first standard deviation of normal distribution

func TestScores(t *testing.T) {
	baseline, err := scoreFolder(*baselineFlag)
	test.That(t, err, test.ShouldBeNil)
	modification, err := scoreFolder(*modifiedFlag)
	test.That(t, err, test.ShouldBeNil)

	// compare folders with results
	test.That(t, compareResults(baseline, modification), test.ShouldBeNil)
}

func scoreFolder(folder string) (*testResult, error) {
	if folder == nilFolder {
		return nil, errors.New("folder not specified for flag baselineDir or modifiedDir")
	}
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

	hashBytes, err := os.ReadFile(filepath.Join(fullPath, "hash"))
	if err != nil {
		return nil, err
	}

	results := &testResult{
		name:  folder,
		score: make(map[int]*testScore, 0),
		sha1:  string(hashBytes),
	}
	for _, run := range runs {
		if path.Ext(run.Name()) == ".txt" {
			// parse file name to determine what the test parameters were
			fileName := strings.Split(run.Name(), "_")
			sceneNum, err := strconv.Atoi(strings.ReplaceAll(fileName[0], "scene", ""))
			if err != nil {
				return nil, err
			}

			// read the file and get the results of the run
			bytes, err := os.ReadFile(filepath.Join(fullPath, run.Name()))
			if err != nil {
				return nil, err
			}
			rundata := strings.Split(string(bytes), ",")

			// Parse whether was successful or not
			pass := rundata[0]

			// Parse time it took to complete
			time, err := strconv.ParseFloat(strings.TrimSpace(rundata[1]), 64)
			if err != nil {
				return nil, err
			}

			score, ok := results.score[sceneNum]
			if !ok {
				score = &testScore{
					qualities:    make(stats.Float64Data, 0),
					performances: make(stats.Float64Data, 0),
				}
			}
			if pass == "true" {
				data, err := readSolutionFromCSV(filepath.Join(fullPath, fileName[0]+"_"+fileName[1]+".csv"))
				if err != nil {
					return nil, err
				}

				jScore, tScore, oScore, err := evaluateSolution(data, allScenes[sceneNum])
				if err != nil {
					return nil, err
				}

				w.Write([]string{
					fileName[0],
					fileName[1],
					pass,
					fmt.Sprintf("%f", time),
					fmt.Sprintf("%f", jScore+tScore+oScore),
					fmt.Sprintf("%f", jScore),
					fmt.Sprintf("%f", tScore),
					fmt.Sprintf("%f", oScore),
				})

				score.successes += 1
				score.qualities = append(score.qualities, jScore)
				score.performances = append(score.performances, time)
			} else {
				w.Write([]string{
					fileName[0],
					fileName[1],
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
	f, err := os.Create(filepath.Join(resultsDirectory, "motion-benchmarks.md"))
	if err != nil {
		return err
	}
	defer f.Close()

	var builder strings.Builder

	builder.WriteString(tableHeaderInts("Availability", baseline.name, modification.name))
	for i := 1; i <= len(allScenes); i++ {
		builder.WriteString(tableEntryInt(i, baseline.score[i].successes, modification.score[i].successes))
	}

	builder.WriteString(tableHeaderFloats("Quality", baseline.name, modification.name))
	for i := 1; i <= len(allScenes); i++ {
		builder.WriteString(tableEntryFloats(i, baseline.score[i].qualities, modification.score[i].qualities))
	}

	builder.WriteString(tableHeaderFloats("Performance", baseline.name, modification.name))
	for i := 1; i <= len(allScenes); i++ {
		builder.WriteString(tableEntryFloats(i, baseline.score[i].performances, modification.score[i].performances))
	}

	builder.WriteString("\nThe above data was generated by running scenes defined in the " +
		"[`motion-testing`](https://github.com/viamrobotics/motion-testing/) repository")
	builder.WriteString(fmt.Sprintf("\nThe SHA1 for %s is: %s", baseline.name, baseline.sha1))
	builder.WriteString(fmt.Sprintf("\nThe SHA1 for %s is: %s", modification.name, modification.sha1))
	builder.WriteString(fmt.Sprintf("\n* **%d samples** were taken for each scene", numTests))
	builder.WriteString(fmt.Sprintf("\n* A timeout of **%.1f seconds** was imposed for each trial", timeout))

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

func tableHeaderInts(name, baseline, modification string) string {
	formatLine := "| :---: | :----: | :---: | :----: | :---: |\n"
	return fmt.Sprintf("\n## %s\n| Scene # | %s | %s | Percent Improvement | Health | \n",
		name,
		baseline,
		modification,
	) + formatLine
}

func tableEntryInt(sceneNum int, initial, final float64) string {
	delta := percentDifference(initial, final)
	return fmt.Sprintf("| %d | %.0f%% | %.0f%% | %.0f%% | %c | \n",
		sceneNum,
		100*initial/numTests,
		100*final/numTests,
		delta,
		healthIndicator(delta, percentImprovementHealthThresholds),
	)
}

func tableHeaderFloats(name, baseline, modification string) string {
	formatLine := "| :---: | :----: | :---: | :---: | :----: | :---: |\n"
	return fmt.Sprintf("\n## %s\n| Scene # | %s | %s | Percent Improvement | Probability of Improvement | Health | \n",
		name,
		baseline,
		modification,
	) + formatLine
}

func tableEntryFloats(sceneNum int, initial, final stats.Float64Data) string {
	// create normal distributions from initial and final float slices
	A, AValid := normal(initial)
	B, BValid := normal(final)

	var probability float64
	switch {
	case AValid && BValid:
		// create normal distribution C = B - A
		C := distuv.Normal{
			Mu:    B.Mu - A.Mu,
			Sigma: math.Sqrt(A.Sigma*A.Sigma + B.Sigma*B.Sigma),
		}

		switch {
		case C.Sigma != 0:
			// probability that B is an improvement over A is found by evaluating the CDF at x=0
			probability = 100 * C.CDF(0)
		case C.Mu < 0:
			probability = 100
		case C.Mu > 0:
			probability = 0
		default:
			probability = 50
		}
	case AValid && !BValid:
		probability = 0
	case !AValid && BValid:
		probability = 100
	default:
		probability = math.NaN()
	}

	delta := percentDifference(A.Mu, B.Mu)
	return fmt.Sprintf("| %d | %.2f\u00B1%.2f | %.2f\u00B1%.2f | %.0f%% | %.0f%% | %c | \n",
		sceneNum,
		A.Mu, A.Sigma,
		B.Mu, B.Sigma,
		-delta, // want to flip it so its an improvement if its less
		probability,
		healthIndicator(probability, probabilityImprovementHealthThresholds),
	)
}

func percentDifference(initial, final float64) float64 {
	return 100.0 * (final - initial) / initial
}

// normal makes a normal distribution from the float slice
func normal(data stats.Float64Data) (distuv.Normal, bool) {
	mean, err1 := data.Mean()
	stdDev, err2 := data.StandardDeviation()
	if err1 != nil || err2 != nil {
		return distuv.Normal{Mu: math.NaN(), Sigma: math.NaN()}, false
	}
	return distuv.Normal{Mu: mean, Sigma: stdDev}, true
}

func healthIndicator(data float64, thresholds [2]float64) rune {
	switch {
	case data < thresholds[0]:
		return '❌'
	case data > thresholds[1]:
		return '✅'
	default:
		return '➖'
	}
}

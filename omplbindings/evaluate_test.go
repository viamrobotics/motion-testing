package main

import (
	"encoding/csv"
	"fmt"
	"os"
	"path"
	"path/filepath"
	"strconv"
	"strings"
	"testing"

	"go.viam.com/test"
)

var statHeader = []string{"scene", "seed", "success", "time", "total_score", "joint_score", "line_score", "orient_score"}

func TestScores(t *testing.T) {
	parentDir := filepath.Join("..", "results")
	files, err := os.ReadDir(parentDir)
	test.That(t, err, test.ShouldBeNil)

	for _, file := range files {
		if file.IsDir() {
			err := scoreFolder(filepath.Join(parentDir, file.Name()))
			test.That(t, err, test.ShouldBeNil)
		}
	}
}

func scoreFolder(folder string) error {
	runs, err := os.ReadDir(folder)
	if err != nil {
		return err
	}

	f, err := os.Create(filepath.Join(folder, "results.csv"))
	if err != nil {
		return err
	}
	defer f.Close()

	w := csv.NewWriter(f)
	defer w.Flush()
	w.Write(statHeader)

	for _, run := range runs {
		if path.Ext(run.Name()) == ".txt" {
			fmt.Println(run.Name())
			res := strings.Split(run.Name(), "_")
			sceneNum := res[0]
			seed := res[1]

			bytes, err := os.ReadFile(filepath.Join(folder, run.Name()))
			if err != nil {
				return err
			}

			rundata := string(bytes)
			res = strings.Split(rundata, ",")
			pass := res[0]

			time, err := strconv.ParseFloat(strings.TrimSpace(res[1]), 64)
			if err != nil {
				return err
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
			if pass == "true" {
				data, err := readSolutionFromCSV(filepath.Join(folder, sceneNum+"_"+seed+".csv"))
				if err != nil {
					return err
				}

				jscore, tscore, oscore, err := evaluateSolution(data, sceneNum)
				if err != nil {
					return err
				}

				w.Write([]string{
					sceneNum,
					seed,
					pass,
					fmt.Sprintf("%f", time),
					fmt.Sprintf("%f", jscore+tscore+oscore),
					fmt.Sprintf("%f", jscore),
					fmt.Sprintf("%f", tscore),
					fmt.Sprintf("%f", oscore),
				})
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
		}
	}
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

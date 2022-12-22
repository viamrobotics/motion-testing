package main

import (
	"context"
	"encoding/csv"
	"errors"
	"fmt"
	"math"
	"os"
	"path"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"gonum.org/v1/gonum/floats"
)

const defaultEpsilon = 0.01

func runScenes(t *testing.T, name string, options map[string]interface{}) {
	outputFolder := "../results/" + name + "/"
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			fmt.Println(err)
		}
	}

	for scene := range allScenes {
		Init(scene)
		for i := 1; i <= 10; i++ {
			testName := scene + "_" + strconv.Itoa(i)
			t.Run(testName, func(t *testing.T) {
				runPlanner(t, outputFolder+testName, options)
			})
		}
	}
}

func runPlanner(t *testing.T, fileName string, options map[string]interface{}) {
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
	test.That(t, err, test.ShouldBeNil)

	// parse output
	success := "true"
	if err != nil {
		success = "false"
	}

	took := time.Since(start)

	// write stats file
	statsFile, err := os.Create(fileName + "_stats.txt")
	test.That(t, err, test.ShouldBeNil)
	defer statsFile.Close()

	w := csv.NewWriter(statsFile)
	defer w.Flush()
	w.Write([]string{success, fmt.Sprintf("%f", float64(took)/float64(time.Second))})

	// write solution to file
	csvFile, err := os.Create(fileName + ".csv")
	test.That(t, err, test.ShouldBeNil)
	defer csvFile.Close()

	if success == "true" {
		path, err := motionplan.FrameStepsFromRobotPath(testArmFrame, planMap)
		test.That(t, err, test.ShouldBeNil)

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
}

func TestVizPlan(t *testing.T) {
	Init("scene7")
	//~ file := "/home/peter/Documents/echo/ompl-evaluation/results/ompl5/scene3_4.csv"
	//~ file := "/home/peter/Documents/echo/ompl-evaluation/results/cbrt_ps/scene6_25.csv"

	//~ file := "/home/peter/Documents/echo/ompl-evaluation/results/rrtStarConnect_parcol/scene8_3.csv"
	file := "/home/peter/Documents/echo/ompl-evaluation/results/cbrt_fast_impr/scene7_3.csv"
	data, err := readCSV(file)
	test.That(t, err, test.ShouldBeNil)
	VisualizeOMPL(data)
}

func TestCBiRRT(t *testing.T) {
	runScenes(t, "cbirtt", map[string]interface{}{
		"motion_profile": motionplan.FreeMotionProfile,
		"planning_alg":   "cbirrt",
	})
}

func TestRRTStar(t *testing.T) {
	runScenes(t, "rrt", map[string]interface{}{
		"planning_alg": "rrtstar",
	})
}

func TestPlanScoring(t *testing.T) {
	outputFolder := "../results/"
	folders, err := os.ReadDir(outputFolder)
	test.That(t, err, test.ShouldBeNil)

	f, err := os.Create(outputFolder + "results.csv")
	test.That(t, err, test.ShouldBeNil)
	w := csv.NewWriter(f)
	w.Write([]string{"alg", "scene", "seed", "success", "time", "total_score", "joint_score", "line_score", "orient_score"})
	for _, alg := range folders {
		if alg.IsDir() && alg.Name() != "archive" {
			runs, err := os.ReadDir(outputFolder + alg.Name())
			test.That(t, err, test.ShouldBeNil)
			for _, run := range runs {
				sceneNum := ""
				seed := ""

				if path.Ext(run.Name()) == ".txt" {
					fmt.Println(run.Name())
					res := strings.Split(run.Name(), "_")
					sceneNum = res[0]
					seed = res[1]

					b, err := os.ReadFile(outputFolder + alg.Name() + "/" + run.Name())
					test.That(t, err, test.ShouldBeNil)
					rundata := string(b)
					res = strings.Split(rundata, ",")
					pass := res[0]

					time, err := strconv.ParseFloat(strings.TrimSpace(res[1]), 64)
					test.That(t, err, test.ShouldBeNil)

					// Fix ompl
					if pass == "1" {
						pass = "true"
						time /= 1e9
					}
					if pass == "0" {
						pass = "false"
						time /= 1e9
					}
					//~ fmt.Println(sceneNum, seed, pass, time)
					jscore := -1.
					tscore := -1.
					oscore := -1.
					if pass == "true" {
						// Only process paths that are valid.
						data, err := readCSV(outputFolder + alg.Name() + "/" + sceneNum + "_" + seed + ".csv")
						test.That(t, err, test.ShouldBeNil)
						jscore, tscore, oscore, err = processPath(data, sceneNum)
						test.That(t, err, test.ShouldBeNil)
						w.Write([]string{
							alg.Name(),
							sceneNum,
							seed,
							pass,
							fmt.Sprintf("%f", time),
							fmt.Sprintf("%f", jscore+tscore+oscore),
							fmt.Sprintf("%f", jscore),
							fmt.Sprintf("%f", tscore),
							fmt.Sprintf("%f", oscore)})
					} else {
						w.Write([]string{
							alg.Name(),
							sceneNum,
							seed,
							pass,
							fmt.Sprintf("%f", -1.),
							fmt.Sprintf("%f", -1.),
							fmt.Sprintf("%f", jscore),
							fmt.Sprintf("%f", tscore),
							fmt.Sprintf("%f", oscore)})
					}
				}
			}
		}
	}
	w.Flush()
	f.Close()
}

func readCSV(filepath string) ([][]float64, error) {
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

	path := [][]float64{}
	for _, waypoint := range fields {
		step := make([]float64, 0, len(waypoint))
		for _, pos := range waypoint {
			posF, err := strconv.ParseFloat(pos, 64)
			if err != nil {
				return nil, err
			}
			step = append(step, posF)
		}
		path = append(path, step)
	}

	return path, nil
}

func processPath(data [][]float64, scene string) (float64, float64, float64, error) {
	// First, calculate L2 joint score
	l2Score := 0.
	lineScore := 0.
	oScore := 0.

	Init(scene)
	thisFrame := sceneFS.Frame(testArmFrame)

	poseStart, err := thisFrame.Transform(referenceframe.FloatsToInputs(data[0]))
	if poseStart == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
		return -1, -1, -1, err
	}
	poseEnd, err := thisFrame.Transform(referenceframe.FloatsToInputs(data[len(data)-1]))
	if poseEnd == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
		return -1, -1, -1, err
	}

	// For each step
	for i := 0; i < len(data)-1; i++ {
		l2Score += L2Distance(data[i], data[i+1])

		// Check linear and orientation excursion every 2 degrees of joint movement
		nSteps := getSteps(data[i], data[i+1])
		for j := 1; j <= nSteps; j++ {
			step := referenceframe.InterpolateInputs(referenceframe.FloatsToInputs(data[i]), referenceframe.FloatsToInputs(data[i+1]), float64(j)/float64(nSteps))
			pose, err := thisFrame.Transform(step)
			if pose == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
				return -1, -1, -1, err
			}
			lineScore += distToLine(poseStart.Point(), poseEnd.Point(), pose.Point())
			oScore += orientScore(poseStart.Orientation(), poseEnd.Orientation(), pose.Orientation())
		}
	}

	//~ fmt.Println("joint_score", l2Score)

	totalLineDist := poseStart.Point().Sub(poseEnd.Point()).Norm()
	//~ fmt.Println("translation_score", lineScore/totalLineDist)

	//~ fmt.Println("orientation_score", oScore, "\n", "")

	return l2Score, lineScore / totalLineDist, oScore, nil
}

// L2Distance returns the L2 normalized difference between two equal length arrays.
func L2Distance(q1, q2 []float64) float64 {
	diff := make([]float64, len(q1))
	for i := 0; i < len(q1); i++ {
		diff[i] = q1[i] - q2[i]
	}
	// 2 is the L value returning a standard L2 Normalization
	return floats.Norm(diff, 2)
}

func distToLine(pt1, pt2, query r3.Vector) float64 {
	ab := pt1.Sub(pt2)
	av := query.Sub(pt2)

	if av.Dot(ab) <= 0.0 { // Point is lagging behind start of the segment, so perpendicular distance is not viable.
		return av.Norm() // Use distance to start of segment instead.
	}

	bv := query.Sub(pt1)

	if bv.Dot(ab) >= 0.0 { // Point is advanced past the end of the segment, so perpendicular distance is not viable.
		return bv.Norm()
	}
	dist := (ab.Cross(av)).Norm() / ab.Norm()
	if dist < defaultEpsilon {
		return 0.
	}

	return dist
}

// How many steps to step between configs 2 degrees at a time
func getSteps(q1, q2 []float64) int {
	nSteps := 1
	for i, j1 := range q1 {
		// convert to degrees
		//~ jDiff := (180 * math.Abs(j1 - q2[i]))/math.Pi
		jDiff := math.Abs(j1 - q2[i])
		if math.Ceil(jDiff/2.) > float64(nSteps) {
			nSteps = int(math.Ceil(jDiff / 2.))
		}
	}
	return nSteps
}

func orientDist(o1, o2 spatialmath.Orientation) float64 {
	return math.Sqrt(spatialmath.QuatToR3AA(spatialmath.OrientationBetween(o1, o2).Quaternion()).Norm2())
}

func orientScore(start, end, query spatialmath.Orientation) float64 {
	origDist := math.Max(orientDist(start, end), defaultEpsilon)

	sDist := math.Max(orientDist(start, query), defaultEpsilon)
	gDist := 0.

	// If origDist is less than or equal to defaultEpsilon, then the starting and ending orientations are the same and we do not need
	// to compute the distance to the ending orientation
	if origDist > defaultEpsilon {
		gDist = orientDist(end, query)
	}
	return (sDist + gDist) - origDist
}

package main

import (
	"context"
	"errors"
	"fmt"
	"time"
	"testing"
	"math"
	"math/rand"
	"os"
	"path"
	"encoding/csv"
	"strings"
	"strconv"

	"gonum.org/v1/gonum/floats"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
	"github.com/edaniels/golog"
	"github.com/golang/geo/r3"
	//~ "github.com/viamrobotics/visualization"
	//~ commonpb "go.viam.com/api/common/v1"
)

const defaultEpsilon = 0.01

type plannerConstructor func(frame referenceframe.Frame, nCPU int, logger golog.Logger) (motionplan.MotionPlanner, error)
type seededPlannerConstructor func(frame referenceframe.Frame, nCPU int, seed *rand.Rand, logger golog.Logger) (motionplan.MotionPlanner, error)

//~ func TestPlanners(t *testing.T) {
	//~ planners := []plannerConstructor{
		//motionplan.NewRRTStarConnectMotionPlanner,
		//~ motionplan.NewCBiRRTMotionPlanner,
	//~ }
	
	//~ sceneName := "scene9"
	//~ for _, planner := range planners {
		
		//~ Init(sceneName)
		//~ cfg := scene
		//~ mp, err := planner(cfg.RobotFrame, 4, logger)
		//~ test.That(t, err, test.ShouldBeNil)
		//~ path, err := mp.Plan(context.Background(), spatialmath.PoseToProtobuf(cfg.Goal), cfg.Start, scenePlanOpts)
		//~ test.That(t, err, test.ShouldBeNil)
		//~ visualization.VisualizePlan(scene.RobotFrame, path, scene.WorldState)
		//~ fmt.Println(path)
	//~ }
//~ }

func plannerRun(t *testing.T, plannerFunc seededPlannerConstructor, plannerName string) {
	outputFolder := "../results/" + plannerName + "/"
	if _, err := os.Stat(outputFolder); errors.Is(err, os.ErrNotExist) {
		err := os.MkdirAll(outputFolder, os.ModePerm)
		if err != nil {
			fmt.Println(err)
		}
	}
	
	for sceneName, _ := range allScenes {
		for i := 1; i <= 10; i++{
			fmt.Println(sceneName)
			Init(sceneName)
			cfg := scene
			mp, err := plannerFunc(cfg.RobotFrame, 4, rand.New(rand.NewSource(int64(i))), logger)
			test.That(t, err, test.ShouldBeNil)
			start := time.Now()
			path, err := mp.Plan(context.Background(), spatialmath.PoseToProtobuf(cfg.Goal), cfg.Start, scenePlanOpts)
			success := "true"
			if err != nil {
				success = "false"
			}
			took := time.Since(start)
			
			f, err := os.Create(outputFolder + sceneName + "_" + strconv.Itoa(i) + ".csv")
			test.That(t, err, test.ShouldBeNil)
			f2, err := os.Create(outputFolder + sceneName + "_" + strconv.Itoa(i) + "_stats.txt")
			test.That(t, err, test.ShouldBeNil)
			
			w := csv.NewWriter(f2)
			w.Write([]string{success, fmt.Sprintf("%f", float64(took) / float64(time.Second))})
			w.Flush()
			
			if success == "true" {
				w = csv.NewWriter(f)
				for _, step := range path {
					stepStr := make([]string, 0, len(path))
					for _, joint := range step {
						stepStr = append(stepStr, fmt.Sprintf("%f", joint.Value))
					}
					w.Write(stepStr)
				}
				w.Flush()
			}
			f.Close()
			f2.Close()
		}
	}
}

func TestCBiRRT(t *testing.T) {
	plannerRun(t, motionplan.NewCBiRRTMotionPlannerWithSeed, "cbirrt")
}

func TestRRTStar(t *testing.T) {
	plannerRun(t, motionplan.NewRRTStarConnectMotionPlannerWithSeed, "rdk_rrtstar")
}

func TestPlanScoring(t *testing.T) {
	outputFolder := "../results/"
	folders, err := os.ReadDir(outputFolder)
	test.That(t, err, test.ShouldBeNil)
	for _, alg := range folders {
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
				time := res[1]
				fmt.Println(sceneNum, seed, pass, time)
				if pass == "true" {
					// Only process paths that are valid.
					//~ fmt.Println(run.Name())
					data, err := readCSV(outputFolder + alg.Name() + "/" + sceneNum + "_" + seed + ".csv")
					test.That(t, err, test.ShouldBeNil)
					err = processPath(data, sceneNum)
					test.That(t, err, test.ShouldBeNil)
				}
			}
		}
	}
}

func TestVizPlan(t *testing.T) {
	Init("scene9")
	file := "/home/peter/Documents/echo/ompl-evaluation/results/cbirrt/scene9_7.csv"
	data, err := readCSV(file)
	test.That(t, err, test.ShouldBeNil)
	VisualizeOMPL(data)
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

func processPath(data [][]float64, scene string) error {
	// First, calculate L2 joint score
	l2Score := 0.
	lineScore := 0.
	oScore := 0.
	
	Init(scene)
	thisFrame := sceneFS.Frame(testArmFrame)
	
	poseStart, err := thisFrame.Transform(referenceframe.FloatsToInputs(data[0]))
	if err != nil {
		return err
	}
	poseEnd, err := thisFrame.Transform(referenceframe.FloatsToInputs(data[len(data)-1]))
	if err != nil {
		return err
	}
	
	// For each step
	for i := 0; i < len(data) - 1; i++ {
		l2Score += L2Distance(data[i], data[i+1])
		
		// Check linear and orientation excursion every 2 degrees of joint movement
		nSteps := getSteps(data[i], data[i+1])
		for j := 1; j <= nSteps; j++ {
			step := referenceframe.InterpolateInputs(referenceframe.FloatsToInputs(data[i]), referenceframe.FloatsToInputs(data[i+1]), float64(j)/float64(nSteps))
			pose, err := thisFrame.Transform(step)
			if err != nil {
				return err
			}
			lineScore += distToLine(poseStart.Point(), poseEnd.Point(), pose.Point())
			oScore += orientScore(poseStart.Orientation(), poseEnd.Orientation(), pose.Orientation())
		}
	}
	
	fmt.Println("l2Score", l2Score)
	
	totalLineDist := poseStart.Point().Sub(poseEnd.Point()).Norm()
	fmt.Println("translation score", lineScore/totalLineDist)
	
	fmt.Println("orientation score", oScore, "\n", "")
	
	return nil
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
			nSteps = int(math.Ceil(jDiff/2.))
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

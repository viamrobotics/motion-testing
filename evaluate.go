package main

import (
	"context"
	"fmt"
	"math"
	"strings"

	"github.com/golang/geo/r3"
	"github.com/pkg/errors"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/floats"
)

const defaultEpsilon = 1e-2

var logger logging.Logger = logging.NewLogger("motion-testing")

var scene *armplanning.PlanRequest

var allScenes = map[int]func(context.Context, logging.Logger) (*armplanning.PlanRequest, error){
	// arm scenes
	1:  scene1,
	2:  scene2,
	3:  scene3,
	4:  scene4,
	5:  scene5,
	6:  scene6,
	7:  scene7,
	8:  scene8,
	9:  scene9,
	10: scene10,
	11: scene11,
	12: scene12,
	// base scenes
	// 13: scene13,
	// 14: scene14,
	// 15: scene15,
	// 16: scene16,
	// 17: scene17,
	// 18: scene18,
}

var baseSceneStart = 13

const ptgDistStartIdx = 2
const ptgDistEndIdx = 3

// initScene takes a scene number and loads the relevant information into memory
func initScene(sceneNum int) (err error) {
	ctx := context.Background()
	logger := logging.NewLogger(fmt.Sprintf("scene-%d", sceneNum))
	if sceneFn, ok := allScenes[sceneNum]; ok {
		scene, err = sceneFn(ctx, logger)
		if err != nil {
			return
		}
		return
	}
	return errors.Errorf("scene %d does not exist", sceneNum)
}

func interpolateInputs(from, to []referenceframe.Input, by float64) []referenceframe.Input {
	var newVals []referenceframe.Input
	for i, j1 := range from {
		newVals = append(newVals, referenceframe.Input{j1.Value + ((to[i].Value - j1.Value) * by)})
	}
	return newVals
}

func evaluateSolution(solution [][]float64, sceneNum int) (float64, float64, float64, error) {
	var l2Score, lineScore, oScore, totalLineDist float64
	var err error

	if err := initScene(sceneNum); err != nil {
		return -1, -1, -1, err
	}
	var poseStart, poseEnd spatialmath.Pose
	sceneFrame := scene.FrameSystem.Frame("test_base")

	if sceneNum < baseSceneStart {
		sceneFrame = scene.FrameSystem.Frame("arm")
		poseStart, err = sceneFrame.Transform(referenceframe.FloatsToInputs(solution[0]))
		if poseStart == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
			return -1, -1, -1, err
		}
		poseEnd, err = sceneFrame.Transform(referenceframe.FloatsToInputs(solution[len(solution)-1]))
		if poseEnd == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
			return -1, -1, -1, err
		}
		totalLineDist = poseStart.Point().Sub(poseEnd.Point()).Norm()
	}

	// For each step
	for i := 0; i < len(solution)-1; i++ {
		l2Score += L2Distance(solution[i], solution[i+1])

		// Check linear and orientation excursion every 2 degrees of joint movement
		if sceneNum < baseSceneStart {
			nSteps := getSteps(solution[i], solution[i+1])
			for j := 1; j <= nSteps; j++ {
				step := interpolateInputs(
					referenceframe.FloatsToInputs(solution[i]),
					referenceframe.FloatsToInputs(solution[i+1]),
					float64(j)/float64(nSteps),
				)
				pose, err := sceneFrame.Transform(step)
				if pose == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
					return -1, -1, -1, err
				}
				lineScore += distToLine(poseStart.Point(), poseEnd.Point(), pose.Point())
				oScore += orientScore(poseStart.Orientation(), poseEnd.Orientation(), pose.Orientation())
			}
		} else {
			lineScore += math.Abs(solution[i][ptgDistEndIdx] - solution[i][ptgDistStartIdx])
		}
	}

	if totalLineDist != 0 {
		lineScore = lineScore / totalLineDist
	}

	return l2Score, lineScore, oScore, nil
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

// L2Distance returns the L2 normalized difference between two equal length arrays.
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
		// jDiff := (180 * math.Abs(j1 - q2[i]))/math.Pi
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

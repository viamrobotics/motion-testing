package main

/*
	struct pose {
		double X;
		double Y;
		double Z;
		double Pitch;
		double Roll;
		double Yaw;
	};
*/
import "C"
import (
	"fmt"
	"reflect"
	"unsafe"

	"github.com/golang/geo/r3"
	"github.com/viamrobotics/visualization"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

var scene *config
var sceneFS referenceframe.FrameSystem

var goalPoseC *C.struct_pose
var collision motionplan.Constraint

//export StartPos
func StartPos() uintptr {
	res := make([]float64, len(scene.Start))
	for i := 0; i < len(scene.Start); i++ {
		res[i] = scene.Start[i].Value
	}
	hdr := (*reflect.SliceHeader)(unsafe.Pointer(&res))
	return hdr.Data
}

//export GoalPose
func GoalPose() *C.struct_pose {
	return goalPoseC
}

//export ComputePositions
func ComputePositions(pos []float64) *C.struct_pose {
	pose := calcPose(pos)
	return poseToC(pose)
}

//export ValidState
func ValidState(pos []float64) bool {
	cInput := &motionplan.ConstraintInput{StartInput: referenceframe.FloatsToInputs(pos), Frame: sceneFS.Frame("arm")}
	valid, _ := collision(cInput)
	return valid
}

//export Visualize
func Visualize(inputs [][]float64) {
	plan := make([][]referenceframe.Input, 0)
	for _, input := range inputs {
		fmt.Println(input)
		plan = append(plan, referenceframe.FloatsToInputs(input))
	}
	visualization.VisualizePlan(scene.RobotFrame, plan, scene.WorldState)
}

//export Init
func Init(name string) {
	sceneFS = referenceframe.NewEmptySimpleFrameSystem("")

	// setup scenes in global vars
	var scene *config
	switch name {
	case "scene1":
		scene = scene1()
	case "scene2":
		scene = scene2()
	default:
	}
	sceneFS.AddFrame(scene.RobotFrame, sceneFS.World())

	var err error
	// generic post-scene setup
	collision, err = motionplan.NewCollisionConstraintFromWorldState(
		sceneFS.Frame("arm"),
		sceneFS,
		scene.WorldState,
		referenceframe.StartPositions(sceneFS),
	)
	if err != nil {
		fmt.Println(err)
		return
	}
	goalPoseC = poseToC(scene.Goal)
}

func calcPose(pos []float64) spatialmath.Pose {
	positions := map[string][]referenceframe.Input{}
	inputs := referenceframe.FloatsToInputs(pos)
	positions["arm"] = inputs
	posFrame := referenceframe.NewPoseInFrame("arm", spatialmath.NewZeroPose())
	tf, err := sceneFS.Transform(positions, posFrame, "world")
	if err != nil {
		fmt.Println(err)
		return nil
	}
	pose, _ := tf.(*referenceframe.PoseInFrame)
	return pose.Pose()
}

func poseToC(pose spatialmath.Pose) *C.struct_pose {
	eulerPose := (*C.struct_pose)(C.malloc(C.size_t(unsafe.Sizeof(C.struct_pose{}))))

	pt := pose.Point()
	eulerPose.X = C.double(pt.X)
	eulerPose.Y = C.double(pt.Y)
	eulerPose.Z = C.double(pt.Z)

	orient := pose.Orientation().EulerAngles()
	eulerPose.Pitch = C.double(orient.Pitch)
	eulerPose.Roll = C.double(orient.Roll)
	eulerPose.Yaw = C.double(orient.Yaw)

	return eulerPose
}

func cToPose(cPose *C.struct_pose) spatialmath.Pose {
	pt := r3.Vector{float64(cPose.X), float64(cPose.Y), float64(cPose.Z)}
	orient := &spatialmath.EulerAngles{Roll: float64(cPose.Roll), Pitch: float64(cPose.Pitch), Yaw: float64(cPose.Yaw)}

	return spatialmath.NewPoseFromOrientation(pt, orient)
}

func weightedSqNormDist(from, to spatialmath.Pose) float64 {
	delta := spatialmath.PoseDelta(from, to)
	// Increase weight for orientation since it's a small number
	return delta.Point().Norm2() + spatialmath.QuatToR3AA(delta.Orientation().Quaternion()).Mul(10.).Norm2()
}

// Needed for C export
func main() {}

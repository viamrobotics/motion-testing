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

	struct limits {
		double Min;
		double Max;
	};
*/
import "C"
import (
	"errors"
	"fmt"
	"unsafe"

	golog "github.com/edaniels/golog"
	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

var scene *config
var sceneFS referenceframe.FrameSystem

var logger golog.Logger = golog.NewLogger("omplbindings")

const testArmFrame = "arm"

//export NumJoints
func NumJoints() C.int {
	return C.int(len(sceneFS.Frame(testArmFrame).DoF()))
}

//export Limits
func Limits() *C.struct_limits {
	arm_limits := sceneFS.Frame(testArmFrame).DoF()

	limits_len := len(arm_limits)
	limits_ptr := C.malloc(C.size_t(limits_len) * C.size_t(unsafe.Sizeof(C.struct_limits{})))
	limits := (*[1 << 30]C.struct_limits)(limits_ptr)[:limits_len:limits_len]

	for i := 0; i < limits_len; i++ {
		limits[i].Min = C.double(arm_limits[i].Min)
		limits[i].Max = C.double(arm_limits[i].Max)
	}

	return (*C.struct_limits)(limits_ptr)
}

//export StartPos
func StartPos() *C.double {
	val_len := len(scene.Start)
	val_ptr := C.malloc(C.size_t(val_len) * C.size_t(unsafe.Sizeof(C.double(0))))
	values := (*[1 << 30]C.double)(val_ptr)[:val_len:val_len]
	for i := 0; i < val_len; i++ {
		values[i] = C.double(scene.Start[i].Value)
	}
	return (*C.double)(val_ptr)
}

//export GoalPose
func GoalPose() *C.struct_pose {
	return poseToC(scene.Goal)
}

//export NearGoal
func NearGoal(pos []float64) bool {
	actual := calcPose(pos)
	// TODO(rb): tie the epsilon to the resolution
	return spatialmath.PoseAlmostCoincidentEps(actual, scene.Goal, 1e-3)
}

//export ComputePositions
func ComputePositions(pos []float64) *C.struct_pose {
	return poseToC(calcPose(pos))
}

//export ComputePose
func ComputePose(targetPoseC *C.struct_pose) *C.double {
	// solutions, err := motionplan.BestIKSolutions(
	// 	context.Background(),
	// 	logger,
	// 	sceneFS,
	// 	scene.RobotFrame,
	// 	scene.Start,
	// 	cToPose(targetPoseC),
	// 	scene.WorldState,
	// 	1,
	// 	options,
	// )
	// if err != nil {
	// 	fmt.Println(err)
	// }
	solution := make([]referenceframe.Input, 6)
	ik_sol_len := len(solution)
	ik_sol_ptr := C.malloc(C.size_t(ik_sol_len) * C.size_t(unsafe.Sizeof(C.double(0))))
	ik_solution := (*[1 << 30]C.double)(ik_sol_ptr)[:ik_sol_len:ik_sol_len]
	for i := 0; i < ik_sol_len; i++ {
		ik_solution[i] = C.double(solution[i].Value)
	}
	return (*C.double)(ik_sol_ptr)
}

//export ValidState
func ValidState(pos []float64) bool {
	// cInput := &motionplan.ConstraintInput{StartInput: referenceframe.FloatsToInputs(pos), Frame: sceneFS.Frame(testArmFrame)}
	// for _, cons := range constraints {
	// 	pass, _ := cons(cInput)
	// 	if !pass {
	// 		return false
	// 	}
	// }
	return true
}

//export VisualizeOMPL
func VisualizeOMPL(inputs [][]float64) error {
	// plan := make([][]referenceframe.Input, 0)
	// nSteps := 0
	// for i, input := range inputs {
	// 	pStep := referenceframe.FloatsToInputs(input)
	// 	plan = append(plan, pStep)
	// 	if i < len(inputs)-1 {
	// 		nextStep := referenceframe.FloatsToInputs(inputs[i+1])
	// 		for j := 1; j <= nSteps; j++ {
	// 			step := referenceframe.InterpolateInputs(pStep, nextStep, float64(j)/float64(nSteps))
	// 			plan = append(plan, step)
	// 		}
	// 	}
	// }
	// worldState, err := referenceframe.WorldStateToProtobuf(scene.WorldState)
	// if err != nil {
	// 	return nil
	// }
	// return visualization.VisualizePlan(scene.RobotFrame, plan, worldState)
	return nil
}

//export Init
func Init(name string) error {
	sceneFS = referenceframe.NewEmptySimpleFrameSystem("")

	// setup scenes in global vars
	initFunc, ok := allScenes[name]
	if !ok {
		msg := "scene '" + name + "'does not exist"
		logger.Error(msg)
		return errors.New(msg)
	}
	var err error
	scene, err = initFunc()
	if err != nil {
		fmt.Println(err)
		return err
	}
	sceneFS.AddFrame(scene.RobotFrame, sceneFS.World())
	return nil
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

	return spatialmath.NewPose(pt, orient)
}

// Needed for C export
func main() {
}

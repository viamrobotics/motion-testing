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
	"context"
	"fmt"
	"math"
	"runtime"
	"unsafe"

	golog "github.com/edaniels/golog"
	"github.com/golang/geo/r3"

	"github.com/viamrobotics/visualization"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

var scene *config
var sceneFS referenceframe.FrameSystem
var scenePlanOpts *motionplan.PlannerOptions

var goalPoseC *C.struct_pose
var constraints []motionplan.Constraint
var ik *motionplan.CombinedIK
var logger golog.Logger

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
	return goalPoseC
}

//export NearGoal
func NearGoal(pos []float64) bool {
	actual := calcPose(pos)
	expected := cToPose(goalPoseC)
	// TODO(rb): tie the epsilon to the resolution
	return spatialmath.PoseAlmostCoincidentEps(actual, expected, 1e-3)
}

//export ComputePositions
func ComputePositions(pos []float64) *C.struct_pose {
	return poseToC(calcPose(pos))
}

//export ComputePose
func ComputePose(targetPoseC *C.struct_pose) *C.double {
	spatial_pose := cToPose(targetPoseC)
	pb_pose := spatialmath.PoseToProtobuf(spatial_pose)

	solutions, err := getIKSolutions(context.Background(), scenePlanOpts, ik, pb_pose, scene.Start, sceneFS.Frame(testArmFrame))
	if err != nil {
		fmt.Println(err)
	}

	ik_sol_len := len(solutions[0].Q())
	ik_sol_ptr := C.malloc(C.size_t(ik_sol_len) * C.size_t(unsafe.Sizeof(C.double(0))))
	ik_solution := (*[1 << 30]C.double)(ik_sol_ptr)[:ik_sol_len:ik_sol_len]
	for i := 0; i < ik_sol_len; i++ {
		ik_solution[i] = C.double(solutions[0].Q()[i].Value)
	}
	return (*C.double)(ik_sol_ptr)
}

//export ValidState
func ValidState(pos []float64) bool {
	cInput := &motionplan.ConstraintInput{StartInput: referenceframe.FloatsToInputs(pos), Frame: sceneFS.Frame(testArmFrame)}
	for _, cons := range constraints {
		pass, _ := cons(cInput)
		if !pass {
			return false
		}
	}
	return true
}

//export VisualizeOMPL
func VisualizeOMPL(inputs [][]float64) {
	plan := make([][]referenceframe.Input, 0)
	nSteps := 2
	for i, input := range inputs {
		pStep := referenceframe.FloatsToInputs(input)
		plan = append(plan, pStep)
		if i < len(inputs) - 1 {
			nextStep := referenceframe.FloatsToInputs(inputs[i+1])
			for j := 1; j <= nSteps; j++ {
				step := referenceframe.InterpolateInputs(pStep, nextStep, float64(j)/float64(nSteps))
				plan = append(plan, step)
			}
		}
	}
	visualization.VisualizePlan(scene.RobotFrame, plan, scene.WorldState)
}

//export Init
func Init(name string) {
	sceneFS = referenceframe.NewEmptySimpleFrameSystem("")

	// setup scenes in global vars
	initFunc, ok := allScenes[name]
	if !ok {
		fmt.Println("Scene '", name, "'does not exist!")
		return
	}
	scene = initFunc()
	sceneFS.AddFrame(scene.RobotFrame, sceneFS.World())

	// generic post-scene setup
	var err error
	collision, err := motionplan.NewCollisionConstraintFromWorldState(
		sceneFS.Frame(testArmFrame),
		sceneFS,
		scene.WorldState,
		referenceframe.StartPositions(sceneFS),
	)
	if err != nil {
		fmt.Println(err)
		return
	}

	// Setup PlanOptions for eventual IK solvers
	scenePlanOpts = motionplan.NewBasicPlannerOptions()
	scenePlanOpts.AddConstraint("collision", collision)
	
	constraints = append(constraints, collision)
	
	if name == "scene10" {
		from, _ := scene.RobotFrame.Transform(scene.Start)
		
		constraint, pathDist := motionplan.NewAbsoluteLinearInterpolatingConstraint(from, scene.Goal, 0.1, 0.05)
		scenePlanOpts.AddConstraint("linear", constraint)
		scenePlanOpts.SetPathDist(pathDist)
	}

	// Setup IK solver after scene buildup
	nCPU := int(math.Max(1.0, float64(runtime.NumCPU()/4)))
	logger = golog.NewLogger("omplbindings")
	ik, err = motionplan.CreateCombinedIKSolver(scene.RobotFrame, logger, nCPU)
	if err != nil {
		fmt.Println(err)
		return
	}

	goalPoseC = poseToC(scene.Goal)
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

// Needed for C export
func main() {
}

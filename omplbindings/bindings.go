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
	"context"
	"fmt"
	"math"
	"runtime"
	"sort"
	"unsafe"

	golog "github.com/edaniels/golog"
	"github.com/pkg/errors"
	"github.com/golang/geo/r3"

	"go.viam.com/utils"
	commonpb "go.viam.com/api/common/v1"
	"github.com/viamrobotics/visualization"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

var scene *config
var sceneFS referenceframe.FrameSystem
var scenePlanOpts *motionplan.PlannerOptions

var goalPoseC *C.struct_pose
var collision motionplan.Constraint
var ik *motionplan.CombinedIK

//export StartPos
func StartPos() *C.double {
	val_len := len(scene.Start)
	val_ptr := C.malloc(C.size_t(val_len) * C.size_t(unsafe.Sizeof(C.double(0))))
	values := (*[1<<30 - 1]C.double)(val_ptr)[:val_len:val_len]
	for i := 0; i < val_len; i++ {
		values[i] = C.double(scene.Start[i].Value)
	}
	return (*C.double)(val_ptr)
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

//export ComputePose
func ComputePose(targetPoseC *C.struct_pose) *C.double {
	spatial_pose := cToPose(targetPoseC)
	pb_pose := spatialmath.PoseToProtobuf(spatial_pose)

	solutions, err := getIKSolutions(context.Background(), scenePlanOpts, ik, pb_pose, referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0}), sceneFS.Frame("arm"))
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println(solutions)

	test_vals := []float64{ 1, 2, 3, 4, 5, 6 }
	val_len := len(test_vals)
	val_ptr := C.malloc(C.size_t(val_len) * C.size_t(unsafe.Sizeof(C.double(0))))
	values := (*[1<<30 - 1]C.double)(val_ptr)[:val_len:val_len]
	for i := 0; i < val_len; i++ {
		values[i] = C.double(test_vals[i])
	}
	return (*C.double)(val_ptr)
}

//export ValidState
func ValidState(pos []float64) bool {
	cInput := &motionplan.ConstraintInput{StartInput: referenceframe.FloatsToInputs(pos), Frame: sceneFS.Frame("arm")}
	valid, _ := collision(cInput)
	return valid
}

//export VisualizeOMPL
func VisualizeOMPL(inputs [][]float64) {
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
	switch name {
	case "scene1":
		scene = scene1()
	case "scene2":
		scene = scene2()
	default:
	}
	sceneFS.AddFrame(scene.RobotFrame, sceneFS.World())

	// generic post-scene setup
	var err error
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

	// Setup PlanOptions for eventual IK solvers
	scenePlanOpts := motionplan.NewBasicPlannerOptions()
	scenePlanOpts.AddConstraint("collision", collision)

	// Setup IK solver after scene buildup
	nCPU := int(math.Max(1.0, float64(runtime.NumCPU()/4)))
	logger := golog.NewLogger("omplbindings")
	ik, err = motionplan.CreateCombinedIKSolver(scene.RobotFrame, logger, nCPU)
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

// node interface is used to wrap a configuration for planning purposes.
type node interface {
	// return the configuration associated with the node
	Q() []referenceframe.Input
}

type basicNode struct {
	q []referenceframe.Input
}

func (n *basicNode) Q() []referenceframe.Input {
	return n.q
}

type costNode struct {
	node
	cost float64
}

func newCostNode(q []referenceframe.Input, cost float64) *costNode {
	return &costNode{&basicNode{q: q}, cost}
}

func fixOvIncrement(pos, seed *commonpb.Pose) *commonpb.Pose {
	epsilon := 0.0001
	if pos.X != seed.X || pos.Y != seed.Y || pos.Z != seed.Z || pos.Theta != seed.Theta {
		return pos
	}
	if 1-math.Abs(seed.OZ) > epsilon || pos.OZ != seed.OZ {
		return pos
	}
	xInc := pos.OX - seed.OX
	yInc := math.Abs(pos.OY - seed.OY)
	var adj float64
	if pos.OX == seed.OX {
		if yInc != 0.1 && yInc != 0.01 {
			return pos
		}
		if pos.OY-seed.OY > 0 {
			adj = 90
		} else {
			adj = -90
		}
	} else {
		if (xInc != -0.1 && xInc != -0.01) || pos.OY != seed.OY {
			return pos
		}
		adj = 180
	}
	if pos.OZ > 0 {
		adj *= -1
	}
	return &commonpb.Pose{
		X:     pos.X,
		Y:     pos.Y,
		Z:     pos.Z,
		Theta: pos.Theta + adj,
		OX:    pos.OX,
		OY:    pos.OY,
		OZ:    pos.OZ,
	}
}

func getIKSolutions(ctx context.Context,
	planOpts *motionplan.PlannerOptions,
	solver motionplan.InverseKinematics,
	goal *commonpb.Pose,
	seed []referenceframe.Input,
	frm referenceframe.Frame,
) ([]*costNode, error) {
	nSolutions := 0 // planOpts.MaxSolutions was causing a panic here
	if nSolutions == 0 {
		nSolutions = 50  // motionplan.defaultSolutionsToSeed uses this value
	}

	seedPos, err := frm.Transform(seed)
	if err != nil {
		return nil, err
	}
	goalPos := spatialmath.NewPoseFromProtobuf(fixOvIncrement(goal, spatialmath.PoseToProtobuf(seedPos)))  // Copied fixOvIncrement to local file

	solutionGen := make(chan []referenceframe.Input)
	ikErr := make(chan error, 1)
	defer func() { <-ikErr }()

	ctxWithCancel, cancel := context.WithCancel(ctx)
	defer cancel()

	// Spawn the IK solver to generate solutions until done
	utils.PanicCapturingGo(func() {
		defer close(ikErr)
		ikErr <- solver.Solve(ctxWithCancel, solutionGen, goalPos, seed, motionplan.NewSquaredNormMetric())  // Not sure if this is copasetic
	})

	solutions := map[float64][]referenceframe.Input{}

	// Solve the IK solver. Loop labels are required because `break` etc in a `select` will break only the `select`.
IK:
	for {
		select {
		case <-ctx.Done():
			return nil, ctx.Err()
		default:
		}

		select {
		case step := <-solutionGen:
			cPass, cScore := planOpts.CheckConstraints(&motionplan.ConstraintInput{
				seedPos,
				goalPos,
				seed,
				step,
				frm,
			})
			endPass, _ := planOpts.CheckConstraints(&motionplan.ConstraintInput{
				goalPos,
				goalPos,
				step,
				step,
				frm,
			})

			if cPass && endPass {
				if cScore < planOpts.MinScore && planOpts.MinScore > 0 {
					solutions = map[float64][]referenceframe.Input{}
					solutions[cScore] = step
					// good solution, stopping early
					break IK
				}

				solutions[cScore] = step
				if len(solutions) >= nSolutions {
					// sufficient solutions found, stopping early
					break IK
				}
			}
			// Skip the return check below until we have nothing left to read from solutionGen
			continue IK
		default:
		}

		select {
		case <-ikErr:
			// If we have a return from the IK solver, there are no more solutions, so we finish processing above
			// until we've drained the channel
			break IK
		default:
		}
	}
	if len(solutions) == 0 {
		return nil, errors.New("unable to solve for position")
	}

	keys := make([]float64, 0, len(solutions))
	for k := range solutions {
		keys = append(keys, k)
	}
	sort.Float64s(keys)

	orderedSolutions := make([]*costNode, 0)
	for _, key := range keys {
		orderedSolutions = append(orderedSolutions, newCostNode(solutions[key], key))
	}
	return orderedSolutions, nil
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

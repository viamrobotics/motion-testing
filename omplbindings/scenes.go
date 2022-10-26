package main

import (
	"fmt"
	"errors"
	"sort"
	"math"
	"math/rand"
	"context"
	
	"go.viam.com/utils"
	"go.viam.com/rdk/motionplan"
	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/components/arm/universalrobots"
	"go.viam.com/rdk/components/arm/xarm"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
)

type config struct {
	Start      []referenceframe.Input
	Goal       spatialmath.Pose
	RobotFrame referenceframe.Frame
	WorldState *commonpb.WorldState
}

var allScenes = map[string]func() *config {
	"scene1": scene1,
	"scene2": scene2,
	"scene3": scene3,
	"scene4": scene4,
	"scene5": scene5,
	"scene6": scene6,
	"scene7": scene7,
	"scene8": scene8,
	"scene9": scene9,
}

// setup a UR5 moving along a linear path in unrestricted space
func scene1() *config {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 100
	goalPt.Y += 100
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{},
	}
}

// setup a xArm7 to move in a straight line, adjacent to two large obstacles that should not impede the most efficient path
func scene2() *config {
	model, _ := xarm.Model("arm", 7)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 200
	goalPt.Z += 100
	testPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 1., Y: -200., Z: 3.},
		&spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.},
	)
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{
			Obstacles: []*commonpb.GeometriesInFrame{
				{
					ReferenceFrame: "world",
					Geometries: []*commonpb.Geometry{
						{
							Center: spatialmath.PoseToProtobuf(testPose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 2000,
									Y: 2000,
									Z: 20,
								}},
							},
						},
						{
							Center: spatialmath.PoseToProtobuf(testPose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 2000,
									Y: 20,
									Z: 2000,
								}},
							},
						},
					},
				},
			},
		},
	}
}

// setup a UR5 to move to the other side of an obstacle that obstructs the direct path
func scene3() *config {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := r3.Vector{X: -400, Y: 350, Z: 0}
	testPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 0., Y: 150., Z: 0.},
		&spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.},
	)
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{
			Obstacles: []*commonpb.GeometriesInFrame{
				{
					ReferenceFrame: "world",
					Geometries: []*commonpb.Geometry{
						{
							Center: spatialmath.PoseToProtobuf(testPose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 2000,
									Y: 20,
									Z: 120,
								}},
							},
						},
					},
				},
			},
		},
	}
}

// a xarm6 moving to the other side of an obstacle that obstructs its path
func scene4() *config {
	model, _ := xarm.Model("arm", 6)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 300
	testPt := startPose.Point()
	testPt.X += 150
	testPose := spatialmath.NewPoseFromOrientation(testPt, startPose.Orientation())
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{
			Obstacles: []*commonpb.GeometriesInFrame{
				{
					ReferenceFrame: "world",
					Geometries: []*commonpb.Geometry{
						{
							Center: spatialmath.PoseToProtobuf(testPose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 20,
									Y: 2000,
									Z: 60,
								}},
							},
						},
					},
				},
			},
		},
	}
}

// a xarm7 moving in a fairly constrained space
func scene5() *config {
	model, _ := xarm.Model("arm", 7)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 400
	wallPose := spatialmath.NewPoseFromPoint(r3.Vector{0, -200, 0})
	obs1Pose := spatialmath.NewPoseFromPoint(r3.Vector{300, 0, 0})
	obs2Pose := spatialmath.NewPoseFromPoint(r3.Vector{300, 0, 500})
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{
			Obstacles: []*commonpb.GeometriesInFrame{
				{
					ReferenceFrame: "world",
					Geometries: []*commonpb.Geometry{
						{
							Center: spatialmath.PoseToProtobuf(wallPose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 2000,
									Y: 50,
									Z: 2000,
								}},
							},
						},
						{
							Center: spatialmath.PoseToProtobuf(obs1Pose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 50,
									Y: 1250,
									Z: 200,
								}},
							},
						},
						{
							Center: spatialmath.PoseToProtobuf(obs2Pose),
							GeometryType: &commonpb.Geometry_Box{
								Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
									X: 50,
									Y: 1250,
									Z: 200,
								}},
							},
						},
					},
				},
			},
		},
	}
}

// scene 5 but with a bonus obstacle
func scene6() *config {
	cfg :=  scene5()
	wall2Pose := spatialmath.NewPoseFromPoint(r3.Vector{-150, 0, 0})
	geom := &commonpb.Geometry{
		Center: spatialmath.PoseToProtobuf(wall2Pose),
		GeometryType: &commonpb.Geometry_Box{
			Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
				X: 20,
				Y: 2000,
				Z: 2000,
			}},
		},
	}
	cfg.WorldState.Obstacles[0].Geometries = append(cfg.WorldState.Obstacles[0].Geometries, geom)
	return cfg
}

// Scene 4 but with a narrow interaction space
func scene7() *config {
	cfg :=  scene4()
	ispacePose := spatialmath.NewPoseFromPoint(r3.Vector{0, 0, 0})
	ispace := []*commonpb.GeometriesInFrame{
		{
			ReferenceFrame: "world",
			Geometries: []*commonpb.Geometry{
				{
					Center: spatialmath.PoseToProtobuf(ispacePose),
					GeometryType: &commonpb.Geometry_Box{
						Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
							X: 2000,
							Y: 260,
							Z: 2000,
						}},
					},
				},
			},
		},
	}
	cfg.WorldState.InteractionSpaces = ispace
	return cfg
}

// Scene 2 but with an orientation change for the goal
func scene8() *config {
	cfg :=  scene2()
	newGoal := spatialmath.NewPoseFromOrientation(cfg.Goal.Point(), &spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.})
	cfg.Goal = newGoal
	return cfg
}

// setup a UR5 moving a big movement with random obstacles
func scene9() *config {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 1100
	goalPt.Y += 600
	
	cube :=  &commonpb.Geometry_Box{
		Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
			X: 1,
			Y: 1,
			Z: 1,
		}},
	}
	
	cfg := &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &commonpb.WorldState{
			Obstacles: []*commonpb.GeometriesInFrame{
				{
					ReferenceFrame: "world",
					Geometries: []*commonpb.Geometry{},
				},
			},
		},
	}
	
	rGen := rand.New(rand.NewSource(int64(1)))
	for i := 0; i < 100; i++ {
		cubePose := spatialmath.NewPoseFromPoint(r3.Vector{
			X: 2000 * (rGen.Float64()- 0.5),
			Y: 2000 * (rGen.Float64()- 0.5),
			Z: 2000 * (rGen.Float64()- 0.5),
		})
		
		geom := &commonpb.Geometry{
			Center: spatialmath.PoseToProtobuf(cubePose),
			GeometryType: cube,
		}
		cfg.WorldState.Obstacles[0].Geometries = append(cfg.WorldState.Obstacles[0].Geometries, geom)
	}
	return cfg
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
	nSolutions := planOpts.MaxSolutions
	if nSolutions == 0 {
		nSolutions = 50  // motionplan.defaultSolutionsToSeed has this value
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

func weightedSqNormDist(from, to spatialmath.Pose) float64 {
	delta := spatialmath.PoseDelta(from, to)
	// Increase weight for orientation since it's a small number
	return delta.Point().Norm2() + spatialmath.QuatToR3AA(delta.Orientation().Quaternion()).Mul(10.).Norm2()
}

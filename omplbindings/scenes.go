package main

import (
	"fmt"
	"math"
	"math/rand"
	"strconv"

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
	WorldState *referenceframe.WorldState
}

var allScenes = map[string]func() (*config, error){
	"scene1": scene1,
	"scene2": scene2,
	"scene3": scene3,
	"scene4": scene4,
	"scene5": scene5,
	"scene6": scene6,
	// TODO(rb): scene 7 is broken for some reason
	// "scene7": scene7,
	"scene8":  scene8,
	"scene9":  scene9,
	"scene10": scene10,
	"scene11": scene11,
	"scene12": scene12,
}

// scene1: setup a UR5 moving along a linear path in unrestricted space
func scene1() (*config, error) {
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
		WorldState: &referenceframe.WorldState{},
	}, nil
}

// scene2: setup a xArm7 to move in a straight line, adjacent to two large obstacles that should not impede the most efficient path
func scene2() (*config, error) {
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
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
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
	})
	if err != nil {
		return nil, err
	}

	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: worldState,
	}, nil
}

// scene3: setup a UR5 to move to the other side of an obstacle that obstructs the direct path
func scene3() (*config, error) {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := r3.Vector{X: -400, Y: 350, Z: 0}
	testPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 0., Y: 150., Z: 0.},
		&spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.},
	)
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
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
	})
	if err != nil {
		return nil, err
	}

	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: worldState,
	}, nil
}

// scene4: a xarm6 moving to the other side of an obstacle that obstructs its path
func scene4() (*config, error) {
	model, _ := xarm.Model("arm", 6)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 300
	testPt := startPose.Point()
	testPt.X += 150
	testPose := spatialmath.NewPoseFromOrientation(testPt, startPose.Orientation())
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
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
	})
	if err != nil {
		return nil, err
	}

	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: worldState,
	}, nil
}

// scene5: a xarm7 moving in a fairly constrained space
func scene5() (*config, error) {
	model, _ := xarm.Model("arm", 7)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 400
	wallPose := spatialmath.NewPoseFromPoint(r3.Vector{0, -200, 0})
	obs1Pose := spatialmath.NewPoseFromPoint(r3.Vector{300, 0, 0})
	obs2Pose := spatialmath.NewPoseFromPoint(r3.Vector{300, 0, 500})
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
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
	})

	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: worldState,
	}, err
}

// scene6: scene 5 but with a bonus obstacle
func scene6() (*config, error) {
	cfg, err := scene5()
	if err != nil {
		return nil, err
	}
	obstacle, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{-150, 0, 0}), r3.Vector{20, 2000, 2000}, "")
	if err != nil {
		return nil, err
	}
	cfg.WorldState.Obstacles = append(
		cfg.WorldState.Obstacles,
		referenceframe.NewGeometriesInFrame(referenceframe.World, map[string]spatialmath.Geometry{"": obstacle}),
	)
	return cfg, err
}

// scene7: scene 4 but with a narrow interaction space
func scene7() (*config, error) {
	cfg, err := scene4()
	if err != nil {
		return nil, err
	}
	ispace, err := spatialmath.NewBox(spatialmath.NewZeroPose(), r3.Vector{2000, 260, 2000}, "")
	if err != nil {
		return nil, err
	}
	cfg.WorldState.InteractionSpaces = append(
		cfg.WorldState.InteractionSpaces,
		referenceframe.NewGeometriesInFrame(referenceframe.World, map[string]spatialmath.Geometry{"": ispace}),
	)
	return cfg, nil
}

// scene8: scene 2 but with an orientation change for the goal
func scene8() (*config, error) {
	cfg, err := scene2()
	if err != nil {
		return nil, err
	}
	cfg.Goal = spatialmath.NewPoseFromOrientation(cfg.Goal.Point(), &spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.})
	return cfg, err
}

// scene9: setup a UR5 moving a big movement with 100 random obstacles
func scene9() (*config, error) {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 1100
	goalPt.Y += 600

	rGen := rand.New(rand.NewSource(int64(1)))
	obstacles := make(map[string]spatialmath.Geometry, 0)
	for i := 0; i < 100; i++ {
		cubePose := spatialmath.NewPoseFromPoint(r3.Vector{
			X: 2000 * (rGen.Float64() - 0.5),
			Y: 2000 * (rGen.Float64() - 0.5),
			Z: 2000 * (rGen.Float64() - 0.5),
		})
		label := strconv.Itoa(i)
		cube, err := spatialmath.NewBox(cubePose, r3.Vector{1, 1, 1}, label)
		if err != nil {
			return nil, err
		}
		obstacles[label] = cube
	}

	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: &referenceframe.WorldState{Obstacles: []*referenceframe.GeometriesInFrame{
			referenceframe.NewGeometriesInFrame(referenceframe.World, obstacles),
		}},
	}, nil

}

// scene10: Move a UR5 a large distance around the base
func scene10() (*config, error) {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{0, -math.Pi / 4, math.Pi / 2, 3 * math.Pi / 4, -math.Pi / 2, 0})
	startPose, _ := model.Transform(startInput)

	// Goal pose
	goalPt := startPose.Point()
	goalPt.X += 1200
	goalPt.Y += 600

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 0., Y: 0., Z: -1000.},
		&spatialmath.R4AA{Theta: 0, RX: 1., RY: 0., RZ: 0.},
	)
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
		Obstacles: []*commonpb.GeometriesInFrame{
			{
				ReferenceFrame: "world",
				Geometries: []*commonpb.Geometry{
					{
						Center: spatialmath.PoseToProtobuf(pillarPose),
						GeometryType: &commonpb.Geometry_Box{
							Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
								X: 130,
								Y: 130,
								Z: 2000,
							}},
						},
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}
	return &config{
		Start:      startInput,
		Goal:       spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()),
		RobotFrame: model,
		WorldState: worldState,
	}, nil
}

// scene 11: Get the UR5 to hit itself when this set of start and goal data is used for motion planning
// Corresponds to move that can cause a self-collision on the UR5 from Rand's move set
func scene11() (*config, error) {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{3.8141, -1.3106, 2.4543, 4.9485, -3.4041, -2.6749})

	// Goal pose
	goalPos := r3.Vector{X: -244.43, Y: -255.12, Z: 676.97}
	goalRot := spatialmath.R3ToR4(r3.Vector{X: 0.233, Y: -1.637, Z: 1.224})
	goalPose := spatialmath.NewPoseFromOrientation(goalPos, goalRot)

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 0., Y: 0., Z: -1000.},
		&spatialmath.R4AA{Theta: 0, RX: 1., RY: 0., RZ: 0.},
	)
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
		Obstacles: []*commonpb.GeometriesInFrame{
			{
				ReferenceFrame: "world",
				Geometries: []*commonpb.Geometry{
					{
						Center: spatialmath.PoseToProtobuf(pillarPose),
						GeometryType: &commonpb.Geometry_Box{
							Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
								X: 130,
								Y: 130,
								Z: 2000,
							}},
						},
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &config{
		Start:      startInput,
		Goal:       goalPose,
		RobotFrame: model,
		WorldState: worldState,
	}, nil
}

// scene12: Move a UR5 that has been tangled up with itself through the workspace across octents
// Corresponds to move that only works with MoveJ from Rand's move set
func scene12() (*config, error) {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{1.2807, -1.4437, -1.3287, 3.7446, 1.4315, -0.2135})

	// Goal pose
	goalPos := r3.Vector{X: -50.47, Y: -366.47, Z: 189.04}
	goalRot := spatialmath.R3ToR4(r3.Vector{X: 0.808, Y: 2.168, Z: 2.916})
	goalPose := spatialmath.NewPoseFromOrientation(goalPos, goalRot)

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 0., Y: 0., Z: -1000.},
		&spatialmath.R4AA{Theta: 0, RX: 1., RY: 0., RZ: 0.},
	)
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
		Obstacles: []*commonpb.GeometriesInFrame{
			{
				ReferenceFrame: "world",
				Geometries: []*commonpb.Geometry{
					{
						Center: spatialmath.PoseToProtobuf(pillarPose),
						GeometryType: &commonpb.Geometry_Box{
							Box: &commonpb.RectangularPrism{DimsMm: &commonpb.Vector3{
								X: 130,
								Y: 130,
								Z: 2000,
							}},
						},
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &config{
		Start:      startInput,
		Goal:       goalPose,
		RobotFrame: model,
		WorldState: worldState,
	}, nil
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

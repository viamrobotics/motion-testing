package scenes

import (
	"context"
	"math"
	"math/rand"
	"strconv"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/arm/fake"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/spatialmath"

	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
)

func newArmModel(ctx context.Context, armModelName string, logger logging.Logger) (referenceframe.Model, error) {
	cfg := resource.Config{
		Name:  arm.Named("arm").Name,
		Model: resource.DefaultModelFamily.WithModel(armModelName),
		ConvertedAttributes: &fake.Config{
			ArmModel: armModelName,
		},
	}
	a, err := fake.NewArm(ctx, nil, cfg, logger)
	if err != nil {
		return nil, err
	}
	return a.Kinematics(ctx)
}

func scene1(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}

	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 100
	goalPt.Y += 100

	// Create PathState for goal
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
	}, nil
}

func scene2(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "xarm7", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 200
	goalPt.Z += 100

	// Create PathState for goal
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Obstacles
	testPose := spatialmath.NewPose(
		r3.Vector{X: 1., Y: -200., Z: 3.},
		&spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.},
	)
	worldState, err := referenceframe.WorldStateFromProtobuf(&commonpb.WorldState{
		Obstacles: []*commonpb.GeometriesInFrame{
			{
				ReferenceFrame: referenceframe.World,
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
						Label: "floor",
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
						Label: "wall",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

func scene3(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := r3.Vector{X: -400, Y: 350, Z: 0}
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Obstacles
	testPose := spatialmath.NewPose(
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
						Label: "blocker",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

func scene4(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "xarm6", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 300
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Obstacles
	testPt := startPose.Point()
	testPt.X += 150
	testPose := spatialmath.NewPose(testPt, startPose.Orientation())
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
						Label: "blocker",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

func scene5(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "xarm7", logger)
	if err != nil {
		return nil, err
	}
	// model, _ := xarm.MakeModelFrame(xarm.ModelName7DOF, nil, nil, nil)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 400
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Obstacles
	wallPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: -200, Z: 0})
	obs1Pose := spatialmath.NewPoseFromPoint(r3.Vector{X: 300, Y: 0, Z: 0})
	obs2Pose := spatialmath.NewPoseFromPoint(r3.Vector{X: 300, Y: 0, Z: 500})
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
						Label: "wall",
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
						Label: "lower_window",
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
						Label: "upper_window",
					},
				},
			},
		},
	})

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, err
}

func Scene6(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	cfg, err := scene5(ctx, logger)
	if err != nil {
		return nil, err
	}

	newGeometries := make([]*referenceframe.GeometriesInFrame, 0)
	oldWorld, err := cfg.WorldState.ToProtobuf()
	for _, protoGeometries := range oldWorld.GetObstacles() {
		oldGeometries, err := referenceframe.ProtobufToGeometriesInFrame(protoGeometries)
		if err != nil {
			return nil, err
		}
		newGeometries = append(newGeometries, oldGeometries)
	}

	obstacle, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: -150, Y: 0, Z: 0}), r3.Vector{X: 20, Y: 2000, Z: 2000}, "extra_obs")
	if err != nil {
		return nil, err
	}
	obstaclesInFrame := referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{obstacle})
	newGeometries = append(newGeometries, obstaclesInFrame)

	newWorldState, err := referenceframe.NewWorldState(newGeometries, nil)
	if err != nil {
		return nil, err
	}
	cfg.WorldState = newWorldState

	return cfg, err
}

func scene7(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	cfg, err := scene4(ctx, logger)
	if err != nil {
		return nil, err
	}

	newGeometries := make([]*referenceframe.GeometriesInFrame, 0)
	oldWorld, err := cfg.WorldState.ToProtobuf()
	for _, protoGeometries := range oldWorld.GetObstacles() {
		oldGeometries, err := referenceframe.ProtobufToGeometriesInFrame(protoGeometries)
		if err != nil {
			return nil, err
		}
		newGeometries = append(newGeometries, oldGeometries)
	}

	left_wall, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: 140, Z: 0}), r3.Vector{X: 2000, Y: 20, Z: 2000}, "left_wall")
	if err != nil {
		return nil, err
	}
	right_wall, err := spatialmath.NewBox(spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: -140, Z: 0}), r3.Vector{X: 2000, Y: 20, Z: 2000}, "right_wall")
	if err != nil {
		return nil, err
	}
	obstaclesInFrame := referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{left_wall, right_wall})
	newGeometries = append(newGeometries, obstaclesInFrame)

	newWorldState, err := referenceframe.NewWorldState(newGeometries, nil)
	if err != nil {
		return nil, err
	}
	cfg.WorldState = newWorldState

	return cfg, err
}

func scene8(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	cfg, err := scene2(ctx, logger)
	if err != nil {
		return nil, err
	}

	// Update the goal to only include pose information
	goalPose := cfg.Goals[0].Poses()["arm"].Pose()
	goalPathState := referenceframe.FrameSystemPoses{"arm": referenceframe.NewPoseInFrame(referenceframe.World, goalPose)}
	cfg.Goals = []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)}

	return cfg, nil
}

func scene9(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 1100
	goalPt.Y += 600
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	rGen := rand.New(rand.NewSource(int64(1)))
	obstacles := make([]spatialmath.Geometry, 0)
	for i := 0; i < 100; i++ {
		cubePose := spatialmath.NewPoseFromPoint(r3.Vector{
			X: 2000 * (rGen.Float64() - 0.5),
			Y: 2000 * (rGen.Float64() - 0.5),
			Z: 2000 * (rGen.Float64() - 0.5),
		})
		label := strconv.Itoa(i)
		cube, err := spatialmath.NewBox(cubePose, r3.Vector{X: 1, Y: 1, Z: 1}, label)
		if err != nil {
			return nil, err
		}
		obstacles = append(obstacles, cube)
	}

	obstaclesInFrame := referenceframe.NewGeometriesInFrame(referenceframe.World, obstacles)
	worldState, err := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{obstaclesInFrame}, nil)
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

func scene10(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{0, -math.Pi / 4, math.Pi / 2, 3 * math.Pi / 4, -math.Pi / 2, 0})
	startPose, _ := model.Transform(startInput)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPt := startPose.Point()
	goalPt.X += 1200
	goalPt.Y += 600
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPt, startPose.Orientation()))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPose(
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
						Label: "pillar",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

// Corresponds to move that has been demonstrated to cause a self-collision on the UR5's basic planning
func scene11(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{3.8141, -1.3106, 2.4543, 4.9485, -3.4041, -2.6749})

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPos := r3.Vector{X: -244.43, Y: -255.12, Z: 676.97}
	goalRot := spatialmath.R3ToR4(r3.Vector{X: 0.233, Y: -1.637, Z: 1.224})
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPos, goalRot))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPose(
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
						Label: "pillar",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

// Corresponds to move that only works with MoveJ from an engineering move set
func scene12(ctx context.Context, logger logging.Logger) (*armplanning.PlanRequest, error) {
	model, err := newArmModel(ctx, "ur5e", logger)
	if err != nil {
		return nil, err
	}
	startInput := referenceframe.FloatsToInputs([]float64{1.2807, -1.4437, -1.3287, 3.7446, 1.4315, -0.2135})

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(model, fs.World())

	startMap := map[string][]referenceframe.Input{"arm": startInput}

	// Goal specification
	goalPos := r3.Vector{X: -50.47, Y: -366.47, Z: 189.04}
	goalRot := spatialmath.R3ToR4(r3.Vector{X: 0.808, Y: 2.168, Z: 2.916})
	goalPose := referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewPose(goalPos, goalRot))
	goalPathState := referenceframe.FrameSystemPoses{"arm": goalPose}

	// Pose of UR5 mount pillar
	pillarPose := spatialmath.NewPose(
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
						Label: "pillar",
					},
				},
			},
		},
	})
	if err != nil {
		return nil, err
	}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(nil, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		FrameSystem: fs,
		WorldState:  worldState,
	}, nil
}

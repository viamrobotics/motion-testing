package main

import (
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

// setup a UR5 moving along a linear path in unrestricted space
func scene1() *config {
	model, _ := universalrobots.Model("arm")
	startInput := referenceframe.FloatsToInputs([]float64{1, 0, 0, 0, 0, 0})
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

// setup a xArm6 to move in a straight line, adjacent to a large obstacle that should not imede the most efficient path
func scene2() *config {
	model, _ := xarm.Model("arm", 6)
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0, 0, 0, 0})
	startPose, _ := model.Transform(startInput)
	goalPt := startPose.Point()
	goalPt.X += 200
	goalPt.Z += 100
	testPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 1., Y: -100., Z: 3.},
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
					},
				},
			},
		},
	}
}

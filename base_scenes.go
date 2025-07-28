package main

import (
	"bytes"
	"context"
	"os"
	"path/filepath"

	geo "github.com/kellydunn/golang-geo"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/base/fake"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/testutils/inject"
	"go.viam.com/utils"
	"go.viam.com/utils/artifact"
)

func getPointCloudMap(path string) (func() ([]byte, error), error) {
	const chunkSizeBytes = 1 * 1024 * 1024
	file, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	chunk := make([]byte, chunkSizeBytes)
	f := func() ([]byte, error) {
		bytesRead, err := file.Read(chunk)
		if err != nil {
			defer utils.UncheckedErrorFunc(file.Close)
			return nil, err
		}
		return chunk[:bytesRead], err
	}
	return f, nil
}

func createBaseSceneConfig(
	startInput []referenceframe.Input,
	goalPose spatialmath.Pose,
	artifactPath string,
) (*armplanning.PlanRequest, error) {
	injectSlam := inject.NewSLAMService("test_slam")
	injectSlam.PointCloudMapFunc = func(ctx context.Context, returnEditedMap bool) (func() ([]byte, error), error) {

		return getPointCloudMap(filepath.Clean(artifact.MustPath(artifactPath)))
	}
	injectSlam.PositionFunc = func(ctx context.Context) (spatialmath.Pose, error) {
		return spatialmath.NewZeroPose(), nil
	}

	// create fake base
	baseCfg := resource.Config{
		Name:  "test_base",
		API:   base.API,
		Frame: &referenceframe.LinkConfig{Geometry: &spatialmath.GeometryConfig{R: 20}},
	}

	ms := inject.NewMovementSensor("movement_sensor")
	gpOrigin := geo.NewPoint(0, 0)
	ms.PositionFunc = func(ctx context.Context, extra map[string]interface{}) (*geo.Point, float64, error) {
		return gpOrigin, 0, nil
	}
	ms.CompassHeadingFunc = func(ctx context.Context, extra map[string]interface{}) (float64, error) {
		return 0, nil
	}
	ms.PropertiesFunc = func(ctx context.Context, extra map[string]interface{}) (*movementsensor.Properties, error) {
		return &movementsensor.Properties{CompassHeadingSupported: true}, nil
	}
	localizer := motion.NewMovementSensorLocalizer(ms, gpOrigin, spatialmath.NewZeroPose())
	fakeBase, _ := fake.NewBase(context.Background(), nil, baseCfg, logger)
	kb, _ := kinematicbase.WrapWithKinematics(
		context.Background(),
		fakeBase.(*fake.Base),
		logger,
		localizer,
		nil,
		kinematicbase.NewKinematicBaseOptions(),
	)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	m, err := kb.Kinematics(context.Background())
	if err != nil {
		return nil, err
	}
	fs.AddFrame(m, fs.World())

	// get point cloud data in the form of bytes from pcd
	pointCloudData, err := slam.PointCloudMapFull(context.Background(), injectSlam, false)
	if err != nil {
		return nil, err
	}

	// store slam point cloud data  in the form of a recursive octree for collision checking
	octree, err := pointcloud.ReadPCD(bytes.NewReader(pointCloudData), "octree")
	if err != nil {
		return nil, err
	}

	worldState, _ := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{octree.(spatialmath.Geometry)}),
	}, nil)
	goalPathState := referenceframe.FrameSystemPoses{m.Name(): referenceframe.NewPoseInFrame(referenceframe.World, goalPose)}
	startMap := referenceframe.NewZeroInputs(fs)
	startPathState := referenceframe.FrameSystemPoses{m.Name(): referenceframe.NewZeroPoseInFrame(referenceframe.World)}

	return &armplanning.PlanRequest{
		StartState:  armplanning.NewPlanState(startPathState, startMap),
		Goals:       []*armplanning.PlanState{armplanning.NewPlanState(goalPathState, nil)},
		WorldState:  worldState,
		FrameSystem: fs,
	}, nil
}

func scene13() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 0.277 * 1000, Y: 0.593 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "pointcloud/octagonspace.pcd")
}

func scene14() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 1.32 * 1000, Y: 0})
	return createBaseSceneConfig(startInput, goalPose, "pointcloud/octagonspace.pcd")
}

func scene15() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{-6.905 * 1000, 0.623 * 1000, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -29.164 * 1000, Y: 3.433 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene16() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{-19.376 * 1000, 2.305 * 1000, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -27.946 * 1000, Y: -4.406 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene17() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -5.959 * 1000, Y: -5.542 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene18() (*armplanning.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -52.555 * 1000, Y: -27.215 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

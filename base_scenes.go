package main

import (
	"bytes"
	"context"
	"os"
	"path/filepath"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/base/fake"
	"go.viam.com/rdk/components/base/kinematicbase"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
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
) (*motionplan.PlanRequest, error) {
	injectSlam := inject.NewSLAMService("test_slam")
	injectSlam.PointCloudMapFunc = func(ctx context.Context) (func() ([]byte, error), error) {

		return getPointCloudMap(filepath.Clean(artifact.MustPath(artifactPath)))
	}
	injectSlam.PositionFunc = func(ctx context.Context) (spatialmath.Pose, string, error) {
		return spatialmath.NewZeroPose(), "", nil
	}

	// create fake base
	baseCfg := resource.Config{
		Name:  "test_base",
		API:   base.API,
		Frame: &referenceframe.LinkConfig{Geometry: &spatialmath.GeometryConfig{R: 20}},
	}
	fakeBase, _ := fake.NewBase(context.Background(), nil, baseCfg, logger)
	kb, _ := kinematicbase.WrapWithFakePTGKinematics(
		context.Background(),
		fakeBase.(*fake.Base),
		logger,
		referenceframe.NewPoseInFrame(referenceframe.World, spatialmath.NewZeroPose()),
		kinematicbase.NewKinematicBaseOptions(),
		nil,
		5,
	)

	// Add frame system and needed frames
	fs := referenceframe.NewEmptyFrameSystem("test")
	fs.AddFrame(kb.Kinematics(), fs.World())

	// get point cloud data in the form of bytes from pcd
	pointCloudData, _ := slam.PointCloudMapFull(context.Background(), injectSlam, false)
	// store slam point cloud data  in the form of a recursive octree for collision checking
	octree, _ := pointcloud.ReadPCDToBasicOctree(bytes.NewReader(pointCloudData))
	worldState, _ := referenceframe.NewWorldState([]*referenceframe.GeometriesInFrame{
		referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{octree}),
	}, nil)

	startMap := referenceframe.StartPositions(fs)

	return &motionplan.PlanRequest{
		StartConfiguration: startMap,
		Goal:               referenceframe.NewPoseInFrame(referenceframe.World, goalPose),
		Frame:              kb.Kinematics(),
		WorldState:         worldState,
		FrameSystem:        fs,
	}, nil
}

func scene13() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 0.277 * 1000, Y: 0.593 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "pointcloud/octagonspace.pcd")
}

func scene14() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 1.32 * 1000, Y: 0})
	return createBaseSceneConfig(startInput, goalPose, "pointcloud/octagonspace.pcd")
}

func scene15() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{-6.905 * 1000, 0.623 * 1000, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -29.164 * 1000, Y: 3.433 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene16() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{-19.376 * 1000, 2.305 * 1000, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -27.946 * 1000, Y: -4.406 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene17() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -5.959 * 1000, Y: -5.542 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

func scene18() (*motionplan.PlanRequest, error) {
	startInput := referenceframe.FloatsToInputs([]float64{0, 0, 0})
	goalPose := spatialmath.NewPoseFromPoint(r3.Vector{X: -52.555 * 1000, Y: -27.215 * 1000})
	return createBaseSceneConfig(startInput, goalPose, "slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd")
}

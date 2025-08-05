package scenes

import (
	"context"
	"math"
	"runtime"
	"strings"
	"testing"
	"time"

	"github.com/golang/geo/r3"
	"github.com/google/uuid"
	geo "github.com/kellydunn/golang-geo"
	"go.viam.com/rdk/components/base"
	"go.viam.com/rdk/components/movementsensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/services/motion/builtin"
	"go.viam.com/rdk/services/slam"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/test"
)

var (
	leftMotorName  = "leftmotor"
	rightMotorName = "rightmotor"
	baseName       = "test-base"
	moveSensorName = "test-movement-sensor"

	moveSensorResource        = resource.NewName(movementsensor.API, moveSensorName)
	baseResource              = resource.NewName(base.API, baseName)
	movementSensorInBasePoint = r3.Vector{X: -10, Y: 0, Z: 0}
	updateRate                = 33
	PlanDeviationMM           = 150.
	timeoutSec                = time.Duration(45)
)

func TestMotionExtendedGlobe(t *testing.T) {
	ctx := context.Background()
	ctx, cFunc := context.WithCancel(ctx)
	defer cFunc()
	// Near antarctica 🐧
	gpsPoint := geo.NewPoint(-70, 40)
	dst := geo.NewPoint(gpsPoint.Lat(), gpsPoint.Lng()+7e-5)
	expectedDst := r3.Vector{X: 2662.16, Y: 0, Z: 0} // Relative pose to the starting point of the base; facing north, Y = forwards
	// create motion config
	extra := map[string]interface{}{
		"motion_profile": "position_only",
		"timeout":        timeoutSec,
	}

	t.Run("is able to reach a nearby geo point with empty values", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			MovementSensorName: moveSensorResource,
			Destination:        dst,
			Extra:              extra,
		}
		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)
	})

	t.Run("is able to reach a nearby geo point with a requested NaN heading", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			MovementSensorName: moveSensorResource,
			Heading:            math.NaN(),
			Destination:        dst,
			Extra:              extra,
		}
		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)
	})

	t.Run("is able to reach a nearby geo point with a requested positive heading", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			MovementSensorName: moveSensorResource,
			Heading:            10000000,
			Destination:        dst,
			Extra:              extra,
		}
		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)
	})

	t.Run("is able to reach a nearby geo point with a requested negative heading", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			MovementSensorName: moveSensorResource,
			Heading:            -10000000,
			Destination:        dst,
			Extra:              extra,
		}
		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)
	})

	t.Run("is able to reach a nearby geo point when the motion configuration is empty", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			MovementSensorName: moveSensorResource,
			Heading:            90,
			Destination:        dst,
			MotionCfg:          &motion.MotionConfiguration{},
			Extra:              extra,
		}
		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)
	})

	t.Run("go around an obstacle", func(t *testing.T) {
		localizer, ms, closeFunc := builtin.CreateMoveOnGlobeTestEnvironment(ctx, t, gpsPoint, 80, nil)
		defer closeFunc(ctx)
		planDeviationMM := 100.
		var positionPollingFreqHz float64
		motionCfg := &motion.MotionConfiguration{PositionPollingFreqHz: &positionPollingFreqHz, LinearMPerSec: 0.2, AngularDegsPerSec: 60}

		boxPose := spatialmath.NewPoseFromPoint(r3.Vector{X: 50, Y: 0, Z: 0})
		boxDims := r3.Vector{X: 5, Y: 50, Z: 10}
		geometries, err := spatialmath.NewBox(boxPose, boxDims, "wall")
		test.That(t, err, test.ShouldBeNil)
		geoGeometry := spatialmath.NewGeoGeometry(gpsPoint, []spatialmath.Geometry{geometries})
		startPose, err := localizer.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)

		req := motion.MoveOnGlobeReq{
			ComponentName:      baseResource,
			Destination:        dst,
			MovementSensorName: moveSensorResource,
			Obstacles:          []*spatialmath.GeoGeometry{geoGeometry},
			MotionCfg:          motionCfg,
			Extra:              extra,
		}

		executionID, err := ms.MoveOnGlobe(ctx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

		timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
			ComponentName: req.ComponentName,
			ExecutionID:   executionID,
			LastPlanOnly:  true,
		})
		test.That(t, err, test.ShouldBeNil)

		endPose, err := localizer.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)
		movedPose := spatialmath.PoseBetween(startPose.Pose(), endPose.Pose())
		test.That(t, movedPose.Point().X, test.ShouldAlmostEqual, expectedDst.X, planDeviationMM)
		test.That(t, movedPose.Point().Y, test.ShouldAlmostEqual, expectedDst.Y, planDeviationMM)
	})
}

func TestMotionExtendedMapSimple(t *testing.T) {
	ctx := context.Background()
	logger := logging.NewTestLogger(t)
	var positionPollingFreqHz float64
	motionCfg := &motion.MotionConfiguration{PositionPollingFreqHz: &positionPollingFreqHz, PlanDeviationMM: 150, LinearMPerSec: 0.3, AngularDegsPerSec: 60}

	t.Run("Long distance", func(t *testing.T) {
		if runtime.GOARCH == "arm" {
			t.Skip("skipping on 32-bit ARM, large maps use too much memory")
		}
		// Driving faster makes
		motionCfgFast := &motion.MotionConfiguration{
			PositionPollingFreqHz: &positionPollingFreqHz,
			PlanDeviationMM:       250,
			LinearMPerSec:         0.9,
			AngularDegsPerSec:     60,
		}
		extra := map[string]interface{}{"motion_profile": "position_only"}
		// goal position is scaled to be in mm
		goalInBaseFrame := spatialmath.NewPoseFromPoint(r3.Vector{X: -32.508 * 1000, Y: -2.092 * 1000})
		goalInSLAMFrame := spatialmath.PoseBetweenInverse(motion.SLAMOrientationAdjustment, goalInBaseFrame)

		kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(
			ctx,
			t,
			"slam/example_cartographer_outputs/viam-office-02-22-3/pointcloud/pointcloud_4.pcd",
			110,
			spatialmath.NewPoseFromPoint(r3.Vector{0, -1600, 0}),
		)
		defer closeFunc(ctx)
		req := motion.MoveOnMapReq{
			ComponentName: base.Named("test-base"),
			Destination:   goalInSLAMFrame,
			SlamName:      slam.Named("test_slam"),
			Extra:         extra,
			MotionCfg:     motionCfgFast,
		}

		timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec*3)
		defer timeoutFn()
		executionID, err := ms.MoveOnMap(timeoutCtx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

		timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec*3)
		defer timeoutFn()
		err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
			ComponentName: req.ComponentName,
			ExecutionID:   executionID,
			LastPlanOnly:  true,
		})
		test.That(t, err, test.ShouldBeNil)

		endPos, err := kb.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)

		logger.Debug("long dist end pose", spatialmath.PoseToProtobuf(endPos.Pose()))

		test.That(t, spatialmath.PoseAlmostCoincidentEps(endPos.Pose(), goalInBaseFrame, PlanDeviationMM), test.ShouldBeTrue)
	})

	t.Run("Plans", func(t *testing.T) {
		// goal x-position of 1.32m is scaled to be in mm
		// Orientation theta should be at least 3 degrees away from an integer multiple of 22.5 to ensure the position-only test functions.
		goalInBaseFrame := spatialmath.NewPose(r3.Vector{X: 1.32 * 1000, Y: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 33})
		goalInSLAMFrame := spatialmath.PoseBetweenInverse(motion.SLAMOrientationAdjustment, goalInBaseFrame)
		extra := map[string]interface{}{}
		extraPosOnly := map[string]interface{}{"motion_profile": "position_only"}

		t.Run("ensure success of movement around obstacle", func(t *testing.T) {
			kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: -50}))
			defer closeFunc(ctx)

			req := motion.MoveOnMapReq{
				ComponentName: base.Named("test-base"),
				Destination:   goalInSLAMFrame,
				SlamName:      slam.Named("test_slam"),
				Extra:         extra,
				MotionCfg:     motionCfg,
			}

			timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			executionID, err := ms.MoveOnMap(timeoutCtx, req)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

			timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
				ComponentName: req.ComponentName,
				ExecutionID:   executionID,
				LastPlanOnly:  true,
			})
			test.That(t, err, test.ShouldBeNil)

			endPos, err := kb.CurrentPosition(ctx)
			test.That(t, err, test.ShouldBeNil)

			test.That(t, spatialmath.PoseAlmostCoincidentEps(endPos.Pose(), goalInBaseFrame, PlanDeviationMM), test.ShouldBeTrue)
		})
		t.Run("check that straight line path executes", func(t *testing.T) {
			kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, nil)
			defer closeFunc(ctx)
			easyGoalInBaseFrame := spatialmath.NewPoseFromPoint(r3.Vector{X: 0.277 * 1000, Y: 0.593 * 1000})
			easyGoalInSLAMFrame := spatialmath.PoseBetweenInverse(motion.SLAMOrientationAdjustment, easyGoalInBaseFrame)

			req := motion.MoveOnMapReq{
				ComponentName: base.Named("test-base"),
				Destination:   easyGoalInSLAMFrame,
				MotionCfg:     motionCfg,
				SlamName:      slam.Named("test_slam"),
				Extra:         extra,
			}

			timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			executionID, err := ms.MoveOnMap(timeoutCtx, req)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

			timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
				ComponentName: req.ComponentName,
				ExecutionID:   executionID,
				LastPlanOnly:  true,
			})
			test.That(t, err, test.ShouldBeNil)

			endPos, err := kb.CurrentPosition(ctx)
			test.That(t, err, test.ShouldBeNil)

			test.That(t, spatialmath.PoseAlmostEqualEps(endPos.Pose(), easyGoalInBaseFrame, PlanDeviationMM), test.ShouldBeTrue)
		})

		t.Run("check that position-only mode executes", func(t *testing.T) {
			kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, nil)
			defer closeFunc(ctx)

			req := motion.MoveOnMapReq{
				ComponentName: base.Named("test-base"),
				Destination:   goalInSLAMFrame,
				SlamName:      slam.Named("test_slam"),
				MotionCfg:     motionCfg,
				Extra:         extraPosOnly,
			}

			timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			executionID, err := ms.MoveOnMap(timeoutCtx, req)
			test.That(t, err, test.ShouldBeNil)
			test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

			timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
			defer timeoutFn()
			err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
				ComponentName: req.ComponentName,
				ExecutionID:   executionID,
				LastPlanOnly:  true,
			})
			test.That(t, err, test.ShouldBeNil)

			endPos, err := kb.CurrentPosition(ctx)
			test.That(t, err, test.ShouldBeNil)

			test.That(t, spatialmath.PoseAlmostCoincidentEps(endPos.Pose(), goalInBaseFrame, PlanDeviationMM), test.ShouldBeTrue)
			// Position only mode should not yield the goal orientation.
			test.That(t, spatialmath.OrientationAlmostEqualEps(
				endPos.Pose().Orientation(),
				goalInBaseFrame.Orientation(),
				0.035), test.ShouldBeFalse)
		})

		t.Run("should fail due to map collision", func(t *testing.T) {
			_, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, spatialmath.NewPoseFromPoint(r3.Vector{X: 0, Y: -500}))
			defer closeFunc(ctx)
			easyGoalInBaseFrame := spatialmath.NewPoseFromPoint(r3.Vector{X: 0.277 * 1000, Y: 0.593 * 1000})
			easyGoalInSLAMFrame := spatialmath.PoseBetweenInverse(motion.SLAMOrientationAdjustment, easyGoalInBaseFrame)
			executionID, err := ms.MoveOnMap(
				context.Background(),
				motion.MoveOnMapReq{
					ComponentName: base.Named("test-base"),
					Destination:   easyGoalInSLAMFrame,
					SlamName:      slam.Named("test_slam"),
					Extra:         extra,
				},
			)
			test.That(t, err, test.ShouldNotBeNil)
			logger.Debug(err.Error())
			test.That(t, strings.Contains(err.Error(), "starting collision between SLAM map and "), test.ShouldBeTrue)
			test.That(t, executionID, test.ShouldResemble, uuid.Nil)
		})
	})

	t.Run("Subsequent", func(t *testing.T) {
		// goal x-position of 1.32m is scaled to be in mm
		goal1SLAMFrame := spatialmath.NewPose(r3.Vector{X: 1.32 * 1000, Y: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 55})
		goal1BaseFrame := spatialmath.Compose(goal1SLAMFrame, motion.SLAMOrientationAdjustment)
		goal2SLAMFrame := spatialmath.NewPose(r3.Vector{X: 277, Y: 593}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 150})
		goal2BaseFrame := spatialmath.Compose(goal2SLAMFrame, motion.SLAMOrientationAdjustment)

		kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, nil)
		defer closeFunc(ctx)

		req := motion.MoveOnMapReq{
			ComponentName: base.Named("test-base"),
			Destination:   goal1SLAMFrame,
			MotionCfg:     motionCfg,
			SlamName:      slam.Named("test_slam"),
			Extra:         map[string]interface{}{"smooth_iter": 5},
		}

		timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		executionID, err := ms.MoveOnMap(timeoutCtx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

		timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
			ComponentName: req.ComponentName,
			ExecutionID:   executionID,
			LastPlanOnly:  true,
		})
		test.That(t, err, test.ShouldBeNil)

		endPos, err := kb.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)

		logger.Debug(spatialmath.PoseToProtobuf(endPos.Pose()))
		test.That(t, spatialmath.PoseAlmostEqualEps(endPos.Pose(), goal1BaseFrame, PlanDeviationMM), test.ShouldBeTrue)

		// Now, we try to go to the second goal. Since the `CurrentPosition` of our base is at `goal1`, the pose that motion solves for and
		// logs should be {x:-1043  y:593}
		req = motion.MoveOnMapReq{
			ComponentName: base.Named("test-base"),
			Destination:   goal2SLAMFrame,
			SlamName:      slam.Named("test_slam"),
			MotionCfg:     motionCfg,
			Extra:         map[string]interface{}{"smooth_iter": 5},
		}
		timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		executionID, err = ms.MoveOnMap(timeoutCtx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

		timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
			ComponentName: req.ComponentName,
			ExecutionID:   executionID,
			LastPlanOnly:  true,
		})
		test.That(t, err, test.ShouldBeNil)

		endPos, err = kb.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)
		logger.Debug(spatialmath.PoseToProtobuf(endPos.Pose()))
		test.That(t, spatialmath.PoseAlmostEqualEps(endPos.Pose(), goal2BaseFrame, PlanDeviationMM), test.ShouldBeTrue)

		plans, err := ms.PlanHistory(ctx, motion.PlanHistoryReq{
			ComponentName: base.Named("test-base"),
			LastPlanOnly:  false,
			ExecutionID:   executionID,
		})
		test.That(t, err, test.ShouldBeNil)

		goalPose1 := plans[0].Plan.Path()[0]["test-base"].Pose()
		goalPose2 := spatialmath.PoseBetween(
			plans[0].Plan.Path()[0]["test-base"].Pose(),
			plans[0].Plan.Path()[len(plans[0].Plan.Path())-1]["test-base"].Pose(),
		)

		// We don't actually surface the internal motion planning goal; we report to the user in terms of what the user provided us.
		// Thus, we use PlanHistory to get the plan steps of the latest plan.
		// The zeroth index of the plan steps is the relative position of goal1 and the pose inverse between the first and last value of
		// plan steps gives us the relative pose we solved for goal2.
		test.That(t, spatialmath.PoseAlmostEqualEps(goalPose1, goal1BaseFrame, PlanDeviationMM), test.ShouldBeTrue)

		// This is the important test.
		test.That(t, spatialmath.PoseAlmostEqualEps(goalPose2, spatialmath.PoseBetween(goal1BaseFrame, goal2BaseFrame), PlanDeviationMM), test.ShouldBeTrue)
	})

	t.Run("pass when within plan dev m of goal without position_only due to theta difference in goal", func(t *testing.T) {
		_, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, nil)
		defer closeFunc(ctx)

		req := motion.MoveOnMapReq{
			ComponentName: base.Named("test-base"),
			MotionCfg:     motionCfg,
			Destination:   spatialmath.NewPoseFromOrientation(&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 150}),
			SlamName:      slam.Named("test_slam"),
		}

		timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		executionID, err := ms.MoveOnMap(timeoutCtx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotBeEmpty)
	})
}

func TestMotionExtendedAskewIMU(t *testing.T) {
	t.Parallel()
	motionCfg := &motion.MotionConfiguration{PlanDeviationMM: 150, LinearMPerSec: 0.3, AngularDegsPerSec: 60}
	extraPosOnly := map[string]interface{}{"smooth_iter": 5, "motion_profile": "position_only"}
	t.Run("Askew but valid base should be able to plan", func(t *testing.T) {
		t.Parallel()
		logger := logging.NewTestLogger(t)
		ctx := context.Background()
		askewOrient := &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 35}
		// goal x-position of 1.32m is scaled to be in mm
		goal1SLAMFrame := spatialmath.NewPose(r3.Vector{X: 1.32 * 1000, Y: 0}, &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 55})
		goal1BaseFrame := spatialmath.Compose(goal1SLAMFrame, motion.SLAMOrientationAdjustment)

		kb, ms, closeFunc := builtin.CreateMoveOnMapTestEnvironment(ctx, t, "pointcloud/octagonspace.pcd", 40, spatialmath.NewPoseFromOrientation(askewOrient))
		defer closeFunc(ctx)

		req := motion.MoveOnMapReq{
			ComponentName: base.Named("test-base"),
			Destination:   goal1SLAMFrame,
			SlamName:      slam.Named("test_slam"),
			Extra:         extraPosOnly,
			MotionCfg:     motionCfg,
		}

		timeoutCtx, timeoutFn := context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		executionID, err := ms.MoveOnMap(timeoutCtx, req)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, executionID, test.ShouldNotResemble, uuid.Nil)

		timeoutCtx, timeoutFn = context.WithTimeout(ctx, time.Second*timeoutSec)
		defer timeoutFn()
		err = motion.PollHistoryUntilSuccessOrError(timeoutCtx, ms, time.Millisecond*5, motion.PlanHistoryReq{
			ComponentName: req.ComponentName,
			ExecutionID:   executionID,
			LastPlanOnly:  true,
		})
		test.That(t, err, test.ShouldBeNil)

		endPIF, err := kb.CurrentPosition(ctx)
		test.That(t, err, test.ShouldBeNil)

		// We need to transform the endPos by the corrected orientation in order to properly place it, otherwise it will go off in +Z somewhere.
		// In a real robot this will be taken care of by gravity.
		correctedPose := spatialmath.NewPoseFromOrientation(askewOrient)
		endPos := spatialmath.Compose(correctedPose, spatialmath.PoseBetween(spatialmath.NewPoseFromOrientation(askewOrient), endPIF.Pose()))
		logger.Debug(spatialmath.PoseToProtobuf(endPos))
		logger.Debug(spatialmath.PoseToProtobuf(goal1BaseFrame))

		test.That(t, spatialmath.PoseAlmostEqualEps(endPos, goal1BaseFrame, PlanDeviationMM), test.ShouldBeTrue)
	})
}

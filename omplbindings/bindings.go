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
	//~ "os"
	"fmt"
	"unsafe"
	
	"github.com/golang/geo/r3"
	commonpb "go.viam.com/api/common/v1"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/components/arm/xarm"
	"go.viam.com/rdk/components/arm/universalrobots"
	//~ "go.viam.com/rdk/motionplan"
)

var sceneFS referenceframe.FrameSystem
var sceneWS *commonpb.WorldState

var startPos []float64
var goalPose *C.struct_pose



//export ComputePositions
func ComputePositions(pos []float64) *C.struct_pose {
	pose := calcPose(pos)
	return poseToC(pose)
}

//export Init
func Init(sceneNum string) {
	sceneFS = referenceframe.NewEmptySimpleFrameSystem("")
	goalPose = (*C.struct_pose)(C.malloc(C.size_t(unsafe.Sizeof(C.struct_pose{}))))
	
	switch sceneNum {
	case "scene1":
		setupScene1()
	case "scene2":
		setupScene2()
	default:
	}
}

func calcPose(pos []float64) spatialmath.Pose {
	positions := map[string][]referenceframe.Input{}
	inputs := referenceframe.FloatsToInputs(pos)
	positions["arm"] = inputs
	posFrame := referenceframe.NewPoseInFrame("arm", spatialmath.NewZeroPose())
	tf, err := sceneFS.Transform(positions, posFrame, "world")
	if err != nil{
		fmt.Println(err)
		return nil
	}
	pose, _ := tf.(*referenceframe.PoseInFrame)
	return pose.Pose()
}

func poseToC(pose spatialmath.Pose) *C.struct_pose{
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

func cToPose(cPose *C.struct_pose) spatialmath.Pose{
	pt := r3.Vector{float64(cPose.X), float64(cPose.Y), float64(cPose.Z)}
	orient := &spatialmath.EulerAngles{Roll: float64(cPose.Roll), Pitch: float64(cPose.Pitch), Yaw: float64(cPose.Yaw)}
	
	return spatialmath.NewPoseFromOrientation(pt, orient)
}

// setup a UR5 moving along a linear path in unrestricted space
func setupScene1() {
	arm, _ := universalrobots.Model("arm")
	sceneFS.AddFrame(arm, sceneFS.World())

	sceneWS = &commonpb.WorldState{}
	startPos = []float64{0,0,0,0,0,0}
	
	startPose := calcPose(startPos)
	goalPt := startPose.Point()
	goalPt.X += 100
	goalPt.Y += 100
	
	goalPose = poseToC(spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()))
}

// setup a xArm7 to move in a straight line, adjacent to a large obstacle that should not imede the most efficient path
func setupScene2() {
	arm, _ := xarm.Model("arm", 7)
	sceneFS.AddFrame(arm, sceneFS.World())
	testPose := spatialmath.NewPoseFromOrientation(
		r3.Vector{X: 1., Y: -100., Z: 3.},
		&spatialmath.R4AA{Theta: 0, RX: 0., RY: 0., RZ: 1.},
	)
	obsMsgs := []*commonpb.GeometriesInFrame{
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
	}
	sceneWS = &commonpb.WorldState{Obstacles: obsMsgs}
	startPos = []float64{0,0,0,0,0,0, 0}
	
	startPose := calcPose(startPos)
	goalPt := startPose.Point()
	goalPt.X += 200
	goalPt.Z += 100
	
	goalPose = poseToC(spatialmath.NewPoseFromOrientation(goalPt, startPose.Orientation()))
}

func weightedSqNormDist(from, to spatialmath.Pose) float64 {
	delta := spatialmath.PoseDelta(from, to)
	// Increase weight for orientation since it's a small number
	return delta.Point().Norm2() + spatialmath.QuatToR3AA(delta.Orientation().Quaternion()).Mul(10.).Norm2()
}

// Needed for C export
func main() {}

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
	"os"
	"fmt"
	"unsafe"
	
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	//~ "go.viam.com/rdk/motionplan"
)

var sceneFS referenceframe.FrameSystem

//export ComputePositions
func ComputePositions(frameName string, pos []float64) *C.struct_pose {
	
	eulerPose := (*C.struct_pose)(C.malloc(C.size_t(unsafe.Sizeof(C.struct_pose{}))))

	
	positions := map[string][]referenceframe.Input{}
	inputs := referenceframe.FloatsToInputs(pos)
	positions[frameName] = inputs
	posFrame := referenceframe.NewPoseInFrame(frameName, spatialmath.NewZeroPose())
	tf, err := sceneFS.Transform(positions, posFrame, "world")
	if err != nil{
		fmt.Println(err)
		return eulerPose
		//~ return
	}
	pose, _ := tf.(*referenceframe.PoseInFrame)
	pt := pose.Pose().Point()
	eulerPose.X = C.double(pt.X)
	eulerPose.Y = C.double(pt.Y)
	eulerPose.Z = C.double(pt.Z)
	
	orient := pose.Pose().Orientation().EulerAngles()
	eulerPose.Pitch = C.double(orient.Pitch)
	eulerPose.Roll = C.double(orient.Roll)
	eulerPose.Yaw = C.double(orient.Yaw)
	
	fmt.Println(eulerPose)
	return eulerPose
}

//export Init
func Init(modelFile string) {
	sceneFS = referenceframe.NewEmptySimpleFrameSystem("")
	b, err := os.ReadFile(modelFile) // just pass the file name
	if err != nil {
		fmt.Print(err)
	}
	model, err := referenceframe.UnmarshalModelJSON(b, "arm")
	if err != nil {
		fmt.Print(err)
	}
	
	sceneFS.AddFrame(model, sceneFS.World())
}

// Needed for C export
func main() {}

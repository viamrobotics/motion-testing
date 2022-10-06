package main

import (
	"C"
	"os"
	"fmt"
	
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	//~ "go.viam.com/rdk/motionplan"
)

var sceneFS referenceframe.FrameSystem

//export ComputePositions
func ComputePositions(frameName string, pos []float64) []float64 {
	
	var eulerPose []float64
	
	positions := map[string][]referenceframe.Input{}
	inputs := referenceframe.FloatsToInputs(pos)
	positions[frameName] = inputs
	posFrame := referenceframe.NewPoseInFrame(frameName, spatialmath.NewZeroPose())
	tf, err := sceneFS.Transform(positions, posFrame, "world")
	if err != nil{
		fmt.Println(err)
		return eulerPose
	}
	pose, _ := tf.(*referenceframe.PoseInFrame)
	pt := pose.Pose().Point()
	eulerPose = append(eulerPose, pt.X)
	eulerPose = append(eulerPose, pt.Y)
	eulerPose = append(eulerPose, pt.Z)
	
	orient := pose.Pose().Orientation().EulerAngles()
	eulerPose = append(eulerPose, orient.Pitch)
	eulerPose = append(eulerPose, orient.Roll)
	eulerPose = append(eulerPose, orient.Yaw)
	
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

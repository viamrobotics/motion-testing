package main

import (
	"context"
	"encoding/csv"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"strconv"

	"github.com/edaniels/golog"
	pb "go.viam.com/api/component/arm/v1"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/robot/client"
	rdkutils "go.viam.com/rdk/utils"
	"go.viam.com/utils"
	"go.viam.com/utils/rpc"
)

var logger = golog.NewDevelopmentLogger("client")

func main() {
	utils.ContextualMain(mainWithArgs, logger)
}

func mainWithArgs(ctx context.Context, args []string, logger golog.Logger) error {
	// parse command line input
	address := flag.String("address", "", "address of robot on app.viam.com")
	payload := flag.String("payload", "", "payload corresponding to address")
	sceneName := flag.String("scene_name", "", "name of the scene plan file to parse and execute")
	armName := flag.String("arm", "arm", "name of the arm being planned with")
	flag.Parse()

	// Connect to robot arm
	robot, err := connect(*address, *payload)
	if err != nil {
		return err
	}
	defer robot.Close(context.Background())
	fmt.Println(">>> Successfully connected to robot")
	arm, err := arm.FromRobot(robot, *armName)
	if err != nil {
		return err
	}
	fmt.Printf(">>> Found arm with name [ %s ]\n", *armName)

	// Parse the plan for the given scene name
	path := resolvePath(*sceneName + ".csv")
	fmt.Printf(">>> Searching for plan path file [ %s ]\n", path)
	plan, err := planFromCSV(path)
	if err != nil {
		return err
	}

	// Execute the plan
	fmt.Println(">>> Executing plan ...")
	for i, step := range plan {
		fmt.Printf("\t\tStep [ %d / %d ] - %.2v\n", i, len(plan), step.Values)
		err := arm.MoveToJointPositions(context.Background(), step, nil)
		if err != nil {
			return err
		}
	}
	return nil
}

func resolvePath(filename string) string {
	_, thisFilePath, _, _ := runtime.Caller(0)
	thisDirPath, err := filepath.Abs(filepath.Dir(thisFilePath))
	if err != nil {
		panic(err)
	}
	return filepath.Join(thisDirPath, "..", filename)
}

func planFromCSV(filepath string) ([]*pb.JointPositions, error) {
	// open file
	f, err := os.Open(filepath)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	// read csv values using csv.Reader
	csvReader := csv.NewReader(f)
	data, err := csvReader.ReadAll()
	if err != nil {
		return nil, err
	}

	// convert to a plan
	plan := make([]*pb.JointPositions, 0)
	for _, line := range data {
		floats := make([]float64, 0)
		for _, elem := range line {
			float, err := strconv.ParseFloat(elem, 64)
			if err != nil {
				return nil, err
			}
			floats = append(floats, rdkutils.RadToDeg(float))
		}
		plan = append(plan, &pb.JointPositions{Values: floats})
	}
	return plan, nil
}

func connect(address, payload string) (*client.RobotClient, error) {
	credentials := rpc.WithCredentials(rpc.Credentials{Type: rdkutils.CredentialsTypeRobotLocationSecret, Payload: payload})
	return client.New(context.Background(), address, logger, client.WithDialOptions(credentials))
}

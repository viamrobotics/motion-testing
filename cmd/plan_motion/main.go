package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"os"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
)

func main() {
	ctx := context.Background()
	logger := logging.NewDebugLogger("scene6")
	if len(os.Args) != 2 {
		fmt.Printf("usage: %s plan_request.json\n", os.Args[0])
		os.Exit(1)
	}
	// scene, err := scenes.Scene6(ctx, logger)
	// if err != nil {
	// 	log.Fatal(err.Error())
	// }
	// b, err := json.Marshal(scene)
	// if err != nil {
	// 	log.Fatal(err.Error())
	// }
	//
	// if _, err = os.Stdout.Write(b); err != nil {
	// 	log.Fatal(err.Error())
	// }
	b, err := os.ReadFile(os.Args[1])
	if err != nil {
		log.Fatal(err.Error())
	}
	var planRequest armplanning.PlanRequest
	if err := json.Unmarshal(b, &planRequest); err != nil {
		log.Fatal(err.Error())
	}

	_, err = armplanning.PlanMotion(ctx, logger, &planRequest)
	if err != nil {
		log.Fatal(err.Error())
	}
}

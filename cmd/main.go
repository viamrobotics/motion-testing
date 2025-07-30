package main

import (
	"context"
	"log"

	scenes "go.viam.com/motion-testing"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
)

func main() {
	ctx := context.Background()
	logger := logging.NewDebugLogger("scene6")
	scene, err := scenes.Scene6(ctx, logger)
	if err != nil {
		log.Fatal(err.Error())
	}
	_, err = armplanning.PlanMotion(ctx, logger, scene)
	if err != nil {
		log.Fatal(err.Error())
	}
}

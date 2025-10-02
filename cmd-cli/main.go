package main

import (
	"flag"
	"fmt"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"

	"go.viam.com/motiontesting"
)

func main() {
	err := realMain()
	if err != nil {
		panic(err)
	}
}

func realMain() error {
	logger := logging.NewLogger("motion-testing-cli")

	debug := flag.Bool("v", false, "debugging")

	flag.Parse()

	if *debug {
		logger.SetLevel(logging.DEBUG)
	}

	if len(flag.Args()) < 1 {
		return fmt.Errorf("need a command")
	}

	cmd := flag.Args()[0]
	args := flag.Args()[1:]

	switch cmd {
	case "run":
		if len(args) < 1 {
			return fmt.Errorf("need name")
		}

		return motiontesting.RunScenes(args[0], armplanning.NewBasicPlannerOptions(), logger)

	case "score":
		if len(args) < 2 {
			return fmt.Errorf("need 2 directories to score")
		}

		res1, err := motiontesting.ScoreFolder(args[0], logger)
		if err != nil {
			return fmt.Errorf("couldn't score %s : %w", args[0], err)
		}

		res2, err := motiontesting.ScoreFolder(args[1], logger)
		if err != nil {
			return fmt.Errorf("couldn't score %s : %w", args[1], err)
		}

		return motiontesting.CompareResults(res1, res2)
	default:
		return fmt.Errorf("unknown command [%s]", cmd)
	}

}

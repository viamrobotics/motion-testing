package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"maps"
	"os"
	"slices"
	"strconv"

	scenes "go.viam.com/motion-testing"
	"go.viam.com/rdk/logging"
)

func main() {
	ctx := context.Background()
	logger := logging.NewDebugLogger("scene6")
	if len(os.Args) != 2 {
		fmt.Printf("usage: %s <int>\n", os.Args[0])
		os.Exit(1)
	}
	i, err := strconv.Atoi(os.Args[1])
	if err != nil {
		fmt.Printf("usage: %s <int>, err: %s\n", os.Args[0], err.Error())
		os.Exit(1)
	}
	sceneFn, ok := scenes.AllScenes[i]
	keys := slices.Collect(maps.Keys(scenes.AllScenes))

	if !ok {
		fmt.Printf("usage: %s <%d-%d>, err: s\n", os.Args[0], slices.Min(keys), slices.Max(keys))
		os.Exit(1)
	}
	scene, err := sceneFn(ctx, logger)
	if err != nil {
		log.Fatal(err.Error())
	}
	b, err := json.Marshal(scene)
	if err != nil {
		log.Fatal(err.Error())
	}

	if _, err = os.Stdout.Write(b); err != nil {
		log.Fatal(err.Error())
	}
}

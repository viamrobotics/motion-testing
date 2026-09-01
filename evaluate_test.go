package motiontesting

import (
	"flag"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

// flags used to define folders insider results folder to compare
var (
	baselineFlag = flag.String("baselineDir", "baseline", "name of test to use as a baseline")
	modifiedFlag = flag.String("modifiedDir", "default", "name of test to compare to the baseline")
)

func TestScores(t *testing.T) {
	logger := logging.NewTestLogger(t)
	baseline, err := ScoreFolder(*baselineFlag, logger)
	test.That(t, err, test.ShouldBeNil)
	modification, err := ScoreFolder(*modifiedFlag, logger)
	test.That(t, err, test.ShouldBeNil)

	// compare folders with results
	test.That(t, CompareResults(baseline, modification), test.ShouldBeNil)
}

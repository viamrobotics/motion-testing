package motiontesting

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"

	"go.viam.com/rdk/logging"
	"go.viam.com/test"
)

// writeSequenceResult writes one synthetic sequence-plan result the way runOrderedSequence would: a stats
// file, and a two-step solution CSV when the plan succeeded.
func writeSequenceResult(t *testing.T, dir string, sceneNum, planIdx, pass int, success bool, seconds float64) {
	t.Helper()
	base := filepath.Join(dir, fmt.Sprintf("scene%d-%03d_%d", sceneNum, planIdx, pass))
	stats := fmt.Sprintf("%t,%f\n", success, seconds)
	test.That(t, os.WriteFile(base+"_stats.txt", []byte(stats), 0o644), test.ShouldBeNil)
	if success {
		test.That(t, os.WriteFile(base+".csv", []byte("0.0,0.0\n1.0,0.0\n"), 0o644), test.ShouldBeNil)
	}
}

func makeSequenceResultDir(t *testing.T, name string) string {
	t.Helper()
	dir := filepath.Join(resultsDirectory, name)
	test.That(t, os.MkdirAll(dir, 0o755), test.ShouldBeNil)
	t.Cleanup(func() { os.RemoveAll(dir) })
	test.That(t, os.WriteFile(filepath.Join(dir, "hash"), []byte("test-sha"), 0o644), test.ShouldBeNil)
	return dir
}

func TestOrderedSequenceScoringAndRendering(t *testing.T) {
	logger := logging.NewTestLogger(t)

	// Baseline: a clean cold pass (pass 0, slower — caches are empty) then two clean warm passes
	// over a three-plan sequence.
	baseDir := makeSequenceResultDir(t, "unit-sequence-base")
	writeSequenceResult(t, baseDir, 100, 0, 0, true, 3.0)
	writeSequenceResult(t, baseDir, 100, 1, 0, true, 1.5)
	writeSequenceResult(t, baseDir, 100, 2, 0, true, 0.5)
	for pass := 1; pass <= 2; pass++ {
		writeSequenceResult(t, baseDir, 100, 0, pass, true, 1.0)
		writeSequenceResult(t, baseDir, 100, 1, pass, true, 0.5)
		writeSequenceResult(t, baseDir, 100, 2, pass, true, 0.2)
	}

	// Modified: plan 0 got much slower, plan 1 fails every pass including the cold one, plan 2
	// is unchanged.
	modDir := makeSequenceResultDir(t, "unit-sequence-mod")
	writeSequenceResult(t, modDir, 100, 0, 0, true, 6.0)
	writeSequenceResult(t, modDir, 100, 1, 0, false, 0.01)
	writeSequenceResult(t, modDir, 100, 2, 0, true, 0.5)
	for pass := 1; pass <= 2; pass++ {
		writeSequenceResult(t, modDir, 100, 0, pass, true, 2.0)
		writeSequenceResult(t, modDir, 100, 1, pass, false, 0.01)
		writeSequenceResult(t, modDir, 100, 2, pass, true, 0.2)
	}

	base, err := ScoreFolder("unit-sequence-base", logger)
	test.That(t, err, test.ShouldBeNil)
	mod, err := ScoreFolder("unit-sequence-mod", logger)
	test.That(t, err, test.ShouldBeNil)

	// The baseline's clean warm passes each yield one whole-sequence sample; the cold pass is
	// aggregated separately.
	test.That(t, base.score[100].samples, test.ShouldEqual, 2.)
	test.That(t, base.score[100].successes, test.ShouldEqual, 2.)
	test.That(t, len(base.score[100].performances), test.ShouldEqual, 2)
	for _, seconds := range base.score[100].performances {
		test.That(t, seconds, test.ShouldAlmostEqual, 1.7)
	}
	test.That(t, base.coldScore[100].samples, test.ShouldEqual, 1.)
	test.That(t, base.coldScore[100].successes, test.ShouldEqual, 1.)
	test.That(t, len(base.coldScore[100].performances), test.ShouldEqual, 1)
	test.That(t, base.coldScore[100].performances[0], test.ShouldAlmostEqual, 5.0)

	// The modified passes each failed a plan: availability records it, and no quality or
	// performance sample may exist — a pass that did less work must not read as faster.
	test.That(t, mod.score[100].samples, test.ShouldEqual, 2.)
	test.That(t, mod.score[100].successes, test.ShouldAlmostEqual, 4./3.)
	test.That(t, len(mod.score[100].performances), test.ShouldEqual, 0)
	test.That(t, len(mod.score[100].qualities), test.ShouldEqual, 0)
	test.That(t, mod.coldScore[100].samples, test.ShouldEqual, 1.)
	test.That(t, mod.coldScore[100].successes, test.ShouldAlmostEqual, 2./3.)
	test.That(t, len(mod.coldScore[100].performances), test.ShouldEqual, 0)

	// Failed plans contribute no per-plan samples, and the cold pass stays out of the per-plan
	// scores entirely.
	test.That(t, mod.sequencePlans[100][1].samples, test.ShouldEqual, 2.)
	test.That(t, mod.sequencePlans[100][1].successes, test.ShouldEqual, 0.)
	test.That(t, len(mod.sequencePlans[100][1].performances), test.ShouldEqual, 0)
	test.That(t, base.sequencePlans[100][0].samples, test.ShouldEqual, 2.)

	test.That(t, CompareResults(base, mod), test.ShouldBeNil)
	mdBytes, err := os.ReadFile(filepath.Join(resultsDirectory, "motion-benchmarks.md"))
	test.That(t, err, test.ShouldBeNil)
	md := string(mdBytes)

	// The ordered sequence renders as a cold and a warm row in the aggregate tables, and the
	// plan-by-plan section reports the newly failing plan and the plan that slowed down.
	test.That(t, md, test.ShouldContainSubstring, "| 100 (cold) | 100% | 67% |")
	test.That(t, md, test.ShouldContainSubstring, "| 100 (warm) | 100% | 67% |")
	test.That(t, md, test.ShouldContainSubstring, "| 100 (cold) | 5.00±0.00 |")
	test.That(t, md, test.ShouldContainSubstring, "Newly failing plans (1)")
	test.That(t, md, test.ShouldContainSubstring, "`001 grinding/move` failed 2 of 2 warm passes")
	test.That(t, md, test.ShouldContainSubstring, "| `000 grinding/move` | 1.00±0.00 | 2.00±0.00 | -100% | 0% | ❌ |")
	// The failing plan and the unchanged plan must not appear in the time table.
	test.That(t, md, test.ShouldNotContainSubstring, "| `001 grinding/move` |")
	test.That(t, md, test.ShouldNotContainSubstring, "| `002 grinding/move` |")
}

func TestOrderedSequenceOneSidedComparison(t *testing.T) {
	logger := logging.NewTestLogger(t)

	baseDir := makeSequenceResultDir(t, "unit-sequence-only")
	writeSequenceResult(t, baseDir, 100, 0, 1, true, 1.0)
	makeSequenceResultDir(t, "unit-sequence-empty")

	base, err := ScoreFolder("unit-sequence-only", logger)
	test.That(t, err, test.ShouldBeNil)
	mod, err := ScoreFolder("unit-sequence-empty", logger)
	test.That(t, err, test.ShouldBeNil)

	test.That(t, CompareResults(base, mod), test.ShouldBeNil)
	mdBytes, err := os.ReadFile(filepath.Join(resultsDirectory, "motion-benchmarks.md"))
	test.That(t, err, test.ShouldBeNil)

	// One side never ran the ordered sequence: that must be said outright, not rendered as "nothing
	// moved".
	test.That(t, string(mdBytes), test.ShouldContainSubstring, "has no ordered-sequence data")
	test.That(t, string(mdBytes), test.ShouldNotContainSubstring, "No individual plan moved appreciably")
}

func TestOrderedSequenceNumbersDisjoint(t *testing.T) {
	for sceneNum := range allOrderedSequences {
		_, collides := allScenes[sceneNum]
		test.That(t, collides, test.ShouldBeFalse)
	}
}

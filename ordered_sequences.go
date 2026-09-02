package motiontesting

import (
	"crypto/sha256"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"runtime/debug"

	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/utils/artifact"
)

// An ordered sequence replays the plan requests captured from one production robot set of actions, in the
// order the robot planned them. Single-request scenes measure a plan in isolation; ordered
// sequences exist because armplanning carries learned roadmaps, smart-seed caches and a
// static-scene SDF registry between calls within a process, so what a plan costs depends on which
// plans ran before it. Replaying a whole sequence is the shape production actually runs.
//
// The payloads are far too large for git and live in the artifact store; the committed manifest
// pins the replay order, the content, and the labels used to join plans across two runs.

// allOrderedSequences maps a scene number to the manifest of the sequence it replays. Ordered
// sequences share the numbering space with allScenes, so they start at 100: numbers below that
// are reserved for single-request scenes (the README already promises 13-18 to base scenes).
// RunScenes refuses to run if the two maps collide.
var allOrderedSequences = map[int]string{
	100: "data/sequences/cappuccina-5fb95a4c.json",
}

// numSequencePasses is how many warm replays of each sequence are taken, after the cold pass.
// Replaying an entire sequence is orders of magnitude more work than a single-request scene, so
// sequences get fewer samples than the numTests taken per single scene.
var numSequencePasses = 3

// sequencePlanTimeout caps each replayed plan, in seconds. Captured requests carry production
// timeouts of hundreds of seconds; one pathologically regressed plan should fail fast rather
// than burn the whole CI budget.
const sequencePlanTimeout = 30.

// sequenceGCPercent pins the garbage collector during a replay. The planner is allocation-heavy
// enough that the GC target moves plan times materially, so it is set explicitly — an inherited
// GOGC difference between two runs must not masquerade as a planner change. 300 is what
// production planning uses.
const sequenceGCPercent = 300

type sequenceEntry struct {
	Index  int    `json:"index"`
	File   string `json:"file"`
	Step   string `json:"step"`
	Motion string `json:"motion"`
	SHA256 string `json:"sha256"`
}

// name is the stable label used to join a plan across two benchmark runs. It must not depend on
// timing or on the rdk revision under test.
func (e sequenceEntry) name() string {
	return fmt.Sprintf("%03d %s/%s", e.Index, e.Step, e.Motion)
}

type sequenceManifest struct {
	// Order is the data-capture tag the corpus was exported from.
	Order string `json:"order"`
	// ArtifactPath is where the payloads live in the artifact tree.
	ArtifactPath string          `json:"artifact_path"`
	Entries      []sequenceEntry `json:"entries"`
}

func loadSequenceManifest(path string) (*sequenceManifest, error) {
	data, err := os.ReadFile(filepath.Clean(path))
	if err != nil {
		return nil, err
	}
	m := &sequenceManifest{}
	if err := json.Unmarshal(data, m); err != nil {
		return nil, err
	}
	if len(m.Entries) == 0 {
		return nil, fmt.Errorf("sequence manifest %q has no entries", path)
	}
	return m, nil
}

// resolveOrderedSequence loads a sequence's manifest and fetches its payloads from the artifact
// store, returning the directory holding them. Every payload is verified against the manifest's
// content hash first: a corpus that silently drifted in the store would invalidate every
// comparison made against it.
func resolveOrderedSequence(manifestPath string) (*sequenceManifest, string, error) {
	m, err := loadSequenceManifest(manifestPath)
	if err != nil {
		return nil, "", err
	}
	dir, err := artifact.Path(m.ArtifactPath)
	if err != nil {
		return nil, "", fmt.Errorf("fetching sequence %q from the artifact store: %w", m.ArtifactPath, err)
	}
	for _, e := range m.Entries {
		data, err := os.ReadFile(filepath.Join(dir, e.File))
		if err != nil {
			return nil, "", fmt.Errorf("sequence entry %s: %w", e.name(), err)
		}
		if sum := sha256.Sum256(data); e.SHA256 != hex.EncodeToString(sum[:]) {
			return nil, "", fmt.Errorf("sequence entry %s: content does not match the manifest hash", e.name())
		}
	}
	return m, dir, nil
}

func runOrderedSequence(outputFolder string, sceneNum int, manifestPath string, logger logging.Logger) error {
	m, dir, err := resolveOrderedSequence(manifestPath)
	if err != nil {
		return err
	}

	// The planner logs multi-kilobyte debug lines per subgoal, and rdk subloggers do not follow
	// a runtime level set on their parent; over ~100 plans per pass that formatting costs whole
	// minutes of benchmark budget, so planning gets a silent logger. Failures still land in the
	// stats files.
	planLogger := logging.NewBlankLogger("sequence-plan")

	prevGC := debug.SetGCPercent(sequenceGCPercent)
	defer debug.SetGCPercent(prevGC)

	// The sequence starts from a cold planner so that pass 0 measures a genuine cold start: the
	// smart-seed cache is cleared outright, and the on-disk roadmap cache is disabled the way
	// viam-server runs (on rdk revisions without that cache the env var is inert). Newer rdk
	// revisions also keep an in-process learned-roadmap registry with no public reset; sequences
	// are the first users of their frame systems in this process, so pass 0 still plans cold.
	armplanning.ClearSeedCache()
	if err := os.Setenv("MOTION_ROADMAP_CACHE_DIR", ""); err != nil {
		return err
	}

	// Pass 0 replays the sequence cold and is recorded as the sequence's cold-start metrics; it
	// doubles as the cache warm-up, so the following passes all measure the planner in its
	// warmed, steady-state shape. Scoring keys off the pass number: 0 is cold, the rest are warm.
	for pass := 0; pass <= numSequencePasses; pass++ {
		logger.Infof("ordered sequence sceneNum: %d pass: %d of %d (0 is the cold start)", sceneNum, pass, numSequencePasses)
		for _, entry := range m.Entries {
			req, err := armplanning.ReadRequestFromFile(filepath.Join(dir, entry.File))
			if err != nil {
				return fmt.Errorf("sequence entry %s: %w", entry.name(), err)
			}

			// The recorded planner options are kept so each request is planned the way
			// production planned it; only the seed varies per pass, and the timeout is capped.
			// The cold pass runs seed 0, which is what production ran.
			if req.PlannerOptions == nil {
				req.PlannerOptions = armplanning.NewBasicPlannerOptions()
			}
			req.PlannerOptions.RandomSeed = pass
			if req.PlannerOptions.Timeout > sequencePlanTimeout {
				req.PlannerOptions.Timeout = sequencePlanTimeout
			}

			fileName := filepath.Join(outputFolder, fmt.Sprintf("scene%d-%03d_%d", sceneNum, entry.Index, pass))
			if err := planAndRecord(fileName, req, planLogger); err != nil {
				return fmt.Errorf("sequence entry %s: %w", entry.name(), err)
			}
		}
	}
	return nil
}

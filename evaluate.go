package motiontesting

import (
	"encoding/csv"
	"fmt"
	"math"
	"os"
	"path"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	"github.com/golang/geo/r3"
	"github.com/montanaflynn/stats"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/stat/distuv"
)

const defaultEpsilon = 1e-2

const (
	ptgDistStartIdx = 2
	ptgDistEndIdx   = 3
)

// TestResult holds the per-scene scores for a single results folder.
type TestResult struct {
	name  string
	score map[int]*testScore
	// coldScore holds each ordered sequence's cold-start aggregate: the metrics of the single
	// pass replayed against cleared caches. score holds the warm aggregate over the remaining
	// passes, alongside the single scenes.
	coldScore map[int]*testScore
	// sequencePlans holds, for each ordered sequence, the per-plan scores keyed by the plan's
	// index in the recorded order, over the warm passes only — one cold sample per plan is too
	// noisy to compare plan-by-plan. The scene-level aggregates live in score and coldScore; this
	// is what lets the comparison say which plans inside the sequence moved.
	sequencePlans map[int]map[int]*testScore
	sha1          string
}

type testScore struct {
	successes    float64
	samples      float64
	qualities    stats.Float64Data
	performances stats.Float64Data
}

// these variables represent the lower and higher bounds (exclusive) for unacceptable and acceptable values respectively
var (
	percentImprovementHealthThresholds     = [2]float64{0, 0}
	probabilityImprovementHealthThresholds = [2]float64{16, 84} // numbers derive from first standard deviation of normal distribution
)

func evaluateSolution(solution [][]float64, scene sceneFunc, logger logging.Logger) (float64, float64, float64, error) {
	var l2Score, lineScore, oScore, totalLineDist float64
	var err error

	req, err := scene(logger)
	if err != nil {
		return -1, -1, -1, err
	}

	var poseStart, poseEnd spatialmath.Pose
	sceneFrame := req.FrameSystem.Frame("test_base")
	if req.FrameSystem.Frame("arm") != nil {
		sceneFrame = req.FrameSystem.Frame("arm")
		poseStart, err = sceneFrame.Transform(solution[0])
		if poseStart == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
			return -1, -1, -1, err
		}
		poseEnd, err = sceneFrame.Transform(solution[len(solution)-1])
		if poseEnd == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
			return -1, -1, -1, err
		}
		totalLineDist = poseStart.Point().Sub(poseEnd.Point()).Norm()
	}

	// For each step
	for i := 0; i < len(solution)-1; i++ {
		l2Score += referenceframe.InputsL2Distance(solution[i], solution[i+1])

		// Check linear and orientation excursion every 2 degrees of joint movement
		if req.FrameSystem.Frame("arm") != nil {
			nSteps := getSteps(solution[i], solution[i+1])
			for j := 1; j <= nSteps; j++ {
				step, err := sceneFrame.Interpolate(
					solution[i],
					solution[i+1],
					float64(j)/float64(nSteps),
				)
				if err != nil {
					return -1, -1, -1, err
				}
				pose, err := sceneFrame.Transform(step)
				if pose == nil || (err != nil && !strings.Contains(err.Error(), referenceframe.OOBErrString)) {
					return -1, -1, -1, err
				}
				lineScore += distToLine(poseStart.Point(), poseEnd.Point(), pose.Point())
				oScore += orientScore(poseStart.Orientation(), poseEnd.Orientation(), pose.Orientation())
			}
		} else {
			lineScore += math.Abs(solution[i][ptgDistEndIdx] - solution[i][ptgDistStartIdx])
		}
	}

	if totalLineDist != 0 {
		lineScore /= totalLineDist
	}

	return l2Score, lineScore, oScore, nil
}

// L2Distance returns the L2 normalized difference between two equal length arrays.
func distToLine(pt1, pt2, query r3.Vector) float64 {
	ab := pt1.Sub(pt2)
	av := query.Sub(pt2)

	if av.Dot(ab) <= 0.0 { // Point is lagging behind start of the segment, so perpendicular distance is not viable.
		return av.Norm() // Use distance to start of segment instead.
	}

	bv := query.Sub(pt1)

	if bv.Dot(ab) >= 0.0 { // Point is advanced past the end of the segment, so perpendicular distance is not viable.
		return bv.Norm()
	}
	dist := (ab.Cross(av)).Norm() / ab.Norm()
	if dist < defaultEpsilon {
		return 0.
	}

	return dist
}

// How many steps to step between configs 2 degrees at a time
func getSteps(q1, q2 []float64) int {
	nSteps := 1
	for i, j1 := range q1 {
		// convert to degrees
		// jDiff := (180 * math.Abs(j1 - q2[i]))/math.Pi
		jDiff := math.Abs(j1 - q2[i])
		if math.Ceil(jDiff/2.) > float64(nSteps) {
			nSteps = int(math.Ceil(jDiff / 2.))
		}
	}
	return nSteps
}

func orientDist(o1, o2 spatialmath.Orientation) float64 {
	return math.Sqrt(spatialmath.QuatToR3AA(spatialmath.OrientationBetween(o1, o2).Quaternion()).Norm2())
}

func orientScore(start, end, query spatialmath.Orientation) float64 {
	origDist := math.Max(orientDist(start, end), defaultEpsilon)

	sDist := math.Max(orientDist(start, query), defaultEpsilon)
	gDist := 0.

	// If origDist is less than or equal to defaultEpsilon, then the starting and ending orientations are the same and we do not need
	// to compute the distance to the ending orientation
	if origDist > defaultEpsilon {
		gDist = orientDist(end, query)
	}
	return (sDist + gDist) - origDist
}

// ScoreFolder scores every run recorded in results/<folder>, writing a per-run breakdown to
// results/<folder>/results.csv and returning the scores aggregated by scene.
func ScoreFolder(folder string, logger logging.Logger) (*TestResult, error) {
	fullPath := filepath.Join(resultsDirectory, folder)
	fileInfo, err := os.Stat(fullPath)
	if err != nil || !fileInfo.IsDir() {
		return nil, fmt.Errorf("could not open folder: %s", fullPath)
	}

	runs, err := os.ReadDir(fullPath)
	if err != nil {
		return nil, err
	}

	records := [][]string{
		{"scene", "seed", "success", "time", "total_score", "joint_score", "line_score", "orient_score"},
	}

	hashBytes, err := os.ReadFile(filepath.Join(fullPath, "hash"))
	if err != nil {
		return nil, err
	}

	results := &TestResult{
		name:          folder,
		score:         make(map[int]*testScore, 0),
		coldScore:     make(map[int]*testScore),
		sequencePlans: make(map[int]map[int]*testScore),
		sha1:          string(hashBytes),
	}

	// Totals of one replay pass over a sequence, folded into the sequence's scene-level score
	// after the scan. Keyed sceneNum → pass.
	type sequencePassTotals struct {
		seconds float64
		quality float64
		solved  int
		plans   int
	}
	sequencePasses := map[int]map[int]*sequencePassTotals{}

	for _, run := range runs {
		if path.Ext(run.Name()) == ".txt" {
			// parse file name to determine what the test parameters were: single scenes write
			// scene<N>_<seed>_stats.txt, ordered sequences scene<N>-<planIdx>_<pass>_stats.txt
			fileName := strings.Split(run.Name(), "_")
			sceneName, planIdxStr, isSequencePlan := strings.Cut(strings.TrimPrefix(fileName[0], "scene"), "-")
			sceneNum, err := strconv.Atoi(sceneName)
			if err != nil {
				return nil, err
			}

			// read the file and get the results of the run
			bytes, err := os.ReadFile(filepath.Join(fullPath, run.Name()))
			if err != nil {
				return nil, err
			}
			rundata := strings.Split(string(bytes), ",")

			// Parse whether was successful or not
			pass := rundata[0]

			// Parse time it took to complete
			time, err := strconv.ParseFloat(strings.TrimSpace(rundata[1]), 64)
			if err != nil {
				return nil, err
			}

			if isSequencePlan {
				planIdx, err := strconv.Atoi(planIdxStr)
				if err != nil {
					return nil, err
				}
				passNum, err := strconv.Atoi(fileName[1])
				if err != nil {
					return nil, err
				}

				passes, ok := sequencePasses[sceneNum]
				if !ok {
					passes = map[int]*sequencePassTotals{}
					sequencePasses[sceneNum] = passes
				}
				totals, ok := passes[passNum]
				if !ok {
					totals = &sequencePassTotals{}
					passes[passNum] = totals
				}

				// Pass 0 is the cold-start pass: it counts toward the pass totals, which is
				// where the cold aggregate comes from, but not toward the per-plan scores —
				// a single cold sample per plan is too noisy to compare plan-by-plan.
				var plan *testScore
				if passNum > 0 {
					plans, ok := results.sequencePlans[sceneNum]
					if !ok {
						plans = map[int]*testScore{}
						results.sequencePlans[sceneNum] = plans
					}
					plan, ok = plans[planIdx]
					if !ok {
						plan = &testScore{}
						plans[planIdx] = plan
					}
					plan.samples++
				}

				// Like single scenes, a failed plan contributes no quality or performance
				// sample — a plan that fails fast must not read as one that got faster.
				totals.plans++
				jScore, csvTime := -1., -1.
				if pass == "true" {
					data, err := readSolutionFromCSV(filepath.Join(fullPath, fileName[0]+"_"+fileName[1]+".csv"))
					if err != nil {
						return nil, err
					}
					jScore, csvTime = solutionL2(data), time
					totals.quality += jScore
					totals.seconds += time
					totals.solved++
					if plan != nil {
						plan.successes++
						plan.qualities = append(plan.qualities, jScore)
						plan.performances = append(plan.performances, time)
					}
				}
				records = append(records, []string{
					fileName[0],
					fileName[1],
					pass,
					fmt.Sprintf("%f", csvTime),
					fmt.Sprintf("%f", jScore),
					fmt.Sprintf("%f", jScore),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
				})
				continue
			}

			score, ok := results.score[sceneNum]
			if !ok {
				score = &testScore{
					qualities:    make(stats.Float64Data, 0),
					performances: make(stats.Float64Data, 0),
				}
			}
			score.samples++
			if pass == "true" {
				data, err := readSolutionFromCSV(filepath.Join(fullPath, fileName[0]+"_"+fileName[1]+".csv"))
				if err != nil {
					return nil, err
				}

				jScore, tScore, oScore, err := evaluateSolution(data, allScenes[sceneNum], logger)
				if err != nil {
					return nil, err
				}

				records = append(records, []string{
					fileName[0],
					fileName[1],
					pass,
					fmt.Sprintf("%f", time),
					fmt.Sprintf("%f", jScore+tScore+oScore),
					fmt.Sprintf("%f", jScore),
					fmt.Sprintf("%f", tScore),
					fmt.Sprintf("%f", oScore),
				})

				score.successes++
				score.qualities = append(score.qualities, jScore) // joint score is the scope we will use for quality
				score.performances = append(score.performances, time)
			} else {
				records = append(records, []string{
					fileName[0],
					fileName[1],
					pass,
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
					fmt.Sprintf("%f", -1.),
				})
			}
			results.score[sceneNum] = score
		}
	}

	// A sequence's scene-level rows aggregate whole passes: one availability, quality and
	// performance sample per replay of the entire sequence, with pass 0 — replayed against
	// cleared caches — reported separately as the cold start. A pass with any failed plan yields
	// no quality or performance sample — its totals cover less work, so comparing them against a
	// clean pass would let a regression that fails plans render as an improvement. Availability
	// is what reports such a pass.
	for sceneNum, passes := range sequencePasses {
		if _, ok := results.score[sceneNum]; ok {
			return nil, fmt.Errorf("scene number %d has both single-scene and ordered-sequence results", sceneNum)
		}
		warm, cold := &testScore{}, &testScore{}
		for passNum, totals := range passes {
			score := warm
			if passNum == 0 {
				score = cold
			}
			score.samples++
			score.successes += float64(totals.solved) / float64(totals.plans)
			if totals.solved == totals.plans {
				score.performances = append(score.performances, totals.seconds)
				score.qualities = append(score.qualities, totals.quality)
			}
		}
		results.score[sceneNum] = warm
		if cold.samples > 0 {
			results.coldScore[sceneNum] = cold
		}
	}

	if err := writeCSV(filepath.Join(fullPath, "results.csv"), records); err != nil {
		return nil, err
	}
	return results, nil
}

// solutionL2 is the joint-space length of a solution: the same joint score evaluateSolution
// computes, without needing the scene rebuilt. Ordered-sequence plans are scored this way because
// rebuilding a scene means re-reading a multi-megabyte captured request for every pass.
func solutionL2(solution [][]float64) float64 {
	total := 0.
	for i := 0; i < len(solution)-1; i++ {
		total += referenceframe.InputsL2Distance(solution[i], solution[i+1])
	}
	return total
}

// CompareResults writes a markdown report to results/motion-benchmarks.md contrasting the
// availability, quality and performance scores of the two given result sets.
func CompareResults(baseline, modification *TestResult) error {
	sceneNums := sortedSceneNums(baseline, modification)
	rows := comparisonRows(sceneNums, baseline, modification)

	var builder strings.Builder

	builder.WriteString(tableHeaderInts("Availability", baseline.name, modification.name))
	for _, row := range rows {
		builder.WriteString(tableEntryInt(row.label, row.baseline, row.modification))
	}

	builder.WriteString(tableHeaderFloats("Quality", baseline.name, modification.name))
	for _, row := range rows {
		builder.WriteString(tableEntryFloats(row.label, row.baseline, row.modification, (*testScore).qualityData))
	}

	builder.WriteString(tableHeaderFloats("Performance", baseline.name, modification.name))
	for _, row := range rows {
		builder.WriteString(tableEntryFloats(row.label, row.baseline, row.modification, (*testScore).performanceData))
	}

	for _, sceneNum := range sceneNums {
		builder.WriteString(orderedSequenceSection(sceneNum, baseline, modification))
	}

	builder.WriteString("\nThe above data was generated by running scenes defined in the " +
		"[`motion-testing`](https://github.com/viamrobotics/motion-testing/) repository")
	builder.WriteString(fmt.Sprintf("\nThe SHA1 for %s is: %s", baseline.name, baseline.sha1))
	builder.WriteString(fmt.Sprintf("\nThe SHA1 for %s is: %s", modification.name, modification.sha1))
	builder.WriteString(fmt.Sprintf("\n* **%d samples** were taken for each scene, **1 cold + %d warm passes** for each ordered sequence",
		numTests, numSequencePasses))

	return os.WriteFile(filepath.Join(resultsDirectory, "motion-benchmarks.md"), []byte(builder.String()), 0o600)
}

// comparisonRow is one line of the aggregate tables: a label and the two scores it compares. An
// ordered sequence contributes two rows, its cold start and its warm aggregate.
type comparisonRow struct {
	label                  string
	baseline, modification *testScore
}

func comparisonRows(sceneNums []int, baseline, modification *TestResult) []comparisonRow {
	rows := make([]comparisonRow, 0, len(sceneNums))
	for _, i := range sceneNums {
		coldBase, coldMod := baseline.coldScore[i], modification.coldScore[i]
		if coldBase == nil && coldMod == nil {
			rows = append(rows, comparisonRow{strconv.Itoa(i), baseline.score[i], modification.score[i]})
			continue
		}
		rows = append(rows,
			comparisonRow{fmt.Sprintf("%d (cold)", i), coldBase, coldMod},
			comparisonRow{fmt.Sprintf("%d (warm)", i), baseline.score[i], modification.score[i]},
		)
	}
	return rows
}

// sortedSceneNums returns every scene number either result has data for, in order, so the tables
// stay complete even when the two runs did not execute the same scene set.
func sortedSceneNums(baseline, modification *TestResult) []int {
	seen := map[int]bool{}
	for i := range baseline.score {
		seen[i] = true
	}
	for i := range modification.score {
		seen[i] = true
	}
	nums := make([]int, 0, len(seen))
	for i := range seen {
		nums = append(nums, i)
	}
	sort.Ints(nums)
	return nums
}

func (ts *testScore) qualityData() stats.Float64Data {
	if ts == nil {
		return nil
	}
	return ts.qualities
}

func (ts *testScore) performanceData() stats.Float64Data {
	if ts == nil {
		return nil
	}
	return ts.performances
}

// minNotablePlanSeconds keeps trivially short plans out of the per-plan tables: a plan going
// from 3ms to 9ms is a 3x that means nothing to the sequence.
const minNotablePlanSeconds = 0.05

// maxPlanRows caps each per-plan table; a comment that runs to a hundred rows does not get read.
const maxPlanRows = 15

// orderedSequenceSection renders the plan-by-plan breakdown of an ordered sequence over its warm
// passes: which plans newly fail and which moved appreciably in time or joint travel. The cold
// and warm rows in the tables above say whether the sequence as a whole moved; this section says
// which plans carried it.
func orderedSequenceSection(sceneNum int, baseline, modification *TestResult) string {
	basePlans := baseline.sequencePlans[sceneNum]
	modPlans := modification.sequencePlans[sceneNum]
	if len(basePlans) == 0 && len(modPlans) == 0 {
		return ""
	}
	if len(basePlans) == 0 || len(modPlans) == 0 {
		missing := baseline.name
		if len(basePlans) > 0 {
			missing = modification.name
		}
		return fmt.Sprintf("\n## Scene %d plan-by-plan\n\n%s has no ordered-sequence data for scene %d, so plans cannot be compared.\n",
			sceneNum, missing, sceneNum)
	}

	planNums := map[int]bool{}
	for i := range basePlans {
		planNums[i] = true
	}
	for i := range modPlans {
		planNums[i] = true
	}
	nums := make([]int, 0, len(planNums))
	for i := range planNums {
		nums = append(nums, i)
	}
	sort.Ints(nums)

	title, labels := sequenceManifestInfo(sceneNum, len(nums))
	label := func(idx int) string {
		if l, ok := labels[idx]; ok {
			return l
		}
		return fmt.Sprintf("%03d", idx)
	}

	var builder strings.Builder
	builder.WriteString(fmt.Sprintf("\n## Scene %d plan-by-plan: %s\n", sceneNum, title))

	var newlyFailing, newlyFixed []string
	type planRow struct {
		label    string
		cmp      floatComparison
		absDelta float64
	}
	var timeRows, qualityRows []planRow

	for _, idx := range nums {
		b, m := basePlans[idx], modPlans[idx]
		if b == nil || m == nil {
			continue
		}
		switch {
		case b.successes == b.samples && m.successes < m.samples:
			newlyFailing = append(newlyFailing, fmt.Sprintf("`%s` failed %.0f of %.0f warm passes", label(idx), m.samples-m.successes, m.samples))
		case b.successes < b.samples && m.successes == m.samples:
			newlyFixed = append(newlyFixed, fmt.Sprintf("`%s`", label(idx)))
		}

		// A plan with no successful pass on a side has no quality or performance samples to
		// compare; the newly failing/passing lists are what report it.
		if b.successes == 0 || m.successes == 0 {
			continue
		}
		if cmp := compareFloatData(b.performances, m.performances); cmp.notable() &&
			(cmp.baseMu >= minNotablePlanSeconds || cmp.modMu >= minNotablePlanSeconds) {
			timeRows = append(timeRows, planRow{label(idx), cmp, math.Abs(cmp.modMu - cmp.baseMu)})
		}
		if cmp := compareFloatData(b.qualities, m.qualities); cmp.notable() {
			qualityRows = append(qualityRows, planRow{label(idx), cmp, math.Abs(cmp.modMu - cmp.baseMu)})
		}
	}

	if len(newlyFailing) > 0 {
		builder.WriteString(fmt.Sprintf("\n⛔ **Newly failing plans (%d):**\n", len(newlyFailing)))
		for _, line := range newlyFailing {
			builder.WriteString("* " + line + "\n")
		}
	}
	if len(newlyFixed) > 0 {
		builder.WriteString(fmt.Sprintf("\n✅ **Newly passing plans (%d):** %s\n", len(newlyFixed), strings.Join(newlyFixed, ", ")))
	}

	writeRows := func(title string, rows []planRow) {
		if len(rows) == 0 {
			return
		}
		sort.Slice(rows, func(i, j int) bool { return rows[i].absDelta > rows[j].absDelta })
		shown := rows
		if len(shown) > maxPlanRows {
			shown = shown[:maxPlanRows]
		}
		builder.WriteString(fmt.Sprintf("\n### %s\n| Plan | %s | %s | Percent Improvement | Probability of Improvement | Health | \n",
			title, baseline.name, modification.name))
		builder.WriteString("| :--- | :----: | :---: | :---: | :----: | :---: |\n")
		for _, row := range shown {
			builder.WriteString(fmt.Sprintf("| `%s` | %.2f±%.2f | %.2f±%.2f | %.0f%% | %.0f%% | %c | \n",
				row.label,
				row.cmp.baseMu, row.cmp.baseSigma,
				row.cmp.modMu, row.cmp.modSigma,
				row.cmp.improvement,
				row.cmp.probability,
				row.cmp.health,
			))
		}
		if len(rows) > len(shown) {
			builder.WriteString(fmt.Sprintf("\n…and %d more.\n", len(rows)-len(shown)))
		}
	}
	writeRows("Time moved (seconds, warm passes, by absolute change)", timeRows)
	writeRows("Joint travel moved (L2, warm passes, by absolute change)", qualityRows)

	if len(newlyFailing) == 0 && len(newlyFixed) == 0 && len(timeRows) == 0 && len(qualityRows) == 0 {
		builder.WriteString("\nNo individual plan moved appreciably.\n")
	}
	return builder.String()
}

// sequenceManifestInfo names an ordered sequence and its plans from the sequence's manifest;
// scoring results from a sequence whose manifest is no longer registered still renders, just
// without the labels.
func sequenceManifestInfo(sceneNum, planCount int) (string, map[int]string) {
	title := fmt.Sprintf("%d plans replayed in recorded order", planCount)
	manifestPath, ok := allOrderedSequences[sceneNum]
	if !ok {
		return title, nil
	}
	m, err := loadSequenceManifest(manifestPath)
	if err != nil {
		return title, nil
	}
	title = fmt.Sprintf("%s (%d plans replayed in recorded order)", strings.TrimSuffix(filepath.Base(manifestPath), ".json"), len(m.Entries))
	labels := make(map[int]string, len(m.Entries))
	for _, e := range m.Entries {
		labels[e.Index] = e.name()
	}
	return title, labels
}

func readSolutionFromCSV(filepath string) ([][]float64, error) {
	csvfile, err := os.Open(filepath)
	if err != nil {
		return nil, err
	}
	defer func() { _ = csvfile.Close() }()

	reader := csv.NewReader(csvfile)
	fields, err := reader.ReadAll()
	if err != nil {
		return nil, err
	}

	solution := [][]float64{}
	for _, waypoint := range fields {
		step := make([]float64, 0, len(waypoint))
		for _, pos := range waypoint {
			posF, err := strconv.ParseFloat(pos, 64)
			if err != nil {
				return nil, err
			}
			step = append(step, posF)
		}
		solution = append(solution, step)
	}

	return solution, nil
}

func tableHeaderInts(name, baseline, modification string) string {
	formatLine := "| :---: | :----: | :---: | :----: | :---: |\n"
	return fmt.Sprintf("\n## %s\n| Scene # | %s | %s | Percent Improvement | Health | \n",
		name,
		baseline,
		modification,
	) + formatLine
}

func tableEntryInt(label string, initial, final *testScore) string {
	initialPct, finalPct := initial.successPercent(), final.successPercent()
	delta := percentDifference(initialPct, finalPct)
	return fmt.Sprintf("| %s | %.0f%% | %.0f%% | %.0f%% | %c | \n",
		label,
		initialPct,
		finalPct,
		delta,
		healthIndicator(delta, delta, percentImprovementHealthThresholds),
	)
}

func (ts *testScore) successPercent() float64 {
	if ts == nil || ts.samples == 0 {
		return math.NaN()
	}
	return 100 * ts.successes / ts.samples
}

func tableHeaderFloats(name, baseline, modification string) string {
	formatLine := "| :---: | :----: | :---: | :---: | :----: | :---: |\n"
	return fmt.Sprintf("\n## %s\n| Scene # | %s | %s | Percent Improvement | Probability of Improvement | Health | \n",
		name,
		baseline,
		modification,
	) + formatLine
}

func tableEntryFloats(label string, initial, final *testScore, data func(*testScore) stats.Float64Data) string {
	cmp := compareFloatData(data(initial), data(final))
	return fmt.Sprintf("| %s | %.2f\u00B1%.2f | %.2f\u00B1%.2f | %.0f%% | %.0f%% | %c | \n",
		label,
		cmp.baseMu, cmp.baseSigma,
		cmp.modMu, cmp.modSigma,
		cmp.improvement,
		cmp.probability,
		cmp.health,
	)
}

// floatComparison is the comparison of one metric's samples across the two runs. Lower is better
// for every metric compared this way, so improvement is positive when the value went down.
type floatComparison struct {
	baseMu, baseSigma float64
	modMu, modSigma   float64
	improvement       float64
	probability       float64
	health            rune
}

// notable reports whether the change is worth a reader's attention: big enough to matter and
// unlikely to be noise, by the same thresholds the health column uses.
func (c floatComparison) notable() bool {
	return c.health != '\u2796'
}

func compareFloatData(initial, final stats.Float64Data) floatComparison {
	// create normal distributions from initial and final float slices
	A, AValid := normal(initial)
	B, BValid := normal(final)

	var probability float64
	switch {
	case AValid && BValid:
		// create normal distribution C = B - A
		C := distuv.Normal{
			Mu:    B.Mu - A.Mu,
			Sigma: math.Sqrt(A.Sigma*A.Sigma + B.Sigma*B.Sigma),
		}

		switch {
		case C.Sigma != 0:
			// probability that B is an improvement over A is found by evaluating the CDF at x=0
			probability = 100 * C.CDF(0)
		case C.Mu < 0:
			probability = 100
		case C.Mu > 0:
			probability = 0
		default:
			probability = 50
		}
	case AValid && !BValid:
		probability = 0
	case !AValid && BValid:
		probability = 100
	default:
		probability = math.NaN()
	}

	delta := percentDifference(A.Mu, B.Mu)

	return floatComparison{
		baseMu: A.Mu, baseSigma: A.Sigma,
		modMu: B.Mu, modSigma: B.Sigma,
		improvement: -delta, // want to flip it so its an improvement if its less
		probability: probability,
		health:      healthIndicator(-delta, probability, probabilityImprovementHealthThresholds),
	}
}

func percentDifference(initial, final float64) float64 {
	return 100.0 * (final - initial) / initial
}

// normal makes a normal distribution from the float slice
func normal(data stats.Float64Data) (distuv.Normal, bool) {
	mean, err1 := data.Mean()
	stdDev, err2 := data.StandardDeviation()
	if err1 != nil || err2 != nil {
		return distuv.Normal{Mu: math.NaN(), Sigma: math.NaN()}, false
	}
	return distuv.Normal{Mu: mean, Sigma: stdDev}, true
}

func healthIndicator(val, prob float64, thresholds [2]float64) rune {
	switch {
	case math.Abs(val) <= 5 || math.IsNaN(prob):
		return '➖'
	case (prob + .1) < thresholds[0]:
		return '❌'
	case (prob - .1) > thresholds[1]:
		return '✅'
	default:
		return '➖'
	}
}

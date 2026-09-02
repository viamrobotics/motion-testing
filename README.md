# motion-testing

At present, development of motion planning within Viam benefits from significant virtualized testing. Checking algorithmic performance, behavior of motion planners, forward and inverse kinematic solver performance, and tracking collision checking performance are all facilitated by the use of **scenes**. Scenes are combinations of specific robotic arms, world configurations, start and goal states, and other input parameters that comprise a repeatable test. These scenes allow the Motion Planning team to evaluate their capabilities in a more integrated fashion. 

The `motion-testing` repository also serves as the foundation for automated testing to track changes in performance that may be rooted in complex interactions. Pull requests against the `motionplan`, `referenceframe`, or `referenceframe` areas of the `rdk` repository initiate a GitHub action to execute these tests on a cloud container and post the results to the PR activity feed. This allows developers to see the impact (if any) of changes to foundational Motion or Motion-adjacent code prior to merging.

Separately, the capability exists to execute these tests on a local resource (such as a developer laptop) in the same manner as the automated CI workflow executes on cloud resources. This allows developers to evaluate performance and quality for the same scenes without needing to open an initial PR and iterate during the review process. A section below outlines how all developers can run the automated scene tests on an ad-hoc basis, compare results against each other, and generate the results table for various desired outcomes.

A comprehensive overview of the scenes (including their objectives, what features they demonstrate, and how they tie into user stories) is included below.

## Manual Execution of Tests

Note that executing these tests requires many of the same dependencies as developing against `rdk` in the first place, most notably an operational Go development environment. See the `rdk` README for details on setting up a development environment for further details.

Two possible use cases of `motion-testing` will be described in this section. First, executing a single test sequence with released `rdk` software can be done to establish baseline performance at any time. Second, comparisons of local changes and their impacts (such as those made during engineering development) can be made against the performance of unmodified code in upstream respositories. Both cases require the `motion-testing` repository to be cloned locally to continue.

### Single Benchmark

For the single benchmarking case, simply navigate to the `motion-testing` directory and execute the following commands from the head of the respository. The `$TEST_NAME` placeholder should be changed to match the desired name of the directory that will contain generated test data from this execution. You can reference individual scene executions and results by navigating to the named subdirectory in the `results` folder.

```bash
go test ./... -v -run TestDefault --name=$TEST_NAME
go test ./... -v -run TestScores --baselineDir=$TEST_NAME --modifiedDir=$TEST_NAME
cat results/motion-benchmarks.md
```

### Comparing Baseline Performance to Performance with Software Changes

In the event that comparisons between modified and unmodified code are needed, additional steps are required. This section assumes the baseline tests will be run first, followed by tests that act on modified software.

The `go.mod` file in this repository indicates a specific release version of `rdk` should be used, but a `replace` directive is commented out at the bottom of this file. Take note of this directive; we will be modifying it later.

Start by executing the first command, replacing the placeholder `$TEST_NAME` with the desired folder name; the example below will use `baseline`. If desired, the `replace` directive mentioned above can be changed to point at a local version of `rdk` (ensure you modify the path to point at the local `rdk` repository). Note that whatever code is checked out in the local `rdk` installation will be evaluated, so the `main` branch target will be the only valid options for providing "baseline" software.

```bash
go test ./... -v -run TestDefault --name=baseline
```

Once this executes and results data is in the `results/baseline` directory, you must now checkout the desired changes in your local copy of the `rdk` repository and uncomment the `replace` directive (if you haven't already). Then execute the following commands:

```bash
go test ./... -v -run TestDefault --name=modified
go test ./... -v -run TestScores --baselineDir=baseline --modifiedDir=modified
cat results/motion-benchmarks.md
```

Note the two directory arguments for `TestScores` and how these compare to the prior test commands.

By changing checked out branches in the `rdk` repository (or other repos), a developer could also benchmark multiple sets of changes against each other.

## Continuous Integration

Every push to `main` and every pull request runs the `CI` workflow
(`.github/workflows/ci.yml`), which has two jobs:

- **Lint** — verifies `go.mod`/`go.sum` are tidy and runs `golangci-lint` against the
  configuration in `.golangci.yaml`. Run the same checks locally with `make lint`, and apply the
  formatter with `make format`.
- **Test** — builds the module, plans every scene with `TestDefault`, then scores the run with
  `TestScores`. The generated `results/` directory is uploaded as a workflow artifact and the
  benchmark table is written to the job summary.

The CI run scores its results against themselves, so it does not detect planning regressions; it
verifies that the scenes, the scorer and the report generator all still work. Regressions are
tracked by the `Motion Benchmarks` workflow in
[`rdk`](https://github.com/viamrobotics/rdk), which runs these same scenes against a PR branch and
its base and posts the comparison to the PR.

## Arm Scenes

Basic Arm Scenes 1-12 are derived from brainstorming sessions the Motion team participated in to get automated performance testing off the ground. These do not have any specific user in mind, but could be thought of as possible situations users might encounter during workspace setup or application development.

Later, named scenes represent customer-specific applications or demos that we supported or observed. By tailoring specific scenes to these applications, we should be able to anticipate impacts to certain user applications before our users make such discoveries in production.

At this time, each scene is tightly coupled to a specific robot configuration. Future expansion is planned to decouple these elements, enabling the execution of all scenes with all major robotic arm models that Viam currently supports.

| Basic Scenes | Description of Task                                                                                                      | Arm + EE Used |
| ------------ | ------------------------------------------------------------------------------------------------------------------------ | ------------- |
| armScene1       | Straight-line move in unrestricted space                                                                                 | UR5 + None    |
| armScene2       | Straight-line move with large obstacles nearby                                                                           | UR5 + None    |
| armScene3       | Reach over a short obstacle that obstructs the direct path                                                               | UR5 + None    |
| armScene4       | Reach over a short obstacle that obstructs the direct path                                                               | xArm6 + None  |
| armScene5       | Reach through a window with a large wall on one side                                                                     | xArm7 + None  |
| armScene6       | Reach through a window with a large wall behind and to the side                                                          | xArm7 + None  |
| armScene7       | Reach over a short obstacle, staying within a narrow corridor                                                            | xArm6 + None  |
| armScene8       | Pouring motion with end effector, with large obstacles nearby                                                            | xArm7 + None  |
| armScene9       | Large arm motion within a “forest”, numerous small obstacles in a random distribution surrounding the robot              | UR5 + None    |

## Ordered Sequences

Where the scenes above measure a single plan request in isolation, **ordered sequences** replay a series of plan requests captured from production, in the order the robot planned them. They exist because `armplanning` carries learned roadmaps, smart-seed caches and a static-scene SDF registry between calls within a process, so what a plan costs depends on which plans ran before it — replaying a whole sequence is the shape production actually runs, and it is what catches regressions in that cross-plan state.

Ordered sequences are numbered from 100 internally to keep the lower numbers free for single-request scenes; reports label them by their ordinal instead (`Ordered Sequence 1`, ...).

| Ordered Sequences | Description                                                                                 | Plans |
| ----------------- | ------------------------------------------------------------------------------------------- | ----- |
| 100               | One full espresso-drink order captured from a Cappuccina machine (xArm6), about 7.5 minutes  | 98    |

Mechanically, an ordered sequence is defined by a manifest committed at `data/sequences/<name>.json`, which pins the replay order, the step/motion labels used to join plans across two runs, and the content hashes of the captured requests. The request payloads themselves are far too large for git (~100MB per sequence) and live in the artifact store; they are fetched on demand through the `.artifact` configuration the first time the sequence runs.

Each sequence starts from a cold planner: the smart-seed cache is cleared and the on-disk roadmap cache is disabled before the replay begins, the way production runs. The first pass is recorded as the sequence's **cold-start** metrics — it doubles as the cache warm-up — and the following passes measure the planner **warm**, in its steady-state shape; fewer warm passes are taken than the samples per single scene, since one pass replays the entire sequence. The recorded planner options are kept so each request is planned the way production planned it; only the random seed varies per pass and the per-plan timeout is capped. The garbage collector target is pinned during a replay so that an inherited `GOGC` difference between two runs cannot masquerade as a planner change.

In the results tables an ordered sequence appears as two rows aggregating whole passes — `Ordered Sequence 1 (cold)` and `Ordered Sequence 1 (warm)`: availability is the fraction of plans in the sequence that solved, quality is the summed joint travel of the sequence, and performance is the wall time of the whole sequence. Below the tables, a plan-by-plan section lists the individual plans within the sequence that newly fail, newly pass, or moved appreciably in time or joint travel between the two runs' warm passes — a handful of plans usually carry a regression that the sequence total alone would blur. A collapsed details block under that section holds the full per-plan data: every plan's cold-pass values and warm-pass statistics, for both time and joint travel.

## Base Scenes

Basic Base Scenes 13-18 cover a variety of situations where planning for a base on a SLAM map might be used. These scenes aren't specific to any type of base.

| Basic Scenes | Description of Task                                                                    |
| ------------ | ---------------------------------------------------------------------------------------|
| baseScene1      | Straight-line move on a small map with no obstacles                                    |
| baseScene2      | Move around an obstacle on a small map                                                 |
| baseScene3      | Straight-line move on a large office map                                               |
| baseScene4      | Move around one corner on a large office map                                           |
| baseScene5      | Move a medium distance on a large office map with narrow hallways along the path       |
| baseScene6      | Move a large distance on a large office map with many obstacles in the way             |

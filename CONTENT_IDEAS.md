# MARS — Content Ideas

Grounded in things actually built and measured in this repo, not generic ML/robotics content.
Each entry cites the real artifact behind it. Draft copy is ready to paste and tweak;
visual specs describe exactly what to capture/build — you make the actual assets.

Format assumed: LinkedIn/blog-length posts (300-500 words), each easily trimmed into a
Twitter/X thread (first 2-3 lines are already written as a strong hook for that purpose).

---

## Post 1 — "MAPPO lost to a hand-written heuristic — here's why we published that anyway"

**Draft copy:**

> We trained a Multi-Agent PPO policy for 15 iterations to coordinate a 3-robot
> swarm exploring a room. Then we benchmarked it against a dumb, hand-written
> "go to the nearest unexplored cell" heuristic.
>
> The heuristic won. 21.1% area coverage vs. our RL policy's 10.4%. Higher
> collision rate too.
>
> Most projects would quietly retrain until the number looked better, or just
> not publish this comparison. We put it in the paper instead — Table I, no
> asterisks — because the honest failure mode is more useful to other people
> building this than a cherry-picked win would be.
>
> What actually happened: with only ~4,500 environment steps of training (this
> runs real-time physics in Gazebo, not a fast simulator — no shortcuts), the
> policy's entropy collapsed to -0.53 by the final iteration. It converged to a
> locally "safe" near-stationary behavior before it ever saw enough episodes to
> learn that exploring pays off. Classic tiny-batch, sparse-reward RL failure —
> and a genuinely useful one to see in public, logs and all.
>
> We've since diagnosed the exact fix (GAE lambda, entropy bonus, reward
> rebalancing — details in the next post) and are re-running training now.
> Paper, code, and the honest first result: [link].

**Visual assets needed:**
- **Screenshot**: `paper/main.tex` Table I rendered (the actual PDF table) — crop tightly to just the comparison table, high-res.
- **Screenshot**: a snippet of the raw RLlib console log showing `entropy: -0.53` alongside the iteration number — authenticity matters here, use the real log, don't recreate it.
- **Optional infographic** (simple bar chart, 3 bars): ACR% for Random Walk / Frontier Heuristic / MAPPO, colored so MAPPO is visually the "loser" — the point of the post is owning that.

---

## Post 2 — "We debugged a robot swarm stuck spinning in a doorway"

**Draft copy:**

> Three robots. One coverage grid. And a bug that took real debugging, not
> guessing, to find.
>
> Symptom: robots would explore fine for a while, then just... stop. Frozen.
> Not crashed — LiDAR showed zero obstacles nearby. They just wouldn't move.
>
> We could have thrown more training iterations at it and hoped. Instead we
> added targeted debug logging to one robot and watched exactly what it was
> doing, step by step.
>
> Turned out: the robot was picking a target grid cell, driving to within 0.2m
> of that cell's center, and stopping — "close enough." Except the grid cell
> itself was only 0.275m wide. Stopping 0.2m short of the center meant it
> sometimes never actually crossed into the cell. The cell never got marked
> "visited." So the exact same cell got picked as the target again next step.
> Forever.
>
> A geometry mismatch between a stopping tolerance and a grid resolution,
> invisible unless you're watching the exact numbers. Fixed by scaling the
> grid resolution to the world size instead of reusing a fixed constant.
> Coverage went from stuck-at-15% to climbing cleanly toward 100%.
>
> The lesson: when a swarm "just stops," don't retrain — instrument it and
> watch one agent's raw numbers for 100 steps. The bug is almost always
> boring and specific.

**Visual assets needed:**
- **Screen recording (30-60s)**: RViz or Gazebo showing a robot visibly stuck/oscillating in place while others move — captures the actual bug visually.
- **Screenshot**: terminal output showing the debug print lines (`state=`, `target=`, `n_unvisited=` repeating identically across many step numbers) — this is the "aha" evidence.
- **Simple diagram/infographic**: a small grid with one cell highlighted, a robot icon just outside a 0.2m circle drawn around the cell center but outside the cell boundary — visually explains the geometry mismatch in one image.

---

## Post 3 — "Don't trust a 3D mesh you haven't inspected"

**Draft copy:**

> Before running our robot swarm in a new Gazebo world downloaded from an
> online model library, we didn't just load it and hope. We downloaded the
> raw collision mesh (an STL file) and inspected it in Python first.
>
> Why: we'd already lost hours earlier debugging a world where robots got
> wedged in furniture we didn't know was there. Not doing that twice.
>
> ~150 lines of Python: parse the STL's binary triangle data, slice it at the
> robot's LiDAR height (10-30cm off the ground — tall shelving above that
> doesn't matter to a 2D LiDAR), and rasterize what's actually at floor level
> onto a grid. Turned an unknown 30x50m mesh into a clear picture: mostly open
> floor, a few small support pillars at known coordinates, walls only at the
> perimeter.
>
> That gave us exact, verified pillar coordinates before ever booting the
> simulator — enough to deliberately design a test zone that includes real
> obstacles instead of guessing where they might be.
>
> Simulation is supposed to save you from surprises. A mesh you haven't
> looked at is still a surprise waiting to happen.

**Visual assets needed:**
- **Screenshot**: the ASCII-art occupancy map printed by the analysis script (the `#`/`.` grid showing walls and pillar positions) — genuinely a nice piece of "here's the actual output" evidence.
- **Screenshot or clip**: Gazebo GUI showing the real warehouse model from a wide angle, with the pillar locations visible.
- **Code screenshot**: the ~10-line core of the STL bounding-box/slab-intersection logic — short enough to read in a screenshot.

---

## Post 4 — "Control Barrier Functions don't make a bad policy good"

**Draft copy:**

> There's a seductive idea in safe robotics: wrap your RL policy in a Control
> Barrier Function filter, and now it's safe, regardless of how good the
> policy is. We built exactly this — a dual-lookahead CBF/ACAS filter that
> checks every commanded velocity against LiDAR and neighbor robots before
> it's sent to the motors.
>
> Here's the honest boundary of what that buys you.
>
> The filter guarantees: the robot will never be commanded to drive into a
> detected obstacle. If the safety QP is infeasible, it defaults to halting
> forward motion rather than blindly executing the raw command.
>
> The filter does NOT guarantee: that the robot does anything useful. If the
> underlying policy never learns to move away from its own spawn point (which
> ours didn't, early on), the filter can't invent an escape maneuver for it.
> Safe and stationary is still "safe" by the filter's definition — and
> useless by every other definition.
>
> We measured this directly: our undertrained MAPPO policy still logged
> "collisions" during benchmarking (a soft metric — minimum LiDAR range
> dropping below a threshold — not necessarily a hard crash) despite the
> filter running every step. Safety-filtered and competent are two separate
> properties. Don't let a paper (including ours, before we fixed the framing)
> claim the first proves the second.

**Visual assets needed:**
- **Infographic (2-panel diagram)**: Panel A "What CBF guarantees" (velocity bounded away from obstacle) vs Panel B "What it doesn't" (policy can still be incompetent/stationary) — a clean conceptual diagram, not a screenshot.
- **Screenshot**: the relevant paragraph from `paper/main.tex`'s safety-filter subsection, to show this is a documented, not hand-wavy, finding.
- **Optional short clip**: a robot in Gazebo backing away cleanly from a wall (the CBF working correctly) — good positive-example B-roll.

---

## Post 5 — "Same weights, 3 robots or 10"

**Draft copy:**

> Most multi-agent RL policies are trained for a fixed team size and break
> the moment you add or remove a robot — different input dimensions, retrain
> from scratch.
>
> Ours doesn't. The actor processes each neighbor's relative position through
> a small shared MLP, then mean-pools the results (masking out any padding
> for absent neighbors). That mean is invariant to how many neighbors there
> are and what order they come in.
>
> Practically: we trained on 3 robots and ran the identical network, zero
> code changes, zero retraining, at 5 and at 10 robots. It just works —
> verified with a dedicated unit test that feeds the model observations for
> both swarm sizes and checks the output shapes and value function behave
> correctly.
>
> This is the deep-sets / permutation-invariance trick applied to swarm
> robotics: scale isn't a training-time decision, it's a property of the
> architecture.

**Visual assets needed:**
- **Diagram**: a simple neural-net architecture sketch — N neighbor inputs → shared MLP (drawn once, arrows fanning into it from multiple neighbor boxes) → mean-pool → concatenated with ego features → policy output. This is the single most useful visual for this post.
- **Screenshot**: terminal output of `test_gnn_scale_invariance` passing, showing the N=5 and N=10 assertions.
- **Optional**: side-by-side Gazebo/RViz screenshots — one with 3 robots, one with a larger swarm (if you run a quick demo) — visually reinforces "same policy, different swarm size."

---

## Post 6 — "From 87.5% to 100%: adding A* to a reactive heuristic"

**Draft copy:**

> Our frontier-exploration heuristic (drive to the nearest unexplored cell,
> react to obstacles as you see them) plateaued hard at 87.5% area coverage
> no matter how long we ran it. The last 12% just never got covered.
>
> Root cause: it's purely reactive. If the nearest unexplored cell happened
> to be behind a wall the robot couldn't see a straight line to, it would
> aim straight at it, get blocked, and never route around. Same unreachable
> cells, forever.
>
> First fix attempt made it worse — 81.9%, down from 87.5%. Turns out
> filtering target selection by "is this reachable" and recomputing every
> step caused the robot to flip-flop between near-equidistant targets instead
> of committing to a direction. Lesson: don't touch what's stable if the
> actual bug is somewhere else.
>
> Real fix: keep the original, stable target selection. Only add A* pathing
> as a fallback — check if the direct line to the target is blocked, and only
> then compute a route around the obstacle, with a small safety margin so the
> path doesn't hug walls. Coverage: 87.5% → 90.4%. Then on a second, more open
> world: 99.0%.
>
> Two lessons in one bug: the fix that seems obviously correct isn't always,
> and adding complexity only where the evidence says it's needed beats
> rewriting the whole thing.

**Visual assets needed:**
- **Line chart / infographic**: coverage % over time for 3 curves on one chart — "no A*" (plateaus ~87.5%), "broken A* attempt" (worse, ~82%), "fixed A*" (climbs to ~90-99%). This is the single strongest visual for this post — makes the whole story legible in one image.
- **Screen recording (60-90s, sped up)**: RViz `/map` panel filling in over a full run, watch it visibly slow down/plateau then (in the fixed version) keep climbing.
- **Screenshot**: before/after of the relevant code diff (target-selection logic, ~15 lines) for a technical-audience cut of the post.

---

## Post 7 — "Benchmarking a swarm under sensor noise and robot failure"

**Draft copy:**

> A policy that works in perfect conditions tells you almost nothing about
> whether it'll work in the real world. So our benchmark harness doesn't just
> run one clean scenario — it runs every method under three regimes:
> nominal, Gaussian noise injected into every LiDAR reading, and a robot
> going offline mid-episode.
>
> Same metrics across all three: area coverage rate, overlap redundancy (are
> robots wastefully re-covering the same ground), swarm path length, and
> collision rate.
>
> What we found: our RL policy's performance barely changed across the three
> regimes — but not because it was robust. It was because it had already
> failed nominally (parking near its spawn point), so noise and failure
> couldn't make it meaningfully worse. That's an important distinction a
> single-scenario benchmark would have completely hidden.
>
> A benchmark that only reports the happy path isn't measuring robustness,
> it's measuring optimism.

**Visual assets needed:**
- **Table screenshot**: the actual benchmark summary table (nominal/noise/failure rows) from a real run's terminal output.
- **Infographic**: 3 small icons/panels representing the regimes (clean sensor icon / noisy static icon / robot-with-X icon for failure) as a header graphic.
- **Optional clip**: the `robot_killer` node visibly disabling a robot mid-run in Gazebo, other two robots continuing — good concrete "here's what failure injection looks like" B-roll.

---

## Post 8 — "A decentralized map-merging protocol in under 20 lines"

**Draft copy:**

> No central server. No robot with a "master map." Just three robots, each
> keeping its own private belief of what it's explored, occasionally close
> enough to talk to each other.
>
> The merge rule when two robots come within radio range (3 meters, in our
> case): OR their visited-cell grids together. That's it. Bitwise OR. Both
> robots walk away knowing the union of what either of them has seen.
>
> It's not fancy — no negotiation protocol, no conflict resolution, no
> centralized coordinator to go down. It's also exactly the property you
> want for swarm robustness: any subset of robots that can talk to each
> other converges on a shared picture, and the whole system degrades
> gracefully if robots lose contact instead of breaking.
>
> Sometimes the simplest primitive is the right one — full source below,
> it really is about 15 lines.

**Visual assets needed:**
- **Code screenshot**: the actual consensus loop (the `d_comm` distance check + bitwise-OR merge) — needs to look genuinely short to land the "under 20 lines" claim.
- **Diagram**: two robot icons with a dashed circle (communication radius) around each, overlapping — arrows showing the OR-merge when circles overlap. Simple network/graph-style diagram.
- **Screen recording**: RViz `/map` showing two robots' explored regions visibly merging into one contiguous patch the moment they get close — this is a great, satisfying visual if you can capture the exact moment.

---

## Post 9 — "What a real MAPPO training run looks like when it's not working"

**Draft copy:**

> Everyone shows the reward curve going up and to the right. Here's what one
> looks like when it isn't.
>
> [raw log excerpt — see visual spec]
>
> Policy entropy: started reasonable, dropped to -0.53 by the last iteration.
> Episode returns: swung from -227 to +3 within the same batch of 3-4
> episodes. That's not noise, that's a policy that hasn't seen enough data to
> have a consistent behavior yet, being asked to converge anyway.
>
> If you're learning RL and every tutorial you've seen has a clean upward
> curve, it's worth seeing what the messy, real, "this isn't working and here's
> exactly why" version looks like — because that's the version you'll
> actually encounter first.

**Visual assets needed:**
- **Screenshot**: the raw, unedited RLlib training log for 3-4 consecutive iterations, cropped to show `entropy`, `episode_reward_mean`, `episode_reward_min/max` — authenticity is the entire point of this post, don't clean it up.
- **Simple line/scatter plot**: entropy value per iteration across the training run — one line, clearly trending down/collapsing.
- **Optional**: side-by-side of the "before" log excerpt vs. an "after" excerpt once the retrained run's numbers are in (entropy staying positive) — a strong before/after if timing allows.

---

## Post 10 — "Open-sourcing MARS"

**Draft copy:**

> We built a full ROS2 + Gazebo + Ray RLlib framework for multi-robot swarm
> coverage: MAPPO with a centralized critic, a permutation-invariant GNN
> policy that scales to any swarm size, a Control Barrier Function safety
> filter, decentralized map consensus, and a benchmark harness that tests all
> of it under noise and failure — plus two real-world-verified simulation
> case studies (a furnished cafe capping out around 90% coverage, an open
> warehouse reaching 99-100%).
>
> Everything's in the repo: the training code, the environment, the safety
> filter, the benchmarks, the paper (including the honest negative results,
> not just the wins), and the debugging notes from every bug we hit along the
> way.
>
> If you're building multi-robot RL and want a real, warts-and-all reference
> point instead of a polished toy example — here it is. [repo link]

**Visual assets needed:**
- **GitHub repo screenshot**: the repo's root file listing (README visible, clean folder structure: `src/`, `paper/`, `scratch/`, etc.) — the classic "here's the repo" screenshot. Take this once everything is merged to `main` and the README is current.
- **Screenshot**: the README's architecture diagram (the mermaid graph already in `README.md`) rendered — GitHub renders mermaid natively, so just screenshot the rendered version on the repo page.
- **Optional montage**: 2x2 grid of stills — Gazebo GUI, RViz map, a benchmark plot, the paper's title page — a single "everything in one image" summary graphic for the announcement post.

---

## 2-3 research paper topics (extensions beyond the current paper)

1. **Sample-efficient MAPPO for physics-in-the-loop multi-robot training.**
   Characterize why a tiny env-step budget (thousands, not millions) collapses
   policy entropy in real-time-physics MARL (no fast simulator available), and
   empirically compare mitigations (GAE lambda, entropy scheduling, reward
   rebalancing, behavior-cloning warm starts) — positioned for a systems/
   robotics-RL venue where "no fast simulator" is a real constraint, not a
   simplifying assumption. Directly extends this project's diagnosed failure
   mode and fix (see `paper/RUN_PLAN.md` and the training-fix plan).

2. **Decoupling safety guarantees from policy competence in multi-robot CBF
   filters.** A more formal treatment of this project's safety-filter honesty
   finding: characterize exactly what a dual-lookahead CBF/ACAS filter
   guarantees under a degenerate or undertrained policy, using both the MAPPO
   and frontier-heuristic runs as empirical evidence of the "filtered-safe but
   not competent" regime.

3. **Generalization of permutation-invariant coverage policies across
   structurally different environments.** Uses the cafe (dense furniture) vs.
   warehouse (open floor + isolated pillars) case studies as a controlled
   comparison of how environment structure affects reachable coverage
   ceilings, extended toward sim-to-real hardware transfer.

---

## ⚠️ Blocked: new research paper referencing papers.ssrn.com/abstract=7137361

SSRN blocks automated access to that URL (bot-protection wall — tried direct
fetch and a text-extraction proxy, both hit a CAPTCHA/security challenge
page, no title or abstract text was retrievable). I'm not going to cite or
build a paper around a source I haven't actually read — that risks a
fabricated reference, which is a real integrity problem in an academic
document, not a minor one.

To proceed, paste one of the following and I'll draft the paper against it:
- The paper's title + authors (I can then try to locate a non-paywalled
  version, e.g. an arXiv/preprint mirror, or a publisher page that isn't
  behind the same wall), or
- The abstract text directly, or
- A PDF/text export of the paper itself.

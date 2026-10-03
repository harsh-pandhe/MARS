"""Analyze the Paper 2 A/B runs (original RLlib defaults vs corrected config).

Pre-specified before any data were collected:
  Primary:   entropy at iteration 15, original < corrected.
  Secondary: entropy change (iter 15 - iter 1), original more negative.
  Descriptive only: within-iteration return spread (scale differs between
             arms: different collision penalty and episode length).
Unit of independence: the training run (rep), n per arm = number of reps.
"""
import glob, json, os, sys, itertools, statistics as st

D = sys.argv[1] if len(sys.argv) > 1 else "checkpoints/paper2_ab"
runs = {}
for f in sorted(glob.glob(os.path.join(D, "*_rep*.jsonl"))):
    arm, rep = os.path.basename(f)[:-6].split("_rep")
    recs = [json.loads(l) for l in open(f) if l.strip()]
    if len(recs) >= 15:                      # complete runs only
        runs.setdefault(arm, {})[int(rep)] = recs

def series(recs, key): return [r[key] for r in recs]
summary = {"per_run": {}, "per_arm": {}}
for arm, reps in runs.items():
    for rep, recs in reps.items():
        ent = series(recs, "entropy")
        spread = [r["episode_reward_max"] - r["episode_reward_min"] for r in recs if r["episodes_this_iter"] > 1]
        summary["per_run"][f"{arm}_rep{rep}"] = {
            "entropy_iter1": ent[0], "entropy_iter15": ent[14], "entropy_delta": ent[14] - ent[0],
            "entropy_min": min(ent), "entropy_mean": st.mean(ent),
            "mean_return": st.mean(series(recs, "episode_reward_mean")),
            "mean_within_iter_return_spread": st.mean(spread) if spread else None,
            "mean_episodes_per_iter": st.mean(series(recs, "episodes_this_iter")),
            "env_steps_total": recs[-1]["num_env_steps_sampled"],
            "vf_explained_var_final": recs[14]["vf_explained_var"],
        }
for arm, reps in runs.items():
    pr = [summary["per_run"][f"{arm}_rep{r}"] for r in reps]
    def agg(k):
        v = [p[k] for p in pr if p[k] is not None]
        return {"values": [round(x, 3) for x in v], "mean": round(st.mean(v), 3),
                "sd": round(st.stdev(v), 3) if len(v) > 1 else None}
    summary["per_arm"][arm] = {"n": len(pr), **{k: agg(k) for k in
        ["entropy_iter1", "entropy_iter15", "entropy_delta", "entropy_min", "mean_return",
         "mean_within_iter_return_spread", "mean_episodes_per_iter", "env_steps_total"]}}

def perm_p(a, b):
    """Exact two-sided permutation test on the difference of means."""
    obs = abs(st.mean(a) - st.mean(b)); pool = a + b; n = len(a); cnt = tot = 0
    for idx in itertools.combinations(range(len(pool)), n):
        ga = [pool[i] for i in idx]; gb = [pool[i] for i in range(len(pool)) if i not in idx]
        tot += 1; cnt += abs(st.mean(ga) - st.mean(gb)) >= obs - 1e-12
    return cnt / tot
if {"old", "corrected"} <= set(runs):
    tests = {}
    for k in ["entropy_iter15", "entropy_delta", "mean_within_iter_return_spread"]:
        a = [summary["per_run"][f"old_rep{r}"][k] for r in runs["old"]]
        b = [summary["per_run"][f"corrected_rep{r}"][k] for r in runs["corrected"]]
        tests[k] = {"old_mean": round(st.mean(a), 3), "corrected_mean": round(st.mean(b), 3),
                    "diff_old_minus_corrected": round(st.mean(a) - st.mean(b), 3),
                    "exact_permutation_p_two_sided": round(perm_p(a, b), 4),
                    "min_attainable_p": round(2 / len(list(itertools.combinations(range(len(a)+len(b)), len(a)))), 4)}
    summary["tests"] = tests
json.dump(summary, open(os.path.join(D, "summary.json"), "w"), indent=2)
print(json.dumps(summary["per_arm"], indent=1)); print(json.dumps(summary.get("tests"), indent=1))

try:                                         # figure
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    col = {"old": "tab:red", "corrected": "tab:blue"}
    lab = {"old": "original defaults", "corrected": "corrected config"}
    for arm, reps in runs.items():
        for j, (rep, recs) in enumerate(sorted(reps.items())):
            x = [r["iteration"] for r in recs]
            ax[0].plot(x, series(recs, "entropy"), color=col[arm], alpha=.7, label=lab[arm] if j == 0 else None)
            ax[1].plot(x, series(recs, "episode_reward_mean"), color=col[arm], alpha=.7, label=lab[arm] if j == 0 else None)
    ax[0].set_xlabel("training iteration"); ax[0].set_ylabel("policy entropy"); ax[0].legend()
    ax[1].set_xlabel("training iteration"); ax[1].set_ylabel("mean episode return"); ax[1].legend()
    fig.tight_layout(); fig.savefig(os.path.join(D, "entropy_returns.png"), dpi=200)
except Exception as e:
    print("figure skipped:", e)

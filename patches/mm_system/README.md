# mm_system patches (backup)

Our changes to the real-robot code `nycu-acm/mm_system`, which the sim runs from
`references/upstream/mm_system` (kept out of this repo). Only the diffs live here, as a
backup in case that local clone is deleted or re-cloned.

| Patch | Branch on GitHub | Base | Status |
|---|---|---|---|
| `0001-fix-place-hold-grasp-width-while-carrying-to-the-pla.patch` | `fix/place-hold-grip` (pushed 2026-09-23, not merged) | `main` a99e3d7 | sim-verified, not yet tested on the real arm |

Restore into a fresh clone:

```bash
cd references/upstream/mm_system
git checkout -b fix/place-hold-grip a99e3d7
git am ../../../patches/mm_system/*.patch
```

Or just `git fetch origin && git checkout fix/place-hold-grip` while the branch exists on GitHub.
When new commits go onto the branch, regenerate: `git format-patch -o <this dir> a99e3d7..fix/place-hold-grip`.
Details: docs/dev_guide_paper_core_and_dashboard_plan.md §5.4.5.

# Triage labels

`/triage` and the PR skills use role names. This file maps each role to the label string in this repository's tracker, for an issue and for a PR.

| Role              | On an issue       | On a PR                                        |
| ----------------- | ----------------- | ---------------------------------------------- |
| `bug`             | `type:bug`        | none, the conventional-commit title carries it |
| `enhancement`     | `type:feature`    | none, the conventional-commit title carries it |
| `needs-triage`    | `needs-triage`    | no verdict label, non-draft                    |
| `needs-info`      | `needs-info`      | `recommend-revise`                             |
| `ready-for-agent` | `ready-for-agent` | no verdict label, non-draft                    |
| `ready-for-human` | `ready-for-human` | `recommend-merge`                              |
| `wontfix`         | `wontfix`         | `recommend-close`                              |

Use `type:task` instead of `type:feature` when an `enhancement` is maintenance, docs, or refactor work.

Rules:

- A triaged issue carries exactly one `type:` label and one triage state. An unlabeled issue is untriaged; `needs-triage` means under evaluation.
- A non-draft PR with no verdict is the agent's to finalize. The draft flag is the "still being built" state; there is no label for it.
- The agent emits at most one `recommend-*` verdict per head. `recommend-revise` hands the PR back to its author; the other three hand it to the maintainer.
- Verdicts are recommendations. The agent never merges and never closes. Never rename `recommend-merge` to `ready-to-merge`: merge bots watch that string.
- `maintainer-approved` is set only on explicit maintainer direction and may coexist with a `recommend-*` label.
- A new push makes any verdict stale. The authority that set it clears and renews it.

# T8 held-out-style agent evaluation

Run the eight real-agent tasks with `ANTHROPIC_API_KEY` set:

```sh
npm run run:agent-evaluation -- claude-sonnet-4-5
```

The model argument is optional and can also be supplied as `ANTHROPIC_MODEL`. The report path is already configured by the npm script. Each task uses a fresh message history and only the six MCP simulator tools. Reports and full transcripts are written beneath `artifacts/agent-evaluation/`.

The benchmark contains 28 tasks repeated three times (84 paid model trials): 6 valid, 12 invalid-request variants, 6 ambiguous, and 4 provenance tasks. The 12 invalid tasks implement both required variants for each of the six validator branches. Tasks were authored alongside the interface rather than by an independent held-out author; results are single-model and include no ablation, so this remains limited bonus evidence.

Before spending API calls, run `npm run smoke:agent-evaluation`. This exercises all six validator branches, every registered vehicle through construct/run/read-back, discovery, metric persistence, and checksum separation without calling Anthropic. The paid runner checkpoints after every completed trial. Re-running the same model command resumes a matching partial benchmark instead of paying for completed trials again.

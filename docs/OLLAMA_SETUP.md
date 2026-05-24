# Ollama + gemma3 Setup for PET

This guide explains how to run PET with a local LLM that observes touch events
and chooses movement responses in real time.

---

## What is Ollama?

**Ollama** is a runtime that lets you run open-source language models locally on
your machine — no internet connection, no API key, no cloud service. It handles
model downloading, GPU/CPU scheduling, and exposes a simple HTTP API on
`localhost:11434`.

**gemma3:4b** is Google's Gemma 3 model at the 4-billion parameter size. At 4B
parameters it is fast on a modern laptop (2–5 seconds per response on CPU,
under 1 second with Apple Silicon GPU). It fits in about 4 GB of RAM and
produces reliable JSON output when asked.

---

## How the Integration Works

```
Touch sensors (30 Hz)
       ↓
StrokeDetector / HoldDetector / ContactClassifier
       ↓ (when touch changes, ~once per second)
TouchFormatter → short natural-language description
       ↓
OllamaClient → POST localhost:11434/api/chat
       ↓
gemma3:4b reads:
  • robot_context.md  — what the robot is, what movements are available
  • behavior_guide.md — YOUR rules for how touch maps to movement
       ↓
Returns: {"movement": "ripple", "intensity": 0.55, "speed": 0.3}
       ↓
OllamaControlScheme generates ServoCommands every tick
```

The LLM runs in a background thread so the 30 Hz control loop is never blocked.
Between LLM responses, the robot keeps doing the last commanded movement.

---

## Installation

### 1. Install Ollama

**macOS (Homebrew):**
```bash
brew install ollama
```

**macOS (direct download):**
Download the installer from [ollama.com](https://ollama.com) and drag to Applications.

**Linux:**
```bash
curl -fsSL https://ollama.com/install.sh | sh
```

### 2. Pull the gemma3 model

```bash
ollama pull gemma3:4b
```

This downloads ~2.5 GB. It only needs to happen once.

### 3. Start the Ollama server

```bash
ollama serve
```

Leave this running in a terminal. On macOS, if you installed the desktop app it
starts automatically when you launch it. You can verify it's running:

```bash
curl http://localhost:11434
# Should print: Ollama is running
```

---

## Running PET with Ollama

```bash
# Mock backend (no robot hardware required)
petctl run --control ollama --backend mock --mode sine

# Real robot
petctl run --control ollama --backend robot
```

On startup you'll see a log line confirming the LLM connection:
```
[Ollama] connected, model=gemma3:4b, 7 active servos.
```

If Ollama isn't running you'll see:
```
[Ollama] server not reachable at http://localhost:11434/api/chat — start Ollama with 'ollama serve' then restart PET.
```

The robot will still run (in `freeze` mode), but won't respond to touch until
Ollama is reachable.

---

## Configuring Behavior

All behavior configuration lives in two markdown files in `petctl/prompts/`:

### `robot_context.md` — technical reference (read-only)

Describes the robot's anatomy, the available movement vocabulary, and the
required JSON response format. You generally don't need to edit this unless
you're adding new movement types to the code.

### `behavior_guide.md` — YOUR personality file (edit freely)

This is the main file you'll use to shape how PET responds to touch. It has
three sections:

**`## Character`** — Describe PET's emotional personality in plain English.
This becomes part of the LLM's system prompt, so write it like you're briefing
someone on who the robot "is."

```markdown
## Character
PET is a curious, cautious creature. It warms up to gentle touch but
freezes when grabbed suddenly.
```

**`## Touch → Response Rules`** — Numbered rules that map touch events to
movements. The LLM reads these and tries to follow them. Write them as
clear if/then statements:

```markdown
3. **Stroke from head to tail**: choose `sway`, intensity 0.4, speed 0.25.
```

**`## General Principles`** — Catch-all guidance for cases not covered by
specific rules. Good for intensity and speed preferences.

**Editing workflow:**
1. Edit `behavior_guide.md` in your text editor
2. Restart `petctl run --control ollama`
3. Touch the robot and observe the new behavior

Changes take effect immediately on restart — the file is loaded fresh each run.

---

## Tips for Writing Good Behavior Rules

**Be concrete.** Instead of "move gently," write "choose `sway`, intensity 0.3."
The model is 4B parameters — it follows explicit instructions better than
abstract descriptions.

**Use the exact movement names.** The available movements are listed in
`robot_context.md`. If you write an unknown movement name, the LLM response
will be ignored and a warning logged.

**Keep rules short.** Each rule should fit on one or two lines. Long explanations
are more likely to confuse a small model than help it.

**Avoid contradictions.** If rule 3 and rule 6 apply to the same touch event,
the model will pick arbitrarily. Order your rules from most specific to least
specific, and note "the first matching rule wins."

**Test with mock backend.** `--backend mock --mode sine` creates simulated sensor
activity. You can watch what the LLM decides and tune the rules without using
the real robot.

---

## Logging and Debugging

Enable verbose logging to see LLM calls:

```bash
petctl run --control ollama --backend mock 2>&1 | grep -E "\[Ollama\]"
```

Each touch event you'll see:
```
[Ollama] → ripple (intensity=0.55, speed=0.30)
```

If the LLM returns an invalid movement name:
```
[Ollama] unknown movement 'wave' — ignoring. Valid: curl_left, curl_right, ...
```

If a response can't be parsed:
```
[Ollama] could not parse response: ...
```

---

## Troubleshooting

**"server not reachable"** — Ollama isn't running. Run `ollama serve`.

**Very slow responses (>10s)** — The model is running on CPU only. On Apple
Silicon, Ollama uses the GPU Metal backend automatically. On Intel Mac or Linux,
consider a smaller model: `ollama pull gemma3:2b` and pass `--model gemma3:2b`
(this requires a code change in `OllamaControlScheme.__init__` until a CLI flag
is added).

**Robot moves erratically** — The LLM may be choosing high-intensity movements.
Add to `behavior_guide.md`:
```markdown
## General Principles
- Keep intensity below 0.5 unless contact is very firm.
```

**LLM ignores the rules** — Rephrase rules more explicitly. State the exact
movement name and numeric values rather than adjectives like "gently."

**"unknown movement" warnings** — The LLM hallucinated a movement name. Add to
`robot_context.md` under the movements table: a note that all valid names are
listed there and no others exist.

---

## Available Movement Reference

| Name        | Description |
|-------------|-------------|
| `freeze`    | Hold perfectly still at neutral position. |
| `home`      | Smoothly return all joints to neutral. |
| `pulse`     | All joints flex and release in unison — whole-body breath. |
| `ripple`    | A wave travels from head to tail. |
| `sway`      | Slow, gentle side-to-side oscillation. |
| `curl_right`| Body curves progressively to the right. |
| `curl_left` | Body curves progressively to the left. |
| `twitch`    | Small organic per-joint jitter. |

Both `intensity` (0–1) and `speed` (0–1) are available for all movements
except `freeze` and `home` (which always return to neutral).

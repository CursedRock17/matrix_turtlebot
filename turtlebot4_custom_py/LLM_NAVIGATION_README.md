# LLM-Based Navigation for TurtleBot4

Natural-language navigation: publish "Bring Harold this book" on a topic and
the robot drives to Harold's Room. A small local LLM (Llama-3.2-3B-Instruct,
4-bit GGUF, run by llama.cpp on the laptop CPU) extracts the destination from
the command; no internet, no API keys — it works on BaleNet.

## The pieces

1. **llm_location_mapper.py** — the core library. Builds a prompt listing
   the known location names, asks the model which one the command means, and
   fuzzy-matches the answer back to a known name. Answers `UNKNOWN` (and the
   node refuses to move) when no destination can be inferred.
2. **llm_navigation_node.py** (`ros2 run turtlebot4_custom_py llm_navigation`)
   — subscribes to `navigation_command` and navigates to whatever the mapper
   extracts.
3. **patrol_with_llm_node.py** (`... patrol_with_llm`) — the patrol loop with
   LLM rerouting: patrols the map's waypoints, but a command on
   `navigation_command` diverts it, then the patrol resumes.
4. **location_mapper_eval.py** (`... location_mapper_eval`) — scores the
   mapper against a fixed prompt set, including must-abstain cases. Run it on
   the laptop after changing the model, the prompt, or the location names —
   no robot needed.
5. **locations_map.txt** — fallback location list (`Name: x, y, DIRECTION`
   per line) used only when no map is active (desk testing).

## Where the locations come from

When a map is loaded (i.e. the localization launch is running), the nodes
read the location names from the active map's `maps/<name>.locations.yaml` —
the same surveyed poses navigation uses — so the LLM can only name places
that exist on the map it is driving on. `dock` is always available and is
what charging synonyms ("go recharge", "head home") map to.

Only the standalone desk test (`location_mapper`, `location_mapper_eval`)
falls back to `locations_map.txt`.

## Setup

Needs `llama-cpp-python` and the GGUF weights on the machine running the
node (our demo laptop has both):

```bash
pip install -r requirements.txt   # llama-cpp-python
./download_model.sh               # fetches the ~2GB GGUF from HuggingFace
```

Run the download once while on a network with internet; it puts
`Llama-3.2-3B-Instruct-Q4_K_M.gguf` next to `llm_location_mapper.py` in the
source tree. The model is not git-tracked.

## Usage

With the robot stack up (see [docs/navigate_to_a_goal.md](../docs/navigate_to_a_goal.md)):

```bash
ros2 run turtlebot4_custom_py llm_navigation
```

Then from another sourced terminal:

```bash
ros2 topic pub --once /navigation_command std_msgs/msg/String "data: 'Bring Harold this book'"
ros2 topic pub --once /navigation_command std_msgs/msg/String "data: 'Go back to the charging station'"
```

Parameters (both nodes): `model_path` (override the GGUF location),
`n_threads` (CPU threads for inference, default 4), and for `llm_navigation`
also `command_topic` (default `navigation_command`).

For a namespaced robot, namespace the whole node with the standard remap
(this moves the navigator *and* the command topic together):

```bash
ros2 run turtlebot4_custom_py llm_navigation --ros-args -r __ns:=/matrix_turtlebot1
```

## Testing without a robot

```bash
ros2 run turtlebot4_custom_py location_mapper        # a few demo prompts
ros2 run turtlebot4_custom_py location_mapper_eval   # scored eval, exit code = failures
```

Both use `locations_map.txt`, so they run with no map, no robot, and no ROS
graph — just the model file. The eval's abstain cases are the important
ones: a wrong-but-confident answer sends a real robot somewhere real.

## Adding new locations

Add them to the active map's `maps/<name>.locations.yaml` (survey with
`ros2 run turtlebot4_custom_py survey_locations` — see
[README.md](./README.md)). Prefer real room/person names: the LLM maps
"take this to Jamison" onto a location literally named after Jamison far
more reliably than onto `spot_b`.

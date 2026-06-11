# LLM Integration
-------------------
### Overview
For this task, we wanted to make our Turtlebot's a bit smarter. We needed to give them access to 
LLMs in order to interpret actions more generally (find Harold, show Bill the pool
stick, take this book to Jamison). We needed to poll an LLM without paying (OpenAI and Claude) both
cost money and force us to make API calls constantly — and BaleNet has no internet anyway. So the
model runs locally on the laptop CPU: Llama-3.2-3B-Instruct as a 4-bit GGUF through llama.cpp.

### Prerequisites
  - `llama-cpp-python` (`pip install -r turtlebot4_custom_py/requirements.txt`)
  - The GGUF weights, fetched once on a network with internet:
    `turtlebot4_custom_py/download_model.sh` (~2GB, not git-tracked)

### Where everything lives
The full guide — nodes, topics, parameters, testing without a robot, adding
locations — is [turtlebot4_custom_py/LLM_NAVIGATION_README.md](../turtlebot4_custom_py/LLM_NAVIGATION_README.md).

Quick reference:
  - `ros2 run turtlebot4_custom_py llm_navigation` — natural language in, navigation out
  - `ros2 run turtlebot4_custom_py patrol_with_llm` — patrol loop with LLM rerouting
  - `ros2 run turtlebot4_custom_py location_mapper` — desk test, no robot needed
  - `ros2 run turtlebot4_custom_py location_mapper_eval` — scored prompt eval (run after
    changing the model, prompt, or location names; the abstain cases are the safety net)

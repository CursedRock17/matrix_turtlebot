# LLM Integration
-------------------
### Overview
For this task, we wanted to make our Turtlebot's a bit smarter. We needed to give them access to 
internet based LLMs/VLMs in order to interpret actions more generally (find Harold, show Bill the pool
stick, take this book to Jamison). We needed to poll this LLM without paying (OpenAI and Claude) both
cost money and force us to make API calls constantly. Opted for Huggingface as an open source friendly
backend.


### Prerequistes
  - Utilizes llama.cpp, finding installation - [here](https://github.com/ggml-org/llama.cpp)
  - Wants a GGUF version of a model, specifically Llama-3.2-3B-Instruct-Q4_K_M.gguf

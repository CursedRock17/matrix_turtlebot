#!/bin/bash
# Downloads the Llama 3.2 3B Instruct GGUF model (Q4_K_M quantization, ~2GB)
# into the source directory for use with llm_location_mapper.py

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODEL_DIR="$SCRIPT_DIR/turtlebot4_custom_py"
MODEL_FILE="Llama-3.2-3B-Instruct-Q4_K_M.gguf"
MODEL_PATH="$MODEL_DIR/$MODEL_FILE"

if [ -f "$MODEL_PATH" ]; then
    echo "Model already exists at $MODEL_PATH"
    exit 0
fi

echo "Downloading $MODEL_FILE (~2GB)..."
echo "Destination: $MODEL_PATH"

# Use huggingface-cli if available, otherwise fall back to wget
if command -v huggingface-cli &> /dev/null; then
    huggingface-cli download \
        bartowski/Llama-3.2-3B-Instruct-GGUF \
        "$MODEL_FILE" \
        --local-dir "$MODEL_DIR" \
        --local-dir-use-symlinks False
else
    echo "huggingface-cli not found, using wget..."
    wget -O "$MODEL_PATH" \
        "https://huggingface.co/bartowski/Llama-3.2-3B-Instruct-GGUF/resolve/main/$MODEL_FILE"
fi

echo "Download complete: $MODEL_PATH"

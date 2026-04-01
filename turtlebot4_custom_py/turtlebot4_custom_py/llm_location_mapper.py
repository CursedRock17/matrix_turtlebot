import os
from llama_cpp import Llama
from typing import Optional, Dict, Tuple
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions

# Default GGUF model filename expected in this directory
DEFAULT_MODEL_FILENAME = "Llama-3.2-3B-Instruct-Q4_K_M.gguf"


class LLMLocationMapper:
    """
    Uses a local GGUF model (via llama.cpp) to map natural language commands
    to specific locations.
    """

    def __init__(self, model_path: str = None,
                 locations_file: str = None,
                 n_threads: int = 4):
        """
        Initialize the LLM location mapper.

        Args:
            model_path: Path to a GGUF model file. Defaults to
                        Llama-3.2-3B-Instruct-Q4_K_M.gguf in this directory.
            locations_file: Path to the locations_map.txt file
            n_threads: Number of CPU threads for inference
        """
        print("Initializing LLM Location Mapper...")

        current_dir = os.path.dirname(os.path.abspath(__file__))

        # Load locations from file
        if locations_file is None:
            locations_file = os.path.join("/home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws/turtlebot4_custom_py/turtlebot4_custom_py", "locations_map.txt")

        self.locations = self._load_locations(locations_file)
        print(f"Loaded {len(self.locations)} locations from {locations_file}")

        # Resolve model path
        if model_path is None:
            model_path = os.path.join("/home/cursedrock17/Documents/Electrical/Matrix_Lab/turtlebot4_ws/models/", DEFAULT_MODEL_FILENAME)

        if not os.path.isfile(model_path):
            raise FileNotFoundError(
                f"GGUF model not found at {model_path}\n"
                f"Download it with: ./download_model.sh"
            )

        print(f"Loading model: {model_path}")
        self.model = Llama(
            model_path=model_path,
            n_ctx=512,
            n_threads=n_threads,
            verbose=False,
        )

        print("Model loaded successfully!")

    def _load_locations(self, filepath: str) -> Dict[str, Tuple[float, float, str]]:
        """
        Load locations from the locations_map.txt file.

        Format: Location Name: x, y, DIRECTION

        Returns:
            Dictionary mapping location names to (x, y, direction) tuples
        """
        locations = {}

        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue

                    # Parse: "Location Name: x, y, DIRECTION"
                    if ':' in line:
                        location_name, coords = line.split(':', 1)
                        location_name = location_name.strip()

                        parts = [p.strip() for p in coords.split(',')]
                        if len(parts) == 3:
                            x = float(parts[0])
                            y = float(parts[1])
                            direction = parts[2].upper()
                            locations[location_name] = (x, y, direction)

        except FileNotFoundError:
            print(f"Warning: Locations file not found at {filepath}")
            print("Using empty locations dictionary.")

        return locations

    def _get_direction_enum(self, direction_str: str):
        """Convert direction string to TurtleBot4Directions enum."""
        direction_map = {
            'NORTH': TurtleBot4Directions.NORTH,
            'SOUTH': TurtleBot4Directions.SOUTH,
            'EAST': TurtleBot4Directions.EAST,
            'WEST': TurtleBot4Directions.WEST,
            'NORTH_EAST': TurtleBot4Directions.NORTH_EAST,
            'NORTH_WEST': TurtleBot4Directions.NORTH_WEST,
            'SOUTH_EAST': TurtleBot4Directions.SOUTH_EAST,
            'SOUTH_WEST': TurtleBot4Directions.SOUTH_WEST,
        }
        return direction_map.get(direction_str, TurtleBot4Directions.NORTH)

    def _build_messages(self, command: str) -> list:
        """
        Build chat messages for the LLM to extract a location from a command.

        Args:
            command: Natural language command from user

        Returns:
            List of message dicts for chat completion
        """
        locations_list = "\n".join([f"- {loc}" for loc in self.locations.keys()])

        system_msg = (
            "You are a robot navigation assistant. "
            "Extract the destination location from the user's command.\n\n"
            f"Available locations:\n{locations_list}\n\n"
            "Respond with ONLY the exact location name from the list above. "
            "If no location is mentioned or unclear, respond with \"UNKNOWN\"."
        )

        return [
            {"role": "system", "content": system_msg},
            {"role": "user", "content": command},
        ]

    def extract_location(self, command: str) -> Optional[Tuple[float, float, object]]:
        """
        Extract location from natural language command using LLM.

        Args:
            command: Natural language command (e.g., "Bring Harold this book")

        Returns:
            Tuple of (x, y, direction_enum) or None if location not found
        """
        print(f"\nProcessing command: '{command}'")

        messages = self._build_messages(command)

        output = self.model.create_chat_completion(
            messages=messages,
            max_tokens=50,
            temperature=0.1,
        )

        location_text = output["choices"][0]["message"]["content"].strip()
        # Take only the first line in case the model generates extra text
        location_text = location_text.split('\n')[0].strip()

        print(f"LLM extracted location: '{location_text}'")

        # Match against known locations (case-insensitive)
        for loc_name, (x, y, direction_str) in self.locations.items():
            if loc_name.lower() in location_text.lower() or \
               location_text.lower() in loc_name.lower():
                direction_enum = self._get_direction_enum(direction_str)
                print(f"Matched to: {loc_name} at ({x}, {y}, {direction_str})")
                return (x, y, direction_enum)

        print("No matching location found")
        return None

    def get_location_coords(self, location_name: str) -> Optional[Tuple[float, float, object]]:
        """
        Get coordinates for a specific location name (direct lookup, no LLM).

        Args:
            location_name: Exact location name

        Returns:
            Tuple of (x, y, direction_enum) or None if not found
        """
        if location_name in self.locations:
            x, y, direction_str = self.locations[location_name]
            direction_enum = self._get_direction_enum(direction_str)
            return (x, y, direction_enum)
        return None

    def get_all_locations(self) -> Dict[str, Tuple[float, float, str]]:
        """Return all loaded locations."""
        return self.locations.copy()


# Example usage and testing
if __name__ == '__main__':
    # Initialize mapper
    mapper = LLMLocationMapper()

    # Test commands
    test_commands = [
        "Bring Harold this book",
        "Go to John's room",
        "Navigate to the dock",
        "Take this to Harold",
        "Go to John",
    ]

    print("\n" + "="*60)
    print("Testing LLM Location Mapper")
    print("="*60)

    for cmd in test_commands:
        result = mapper.extract_location(cmd)
        if result:
            x, y, direction = result
            print(f"✓ Result: x={x}, y={y}, direction={direction}")
        else:
            print("✗ No location found")
        print("-"*60)

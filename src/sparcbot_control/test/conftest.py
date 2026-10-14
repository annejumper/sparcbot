import sys
from pathlib import Path

# Make test/reference.py importable as `reference`.
sys.path.insert(0, str(Path(__file__).parent))

import os
import sys
from pathlib import Path

# Get the absolute path to the folder containing this script
MAIN_DIR = Path(__file__).resolve().parent

# Always run as if the current working directory were this script's folder
os.chdir(MAIN_DIR)

# Ensure the project root is in sys.path (for imports like `acsl_pychrono.executor`)
if str(MAIN_DIR.parent) not in sys.path:
  sys.path.insert(0, str(MAIN_DIR.parent))

import acsl_pychrono.executor as Executor
from acsl_pychrono import get_cli_args
  
# This function can be called if imported as external module
def run_experiment(**kwargs):
  """
  Run a simulation programmatically (without using CLI).
  Works only for wrapper_flag = False
  Example:
    from main import *
    run_experiment(uav="X8", controller="PID", simulation_duration=10.0)
  """
  # Convert kwargs to a Namespace just like argparse would produce
  # parser = argparse.ArgumentParser()
  args = get_cli_args()
  for key, val in kwargs.items():
    setattr(args, key, val)
  main(args)

def main(cli_args):
  """Main entry point for both CLI and module use."""
  print("ENTERED MAIN")
  print("CLI ARGS:", cli_args)
  Executor.launchSimulation(cli_args)

# Execution if the sript is directly called
if __name__ == '__main__':
  args = get_cli_args()
  main(args)
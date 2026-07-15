import argparse
import contextlib
import os
import sys
from copy import deepcopy
from pathlib import Path

import numpy as np

import acsl_pychrono.config.config as Cfg
from acsl_pychrono.control.logging import Logging
from acsl_pychrono.executor.simulate_mission import simulateMission
from acsl_pychrono.simulation.simulation import Simulation


def _as_vector(log_dict, group, keys):
  return np.column_stack([np.asarray(log_dict[group][key], dtype=float).reshape(-1) for key in keys])


def tracking_metrics(log_dict):
  position = _as_vector(log_dict, "position", ("x", "y", "z"))
  desired = _as_vector(log_dict, "user_defined_position", ("x", "y", "z"))
  error = position - desired
  norm = np.linalg.norm(error, axis=1)
  return {
    "rms_position_error_m": float(np.sqrt(np.mean(norm ** 2))),
    "max_position_error_m": float(np.max(norm)),
    "final_position_error_m": float(norm[-1]),
  }


def build_config(controller_type, mission_name, duration, visualize):
  mission = Cfg.MissionConfig(
    controller_type=controller_type,
    simulation_duration_seconds=duration,
    visualization_flag=visualize,
    trajectory_type="piecewise_polynomial_trajectory",
    trajectory_data_path="rollercoaster_trajectory1p2.json",
    add_payload_flag=False,
  )
  vehicle = Cfg.VehicleConfig(uav_name="X8")
  environment = Cfg.EnvironmentConfig(include=False)
  wrapper = Cfg.WrapperParams()

  if mission_name == "payload_dither":
    mission.apply_payload_dither = True
    mission.payload_dither_f_fast = 8.0
    mission.payload_dither_f_slow = 0.3
    mission.payload_dither_mean0 = 8
    mission.payload_dither_amp = 2
  elif mission_name == "thrust_dither":
    mission.apply_thrust_dither = True
    mission.thrust_dither_motor_ids = (0,)
    mission.thrust_dither_f_fast = 10.0
    mission.thrust_dither_f_slow = 0.2
    mission.thrust_dither_eps_fast = 0.03
    mission.thrust_dither_eps_slow = 0.02
  elif mission_name == "combined_dither":
    mission.apply_payload_dither = True
    mission.apply_thrust_dither = True
  elif mission_name == "two_ball_drop":
    mission.add_payload_flag = True
    mission.payload_type = "two_steel_balls"
    mission.drop_two_steel_balls = True
    mission.two_steel_balls_drop_time = 1.0
  elif mission_name == "sequential_payload_drop":
    mission.add_payload_flag = True
    mission.payload_type = "ten_steel_balls_in_two_lines"
    mission.sequentially_drop_multiple_balls = True
    mission.sequentially_drop_start_time = 1.0
    mission.sequentially_drop_interval = 0.10
  elif mission_name == "motor_failure":
    mission.apply_motor_failure = True
    mission.motor_failure_time = 1.0
  elif mission_name == "environmentA":
    environment.include = True
    environment.model_relative_path = "environmentA/environmentA.py"
  else:
    raise ValueError(f"Unknown mission: {mission_name}")

  return Cfg.SimulationConfig(
    mission_config=mission,
    vehicle_config=vehicle,
    environment_config=environment,
    wrapper_params=wrapper,
  )


def run_case(controller_type, mission_name, duration, visualize, verbose_sim):
  sim_cfg = build_config(controller_type, mission_name, duration, visualize)
  git_info = Logging.getGitRepoInfo()
  if verbose_sim:
    sim = Simulation(deepcopy(sim_cfg))
    log_dict = simulateMission(sim, git_info)
  else:
    with open(os.devnull, "w", encoding="utf-8") as sink:
      with contextlib.redirect_stdout(sink):
        sim = Simulation(deepcopy(sim_cfg))
        log_dict = simulateMission(sim, git_info)
  return tracking_metrics(log_dict)


def main():
  parser = argparse.ArgumentParser(description="Compare HybridMRAC with and without RKHS on dither disturbance missions.")
  parser.add_argument("--duration", type=float, default=8.0)
  parser.add_argument("--visualize", action="store_true")
  parser.add_argument("--verbose-sim", action="store_true")
  parser.add_argument(
    "--missions",
    nargs="+",
    default=["payload_dither", "thrust_dither"],
    choices=[
      "payload_dither",
      "thrust_dither",
      "combined_dither",
      "two_ball_drop",
      "sequential_payload_drop",
      "motor_failure",
      "environmentA",
    ],
  )
  args = parser.parse_args()

  os.chdir(Path(__file__).resolve().parent)
  controllers = ("HybridMRAC", "HybridMRACwithRKHS")

  print("mission,controller,rms_position_error_m,max_position_error_m,final_position_error_m")
  for mission_name in args.missions:
    for controller_type in controllers:
      metrics = run_case(controller_type, mission_name, args.duration, args.visualize, args.verbose_sim)
      print(
        f"{mission_name},{controller_type},"
        f"{metrics['rms_position_error_m']:.6f},"
        f"{metrics['max_position_error_m']:.6f},"
        f"{metrics['final_position_error_m']:.6f}"
      )


if __name__ == "__main__":
  main()

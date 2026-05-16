# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Examples.ArmCommands.DemoCircle do
  @moduledoc """
  Demo command that traces a circle using DLS IK.

  Traces a small circle around a configurable centre point in one of the
  three axis-aligned planes. By default the circle is in the XZ plane (the
  vertical plane in front of the robot) and is centred on the current
  end-effector position, so the command is safe to run from any reachable
  pose.

  Each waypoint waits for the end-effector to actually arrive (within
  `settle_tolerance_m`) before commanding the next one, so the traced path
  stays faithful to the planned circle.

  ## Goal Parameters

  Required:
  - `ee_link` - The end-effector link name in the robot's topology

  Optional:
  - `centre` - Circle centre as a `BB.Math.Vec3` (default: current EE position)
  - `plane` - One of `:xy`, `:xz`, `:yz` — the plane the circle is traced in
    (default: `:xz`)
  - `radius` - Circle radius in metres (default: `0.03`)
  - `points` - Number of points around the circle (default: `16`)
  - `exclude_joints` - Joints to hold in place during the solve (default: `[]`)
  - `settle_tolerance_m` - EE distance from target to consider arrived (default: `5.0e-3`)
  - `settle_timeout_ms` - Max wait per waypoint before continuing (default: `1500`)

  ## Usage

      {:ok, cmd} = MyArm.Robot.demo_circle(%{ee_link: :ee_link})
      {:ok, :complete} = BB.Command.await(cmd, 30_000)

      # Horizontal circle (XY plane), centred on a specific point:
      {:ok, cmd} = MyArm.Robot.demo_circle(%{
        ee_link: :ee_link,
        plane: :xy,
        centre: BB.Math.Vec3.new(0.25, 0.0, 0.20),
        radius: 0.05
      })
  """
  use BB.Command

  alias BB.IK.DLS
  alias BB.IK.DLS.Motion
  alias BB.Math.Vec3
  alias BB.Motion, as: BBMotion
  alias BB.Robot.Kinematics
  alias BB.Robot.State, as: RobotState

  @default_radius 0.03
  @default_points 16
  @default_plane :xz
  @default_settle_tolerance_m 5.0e-3
  @default_settle_timeout_ms 1500

  @valid_planes [:xy, :xz, :yz]

  # Loose IK tolerance/large step/low damping/few iterations let the warm-started
  # solver converge in 1-3 iterations per waypoint. Tighter values just waste FK
  # calls when the settle tolerance is 5mm anyway.
  @ik_tolerance 2.0e-3
  @ik_step_size 0.3
  @ik_lambda 0.1
  @ik_max_iterations 15

  # Motion.move_to/4 can return errors at runtime but dialyzer can't see
  # through :telemetry.span/3 and thinks it always returns {:ok, meta}
  @dialyzer {:no_match, [run_circle: 5, execute_path: 6]}

  @impl BB.Command
  def handle_command(%{ee_link: ee_link} = goal, context, state) when is_atom(ee_link) do
    plane = Map.get(goal, :plane, @default_plane)

    if plane in @valid_planes do
      run_circle(goal, plane, ee_link, context, state)
    else
      {:stop, :normal, %{state | result: {:error, {:invalid_plane, plane, @valid_planes}}}}
    end
  end

  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:error, :missing_ee_link}}}
  end

  defp run_circle(goal, plane, ee_link, context, state) do
    radius = Map.get(goal, :radius, @default_radius)
    points = Map.get(goal, :points, @default_points)
    tolerance = Map.get(goal, :settle_tolerance_m, @default_settle_tolerance_m)
    timeout = Map.get(goal, :settle_timeout_ms, @default_settle_timeout_ms)
    exclude_joints = Map.get(goal, :exclude_joints, [])
    centre = Map.get_lazy(goal, :centre, fn -> current_ee_position(context, ee_link) end)

    ik_opts = [
      delivery: :direct,
      exclude_joints: exclude_joints,
      tolerance: @ik_tolerance,
      step_size: @ik_step_size,
      lambda: @ik_lambda,
      max_iterations: @ik_max_iterations
    ]

    case Motion.move_to(context, ee_link, centre, ik_opts) do
      {:ok, _meta} ->
        wait_for_arrival(context, ee_link, centre, tolerance, timeout)
        targets = generate_circle_points(centre, radius, points, plane)

        case execute_path(context, ee_link, targets, tolerance, timeout, ik_opts) do
          :ok ->
            Motion.move_to(context, ee_link, centre, ik_opts)
            wait_for_arrival(context, ee_link, centre, tolerance, timeout)
            {:stop, :normal, %{state | result: :complete}}

          {:error, reason} ->
            {:stop, :normal, %{state | result: {:error, reason}}}
        end

      error ->
        {:stop, :normal, %{state | result: {:error, {:failed_to_reach_start, error}}}}
    end
  end

  @impl BB.Command
  def result(%{result: {:error, _} = error}), do: error
  def result(%{result: result}), do: {:ok, result}

  defp current_ee_position(context, ee_link) do
    positions = RobotState.get_all_positions(context.robot_state)
    {x, y, z} = Kinematics.link_position(context.robot, positions, ee_link)
    Vec3.new(x, y, z)
  end

  defp generate_circle_points(%Vec3{} = centre, radius, num_points, plane) do
    {cos_axis, sin_axis} = plane_axes(plane)
    cx = Vec3.x(centre)
    cy = Vec3.y(centre)
    cz = Vec3.z(centre)

    for i <- 0..num_points do
      angle = 2 * :math.pi() * i / num_points
      cos_offset = radius * :math.cos(angle)
      sin_offset = radius * :math.sin(angle)

      offsets = %{cos_axis => cos_offset, sin_axis => sin_offset}

      Vec3.new(
        cx + Map.get(offsets, :x, 0.0),
        cy + Map.get(offsets, :y, 0.0),
        cz + Map.get(offsets, :z, 0.0)
      )
    end
  end

  defp plane_axes(:xy), do: {:x, :y}
  defp plane_axes(:xz), do: {:x, :z}
  defp plane_axes(:yz), do: {:y, :z}

  # Solves each waypoint warm-starting from the previous IK solution rather
  # than from robot_state, which the position estimator overwrites during arm
  # motion. With consecutive waypoints ~3° apart in joint space, IK converges
  # in 1-3 iterations instead of the ~15-25 it takes from a cold start.
  defp execute_path(context, ee_link, targets, tolerance, timeout, ik_opts) do
    seed_positions = RobotState.get_all_positions(context.robot_state)
    solver_opts = Keyword.delete(ik_opts, :delivery)

    targets
    |> Enum.reduce_while({:ok, seed_positions}, fn target, {:ok, positions} ->
      case DLS.solve(context.robot, positions, ee_link, target, solver_opts) do
        {:ok, new_positions, _meta} ->
          BBMotion.send_positions(context, new_positions, delivery: :direct)
          wait_for_arrival(context, ee_link, target, tolerance, timeout)
          {:cont, {:ok, new_positions}}

        {:error, _} = error ->
          {:halt, {:error, {:ik_failed, target, error}}}
      end
    end)
    |> case do
      {:ok, _last_positions} -> :ok
      {:error, _} = error -> error
    end
  end

  # Poll the position estimator until the EE is within `tolerance` of `target`
  # or `timeout_ms` elapses. Motion.move_to/send_positions writes its target
  # into robot_state immediately, so we sleep one estimator tick (~20 ms) first
  # to let the actual interpolated position arrive.
  defp wait_for_arrival(context, ee_link, target, tolerance, timeout_ms) do
    Process.sleep(25)
    deadline = System.monotonic_time(:millisecond) + timeout_ms
    do_wait_for_arrival(context, ee_link, target, tolerance, deadline)
  end

  defp do_wait_for_arrival(context, ee_link, target, tolerance, deadline) do
    distance = current_ee_distance(context, ee_link, target)

    cond do
      distance <= tolerance ->
        :ok

      System.monotonic_time(:millisecond) >= deadline ->
        :timeout

      true ->
        Process.sleep(20)
        do_wait_for_arrival(context, ee_link, target, tolerance, deadline)
    end
  end

  defp current_ee_distance(context, ee_link, target) do
    positions = RobotState.get_all_positions(context.robot_state)
    {x, y, z} = Kinematics.link_position(context.robot, positions, ee_link)

    :math.sqrt(
      :math.pow(x - Vec3.x(target), 2) +
        :math.pow(y - Vec3.y(target), 2) +
        :math.pow(z - Vec3.z(target), 2)
    )
  end
end

# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Examples.ArmCommands.MoveToPose do
  @moduledoc """
  Command to move the end effector to a target position using IK.

  Solves inverse kinematics via `BB.IK.DLS` (Damped Least Squares) for the
  named end-effector link and sends the resulting joint targets directly to
  the actuators.

  ## Goal Parameters

  Required:
  - `target` - A `BB.Math.Vec3` target position
  - `ee_link` - The end-effector link name in the robot's topology

  Optional:
  - `exclude_joints` - Joints to hold in place during the solve (default: `[]`)

  ## Usage

      target = BB.Math.Vec3.new(0.25, 0.0, 0.2)
      {:ok, cmd} = MyArm.Robot.move_to_pose(%{target: target, ee_link: :ee_link})
      {:ok, :reached} = BB.Command.await(cmd, 10_000)

  Holding the gripper while moving:

      MyArm.Robot.move_to_pose(%{
        target: target,
        ee_link: :ee_link,
        exclude_joints: [:gripper]
      })
  """
  use BB.Command

  alias BB.IK.DLS.Motion
  alias BB.Math.Vec3

  # Motion.move_to/4 can return errors at runtime but dialyzer can't see
  # through :telemetry.span/3 and thinks it always returns {:ok, meta}
  @dialyzer {:no_match, handle_command: 3}

  @impl BB.Command
  def handle_command(%{target: %Vec3{} = target, ee_link: ee_link} = goal, context, state)
      when is_atom(ee_link) do
    exclude_joints = Map.get(goal, :exclude_joints, [])
    ik_opts = [delivery: :direct, exclude_joints: exclude_joints]

    case Motion.move_to(context, ee_link, target, ik_opts) do
      {:ok, _meta} ->
        {:stop, :normal, %{state | result: :reached}}

      error ->
        {:stop, :normal, %{state | result: {:error, {:ik_failed, error}}}}
    end
  end

  def handle_command(%{target: %Vec3{}}, _context, state) do
    {:stop, :normal, %{state | result: {:error, :missing_ee_link}}}
  end

  def handle_command(_goal, _context, state) do
    {:stop, :normal, %{state | result: {:error, :missing_target}}}
  end

  @impl BB.Command
  def result(%{result: {:error, _} = error}), do: error
  def result(%{result: result}), do: {:ok, result}
end

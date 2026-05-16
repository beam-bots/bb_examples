# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Examples.ArmCommands.Home do
  @moduledoc """
  Command handler to move joints to a neutral position.

  By default, sends position `0.0` to every movable joint in the robot
  topology. Pass `:position` or `:exclude_joints` in the goal to customise.

  ## Goal Parameters

  All optional:
  - `position` - Position (in radians) to send to each joint (default: `0.0`)
  - `exclude_joints` - List of joint names to hold in place (default: `[]`)

  ## Usage

      commands do
        command :home do
          handler BB.Examples.ArmCommands.Home
          allowed_states [:idle]
        end
      end

  Then execute:

      {:ok, cmd} = MyArm.Robot.home()
      {:ok, :homed} = BB.Command.await(cmd)

      # Hold the gripper while homing everything else:
      {:ok, cmd} = MyArm.Robot.home(%{exclude_joints: [:gripper]})

  """
  use BB.Command

  alias BB.Robot.Joint

  @default_position 0.0

  @impl BB.Command
  def handle_command(goal, context, state) do
    position = Map.get(goal, :position, @default_position)
    excluded = Map.get(goal, :exclude_joints, []) |> MapSet.new()

    positions =
      context.robot.joints
      |> Enum.filter(fn {name, joint} ->
        Joint.movable?(joint) and not MapSet.member?(excluded, name)
      end)
      |> Map.new(fn {name, _joint} -> {name, position} end)

    BB.Motion.send_positions(context, positions, delivery: :direct)

    {:stop, :normal, %{state | result: :homed}}
  end

  @impl BB.Command
  def result(%{result: result}), do: {:ok, result}
end

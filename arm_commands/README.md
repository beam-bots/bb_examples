<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

# `arm_commands`

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)

Demo command handlers for serial-chain robot arms running on
[Beam Bots](https://github.com/beam-bots/bb). Each command is a small,
copy-pasteable handler module suitable for dropping into your robot's
`commands do … end` block.

This package is part of the
[`bb_examples`](https://github.com/beam-bots/bb_examples) repository and is
distributed as a git dependency — there is no Hex release.

## Modules

| Module                              | Purpose                                                                |
|-------------------------------------|------------------------------------------------------------------------|
| `BB.Examples.ArmCommands.Home`      | Move every revolute joint to position `0 rad`                          |
| `BB.Examples.ArmCommands.MoveToPose`| Move the end effector to a target `BB.Math.Vec3` via DLS IK            |
| `BB.Examples.ArmCommands.DemoCircle`| Trace a circle in the XZ plane, waiting for the EE to settle at each waypoint |

## Installation

Add the package to your `mix.exs` as a git dependency:

```elixir
defp deps do
  [
    {:arm_commands,
     git: "https://github.com/beam-bots/bb_examples.git",
     sparse: "arm_commands"}
  ]
end
```

`MoveToPose` and `DemoCircle` need a configured IK solver. The defaults assume
[`bb_ik_dls`](https://github.com/beam-bots/bb_ik_dls); other solvers work if
they implement `BB.IK.Solver`.

## Usage

Reference the modules in your robot's `commands` block:

```elixir
defmodule MyArm.Robot do
  use BB

  commands do
    command :arm,    do: handler(BB.Command.Arm)
    command :disarm, do: handler(BB.Command.Disarm)

    command :home do
      handler BB.Examples.ArmCommands.Home
      allowed_states [:idle]
    end

    command :move_to_pose do
      handler BB.Examples.ArmCommands.MoveToPose
      allowed_states [:idle]
    end

    command :demo_circle do
      handler BB.Examples.ArmCommands.DemoCircle
      allowed_states [:idle]
    end
  end

  # …topology…
end
```

The IK-driven commands need to know the name of your end-effector link, so
pass it in the goal:

```elixir
# Home all movable joints to position 0
{:ok, cmd} = MyArm.Robot.home()
{:ok, :homed} = BB.Command.await(cmd)

# Move the EE to a Cartesian target
target = BB.Math.Vec3.new(0.25, 0.0, 0.2)
{:ok, cmd} = MyArm.Robot.move_to_pose(%{target: target, ee_link: :ee_link})
{:ok, :reached} = BB.Command.await(cmd, 10_000)

# Trace a small circle centred on the current EE position
{:ok, cmd} = MyArm.Robot.demo_circle(%{ee_link: :ee_link})
{:ok, :complete} = BB.Command.await(cmd, 30_000)
```

All three commands accept an optional `:exclude_joints` list (e.g.
`[:gripper]`) to hold specific joints in place while the rest move.

## Customising

These handlers are deliberately small — open the source and tweak the
constants (default radius, settle tolerance, joint set) for your robot. The
[bb_examples](https://github.com/beam-bots/bb_examples) repository is for
starting points, not load-bearing libraries: fork freely.

## Licence

Apache-2.0

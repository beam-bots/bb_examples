<!--
SPDX-FileCopyrightText: 2026 James Harton

SPDX-License-Identifier: Apache-2.0
-->

<img src="https://github.com/beam-bots/bb/blob/main/logos/beam_bots_logo.png?raw=true" alt="Beam Bots Logo" width="250" />

# Beam Bots Examples

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache--2.0-green.svg)](https://opensource.org/licenses/Apache-2.0)
[![REUSE status](https://api.reuse.software/badge/github.com/beam-bots/bb_examples)](https://api.reuse.software/info/github.com/beam-bots/bb_examples)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/beam-bots/bb_examples)

A collection of small, opinionated example mix packages for
[Beam Bots](https://github.com/beam-bots/bb) projects. Each subdirectory is an
independent mix project, distributed as a **git dependency** rather than via
Hex, on the basis that example/demo code doesn't need versioned releases.

## Packages

| Package                        | Description                                                          |
|--------------------------------|----------------------------------------------------------------------|
| [`arm_commands`](./arm_commands) | Reusable demo commands for serial-chain robot arms (home, move-to-pose, demo circle) |

## Using a package

Pick the sub-directory you want, add it to your `mix.exs` as a git dependency
with a `sparse:` checkout so only that sub-project is pulled in:

```elixir
defp deps do
  [
    {:arm_commands,
     git: "https://github.com/beam-bots/bb_examples.git",
     sparse: "arm_commands"}
  ]
end
```

Then `mix deps.get` and use the modules in your robot definition:

```elixir
commands do
  command :home do
    handler BB.Examples.ArmCommands.Home
    allowed_states [:idle]
  end
end
```

See each package's README for the modules it exposes and what each one does.

## Why git, not Hex?

Example code is, by nature, opinionated and likely to drift as the framework
evolves. Distributing via git means:

- No hex versioning treadmill — pin a commit when you want stability
- Easy to fork and customise — these are starting points, not load-bearing libs
- One repo, many packages — adding `motion_recordings/` or `safety_patterns/`
  later is just another subdirectory

## Layout

```
bb_examples/
├── arm_commands/        # Independent mix project
│   ├── mix.exs
│   ├── lib/
│   ├── test/
│   └── README.md
├── (more packages …)
└── README.md            # You're here
```

Each sub-package follows the standard mix project layout. The top-level repo
holds only this README, the licence files, and `.gitignore`.

## Contributing

Submit example packages as PRs against `main`. Aim for:

- A clearly-stated purpose (one paragraph in the package README)
- Useful in more than one project — if it's specific to one robot, it
  probably belongs in that robot's own repo
- No mandatory hardware deps — packages should work in simulation mode at
  minimum

## Licence

Apache-2.0

# SPDX-FileCopyrightText: 2026 James Harton
#
# SPDX-License-Identifier: Apache-2.0

defmodule BB.Examples.ArmCommands.MixProject do
  use Mix.Project

  @moduledoc """
  Reusable demo commands for serial-chain robot arms running on Beam Bots.
  """

  @version "0.1.0"

  def project do
    [
      app: :arm_commands,
      version: @version,
      elixir: "~> 1.19",
      elixirc_paths: elixirc_paths(Mix.env()),
      description: @moduledoc,
      deps: deps(),
      dialyzer: dialyzer(),
      docs: docs(),
      start_permanent: Mix.env() == :prod
    ]
  end

  def application do
    [extra_applications: [:logger]]
  end

  defp dialyzer, do: [plt_add_apps: [:mix]]

  defp docs do
    [
      main: "readme",
      extras: ["README.md", "CHANGELOG.md"],
      source_ref: "main",
      source_url: "https://github.com/beam-bots/bb_examples",
      source_url_pattern: fn path, line ->
        "https://github.com/beam-bots/bb_examples/blob/main/arm_commands/#{path}#L#{line}"
      end
    ]
  end

  defp deps do
    [
      {:bb, "~> 0.15.4"},
      {:bb_ik_dls, "~> 0.3"},

      # dev/test
      {:credo, "~> 1.7", only: [:dev, :test], runtime: false},
      {:dialyxir, "~> 1.4", only: [:dev, :test], runtime: false},
      {:ex_check, "~> 0.16", only: [:dev, :test], runtime: false},
      {:ex_doc, ">= 0.0.0", only: [:dev, :test], runtime: false},
      {:mix_audit, "~> 2.1", only: [:dev, :test], runtime: false}
    ]
  end

  defp elixirc_paths(env) when env in [:dev, :test], do: ["lib", "test/support"]
  defp elixirc_paths(_), do: ["lib"]
end

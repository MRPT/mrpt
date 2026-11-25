\page app_mrpt-ini2yaml mrpt-ini2yaml


Converts an MRPT INI-like config file into YAML, and prints the result to the
standard output. Useful to migrate the config files of MRPT apps or classes to
YAML. The inverse conversion is done by \ref app_mrpt-yaml2ini.

# Usage

    mrpt-ini2yaml <input.ini>

The program takes exactly one argument (`-h` or `--help` prints the usage) and
writes the YAML text to stdout. Errors are reported to stderr, with a non-zero exit
code.

# Example

Using a sample config file from `mrpt_data`:

    mrpt-ini2yaml config_files/simul-beacons/example_simul-beacons.ini > example_simul-beacons.yaml

Output (excerpt):

    Params:
      circularPath: 1
      max_x: 4
      max_y: 4
      ...

# Conversion rules

- Each INI section `[name]` becomes a YAML map `name:` with one entry per key.
- Keys outside of any section become top-level YAML entries.
- All values are written as YAML scalars, as read by
  mrpt::config::CConfigFile, i.e. after evaluating `@define` variables,
  `$eval{}` expressions and `$env{}` environment variables (see
  \ref config_file_format).
- Comments are lost, and keys are sorted alphabetically within each section.

The conversion is implemented by
mrpt::config::CConfigFileBase::getContentAsYAML().

# See also

- \ref app_mrpt-yaml2ini: the inverse conversion.
- mrpt::config::CConfigFile, mrpt::config::CConfigFileMemory.

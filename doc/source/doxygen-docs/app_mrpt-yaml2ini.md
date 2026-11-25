\page app_mrpt-yaml2ini mrpt-yaml2ini


Converts a YAML file into an MRPT INI-like config file, and prints the result
to the standard output. Useful to write configurations in YAML for MRPT apps
or classes that only read INI files. The inverse conversion is done by
\ref app_mrpt-ini2yaml.

# Usage

    mrpt-yaml2ini <input.yaml>

The program takes exactly one argument (it has no other options) and writes
the INI text to stdout. Errors are reported to stderr, with a non-zero exit
code.

# Example

    mrpt-yaml2ini params.yaml > params.ini

with this `params.yaml`:

    rawlog_offset: 0
    Params:
      nBeacons: 14
      nSteps: 100

produces:

    rawlog_offset = 0

    [Params]
    nBeacons = 14
    nSteps = 100

# Conversion rules

INI files only have one level of sections, so the YAML file must have at most
two levels:

- Top-level maps become INI sections `[name]`, with one `key = value` line
  per entry. Values must be scalars.
- Top-level scalars become keys outside of any section, written first.
- Comments are lost, and keys are sorted alphabetically.

The conversion is implemented by
mrpt::config::CConfigFileBase::setContentFromYAML().

# See also

- \ref app_mrpt-ini2yaml: the inverse conversion.
- \ref config_file_format, mrpt::config::CConfigFileMemory.

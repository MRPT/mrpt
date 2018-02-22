/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

/** \page config_file_format Configuration file format in MRPT
 *

# Description

Plain text configuration files in MRPT follows a custom version of the [INI file](https://en.wikipedia.org/wiki/INI_file)
standard format, comprising "sections", "properties" (or "keys") with associated "values" and, optionally, comments.

The following C++ classes are provided to read and write such files:
- mrpt::config::CConfigFile: Access to physical files.
- mrpt::config::CConfigFileMemory: Wrapper around a configuration file "in memory", without an associated physical file.

See also:
- mrpt::config::CConfigFileBase: The base, virtual class underlying the two classes above. Users normally
   inkove the API exposed in this base class.
- mrpt::config::CConfigFilePrefixer: A proxy class to manipulate an object of the two classes above such
   that all accesses to sections and/or properties are *mapped* to modified versions of their names.


# Format specifications

- There exists only one level of hierarchy, i.e. only "toplevel" sections exist, there is no support for nested sections.
  A possible workaround to this limitation is using  mrpt::config::CConfigFilePrefixer.
- Sections are formatted like: `[section_name]`
- Key/values pair follow the format: `key = value`. Whitespaces are ignored before and after the `=` sign, up to the 
first non-blank character of the value.
- API methods exist to read and write different elementary data types (`int`,`double`,`std::string`) and also 
 vectors, matrices and even `enum`s.
- Comments can be included in different formats:
  - Lines starting with `;`. Example: `; Comment line`
  - Lines starting with `#`. Example: `# Comment line`
  - After a value, with `//`. Example: `key = value    // Explanation of this value`
  - An exception to the rule above is hard-coded to allow URLs, e.g. `key = http://www.google.com` is not considered to contain a comment.
- Preprocessor:
  - Just like in C/C++, lines can be ended in a backslash (`\`) to mean "line continuation". [New in MRPT 1.5.0]
  - C preprocessor-like `#define`s are available as `@define VARNAME VALUE`, then using variables as `${VARNAME}` or math expressions as `$eval{...}`. See the example below: [New in MRPT 1.5.0].
		\code
		@define MAXSPEED 10
		@define MAXDIST  $eval{exp(2*MAXSPEED)}
		[test]
		var1=${MAXSPEED}
		var2=$eval{1+2*MAXSPEED}
		var3=$env{MY_ENV_VARIABLE}
		\endcode

# Examples

There are dozens of examples in the subdirectory [MRPT/share/mrpt/config_files](https://github.com/MRPT/mrpt/tree/master/share/mrpt/config_files).

*/


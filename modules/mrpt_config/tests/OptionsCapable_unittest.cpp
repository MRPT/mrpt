/*                    _
                     | |    Mobile Robot Programming Toolkit (MRPT)
 _ __ ___  _ __ _ __ | |_
| '_ ` _ \| '__| '_ \| __|          https://www.mrpt.org/
| | | | | | |  | |_) | |_
|_| |_| |_|_|  | .__/ \__|     https://github.com/MRPT/mrpt/
               | |
               |_|

 Copyright (c) 2005-2026, Individual contributors, see AUTHORS file
 See: https://www.mrpt.org/Authors - All rights reserved.
 SPDX-License-Identifier: BSD-3-Clause
*/

#include <gtest/gtest.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/config/OptionsCapable.h>

namespace
{
struct TDummyInsertionOptions : public mrpt::config::CLoadableOptions
{
  double threshold = 0.5;

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override
  {
    threshold = source.read_double(section, "threshold", threshold);
  }
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& target, const std::string& section) const override
  {
    target.write(section, "threshold", threshold);
  }
};

struct TDummyCreationOptions : public mrpt::config::CLoadableOptions
{
  double voxel_size = 0.1;

  void loadFromConfigFile(
      const mrpt::config::CConfigFileBase& source, const std::string& section) override
  {
    voxel_size = source.read_double(section, "voxel_size", voxel_size);
  }
  void saveToConfigFile(
      mrpt::config::CConfigFileBase& target, const std::string& section) const override
  {
    target.write(section, "voxel_size", voxel_size);
  }
};

/** Minimal, self-contained implementation used only to exercise the
 * mrpt::config::OptionsCapable interface. */
class DummyOptionsCapable : public mrpt::config::OptionsCapable
{
 public:
  TDummyCreationOptions creationOptions;
  TDummyInsertionOptions insertionOptions;

  bool hasData = false;

  [[nodiscard]] std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() override
  {
    return {
        { "creationOptions",  &creationOptions},
        {"insertionOptions", &insertionOptions},
    };
  }

  bool trySetCreationOptions(
      const mrpt::config::CConfigFileBase& cfg, const std::string& section) override
  {
    const double newVoxelSize = cfg.read_double(section, "voxel_size", creationOptions.voxel_size);
    if (hasData && newVoxelSize != creationOptions.voxel_size)
    {
      return false;  // would require rebuilding internal state
    }
    creationOptions.loadFromConfigFile(cfg, section);
    return true;
  }
};

}  // namespace

TEST(OptionsCapable, OptionsByNameExposesLiveMembers)
{
  DummyOptionsCapable obj;

  auto opts = obj.optionsByName();
  ASSERT_EQ(opts.size(), 2U);
  ASSERT_TRUE(opts.count("creationOptions"));
  ASSERT_TRUE(opts.count("insertionOptions"));

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("insertionOptions", "threshold", 1.25);
  opts.at("insertionOptions")->loadFromConfigFile(cfg, "insertionOptions");

  // The map returned by optionsByName() must point to the real, live member:
  EXPECT_DOUBLE_EQ(obj.insertionOptions.threshold, 1.25);
}

TEST(OptionsCapable, DefaultTrySetCreationOptionsReturnsFalse)
{
  class NoCreationOptions : public mrpt::config::OptionsCapable
  {
   public:
    [[nodiscard]] std::map<std::string, mrpt::config::CLoadableOptions*> optionsByName() override
    {
      return {};
    }
  };

  NoCreationOptions obj;
  mrpt::config::CConfigFileMemory cfg;
  EXPECT_FALSE(obj.trySetCreationOptions(cfg, "creationOptions"));
}

TEST(OptionsCapable, TrySetCreationOptionsRefusesIncompatibleChangeWhenDataPresent)
{
  DummyOptionsCapable obj;
  obj.hasData = true;

  mrpt::config::CConfigFileMemory cfg;
  cfg.write("creationOptions", "voxel_size", 0.2);

  EXPECT_FALSE(obj.trySetCreationOptions(cfg, "creationOptions"));
  EXPECT_DOUBLE_EQ(obj.creationOptions.voxel_size, 0.1);  // left unmodified

  obj.hasData = false;
  EXPECT_TRUE(obj.trySetCreationOptions(cfg, "creationOptions"));
  EXPECT_DOUBLE_EQ(obj.creationOptions.voxel_size, 0.2);
}

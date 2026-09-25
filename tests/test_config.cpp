#include <gtest/gtest.h>

#include <stdexcept>

#include "altctl/config.hpp"

using altctl::Config;

TEST(Config, SetOverridesNumericAndTextValues)
{
    Config config;
    config.set("alt_high_m", "12.5");
    config.set("vel_kp", "0.4");
    config.set("connection_url", "udpin://0.0.0.0:14552");
    EXPECT_DOUBLE_EQ(config.alt_high_m, 12.5);
    EXPECT_DOUBLE_EQ(config.velocity_pid.kp, 0.4);
    EXPECT_EQ(config.connection_url, "udpin://0.0.0.0:14552");
}

TEST(Config, UnknownKeyIsRejected)
{
    Config config;
    EXPECT_THROW(config.set("alt_hgih_m", "10"), std::runtime_error);
}

TEST(Config, MalformedNumberIsRejected)
{
    Config config;
    EXPECT_THROW(config.set("alt_high_m", "ten"), std::runtime_error);
    EXPECT_THROW(config.set("alt_high_m", "1.5abc"), std::runtime_error);
    EXPECT_THROW(config.set("alt_high_m", ""), std::runtime_error);
    EXPECT_DOUBLE_EQ(config.alt_high_m, Config{}.alt_high_m);
}

TEST(Config, ShippedConfigFileLoads)
{
    const Config config = Config::load(ALTCTL_SOURCE_DIR "/config/mission.conf");
    EXPECT_DOUBLE_EQ(config.alt_high_m, 10.0);
    EXPECT_DOUBLE_EQ(config.alt_low_m, 5.0);
}

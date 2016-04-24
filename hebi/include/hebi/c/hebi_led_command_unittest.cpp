// Include unit testing code
#include <gtest/gtest.h>

#include "hebi_command.h"
#include "hebi_module.h"

TEST(LedCommandCreateTest, Create) {
  HebiCommand* cmd = hebiCommandCreate();
  ASSERT_TRUE(NULL != cmd);
  HebiLedCommand* ac = hebiCommandGetLedCommand(cmd);
  EXPECT_TRUE(NULL != ac);
  hebiCommandDestroy(cmd);
}

class LedCommandTests : public testing::Test
{
  protected:
    HebiCommand* cmd;
    HebiLedCommand* lc;

    virtual void SetUp()
    {
      cmd = hebiCommandCreate();
      ASSERT_TRUE(NULL != cmd);
      lc = hebiCommandGetLedCommand(cmd);
      ASSERT_TRUE(NULL != lc);
    }
  
    virtual void TearDown()
    {
      hebiCommandDestroy(cmd);
    }
};

TEST_F(LedCommandTests, Override)
{
  // Note: currently the rgb vals in the protocol are ints from 0-255, but the
  // API accepts floats. TODO: FIX THIS!
  const float setR = 25;
  const float setG = 50;
  const float setB = 60;
  float getR, getG, getB;
  bool isOverride;

  // Starts blank:
  ASSERT_EQ(false, hebiLedCommandHasCommand(lc));

  // Set an override command and check:
  hebiLedCommandSetEnableOverride(lc, setR, setG, setB);
  ASSERT_EQ(true, hebiLedCommandHasCommand(lc));
  isOverride = hebiLedCommandGetCommand(lc, &getR, &getG, &getB);
  EXPECT_EQ(true, isOverride);
  EXPECT_FLOAT_EQ(setR, getR);
  EXPECT_FLOAT_EQ(setG, getG);
  EXPECT_FLOAT_EQ(setB, getB);

  // Directly set a disable override:
  hebiLedCommandSetDisableOverride(lc);
  ASSERT_EQ(true, hebiLedCommandHasCommand(lc));
  isOverride = hebiLedCommandGetCommand(lc, &getR, &getG, &getB);
  EXPECT_EQ(false, isOverride);

  // Try removing:
  hebiLedCommandClearCommand(lc);
  ASSERT_EQ(false, hebiLedCommandHasCommand(lc));
}

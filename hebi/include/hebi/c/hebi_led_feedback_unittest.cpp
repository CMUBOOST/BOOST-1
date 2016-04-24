// Include unit testing code
#include <gtest/gtest.h>

// Note: because feedback is read-only, to unit test we need to connect to a
// module.
#include "hebi_lookup.h"
#include "hebi_module.h"
#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_led_feedback.h"

#include "hebi_test_module.h"

// Tests which require connecting to a test module and getting "feedback"
class LedFeedbackModuleTests : public testing::Test
{
  protected:
    HebiModule* module;
    virtual void SetUp()
    {
      printf("Setting up test\n");
      HebiLookup* lookup = hebiCreateLookup();
      sleep(2);
      hebiPrintLookupTable(lookup);
      ASSERT_TRUE(NULL != lookup);
      module = hebiGetModuleFromName(lookup, TEST_MODULE_NAME, TEST_MODULE_GROUP, 5000);
      ASSERT_TRUE(NULL != module);
      // Clear the lookup; we don't need this anymore.
      if (lookup != NULL)
        hebiDeleteLookup(lookup);
    }
  
    virtual void TearDown()
    {
      if (module != NULL)
        hebiReleaseModule(module);
    }
};

TEST_F(LedFeedbackModuleTests, Color)
{
  // TODO: change to 0-1? or to int?
  float setR = 15;
  float setG = 40;
  float setB = 62;
  float getR = 0;
  float getG = 0;
  float getB = 0;

  // Check to be sure that we initially get no LED feedback:
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  EXPECT_FALSE(hebiLedFeedbackHasColor(hebiFeedbackGetLedFeedback(fbk)));
  EXPECT_FALSE(hebiLedFeedbackGetColor(hebiFeedbackGetLedFeedback(fbk), &getR, &getG, &getB));

  // Set the color of the LED on the test module:
  HebiCommand* cmd = hebiCommandCreate();
  HebiLedCommand* ledCmd = hebiCommandGetLedCommand(cmd);
  hebiLedCommandSetEnableOverride(ledCmd, setR, setG, setB);
  ASSERT_TRUE(hebiModuleSendCommandWithAcknowledgement(module, cmd, 500));

  // Check the LED feedback:
  // TODO: should we clear the feedback object? Should this automatically be done
  // when it is passed to request feedback?
  hebiModuleRequestFeedback(module, fbk, 500);
  EXPECT_TRUE(hebiLedFeedbackHasColor(hebiFeedbackGetLedFeedback(fbk)));
  EXPECT_TRUE(hebiLedFeedbackGetColor(hebiFeedbackGetLedFeedback(fbk), &getR, &getG, &getB));
  EXPECT_FLOAT_EQ(setR, getR);
  EXPECT_FLOAT_EQ(setG, getG);
  EXPECT_FLOAT_EQ(setB, getB);

  // Clear the LED on the test module, and recheck:
  hebiLedCommandSetDisableOverride(ledCmd);
  ASSERT_TRUE(hebiModuleSendCommandWithAcknowledgement(module, cmd, 500));
  // TODO: should we clear the feedback object? Should this automatically be done
  // when it is passed to request feedback?
  hebiModuleRequestFeedback(module, fbk, 500);
  EXPECT_FALSE(hebiLedFeedbackHasColor(hebiFeedbackGetLedFeedback(fbk)));
  EXPECT_FALSE(hebiLedFeedbackGetColor(hebiFeedbackGetLedFeedback(fbk), &getR, &getG, &getB));

  hebiFeedbackDestroy(fbk);
  hebiCommandDestroy(cmd);
};

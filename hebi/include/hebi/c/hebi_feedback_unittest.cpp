// Include unit testing code
#include <gtest/gtest.h>

// Note: because feedback is read-only, to unit test we need to connect to a
// module.
#include "hebi_lookup.h"
#include "hebi_module.h"
#include "hebi_command.h"
#include "hebi_feedback.h"

#include "hebi_test_module.h"

TEST(FeedbackCreateTest, Create)
{
  HebiFeedback* fbk = hebiFeedbackCreate();
  ASSERT_TRUE(NULL != fbk);
  hebiFeedbackDestroy(fbk);
}

// A set of tests about retrieving the child feedback types.
class FeedbackChildTests : public testing::Test
{
  protected:
    HebiFeedback* fbk;

    virtual void SetUp()
    {
      fbk = hebiFeedbackCreate();
      ASSERT_TRUE(NULL != fbk);
    }
  
    virtual void TearDown()
    {
      hebiFeedbackDestroy(fbk);
    }
};

TEST_F(FeedbackChildTests, ActuatorFbk)
{
  HebiActuatorFeedback* af = hebiFeedbackGetActuatorFeedback(fbk);
  EXPECT_TRUE(NULL != af);
}

TEST_F(FeedbackChildTests, LedFbk)
{
  HebiLedFeedback* lf = hebiFeedbackGetLedFeedback(fbk);
  EXPECT_TRUE(NULL != lf);
}

TEST_F(FeedbackChildTests, ImuFbk)
{
  HebiImuFeedback* imuf = hebiFeedbackGetImuFeedback(fbk);
  EXPECT_TRUE(NULL != imuf);
}

// Tests which require connecting to a test module and getting "feedback"
class FeedbackModuleTests : public testing::Test
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

TEST_F(FeedbackModuleTests, Temperature)
{
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  // TODO: check temperature values instead of just retrieving them?
  float temperature;
  bool has_data;

  has_data = hebiFeedbackHasAmbientTemperature(fbk);
  EXPECT_TRUE(has_data);
  if (has_data)
    temperature = hebiFeedbackGetAmbientTemperature(fbk);

  has_data = hebiFeedbackHasProcessorTemperature(fbk);
  EXPECT_TRUE(has_data);
  if (has_data)
    temperature = hebiFeedbackGetProcessorTemperature(fbk);
  
  hebiFeedbackDestroy(fbk);
};

TEST_F(FeedbackModuleTests, Power)
{
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);
  // TODO: check values instead of just retrieving them?
  float data;
  bool has_data;

  has_data = hebiFeedbackHasVoltage(fbk);
  EXPECT_TRUE(has_data);
  if (has_data)
    data = hebiFeedbackGetVoltage(fbk);
  
  hebiFeedbackDestroy(fbk);
};

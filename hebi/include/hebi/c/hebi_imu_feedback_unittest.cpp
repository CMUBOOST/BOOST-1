// Include unit testing code
#include <gtest/gtest.h>

// Note: because feedback is read-only, to unit test we need to connect to a
// module.
#include "hebi_lookup.h"
#include "hebi_module.h"
#include "hebi_command.h"
#include "hebi_feedback.h"
#include "hebi_actuator_feedback.h"

#include "hebi_test_module.h"

#include <cmath>

// Tests which require connecting to a test module and getting "feedback"
class ImuFeedbackModuleTests : public testing::Test
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

TEST_F(ImuFeedbackModuleTests, ImuFeedback)
{
  // Check feedback from module:
  HebiFeedback* fbk = hebiFeedbackCreate();
  hebiModuleRequestFeedback(module, fbk, 500);

  // Accel data:
  EXPECT_TRUE(hebiImuFeedbackHasAccelerometer(hebiFeedbackGetImuFeedback(fbk)));
  HebiVector3f accel = hebiImuFeedbackGetAccelerometer(hebiFeedbackGetImuFeedback(fbk));
  EXPECT_FALSE(std::isnan(accel.x));
  EXPECT_FALSE(std::isnan(accel.y));
  EXPECT_FALSE(std::isnan(accel.z));

  hebiModuleRequestFeedback(module, fbk, 500);
  EXPECT_TRUE(hebiImuFeedbackHasGyro(hebiFeedbackGetImuFeedback(fbk)));
  // Can't really say anything about gyro data...
  HebiVector3f gyro = hebiImuFeedbackGetGyro(hebiFeedbackGetImuFeedback(fbk));
  EXPECT_FALSE(std::isnan(gyro.x));
  EXPECT_FALSE(std::isnan(gyro.y));
  EXPECT_FALSE(std::isnan(gyro.z));

  hebiFeedbackDestroy(fbk);
};


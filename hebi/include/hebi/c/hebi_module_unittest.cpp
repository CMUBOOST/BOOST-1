// Include unit testing code
#include <gtest/gtest.h>

#include <unistd.h> // fork/exec
#include <errno.h>

#include "hebi_lookup.h"
#include "hebi_module.h"
#include "hebi_test_module.h"

class ModuleTests : public testing::Test
{
  protected:
    virtual void SetUp()
    {
      printf("Setting up test\n");
    }
  
    virtual void TearDown()
    {
    }
};

TEST_F(ModuleTests, GetDummyModule) {
  printf("Running test\n");
  HebiLookup* lookup = hebiCreateLookup();
  sleep(2);
  hebiPrintLookupTable(lookup);
  ASSERT_TRUE(NULL != lookup);
  HebiModule* module = hebiGetModuleFromName(lookup, TEST_MODULE_NAME, TEST_MODULE_GROUP, 5000);
  EXPECT_TRUE(NULL != module);
  if (module != NULL)
    hebiReleaseModule(module);
  if (lookup != NULL)
    hebiDeleteLookup(lookup);
}

// TODO: different orders of destruction (module vs. lookup), with and without pauses...

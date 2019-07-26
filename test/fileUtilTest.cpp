#include <gtest/gtest.h>
#include "fileUtil.hpp"

TEST(FileUtilTest, addSlashToLast) {
	{
		const std::string kDirPath = "path/to/dir";
		const std::string kResultPath = FileUtil::addSlashToLast(kDirPath);
		EXPECT_STREQ(std::string(kDirPath + "/").c_str(), kResultPath.c_str());
	}

	{
		const std::string kDirPath = "path/to/dir/";
		const std::string kResultPath = FileUtil::addSlashToLast(kDirPath);
		EXPECT_STREQ(kDirPath.c_str(), kResultPath.c_str());
	}
}

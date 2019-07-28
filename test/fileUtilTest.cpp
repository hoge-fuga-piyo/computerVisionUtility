#include <unordered_map>
#include <gtest/gtest.h>
#include "fileUtil.hpp"

TEST(FileUtilTest, readFilesAndDirs) {
	const std::string kDirPath = "../../testdir";

	const auto kFileAndDirPaths = FileUtil::readFilesAndDirs(kDirPath);
	const auto kFilePaths = std::get<0>(kFileAndDirPaths);
	const auto kDirPaths = std::get<1>(kFileAndDirPaths);

	std::unordered_map<std::string, bool> exist_file_paths;
	for (const auto& kFilePath : kFilePaths) {
		exist_file_paths[kFilePath.filename().string()] = true;
	}
	std::unordered_map<std::string, bool> exist_dir_paths;
	for (const auto& kDirPath : kDirPaths) {
		exist_dir_paths[kDirPath.filename().string()] = true;
	}

	EXPECT_EQ(2, kFilePaths.size());
	EXPECT_TRUE(exist_file_paths["file1"]);
	EXPECT_TRUE(exist_file_paths["file2"]);

	EXPECT_EQ(1, kDirPaths.size());
	EXPECT_TRUE(exist_dir_paths["dir"]);
}

TEST(FileUtilTest, readFiles) {
	const std::string kDirPath = "../../testdir";

	const auto kFilePaths = FileUtil::readFiles(kDirPath);
	std::unordered_map<std::string, bool> exist_file_paths;
	for (const auto& kFilePath : kFilePaths) {
		exist_file_paths[kFilePath.filename().string()] = true;
	}

	EXPECT_EQ(2, kFilePaths.size());
	EXPECT_TRUE(exist_file_paths["file1"]);
	EXPECT_TRUE(exist_file_paths["file2"]);
}

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

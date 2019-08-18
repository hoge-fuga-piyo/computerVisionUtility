#ifndef FILE_UTIL_HPP
#define FILE_UTIL_HPP

#include <vector>
#include <filesystem>

class FileUtil {
public:
	static std::tuple<std::vector<std::filesystem::path>, std::vector<std::filesystem::path>> readFilesAndDirs(const std::string& kDirPath);
	static std::vector<std::filesystem::path> readFiles(const std::string& kDirPath);
	static std::string addSlashToLast(const std::string& kDirPath);
private:

};

#endif
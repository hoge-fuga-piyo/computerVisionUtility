#include <gtest/gtest.h>
#include "exifInfo.hpp"

TEST(ExifInfoTest, loadImage) {
	const std::string kDirPath = "../../data/";

	// image has exif
	{
		ExifInfo exif_info;
		EXPECT_TRUE(exif_info.loadImage(kDirPath + "exif_inch.jpg"));
	}

	// image does not have exif
	{
		ExifInfo exif_info;
		EXPECT_FALSE(exif_info.loadImage(kDirPath + "no_exif.jpg"));
	}

	// invalid image path
	{
		testing::internal::CaptureStdout();
		ExifInfo exif_info;
		EXPECT_FALSE(exif_info.loadImage(kDirPath + "invali.jpg"));
		const std::string kErrorString = kDirPath + "invali.jpg: Failed to open the data source: No such file or directory (errno = 2)\n";
		EXPECT_STREQ(kErrorString.c_str(), testing::internal::GetCapturedStdout().c_str());
	}
}

TEST(ExifInfoTest, hasFocalLengthInMm) {
	const std::string kDirPath = "../../data/";

	// image has focal length
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "exif_inch.jpg");
		EXPECT_TRUE(exif_info.hasFocalLengthInMm());
	}

	// image does not have focal length
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "no_focallength.jpg");
		EXPECT_FALSE(exif_info.hasFocalLengthInMm());
	}
}

TEST(ExifInfoTest, getFocalLengthInMm) {
	const std::string kDirPath = "../../data/";

	// image has focal length
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "exif_inch.jpg");
		EXPECT_EQ(50, exif_info.getFocalLengthInMm());
	}
}

TEST(ExifInfoTest, hasFocalLengthInPixel) {
	const std::string kDirPath = "../../data/";

	// focal length expressed in pixel can be computed
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "exif_inch.jpg");
		EXPECT_TRUE(exif_info.hasFocalLengthInPixel());
	}

	// focal length expressed in pixel can not be computed
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "no_focallength.jpg");
		EXPECT_FALSE(exif_info.hasFocalLengthInPixel());
	}
}

TEST(ExifInfoTest, getFocalLengthInPixel) {
	const std::string kDirPath = "../../data/";

	// focal length expressed in pixel can be computed
	{
		ExifInfo exif_info;
		exif_info.loadImage(kDirPath + "exif_inch.jpg");
		EXPECT_DOUBLE_EQ(5000.0, exif_info.getFocalLengthInPixel());
	}
}

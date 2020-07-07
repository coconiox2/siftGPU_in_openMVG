#include "main.h"
//#include "imageListing.hpp"
#include "computeFeaturesSiftGPU.hpp"
//#include "computeFeatures.hpp"

int main(int argc, char** argv)
{
	////计算内参等
	//imageListing::ImageListing sImageListing;
	//sImageListing.checkFileAndPathValidity();
	//sImageListing.imageListing();
	
	//siftGPU特征点检测
	computeFeaturesSiftGPU::ComputeFeaturesSiftGPU sComputeFeatures;
	//sComputeFeatures.checkFileAndPathValidity();
	sComputeFeatures.computeFeatures();

	////openMVG特征点检测
	//ComputeFeatures sComputeFeatures;
	//sComputeFeatures.checkFileAndPathValidity();
	//sComputeFeatures.computeFeatures();

	//getchar();
	return 0;
	////声明SiftGPU并初始化  
	//////////////////////////////////////////////////////
	//SiftGPU sSiftGPU;
	//char* myargv[4] = { "-fo", "-1", "-v", "1" };
	//sSiftGPU.ParseParam(4, myargv);
	////检查硬件是否支持SiftGPU  
	//int support = sSiftGPU.CreateContextGL();
	//if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
	//{
	//	cerr << "SiftGPU is not supported!" << endl;
	//	return 2;
	//}
	/////////////////////////////////////////////////////

	////const std::string sImageInputDir = stlplus::folder_up(imageInputDir, 1) + "/360_school/";
	//std::vector<std::string> vec_image = stlplus::folder_files(sImageInputDir);
	//std::sort(vec_image.begin(), vec_image.end());

	////imgCount 图像数目
	////num 每一幅图像的特征点个数
	////descriptor 每一个特征点的描述符
	////keys 每一个特征点
	////images 所有cv形式的图像
	//int imgCount = 0;
	//vector<int> num;
	//vector<vector<float>> descriptors;
	//vector<vector<SiftGPU::SiftKeypoint>> keys;
	//vector<cv::Mat> images;

	////show progress of imageListing process
	//C_Progress_display my_progress_bar(vec_image.size(),
	//	std::cout, "\n- Image listing -\n");
	//std::ostringstream error_report_stream;
	//for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
	//	iter_image != vec_image.end();
	//	++iter_image, ++my_progress_bar) 
	//{
	//	imgCount++;

	//	const std::string sImageFilename = stlplus::create_filespec(sImageInputDir, *iter_image);
	//	//const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

	//	cv::Mat cvImage = imread(sImageFilename);
	//	images.push_back(cvImage);

	//	const char* cvImageData = sImageFilename.data();
	//	sSiftGPU.RunSIFT(cvImageData);
	//	int _num = sSiftGPU.GetFeatureNum();
	//	num.push_back(_num);
	//	cout << "Keypoints Number of " << sImageFilename << " is= " << _num << endl << endl;
	//	vector<SiftGPU::SiftKeypoint> _keys;
	//	vector<float> _descriptors;
	//	_keys.resize(_num);
	//	_descriptors.resize(128 * _num);
	//	sSiftGPU.GetFeatureVector(&_keys[0], &_descriptors[0]);
	//	keys.push_back(_keys);
	//	descriptors.push_back(_descriptors);
	//}
	//
	////SiftGPU的特征点赋值于OpenCV关键点类型
	//vector<vector<KeyPoint>> cvKeypoints;
	//for (int i = 0; i < imgCount; ++i) {
	//	vector<KeyPoint> cvKeys;
	//	for (int j = 0; j < keys[i].size(); ++j) {
	//		KeyPoint cvTempKey;
	//		cvTempKey.pt.x = keys[i][j].x;
	//		cvTempKey.pt.y = keys[i][j].y;
	//		cvKeys.push_back(cvTempKey);
	//	}
	//	cvKeypoints.push_back(cvKeys);
	//}
	////验证第i张图片的特征点数是否相同
	//for (int i = 0; i < imgCount; ++i) {
	//	if (cvKeypoints[i].size() != keys[i].size())
	//		cerr << "Image " << i << "'s Keypoints Number isn't correct." << endl;
	//}
	////使用OpenCV接口显示关键点
	//for (int i = 0; i < imgCount; ++i) {
	//	cv::Mat featureImage;
	//	drawKeypoints(images[i], cvKeypoints[i], featureImage, Scalar(255, 255, 255), DrawMatchesFlags::DEFAULT);
	//	namedWindow("Sift Keypoints", WINDOW_NORMAL);
	//	imshow("Sift Keypoints", featureImage);
	//	waitKey(0);
	//}

	////SiftGPU的描述子赋值于OpenCV描述子类型
	//vector<cv::Mat> cvDescriptors(imgCount);  //每个Mat内部是num行，128列的特征点描述矩阵
	//for (int inum = 0; inum < imgCount; ++inum) {
	//	//图i的SiftGPU的描述子_des
	//	vector<vector<float>> _des;
	//	for (int sj = 0; sj < num[inum]; ++sj) {
	//		vector<float> temp(128, 0.0);
	//		for (int sk = 0; sk < 128; ++sk) {
	//			temp[sk] = descriptors[inum][sk + 128 * sj];
	//		}
	//		_des.push_back(temp);
	//	}
	//	//图i的OpenCV描述子_cvDes
	//	cv::Mat _cvDes(num[inum], 128, CV_32F);
	//	for (int cj = 0; cj < num[inum]; ++cj) {
	//		float* pxDesMat = _cvDes.ptr<float>(cj);
	//		for (int ck = 0; ck < 128; ++ck) {
	//			pxDesMat[ck] = _des[cj][ck];
	//		}
	//	}
	//	
	//	cvDescriptors.push_back(_cvDes);
	//}
	
	
}


//int main(int argc, char** argv) {
//	ImageListing simageListing;
//	simageListing.checkFileAndPathValidity();
//	simageListing.imageListing();
//
//	
//	
//
//	ComputeFeatures myComputeFeatures;
//	myComputeFeatures.checkFileAndPathValidity();
//
//	
//
//	myComputeFeatures.computeFeatures();
//
//	
//	getchar();
//	return 1;
//}



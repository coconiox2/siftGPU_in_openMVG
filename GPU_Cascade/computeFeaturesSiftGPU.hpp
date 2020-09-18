#pragma once
#ifndef _COMPUTE_FEATURES_SIFTGPU_HPP
#define _COMPUTE_FEATURES_SIFTGPU_HPP

//#include "main.h"
//openMVG
#include <cereal/archives/json.hpp>

#include "openMVG/features/akaze/image_describer_akaze_io.hpp"

#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer_io.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/features/regions_factory_io.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include "nonFree/sift/SIFT_describer_io.hpp"

#include <cereal/details/helpers.hpp>
//c++标准库
#include <atomic>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <utility>
#include <memory>

#ifdef OPENMVG_USE_OPENMP
#include <omp.h>
#endif

// OpenCV图像
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"

// OpenGL
#include <Windows.h>
#include <GL/gl.h>

// SiftGPU模块
#include "SiftGPU.h"



// boost库中计时函数
#include <boost/timer.hpp>


//链接 静态库
#pragma comment(lib,"devil.lib")
//#pragma comment(lib,"ilu.lib")
//#pragma comment(lib,"ilut.lib")

using namespace cv;
using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace std;

namespace computeFeaturesSiftGPU {
	const int group_count = 4;
	const int block_count_per_group = 4;
	const int image_count_per_block = 1;

	

	bool cmp(float &a, float &b)
	{
		if (a>b)
			return true;
		return false;
	}

	class ComputeFeaturesSiftGPU
	{
		using Regions_type = SIFT_Regions;
	public:
		/*///check if sSfM_Data_FilenameDir,sSfM_Data_Filename,sOutDir is valid
		///maybe they don't meet stlplus::folder_up's requirements
		bool checkFileAndPathValidity()
		{
			if (!stlplus::is_folder(sSfM_Data_FilenameDir))
			{
				std::cerr << "sSfM_Data_FilenameDir:" << sSfM_Data_FilenameDir << "is not a stlplus::folder!\n" << std::endl;
				return false;
			}


			if (!stlplus::is_folder(sOutDir))
			{
				std::cerr << "sOutDir:" << sOutDir << "is not a stlplus::folder!\n" << std::endl;
				return false;
			}

			if (!stlplus::is_file(sSfM_Data_Filename))
			{
				std::cerr << "sSfM_Data_Filename" << sSfM_Data_Filename << "is not a stlplus::file!\n" << std::endl;
				return false;
			}

			return true;
		};*/
		features::EDESCRIBER_PRESET stringToEnum(const std::string & sPreset)
		{
			features::EDESCRIBER_PRESET preset;
			if (sPreset == "NORMAL")
				preset = features::NORMAL_PRESET;
			else
				if (sPreset == "HIGH")
					preset = features::HIGH_PRESET;
				else
					if (sPreset == "ULTRA")
						preset = features::ULTRA_PRESET;
					else
						preset = features::EDESCRIBER_PRESET(-1);
			return preset;
		}
		/// - Compute view image description (feature & descriptor extraction)
		/// - Export computed data
		int computeFeatures(std::string sImageListingOutputDirFather)
		{
			const std::string sComputeFeaturesOutputDirFather = sImageListingOutputDirFather;

			std::string sSfM_Data_Filename;
			std::string sOutDir = "";
			bool bUpRight = false;
			std::string sImage_Describer_Method = "SIFT";
			bool bForce = false;
			std::string sFeaturePreset = "";
#ifdef OPENMVG_USE_OPENMP
			int iNumThreads = omp_get_max_threads();
#endif
			
//			// required
//			cmd.add(make_option('i', sSfM_Data_Filename, "input_file"));
//			cmd.add(make_option('o', sOutDir, "outdir"));
//			// Optional
//			cmd.add(make_option('m', sImage_Describer_Method, "describerMethod"));
//			cmd.add(make_option('u', bUpRight, "upright"));
//			cmd.add(make_option('f', bForce, "force"));
//			cmd.add(make_option('p', sFeaturePreset, "describerPreset"));
//
//#ifdef OPENMVG_USE_OPENMP
//			cmd.add(make_option('n', iNumThreads, "numThreads"));
//			//iNumThreads = 4;
//#endif

			/*try {
				if (argc == 1) throw std::string("Invalid command line parameter.");
				cmd.process(argc, argv);
			}
			catch (const std::string& s) {
				std::cerr << "Usage: " << argv[0] << '\n'
					<< "[-i|--input_file] a SfM_Data file \n"
					<< "[-o|--outdir path] \n"
					<< "\n[Optional]\n"
					<< "[-f|--force] Force to recompute data\n"
					<< "[-m|--describerMethod]\n"
					<< "  (method to use to describe an image):\n"
					<< "   SIFT (default),\n"
					<< "   SIFT_ANATOMY,\n"
					<< "   AKAZE_FLOAT: AKAZE with floating point descriptors,\n"
					<< "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
					<< "[-u|--upright] Use Upright feature 0 or 1\n"
					<< "[-p|--describerPreset]\n"
					<< "  (used to control the Image_describer configuration):\n"
					<< "   NORMAL (default),\n"
					<< "   HIGH,\n"
					<< "   ULTRA: !!Can take long time!!\n"
#ifdef OPENMVG_USE_OPENMP
					<< "[-n|--numThreads] number of parallel computations\n"
#endif
					<< std::endl;

				std::cerr << s << std::endl;
				return EXIT_FAILURE;
			}*/
			std::cout << " You called : " << std::endl
				<< "ComputeFeatures" << std::endl
				<< "--input_file " << sSfM_Data_Filename << std::endl
				<< "--outdir " << sOutDir << std::endl
				<< "--describerMethod " << sImage_Describer_Method << std::endl
				<< "--upright " << bUpRight << std::endl
				<< "--describerPreset " << (sFeaturePreset.empty() ? "NORMAL" : sFeaturePreset) << std::endl
				<< "--force " << bForce << std::endl
#ifdef OPENMVG_USE_OPENMP
				<< "--numThreads " << iNumThreads << std::endl
#endif
				<< std::endl;

			for (int i = 0; i < group_count; i++) 
			{
				char temp_i[2] = { ' ','\0' };
				temp_i[0] = i + 48;
				const std::string str_i = temp_i;
				sOutDir = sComputeFeaturesOutputDirFather + "/DJI_" + str_i + "_build/";
				sSfM_Data_Filename = sOutDir + "sfm_data.json";
				
				if (sOutDir.empty()) {
					std::cerr << "\nIt is an invalid output directory" << std::endl;
					return EXIT_FAILURE;
				}

				// Create output dir
				if (!stlplus::folder_exists(sOutDir))
				{
					if (!stlplus::folder_create(sOutDir))
					{
						std::cerr << "Cannot create output directory" << std::endl;
						return EXIT_FAILURE;
					}
				}

				//---------------------------------------
				// a. Load input scene
				//---------------------------------------
				SfM_Data sfm_data;
				if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS | INTRINSICS))) {
					std::cerr << std::endl
						<< "The input file \"" << sSfM_Data_Filename << "\" cannot be read" << std::endl;
					return EXIT_FAILURE;
				}

				// b. Init the image_describer
				// - retrieve the used one in case of pre-computed features
				// - else create the desired one

				using namespace openMVG::features;
				std::unique_ptr<Image_describer> image_describer;

				const std::string sImage_describer = stlplus::create_filespec(sOutDir, "image_describer", "json");
				if (!bForce && stlplus::is_file(sImage_describer))
				{
					// Dynamically load the image_describer from the file (will restore old used settings)
					std::ifstream stream(sImage_describer.c_str());
					if (!stream.is_open())
						return EXIT_FAILURE;

					try
					{
						cereal::JSONInputArchive archive(stream);
						archive(cereal::make_nvp("image_describer", image_describer));
					}
					catch (const cereal::Exception & e)
					{
						std::cerr << e.what() << std::endl
							<< "Cannot dynamically allocate the Image_describer interface." << std::endl;
						return EXIT_FAILURE;
					}
				}
				else
				{
					// Create the desired Image_describer method.
					// Don't use a factory, perform direct allocation
					if (sImage_Describer_Method == "SIFT")
					{
						image_describer.reset(new SIFT_Image_describer
						(SIFT_Image_describer::Params(), !bUpRight));
					}
					else
						if (sImage_Describer_Method == "SIFT_ANATOMY")
						{
							image_describer.reset(
								new SIFT_Anatomy_Image_describer(SIFT_Anatomy_Image_describer::Params()));
						}
						else
							if (sImage_Describer_Method == "AKAZE_FLOAT")
							{
								image_describer = AKAZE_Image_describer::create
								(AKAZE_Image_describer::Params(features::AKAZE::Params(), AKAZE_MSURF), !bUpRight);
							}
							else
								if (sImage_Describer_Method == "AKAZE_MLDB")
								{
									image_describer = AKAZE_Image_describer::create
									(AKAZE_Image_describer::Params(features::AKAZE::Params(), AKAZE_MLDB), !bUpRight);
								}
					if (!image_describer)
					{
						std::cerr << "Cannot create the designed Image_describer:"
							<< sImage_Describer_Method << "." << std::endl;
						return EXIT_FAILURE;
					}
					else
					{
						if (!sFeaturePreset.empty())
							if (!image_describer->Set_configuration_preset(stringToEnum(sFeaturePreset)))
							{
								std::cerr << "Preset configuration failed." << std::endl;
								return EXIT_FAILURE;
							}
					}

					// Export the used Image_describer and region type for:
					// - dynamic future regions computation and/or loading
					{
						std::ofstream stream(sImage_describer.c_str());
						if (!stream.is_open())
							return EXIT_FAILURE;

						cereal::JSONOutputArchive archive(stream);
						archive(cereal::make_nvp("image_describer", image_describer));
						auto regionsType = image_describer->Allocate();
						archive(cereal::make_nvp("regions_type", regionsType));
					}
				}

				// Feature extraction routines
				// For each View of the SfM_Data container:
				// - if regions file exists continue,
				// - if no file, compute features
				{
					//声明SiftGPU并初始化  
					////////////////////////////////////////////////////
					SiftGPU sSiftGPU;
					char* myargv[4] = { "-fo", "-1", "-v", "1" };
					sSiftGPU.ParseParam(4, myargv);
					//检查硬件是否支持SiftGPU  
					int support = sSiftGPU.CreateContextGL();
					if (support != SiftGPU::SIFTGPU_FULL_SUPPORTED)
					{
						cerr << "SiftGPU is not supported!" << endl;
						return 2;
					}
					///////////////////////////////////////////////////

					system::Timer timer;
					Image<unsigned char> imageGray;

					C_Progress_display my_progress_bar(sfm_data.GetViews().size(),
						std::cout, "\n- EXTRACT FEATURES -\n");

					// Use a boolean to track if we must stop feature extraction
					std::atomic<bool> preemptive_exit(false);
#ifdef OPENMVG_USE_OPENMP
					const unsigned int nb_max_thread = omp_get_max_threads();

					if (iNumThreads > 0) {
						omp_set_num_threads(iNumThreads);
					}
					else {
						omp_set_num_threads(nb_max_thread);
					}

//#pragma omp parallel for schedule(dynamic) if (iNumThreads > 0) private(imageGray)
#endif

					for (int i = 0; i < static_cast<int>(sfm_data.views.size()); ++i)
					{
						Views::const_iterator iterViews = sfm_data.views.begin();
						std::advance(iterViews, i);
						const View * view = iterViews->second.get();
						const std::string
							sView_filename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path),
							sFeat = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "feat"),
							sDesc = stlplus::create_filespec(sOutDir, stlplus::basename_part(sView_filename), "desc");

						// If features or descriptors file are missing, compute them
						if (!preemptive_exit && (bForce || !stlplus::file_exists(sFeat) || !stlplus::file_exists(sDesc)))
						{
							//if (!ReadImage(sView_filename.c_str(), &imageGray))
							//	continue;

							////
							//// Look if there is occlusion feature mask
							////
							//Image<unsigned char> * mask = nullptr; // The mask is null by default

							//const std::string
							//	mask_filename_local =
							//	stlplus::create_filespec(sfm_data.s_root_path,
							//		stlplus::basename_part(sView_filename) + "_mask", "png"),
							//	mask__filename_global =
							//	stlplus::create_filespec(sfm_data.s_root_path, "mask", "png");

							//Image<unsigned char> imageMask;
							//// Try to read the local mask
							//if (stlplus::file_exists(mask_filename_local))
							//{
							//	if (!ReadImage(mask_filename_local.c_str(), &imageMask))
							//	{
							//		std::cerr << "Invalid mask: " << mask_filename_local << std::endl
							//			<< "Stopping feature extraction." << std::endl;
							//		preemptive_exit = true;
							//		continue;
							//	}
							//	// Use the local mask only if it fits the current image size
							//	if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
							//		mask = &imageMask;
							//}
							//else
							//{
							//	// Try to read the global mask
							//	if (stlplus::file_exists(mask__filename_global))
							//	{
							//		if (!ReadImage(mask__filename_global.c_str(), &imageMask))
							//		{
							//			std::cerr << "Invalid mask: " << mask__filename_global << std::endl
							//				<< "Stopping feature extraction." << std::endl;
							//			preemptive_exit = true;
							//			continue;
							//		}
							//		// Use the global mask only if it fits the current image size
							//		if (imageMask.Width() == imageGray.Width() && imageMask.Height() == imageGray.Height())
							//			mask = &imageMask;
							//	}
							//}
							/////////////////////////////////////////////////////////////////////////////
							//// Compute features and descriptors and export them to files
							//clock_t startTime, endTime;
							//startTime = clock();

							//auto regions = image_describer->Describe(imageGray, mask);
							//endTime = clock();
							//std::cout << "time cost for 4 images:\n" << endTime - startTime << "\n" << std::endl;
							//if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
							//	std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
							//		<< "Stopping feature extraction." << std::endl;
							//	preemptive_exit = true;
							//	continue;
							//}

							// using siftGPU to Compute features and descriptors and export them to files
							const char* cvImageData = sView_filename.data();
							sSiftGPU.RunSIFT(cvImageData);

							int _num = sSiftGPU.GetFeatureNum();
							cout << "Keypoints Number of " << sView_filename << " is= " << _num << endl << endl;
							vector<SiftGPU::SiftKeypoint> _keys;
							vector<float> _descriptors;
							_keys.resize(_num);
							_descriptors.resize(128 * _num);
							sSiftGPU.GetFeatureVector(&_keys[0], &_descriptors[0]);

							// Build alias to cached data
							auto regions = std::unique_ptr<Regions_type>(new Regions_type);

							// reserve some memory for faster keypoint saving
							//考虑一下2000是否对特征点的提取有限制
							regions->Features().reserve(2000);
							regions->Descriptors().reserve(2000);

							//siftGPU keys -> openMVG::keys
							vector<KeyPoint> cvKeys;
							for (int j = 0; j < _keys.size(); ++j) {
								/*KeyPoint cvTempKey;
								cvTempKey.pt.x = _keys[j].x;
								cvTempKey.pt.y = _keys[j].y;
								cvKeys.push_back(cvTempKey);*/
								const SIOPointFeature fp(_keys[j].x, _keys[j].y,
									_keys[j].s, static_cast<float>(_keys[j].o));
								regions->Features().push_back(fp);
							}
							
							/*const float &temp0 = regions->Features()[100].scale();
							const float &temp1 = regions->Features()[200].scale();
							bool bo = temp0 > temp1;
							int s = 0;*/

							std::vector<float> vecScale;
							//size_t fea_length = regions->Features().size();
							//vecScale.reserve(fea_length);
							//vecScale.resize(fea_length);
							for (int i = 0; i < regions->Features().size(); i++)
							{
								const float &temp1 = regions->Features()[i].scale();
								float temp2 = temp1;
								vecScale.emplace_back(temp2);
							}
							//partial_sort(v.begin(),v.begin()+4,v.end(),cmp);  
							//////////std::partial_sort(vecScale.begin(), vecScale.begin() + 100, vecScale.end(), cmp);

							/*vector<double>::iterator ite1 = find(vec_dis.begin(), vec_dis.end(), vec_dis1[0]);
							vector<double>::iterator ite2 = find(vec_dis.begin(), vec_dis.end(), vec_dis1[1]);
							vector<double>::iterator ite3 = find(vec_dis.begin(), vec_dis.end(), vec_dis1[2]);
							auto index1 = std::distance(std::begin(vec_dis), ite1);
							auto index2 = std::distance(std::begin(vec_dis), ite2);
							auto index3 = std::distance(std::begin(vec_dis), ite3);*/

							
							//siftGPU descriptors -> openMVG::descriptors
							vector<vector<unsigned char>> _des;
							for (int sj = 0; sj < _num; ++sj) {
								Descriptor<vl_sift_pix, 128> temp_descr;
								Descriptor<unsigned char, 128> temp_descriptor;
								for (int sk = 0; sk < 128; ++sk) {
									temp_descr[sk] = _descriptors[sk + 128 * sj];
								}
								siftDescToUChar(&temp_descr[0], temp_descriptor, 0);
								regions->Descriptors().push_back(temp_descriptor);
							}

							//将regions中的特征点和描述符都存到以.feat 和 .desc为后缀名的文件里
							if (regions && !image_describer->Save(regions.get(), sFeat, sDesc)) {
								std::cerr << "Cannot save regions for images: " << sView_filename << std::endl
									<< "Stopping feature extraction." << std::endl;
								preemptive_exit = true;
								continue;
							}

						}
						++my_progress_bar;
					}
					std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
				}
			}
			return EXIT_SUCCESS;
		}
	};
}//namespace computeFeaturesSiftGPU


#endif // !_COMPUTE_FEATURES_SIFTGPU_HPP

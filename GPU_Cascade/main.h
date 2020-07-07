#pragma once
#ifndef _MAIN_H
#define _MAIN_H


////openMVG
//#include "openMVG/graph/graph.hpp"
//
//#include "openMVG/cameras/cameras.hpp"
//#include "openMVG/cameras/Camera_Pinhole.hpp"
//#include "openMVG/cameras/Camera_Pinhole_Radial.hpp"
//#include "openMVG/exif/exif_IO_EasyExif.hpp"
//#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
//#include "openMVG/features/akaze/image_describer_akaze_io.hpp"
//#include "openMVG/features/descriptor.hpp"
//#include "openMVG/features/feature.hpp"
//#include "openMVG/features/regions_factory_io.hpp"
//#include "openMVG/features/sift/SIFT_Anatomy_Image_Describer.hpp"
//#include "openMVG/features/svg_features.hpp"
//#include "openMVG/geodesy/geodesy.hpp"
//#include "openMVG/geometry/pose3.hpp"
//#include "openMVG/image/image_io.hpp"
//#include "openMVG/image/image_concat.hpp"
//#include "openMVG/matching/indMatchDecoratorXY.hpp"
//#include "openMVG/matching/regions_matcher.hpp"
//#include "openMVG/matching/svg_matches.hpp"
////match
//#include "openMVG/matching/indMatch.hpp"
//#include "openMVG/matching/indMatch_utils.hpp"
//#include "openMVG/matching_image_collection/Matcher_Regions.hpp"
//#include "openMVG/matching_image_collection/Cascade_Hashing_Matcher_Regions.hpp"
//#include "openMVG/matching_image_collection/GeometricFilter.hpp"
//#include "openMVG/matching_image_collection/F_ACRobust.hpp"
//#include "openMVG/matching_image_collection/E_ACRobust.hpp"
//#include "openMVG/matching_image_collection/E_ACRobust_Angular.hpp"
//#include "openMVG/matching_image_collection/Eo_Robust.hpp"
//#include "openMVG/matching_image_collection/H_ACRobust.hpp"
//#include "openMVG/matching_image_collection/Pair_Builder.hpp"
//#include "openMVG/matching/pairwiseAdjacencyDisplay.hpp"
//
//#include "openMVG/multiview/triangulation.hpp"
//#include "openMVG/numeric/eigen_alias_definition.hpp"
//#include "openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp"
////match
//#include "openMVG/sfm/pipelines/sfm_features_provider.hpp"
//#include "openMVG/sfm/pipelines/sfm_regions_provider.hpp"
//#include "openMVG/sfm/pipelines/sfm_regions_provider_cache.hpp"
//
//#include "openMVG/sfm/sfm_data.hpp"
//#include "openMVG/sfm/sfm_data_BA.hpp"
//#include "openMVG/sfm/sfm_data_BA_ceres.hpp"
//#include "openMVG/sfm/sfm_data_io.hpp"
//#include "openMVG/sfm/sfm_data_utils.hpp"
//#include "openMVG/sfm/sfm_view.hpp"
//#include "openMVG/sfm/sfm_view_priors.hpp"
////match
//#include "openMVG/stl/stl.hpp"
//#include "openMVG/system/timer.hpp"
//#include "openMVG/types.hpp"
//
//#include <cereal/archives/json.hpp>
//#include <cereal/details/helpers.hpp>
//
//#include "third_party/progress/progress_display.hpp"
//#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
//
//#include "nonFree/sift/SIFT_describer_io.hpp"
//
//// C++标准库
//#include <atomic>
//#include <cstdlib>
//#include <fstream>
//#include <iostream>
//#include <string>
//#include <utility>
//#include <memory>
//
//#ifdef OPENMVG_USE_OPENMP
//#include <omp.h>
//#endif
//
//// OpenCV图像
//#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/core.hpp>
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/features2d/features2d.hpp"
////#include "opencv2/nonfree/nonfree.hpp"
//
//// OpenGL
//#include <Windows.h>
//#include <GL/gl.h>
//
//// SiftGPU模块
//#include "SiftGPU.h"
//
//
//
//// boost库中计时函数
//#include <boost/timer.hpp>
//
//
////链接 静态库
//#pragma comment(lib,"devil.lib")
////#pragma comment(lib,"ilu.lib")
////#pragma comment(lib,"ilut.lib")
//
//using namespace cv;
//using namespace openMVG;
//using namespace openMVG::exif;
//using namespace openMVG::features;
//using namespace openMVG::image;
//using namespace openMVG::matching;
////match
//using namespace openMVG::robust;
//using namespace openMVG::matching_image_collection;
//
//using namespace openMVG::cameras;
//using namespace openMVG::geodesy;
//using namespace openMVG::geometry;
//using namespace openMVG::sfm;
//using namespace std;

#endif // !_MAIN_H

#pragma once
#ifndef _imageListing_H
#define _imageListing_H

//#include "main.h"
#include "openMVG/cameras/cameras.hpp"
#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/exif/sensor_width_database/ParseDatabase.hpp"
#include "openMVG/geodesy/geodesy.hpp"
#include "openMVG/image/image_io.hpp"
#include "openMVG/numeric/eigen_alias_definition.hpp"
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/sfm/sfm_data_io.hpp"
#include "openMVG/sfm/sfm_data_utils.hpp"
#include "openMVG/sfm/sfm_view.hpp"
#include "openMVG/sfm/sfm_view_priors.hpp"
#include "openMVG/types.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress_display.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <memory>
#include <string>
#include <utility>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::exif;
using namespace openMVG::geodesy;
using namespace openMVG::image;
using namespace openMVG::sfm;


namespace imageListing {
	const int group_count = 4;
	const int block_count_per_group = 4;
	const int image_count_per_block = 1;

	

	class ImageListing {
	public:
		///check if sImageInputDir,sfileDatabaseDir,sfileDatabase,sImageListingOutputDir is valid
		///maybe they don't meet stlplus::folder_up's requirements
		/*bool checkFileAndPathValidity()
		{
			if (!stlplus::is_folder(sImageInputDir))
			{
				std::cerr << "sImageInputDir:" << sImageInputDir << "is not a stlplus::folder!\n" << std::endl;
				return false;
			}

			if (!stlplus::is_folder(sImageListingOutputDir))
			{
				std::cerr << "sImageListingOutputDir:" << sImageListingOutputDir << "is not a stlplus::folder!\n" << std::endl;
				return false;
			}

			if (!stlplus::is_folder(sfileDatabaseDir))
			{
				std::cerr << "sfileDatabaseDir:" << sfileDatabaseDir << "is not a stlplus::folder!\n" << std::endl;
				return false;
			}

			if (!stlplus::is_file(sfileDatabase))
			{
				std::cerr << "sfileDatabase" << sfileDatabase << "is not a stlplus::file!\n" << std::endl;
				return false;
			}

			return true;
		};*/
		/// Check that Kmatrix is a string like "f;0;ppx;0;f;ppy;0;0;1"
		/// With f,ppx,ppy as valid numerical value
		bool checkIntrinsicStringValidity(const std::string & Kmatrix, double & focal, double & ppx, double & ppy)
		{
			std::vector<std::string> vec_str;
			stl::split(Kmatrix, ';', vec_str);
			if (vec_str.size() != 9) {
				std::cerr << "\n Missing ';' character" << std::endl;
				return false;
			}
			// Check that all K matrix value are valid numbers
			for (size_t i = 0; i < vec_str.size(); ++i) {
				double readvalue = 0.0;
				std::stringstream ss;
				ss.str(vec_str[i]);
				if (!(ss >> readvalue)) {
					std::cerr << "\n Used an invalid not a number character" << std::endl;
					return false;
				}
				if (i == 0) focal = readvalue;
				if (i == 2) ppx = readvalue;
				if (i == 5) ppy = readvalue;
			}
			return true;
		}

		std::pair<bool, Vec3> checkGPS
		(
			const std::string & filename,
			const int & GPS_to_XYZ_method = 0
		)
		{
			std::pair<bool, Vec3> val(false, Vec3::Zero());
			std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
			if (exifReader)
			{
				// Try to parse EXIF metada & check existence of EXIF data
				if (exifReader->open(filename) && exifReader->doesHaveExifInfo())
				{
					// Check existence of GPS coordinates
					double latitude, longitude, altitude;
					if (exifReader->GPSLatitude(&latitude) &&
						exifReader->GPSLongitude(&longitude) &&
						exifReader->GPSAltitude(&altitude))
					{
						// Add ECEF or UTM XYZ position to the GPS position array
						val.first = true;
						switch (GPS_to_XYZ_method)
						{
						case 1:
							val.second = lla_to_utm(latitude, longitude, altitude);
							break;
						case 0:
						default:
							val.second = lla_to_ecef(latitude, longitude, altitude);
							break;
						}
					}
				}
			}
			return val;
		}


		/// Check string of prior weights
		std::pair<bool, Vec3> checkPriorWeightsString
		(
			const std::string &sWeights
		)
		{
			std::pair<bool, Vec3> val(true, Vec3::Zero());
			std::vector<std::string> vec_str;
			stl::split(sWeights, ';', vec_str);
			if (vec_str.size() != 3)
			{
				std::cerr << "\n Missing ';' character in prior weights" << std::endl;
				val.first = false;
			}
			// Check that all weight values are valid numbers
			for (size_t i = 0; i < vec_str.size(); ++i)
			{
				double readvalue = 0.0;
				std::stringstream ss;
				ss.str(vec_str[i]);
				if (!(ss >> readvalue)) {
					std::cerr << "\n Used an invalid not a number character in local frame origin" << std::endl;
					val.first = false;
				}
				val.second[i] = readvalue;
			}
			return val;
		}
		//
		// Create the description of an input image dataset for OpenMVG toolsuite
		// - Export a SfM_Data file with View & Intrinsic data
		//
		int imageListing(std::string sImageInputDirFather, std::string sfileDatabaseDir, std::string sImageListingOutputDirFather)
		{

			const std::string sfileDatabase = sfileDatabaseDir + "/sensor_width_camera_database.txt";
			std::string sKmatrix;

			std::string sPriorWeights;
			std::pair<bool, Vec3> prior_w_info(false, Vec3(1.0, 1.0, 1.0));

			int i_User_camera_model = PINHOLE_CAMERA_RADIAL3;

			bool b_Group_camera_model = true;

			int i_GPS_XYZ_method = 0;

			double focal_pixels = -1.0;

			for (int i = 0; i < group_count; i++) {
				char temp_i[2] = { ' ','\0' };
				temp_i[0] = i + 48;
				const std::string str_i = temp_i;
				const std::string sImageInputDir = sImageInputDirFather + "/DJI_" + str_i + "/";
				const std::string sImageListingOutputDir = sImageListingOutputDirFather + "/DJI_" + str_i + "_build/";

				//imageListing for every groups and every blocks
				{
					std::cout << " You called : " << std::endl
						<< "imageListing" << std::endl
						<< "--imageDirectory " << sImageInputDir << std::endl
						<< "--sensorWidthDatabase " << sfileDatabase << std::endl
						<< "--outputDirectory " << sImageListingOutputDir << std::endl
						<< "--focal " << focal_pixels << std::endl
						<< "--intrinsics " << sKmatrix << std::endl
						<< "--camera_model " << i_User_camera_model << std::endl
						<< "--group_camera_model " << b_Group_camera_model << std::endl;
					//check filepath's validity
					{
						if (!stlplus::is_folder(sImageInputDir))
						{
							std::cerr << "sImageInputDir:" << sImageInputDir << "is not a stlplus::folder!\n" << std::endl;
							return false;
						}

						/*if (!stlplus::is_folder(sImageListingOutputDir))
						{
							std::cerr << "sImageListingOutputDir:" << sImageListingOutputDir << "is not a stlplus::folder!\n" << std::endl;
							return false;
						}*/

						if (!stlplus::is_folder(sfileDatabaseDir))
						{
							std::cerr << "sfileDatabaseDir:" << sfileDatabaseDir << "is not a stlplus::folder!\n" << std::endl;
							return false;
						}

						if (!stlplus::is_file(sfileDatabase))
						{
							std::cerr << "sfileDatabase" << sfileDatabase << "is not a stlplus::file!\n" << std::endl;
							return false;
						}
					}

					// Expected properties for each image
					double width = -1, height = -1, focal = -1, ppx = -1, ppy = -1;

					const EINTRINSIC e_User_camera_model = EINTRINSIC(i_User_camera_model);

					if (!stlplus::folder_exists(sImageInputDir))
					{
						std::cerr << "\nThe input directory doesn't exist" << std::endl;
						return EXIT_FAILURE;
					}

					if (sImageListingOutputDir.empty())
					{
						std::cerr << "\nInvalid output directory" << std::endl;
						return EXIT_FAILURE;
					}

					if (!stlplus::folder_exists(sImageListingOutputDir))
					{
						if (!stlplus::folder_create(sImageListingOutputDir))
						{
							std::cerr << "\nCannot create output directory" << std::endl;
							return EXIT_FAILURE;
						}
					}

					if (sKmatrix.size() > 0 &&
						!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
					{
						std::cerr << "\nInvalid K matrix input" << std::endl;
						return EXIT_FAILURE;
					}

					if (sKmatrix.size() > 0 && focal_pixels != -1.0)
					{
						std::cerr << "\nCannot combine -f and -k options" << std::endl;
						return EXIT_FAILURE;
					}

					std::vector<Datasheet> vec_database;
					if (!sfileDatabase.empty())
					{
						if (!parseDatabase(sfileDatabase, vec_database))
						{
							std::cerr
								<< "\nInvalid input database: " << sfileDatabase
								<< ", please specify a valid file." << std::endl;
							return EXIT_FAILURE;
						}
					}

					// Check if prior weights are given
					/*if (cmd.used('P') && !sPriorWeights.empty())
					{
					prior_w_info = checkPriorWeightsString(sPriorWeights);
					}
					else if (cmd.used('P'))
					{
					prior_w_info.first = true;
					}*/

					std::vector<std::string> vec_image = stlplus::folder_files(sImageInputDir);
					std::sort(vec_image.begin(), vec_image.end());

					// Configure an empty scene with Views and their corresponding cameras
					SfM_Data sfm_data;
					sfm_data.s_root_path = sImageInputDir; // Setup main image root_path
					Views & views = sfm_data.views;
					Intrinsics & intrinsics = sfm_data.intrinsics;

					//show progress of imageListing process
					C_Progress_display my_progress_bar(vec_image.size(),
						std::cout, "\n- Image listing -\n");
					std::ostringstream error_report_stream;
					for (std::vector<std::string>::const_iterator iter_image = vec_image.begin();
						iter_image != vec_image.end();
						++iter_image, ++my_progress_bar)
					{
						// Read meta data to fill camera parameter (w,h,focal,ppx,ppy) fields.
						width = height = ppx = ppy = focal = -1.0;

						const std::string sImageFilename = stlplus::create_filespec(sImageInputDir, *iter_image);
						const std::string sImFilenamePart = stlplus::filename_part(sImageFilename);

						// Test if the image format is supported:
						if (openMVG::image::GetFormat(sImageFilename.c_str()) == openMVG::image::Unknown)
						{
							error_report_stream
								<< sImFilenamePart << ": Unkown image file format." << "\n";
							continue; // image cannot be opened
						}

						if (sImFilenamePart.find("mask.png") != std::string::npos
							|| sImFilenamePart.find("_mask.png") != std::string::npos)
						{
							error_report_stream
								<< sImFilenamePart << " is a mask image" << "\n";
							continue;
						}

						ImageHeader imgHeader;
						if (!openMVG::image::ReadImageHeader(sImageFilename.c_str(), &imgHeader))
							continue; // image cannot be read

						width = imgHeader.width;
						height = imgHeader.height;
						ppx = width / 2.0;
						ppy = height / 2.0;


						// Consider the case where the focal is provided manually
						if (sKmatrix.size() > 0) // Known user calibration K matrix
						{
							if (!checkIntrinsicStringValidity(sKmatrix, focal, ppx, ppy))
								focal = -1.0;
						}
						else // User provided focal length value
							if (focal_pixels != -1)
								focal = focal_pixels;

						// If not manually provided or wrongly provided
						if (focal == -1)
						{
							std::unique_ptr<Exif_IO> exifReader(new Exif_IO_EasyExif);
							exifReader->open(sImageFilename);

							const bool bHaveValidExifMetadata =
								exifReader->doesHaveExifInfo()
								&& !exifReader->getModel().empty();

							if (bHaveValidExifMetadata) // If image contains meta data
							{
								const std::string sCamModel = exifReader->getModel();

								// Handle case where focal length is equal to 0
								if (exifReader->getFocal() == 0.0f)
								{
									error_report_stream
										<< stlplus::basename_part(sImageFilename) << ": Focal length is missing." << "\n";
									focal = -1.0;
								}
								else
									// Create the image entry in the list file
								{
									Datasheet datasheet;
									if (getInfo(sCamModel, vec_database, datasheet))
									{
										// The camera model was found in the database so we can compute it's approximated focal length
										const double ccdw = datasheet.sensorSize_;
										focal = std::max(width, height) * exifReader->getFocal() / ccdw;
									}
									else
									{
										error_report_stream
											<< stlplus::basename_part(sImageFilename)
											<< "\" model \"" << sCamModel << "\" doesn't exist in the database" << "\n"
											<< "Please consider add your camera model and sensor width in the database." << "\n";
									}
								}
							}
						}
						// Build intrinsic parameter related to the view
						std::shared_ptr<IntrinsicBase> intrinsic;

						if (focal > 0 && ppx > 0 && ppy > 0 && width > 0 && height > 0)
						{
							// Create the desired camera type
							switch (e_User_camera_model)
							{
							case PINHOLE_CAMERA:
								intrinsic = std::make_shared<Pinhole_Intrinsic>
									(width, height, focal, ppx, ppy);
								break;
							case PINHOLE_CAMERA_RADIAL1:
								intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K1>
									(width, height, focal, ppx, ppy, 0.0); // setup no distortion as initial guess
								break;
							case PINHOLE_CAMERA_RADIAL3:
								intrinsic = std::make_shared<Pinhole_Intrinsic_Radial_K3>
									(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0);  // setup no distortion as initial guess
								break;
							case PINHOLE_CAMERA_BROWN:
								intrinsic = std::make_shared<Pinhole_Intrinsic_Brown_T2>
									(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
								break;
							case PINHOLE_CAMERA_FISHEYE:
								intrinsic = std::make_shared<Pinhole_Intrinsic_Fisheye>
									(width, height, focal, ppx, ppy, 0.0, 0.0, 0.0, 0.0); // setup no distortion as initial guess
								break;
							case CAMERA_SPHERICAL:
								intrinsic = std::make_shared<Intrinsic_Spherical>
									(width, height);
								break;
							default:
								std::cerr << "Error: unknown camera model: " << (int)e_User_camera_model << std::endl;
								return EXIT_FAILURE;
							}
						}

						// Build the view corresponding to the image
						const std::pair<bool, Vec3> gps_info = checkGPS(sImageFilename);
						//if (gps_info.first && cmd.used('P'))
						if (gps_info.first && 0)
						{
							ViewPriors v(*iter_image, views.size(), views.size(), views.size(), width, height);

							// Add intrinsic related to the image (if any)
							if (intrinsic == nullptr)
							{
								//Since the view have invalid intrinsic data
								// (export the view, with an invalid intrinsic field value)
								v.id_intrinsic = UndefinedIndexT;
							}
							else
							{
								// Add the defined intrinsic to the sfm_container
								intrinsics[v.id_intrinsic] = intrinsic;
							}

							v.b_use_pose_center_ = true;
							v.pose_center_ = gps_info.second;
							// prior weights
							if (prior_w_info.first == true)
							{
								v.center_weight_ = prior_w_info.second;
							}

							// Add the view to the sfm_container
							views[v.id_view] = std::make_shared<ViewPriors>(v);
						}
						else
						{
							View v(*iter_image, views.size(), views.size(), views.size(), width, height);

							// Add intrinsic related to the image (if any)
							if (intrinsic == nullptr)
							{
								//Since the view have invalid intrinsic data
								// (export the view, with an invalid intrinsic field value)
								v.id_intrinsic = UndefinedIndexT;
							}
							else
							{
								// Add the defined intrinsic to the sfm_container
								intrinsics[v.id_intrinsic] = intrinsic;
							}

							// Add the view to the sfm_container
							views[v.id_view] = std::make_shared<View>(v);
						}
					}

					// Display saved warning & error messages if any.
					if (!error_report_stream.str().empty())
					{
						std::cerr
							<< "\nWarning & Error messages:" << std::endl
							<< error_report_stream.str() << std::endl;
					}

					// Group camera that share common properties if desired (leads to more faster & stable BA).
					if (b_Group_camera_model)
					{
						GroupSharedIntrinsics(sfm_data);
					}

					// Store SfM_Data views & intrinsic data
					if (!Save(
						sfm_data,
						stlplus::create_filespec(sImageListingOutputDir, "sfm_data.json").c_str(),
						ESfM_Data(VIEWS | INTRINSICS)))
					{
						return EXIT_FAILURE;
					}

					std::cout << std::endl
						<< "SfMInit_ImageListing report:\n"
						<< "listed #File(s): " << vec_image.size() << "\n"
						<< "usable #File(s) listed in sfm_data: " << sfm_data.GetViews().size() << "\n"
						<< "usable #Intrinsic(s) listed in sfm_data: " << sfm_data.GetIntrinsics().size() << std::endl;

					
				}
				
			}
			return EXIT_SUCCESS;
		}

	};

}//namespace imageListing

#endif // !_imageListing_H

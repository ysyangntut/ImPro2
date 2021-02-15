#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>

#include "impro_util.h"

#include "FileSeq.h"

using namespace std;

const cv::String keys =
"{help          h usage ? |   | print this message   }"
"{fileseq  fsq            |   | full path of the file sequence file (file must be in same path with photo files): . }"
"{omfile                  |   | full path of output matlab file.}" 
;

int FuncOptflowSeq(int argc, char** argv)
{
	// variables
	string fsq_str;
	string omfile_str;
	FileSeq fsq;
	cv::Size imgSize; 
	cv::Mat imgPrev, imgCurr; 
	cv::Mat flow, exx, eyy, exy, crack_opening, crack_sliding;

	// Step 0: Build parser 
	cv::CommandLineParser parser(argc, argv, keys);
	if (parser.has("fileseq"))
		fsq_str = parser.get<string>("fileseq");
	else
	{
		fsq.setDirFilesByConsole();
	}

	// Step 1: File output setting
	//if (parser.has("omfile"))
	//	omfile_str = parser.get<string>("omfile");
	//else {
	//	cout << "Input full path of output matlab file (input xxx means xxx:\n";
	//	omfile_str = readStringFromIstream(std::cin);
	//}

	// Step 2: Optical flow settings 

	   
	// Step 3: While loop
	printf("There are %d files, from %s to %s.\n", fsq.num_files(), fsq.filename(0).c_str(), fsq.filename(fsq.num_files() - 1).c_str());

	// Step 4: Get image size
	cout << "Waiting for first photo " << fsq.filename(0) << "...";
	fsq.waitForFile(0);
	cout << "Got it.";
	imgPrev = cv::imread(fsq.fullPathOfFile(0), cv::IMREAD_GRAYSCALE);
	imgSize.width = imgPrev.cols;
	imgSize.height = imgPrev.rows;
	printf(" image size (%d x %d).\n", imgSize.height, imgSize.width); 

	// Step 5: Run loop
	for (int iPhoto = 1; iPhoto < fsq.num_files(); iPhoto++)
	{
		// read image
		imgCurr = cv::imread(fsq.fullPathOfFile(iPhoto), cv::IMREAD_GRAYSCALE); 

		// optical flow
		double pyr_scale = 0.5; 
		int level = 5; 
		int winsize = 151;
		int iterations = 10; 
		int poly_n = 5; // Typically, polyN = 5 or 7.
		double poly_sigma = 1.1; // For polyN=5 , you can set polySigma=1.1 . For polyN=7 , a good value would be polySigma=1.5 .
		int flags = cv::OPTFLOW_USE_INITIAL_FLOW | cv::OPTFLOW_FARNEBACK_GAUSSIAN; 
		cv::calcOpticalFlowFarneback(imgPrev, imgCurr, flow, pyr_scale, level,
			winsize, iterations, poly_n, poly_sigma, flags);

		// resize flow to a (much) smaller size
//		cv::resize(flow, flow, cv::Size(0, 0), 0.5, 0.5, cv::INTER_LINEAR); 

		printf("Opt flow of frame %d is sized %dx%d in type %d.\n", iPhoto, flow.rows, flow.cols, flow.type()); 
		if (flow.type() == CV_32FC2)
		{
			float ux_sum = 0.f, uy_sum = 0.f, ux_s2 = 0.f, uy_s2 = 0.f;
			float ux_max, ux_min, uy_max, uy_min; 
			float ux_avg, uy_avg, ux_std, uy_std;
			ux_max = ux_min = flow.at<cv::Point2f>(0, 0).x;
			uy_max = uy_min = flow.at<cv::Point2f>(0, 0).y;
			for (int i = 0; i < flow.rows; i++)
			{
				for (int j = 0; j < flow.cols; j++)
				{
					float ux = flow.at<cv::Point2f>(i, j).x;
					float uy = flow.at<cv::Point2f>(i, j).y;
					ux_sum += ux; 
					uy_sum += uy; 
					ux_s2 += ux * ux; 
					uy_s2 += uy * uy;
					if (ux > ux_max) ux_max = ux; 
					if (ux < ux_min) ux_min = ux; 
					if (uy > uy_max) uy_max = uy;
					if (uy < uy_min) uy_min = uy;
				}
			}
			ux_avg = ux_sum / (flow.rows * flow.cols); 
			uy_avg = uy_sum / (flow.rows * flow.cols);
			ux_std = sqrt(ux_s2 / (flow.rows * flow.cols) - ux_avg * ux_avg); 
			uy_std = sqrt(uy_s2 / (flow.rows * flow.cols) - uy_avg * uy_avg);
			printf("  Ux/Uy max are: %12.4e %12.4e\n", ux_max, uy_max);
			printf("  Ux/Uy min are: %12.4e %12.4e\n", ux_min, uy_min);
			printf("  Ux/Uy avg are: %12.4e %12.4e\n", ux_avg, uy_avg);
			printf("  Ux/Uy std are: %12.4e %12.4e\n", ux_std, uy_std);

			// convert flow to ux and uy to images img_ux and img_uy (in Jet-256 colormap)
			float u_color_max = .5f; 
			float u_color_min = -.5f;
			cv::Mat img_ux(flow.rows, flow.cols, CV_8UC3);
			cv::Mat img_uy(flow.rows, flow.cols, CV_8UC3);
			for (int i = 0; i < flow.rows; i++)
			{
				for (int j = 0; j < flow.cols; j++)
				{
					float ux = flow.at<cv::Point2f>(i, j).x;
					float uy = flow.at<cv::Point2f>(i, j).y;
					int ux_256_i = (int) (255 * ((ux - u_color_min) / (u_color_max - u_color_min)) + .5f);
					int uy_256_i = (int) (255 * ((uy - u_color_min) / (u_color_max - u_color_min)) + .5f);
					unsigned char ux256 = min(255, max(0, ux_256_i)); 
					unsigned char uy256 = min(255, max(0, uy_256_i));
					img_ux.at<cv::Vec3b>(i, j).val[0] = jet_bgr[ux256][0];
					img_ux.at<cv::Vec3b>(i, j).val[1] = jet_bgr[ux256][1];
					img_ux.at<cv::Vec3b>(i, j).val[2] = jet_bgr[ux256][2];
					img_uy.at<cv::Vec3b>(i, j).val[0] = jet_bgr[uy256][0];
					img_uy.at<cv::Vec3b>(i, j).val[1] = jet_bgr[uy256][1];
					img_uy.at<cv::Vec3b>(i, j).val[2] = jet_bgr[uy256][2];
				}
			}
			// print text of max/max values in the images
			char buf[1000];
            snprintf(buf, 1000, "Max(red)/Min(blue): %12.4e %12.4e", u_color_max, u_color_min);
			cv::putText(img_ux, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2); 
            snprintf(buf, 1000, "Max(red)/Min(blue): %12.4e %12.4e", u_color_max, u_color_min);
			cv::putText(img_uy, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
			// write images to files
			string ux_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_ux.JPG"; 
			string uy_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_uy.JPG";
			cv::imwrite(ux_fname, img_ux); 
			cv::imwrite(uy_fname, img_uy); 

			//// flow to strain
			uToStrain(flow, exx, eyy, exy); 
			float exx_sum = 0.f, exx_s2 = 0.f, exx_max, exx_min, exx_avg, exx_std;
			float eyy_sum = 0.f, eyy_s2 = 0.f, eyy_max, eyy_min, eyy_avg, eyy_std;
			float exy_sum = 0.f, exy_s2 = 0.f, exy_max, exy_min, exy_avg, exy_std;
			exx_max = exx_min = exx.at<float>(0, 0);
			eyy_max = eyy_min = eyy.at<float>(0, 0);
			exy_max = exy_min = exy.at<float>(0, 0);
			for (int i = 0; i < flow.rows; i++)
			{
				for (int j = 0; j < flow.cols; j++)
				{
					float exx_ij = exx.at<float>(i, j);
					float eyy_ij = eyy.at<float>(i, j);
					float exy_ij = exy.at<float>(i, j);
					exx_sum += exx_ij;
					eyy_sum += eyy_ij;
					exy_sum += exy_ij;
					exx_s2 += exx_ij * exx_ij;
					eyy_s2 += eyy_ij * eyy_ij;
					exy_s2 += exy_ij * exy_ij;
					if (exx_ij > exx_max) exx_max = exx_ij;
					if (eyy_ij > eyy_max) eyy_max = eyy_ij;
					if (exy_ij > exy_max) exy_max = exy_ij;
					if (exx_ij < exx_min) exx_min = exx_ij;
					if (eyy_ij < eyy_min) eyy_min = eyy_ij;
					if (exy_ij < exy_min) exy_min = exy_ij;
				}
			}
			exx_avg = exx_sum / (flow.rows * flow.cols);
			eyy_avg = eyy_sum / (flow.rows * flow.cols);
			exy_avg = exy_sum / (flow.rows * flow.cols);
			exx_std = sqrt(exx_s2 / (flow.rows * flow.cols) - exx_avg * exx_avg);
			eyy_std = sqrt(eyy_s2 / (flow.rows * flow.cols) - eyy_avg * eyy_avg);
			exy_std = sqrt(exy_s2 / (flow.rows * flow.cols) - exy_avg * exy_avg);
			printf("  Exx/Eyy/Exy max are: %12.4e %12.4e %12.4e\n", exx_max, eyy_max, exy_max);
			printf("  Exx/Eyy/Exy min are: %12.4e %12.4e %12.4e\n", exx_min, eyy_min, exy_min);
			printf("  Exx/Eyy/Exy avg are: %12.4e %12.4e %12.4e\n", exx_avg, eyy_avg, exy_avg);
			printf("  Exx/Eyy/Exy std are: %12.4e %12.4e %12.4e\n", exx_std, eyy_std, exy_std);

			//// convert strain to images img_exx, img_eyy, img_exy (in Jet-256 colormap)
			u_color_max =  .005f;
			u_color_min = -.005f;
			cv::Mat img_exx(exx.rows, exx.cols, CV_8UC3);
			cv::Mat img_eyy(eyy.rows, eyy.cols, CV_8UC3);
			cv::Mat img_exy(exy.rows, exy.cols, CV_8UC3);
			for (int i = 0; i < img_exx.rows; i++)
			{
				for (int j = 0; j < img_exx.cols; j++)
				{
					float exx_ij = exx.at<float>(i, j);
					float eyy_ij = eyy.at<float>(i, j);
					float exy_ij = exy.at<float>(i, j);
					int exx_256_i = (int)(255 * ((exx_ij - u_color_min) / (u_color_max - u_color_min)) + .5f);
					int eyy_256_i = (int)(255 * ((eyy_ij - u_color_min) / (u_color_max - u_color_min)) + .5f);
					int exy_256_i = (int)(255 * ((exy_ij - u_color_min) / (u_color_max - u_color_min)) + .5f);
					unsigned char exx256 = min(255, max(0, exx_256_i));
					unsigned char eyy256 = min(255, max(0, eyy_256_i));
					unsigned char exy256 = min(255, max(0, exy_256_i));
					for (int ch = 0; ch < 3; ch++)
					{
						img_exx.at<cv::Vec3b>(i, j).val[ch] = jet_bgr[exx256][ch];
						img_eyy.at<cv::Vec3b>(i, j).val[ch] = jet_bgr[eyy256][ch];
						img_exy.at<cv::Vec3b>(i, j).val[ch] = jet_bgr[exy256][ch];
					}
				}
			}
			// print text of max/max values in the images
            snprintf(buf, 1000, "Max(red)/Min(blue): %12.4e %12.4e", u_color_max, u_color_min);
			cv::putText(img_exx, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
			cv::putText(img_eyy, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
			cv::putText(img_exy, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
			// write images to files
			string exx_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_exx.JPG";
			string eyy_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_eyy.JPG";
			string exy_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_exy.JPG";
			cv::imwrite(exx_fname, img_exx);
			cv::imwrite(eyy_fname, img_eyy);
			cv::imwrite(exy_fname, img_exy);
			//string opn_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_cr_opn.JPG";
			//string sld_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_cr_sld.JPG";
			//cv::imwrite(opn_fname, img_cr_opn);
			//cv::imwrite(sld_fname, img_cr_sld);

			// flow to crack
			uToCrack(flow, crack_opening, crack_sliding, 999);
			float opn_sum = 0.f, sld_sum = 0.f, opn_s2 = 0.f, sld_s2 = 0.f;
			float opn_max, opn_min, sld_max, sld_min;
			float opn_avg, sld_avg, opn_std, sld_std;
			opn_max = opn_min = crack_opening.at<float>(0, 0);
			sld_max = sld_min = crack_sliding.at<float>(0, 0);
			for (int i = 0; i < flow.rows; i++)
			{
				for (int j = 0; j < flow.cols; j++)
				{
					float opn = crack_opening.at<float>(i, j);
					float sld = crack_sliding.at<float>(i, j);
					opn_sum += opn;
					sld_sum += sld;
					opn_s2 += opn * opn;
					sld_s2 += sld * sld;
					if (opn > opn_max) opn_max = opn;
					if (opn < opn_min) opn_min = opn;
					if (sld > sld_max) sld_max = sld;
					if (sld < sld_min) sld_min = sld;
				}
			}
			opn_avg = opn_sum / (flow.rows * flow.cols);
			sld_avg = sld_sum / (flow.rows * flow.cols);
			opn_std = sqrt(opn_s2 / (flow.rows * flow.cols) - opn_avg * opn_avg);
			sld_std = sqrt(sld_s2 / (flow.rows * flow.cols) - sld_avg * sld_avg);
			printf("  Opn/Sld max are: %12.4e %12.4e\n", opn_max, sld_max);
			printf("  Opn/Sld min are: %12.4e %12.4e\n", opn_min, sld_min);
			printf("  Opn/Sld avg are: %12.4e %12.4e\n", opn_avg, sld_avg);
			printf("  Opn/Sld std are: %12.4e %12.4e\n", opn_std, sld_std);
			// convert crack_opening/sliding to images img_cr_opn and img_cr_sld (in Jet-256 colormap)
			u_color_max = .1f;
			u_color_min = -.1f;
			cv::Mat img_cr_opn(crack_opening.rows, crack_opening.cols, CV_8UC3);
			cv::Mat img_cr_sld(crack_sliding.rows, crack_sliding.cols, CV_8UC3);
			for (int i = 0; i < img_cr_opn.rows; i++)
			{
				for (int j = 0; j < img_cr_opn.cols; j++)
				{
					float opn = crack_opening.at<float>(i, j);
					float sld = crack_sliding.at<float>(i, j);
					int opn_256_i = (int)(255 * ((opn - u_color_min) / (u_color_max - u_color_min)) + .5f);
					int sld_256_i = (int)(255 * ((sld - u_color_min) / (u_color_max - u_color_min)) + .5f);
					unsigned char opn256 = min(255, max(0, opn_256_i));
					unsigned char sld256 = min(255, max(0, sld_256_i));
					img_cr_opn.at<cv::Vec3b>(i, j).val[0] = jet_bgr[opn256][0];
					img_cr_opn.at<cv::Vec3b>(i, j).val[1] = jet_bgr[opn256][1];
					img_cr_opn.at<cv::Vec3b>(i, j).val[2] = jet_bgr[opn256][2];
					img_cr_sld.at<cv::Vec3b>(i, j).val[0] = jet_bgr[sld256][0];
					img_cr_sld.at<cv::Vec3b>(i, j).val[1] = jet_bgr[sld256][1];
					img_cr_sld.at<cv::Vec3b>(i, j).val[2] = jet_bgr[sld256][2];
				}
			}
			// print text of max/max values in the images
            snprintf(buf, 1000, "Max(red)/Min(blue): %12.4e %12.4e", u_color_max, u_color_min);
			cv::putText(img_cr_opn, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
            snprintf(buf, 1000, "Max(red)/Min(blue): %12.4e %12.4e", u_color_max, u_color_min);
			cv::putText(img_cr_sld, buf, cv::Point(100, 100), 0, 3, cv::Scalar(0, 0, 0), 2);
			// write images to files
			string opn_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_cr_opn.JPG";
			string sld_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_cr_sld.JPG";
			cv::imwrite(opn_fname, img_cr_opn);
			cv::imwrite(sld_fname, img_cr_sld);

			// write fields to files (_ux.m, _uy.m, _cr_opn.m, _cr_sld.m, _exx.m, _eyy.m, _exy.m)
			string ux__m_fname = extFilenameRemoved(fsq.fullPathOfFile(iPhoto)) + "_result_fields.m";
			FILE * if_fields_m; // = fopen(ux__m_fname.c_str(), "w");
			int if_fields_ok = fopen_s(&if_fields_m, ux__m_fname.c_str(), "w");
			
			if (if_fields_ok == 0) {
				// write ux
				fprintf(if_fields_m, "ux=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", flow.at<cv::Point2f>(i, j).x);
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write uy
				fprintf(if_fields_m, "uy=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", flow.at<cv::Point2f>(i, j).y);
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write opn
				fprintf(if_fields_m, "opn=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", crack_opening.at<float>(i, j));
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write sld
				fprintf(if_fields_m, "sld=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", crack_sliding.at<float>(i, j));
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write exx
				fprintf(if_fields_m, "exx=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", exx.at<float>(i, j));
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write eyy
				fprintf(if_fields_m, "eyy=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", eyy.at<float>(i, j));
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				// write exy
				fprintf(if_fields_m, "exy=[");
				for (int i = 0; i < flow.rows; i++) {
					for (int j = 0; j < flow.cols; j++)
						fprintf(if_fields_m, "%11.4f ", exy.at<float>(i, j));
					fprintf(if_fields_m, ";");
				}
				fprintf(if_fields_m, "];\n");
				fprintf(if_fields_m, "figure('name','ux '); imagesc(ux ); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','uy '); imagesc(uy ); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','opn'); imagesc(opn); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','sld'); imagesc(sld); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','exx'); imagesc(exx); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','eyy'); imagesc(eyy); colormap('jet'); axis image; colorbar;\n");
				fprintf(if_fields_m, "figure('name','exy'); imagesc(exy); colormap('jet'); axis image; colorbar;\n");

				fclose(if_fields_m);
			} // if m-file is opened
		} // end of if type() is CV_32FC2
	}

	// output to matlab script

	// end of function 
	return 0;
}

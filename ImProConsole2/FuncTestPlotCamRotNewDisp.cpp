#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>


int FuncTestPlotCamRotNewDisp(int argc, char** argv)
{
	cv::Mat newDisp(3, 1, CV_64F), camRot(3, 1, CV_64F);
	for (int i = 0; i < 10000; i++)
	{
		camRot.at<double>(0) = 0.1 * sin(i * 1.0 / 120.0 * 2 * 3.1416);
		camRot.at<double>(1) = 0.1 * cos(i * 1.0 / 120.0 * 2 * 3.1416);
		camRot.at<double>(2) = 0.1 * cos(i * 1.0 / 120.0 * 2 * 3.1416);
		newDisp.at<double>(0) = 8.0 * sin(i * 1.0 / 120.0 * 2 * 3.1416);
		newDisp.at<double>(1) = 8.0 * cos(i * 1.0 / 120.0 * 2 * 3.1416);
		newDisp.at<double>(2) = 8.0 * cos(i * 1.0 / 120.0 * 2 * 3.1416);

		// plot
		bool plot = true;
		float camRotPlotMax[3] = { 0.1f, 0.1f, 0.1f };
		float camRotPlotMin[3] = { -.1f, -.1f, -.1f };
		float newDispPlotMax[3] = { 10.f, 10.f, 10.f };
		float newDispPlotMin[3] = { -10.f, -10.f, -10.f };
		int plotWidth = 1000;
		int plotHeight = 200;
		static std::vector<cv::Mat> plotCamRot(3, cv::Mat(plotHeight, plotWidth, CV_8UC3, cv::Scalar(255, 255, 255)));
		static std::vector<cv::Mat> plotNewDisp(3, cv::Mat(plotHeight, plotWidth, CV_8UC3, cv::Scalar(255, 255, 255)));
		std::vector<std::string> plotCamRotName = { "CamRot X", "CamRot Y", "CamRot Z" };
		std::vector<std::string> plotNewDispName = { "Disp X", "Disp Y", "Disp Z" };
		char buf[1000];
		for (int i = 0; i < 3; i++) {
			snprintf(buf, 1000, ".   Max:%6.2f  Min:%6.2f", camRotPlotMax[i], camRotPlotMin[i]);
			plotCamRotName[i] += std::string(buf);
			snprintf(buf, 1000, ".   Max:%6.2f  Min:%6.2f", newDispPlotMax[i], newDispPlotMin[i]);
			plotNewDispName[i] += std::string(buf);
		}

		if (plot == true)
		{
			static int iPlot = 0;
			// Plot 3 camRot plots
			for (int i = 0; i < 3; i++)
			{
				// clear pixel of this iPlot 
				for (int clearX = 0; clearX < 5; clearX++) {
					if (iPlot + clearX < plotWidth)
					{
						if (clearX == 5 - 1)
							cv::line(plotCamRot[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(128, 128, 128), 1, 8);
						else
							cv::line(plotCamRot[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(255, 255, 255), 1, 8);
					}
				}
				// plot axis
				float y = 0.0f;
				float maxY = camRotPlotMax[i];
				float minY = camRotPlotMin[i];
				int iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
				if (iy < 0) iy = 0;
				if (iy >= plotHeight) iy = plotHeight - 1;
				cv::line(plotCamRot[i], cv::Point(0, iy), cv::Point(plotWidth - 1, iy), cv::Scalar(128, 128, 128), 1, 8);
				// plot data point
				y = (float)camRot.at<double>(i);
				iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
				if (iy < 0) iy = 0;
				if (iy >= plotHeight) iy = plotHeight - 1;
				cv::line(plotCamRot[i], cv::Point(iPlot, iy), cv::Point(iPlot, iy), cv::Scalar(0, 0, 0), 1, 8);
			}
			// Plot 3 newDisp plots
			for (int i = 0; i < 3; i++)
			{
				// clear pixel of this iPlot 
				for (int clearX = 0; clearX < 5; clearX++) {
					if (iPlot + clearX < plotWidth)
					{
						if (clearX == 5 - 1)
							cv::line(plotNewDisp[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(128, 128, 128), 1, 8);
						else
							cv::line(plotNewDisp[i], cv::Point(iPlot + clearX, 0), cv::Point(iPlot + clearX, plotHeight - 1), cv::Scalar(255, 255, 255), 1, 8);
					}
				}
				// plot axis
				float y = 0.0f;
				float maxY = newDispPlotMax[i];
				float minY = newDispPlotMin[i];
				int iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
				if (iy < 0) iy = 0;
				if (iy >= plotHeight) iy = plotHeight - 1;
				cv::line(plotNewDisp[i], cv::Point(0, iy), cv::Point(plotWidth - 1, iy), cv::Scalar(128, 128, 128), 1, 8);
				// plot data point
				y = (float)newDisp.at<double>(i);
				iy = 0 + (int)(plotHeight * (maxY - y) / (maxY - minY) + 0.5f);
				if (iy < 0) iy = 0;
				if (iy >= plotHeight) iy = plotHeight - 1;
				cv::line(plotNewDisp[i], cv::Point(iPlot, iy), cv::Point(iPlot, iy), cv::Scalar(0, 0, 0), 1, 8);
			}
			// show image
			for (int i = 0; i < 3; i++)
				cv::imshow(plotCamRotName[i], plotCamRot[i]);
			for (int i = 0; i < 3; i++)
			 	cv::imshow(plotNewDispName[i], plotNewDisp[i]);
			int ikey = cv::waitKey(1);
			if (ikey == 'q' || ikey == 27 || ikey == 13)
				break;

			iPlot++;
			iPlot = iPlot % plotWidth;
		} // end of if plot == true	
	}

	cv::destroyAllWindows();
	return 0;
}

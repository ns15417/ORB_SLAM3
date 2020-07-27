/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;



int main(int argc, char **argv)
{  
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_ocv_sync path_to_vocabulary path_to_settings left_cam_id right_cam_id add_imu" << endl;
        return 1;
    }

    std::cout << "setting: " << argv[2] << endl;
    int camid_left = std::atoi(argv[3]);
    int camid_right = std::atoi(argv[4]);
    bool add_imu = std::atoi(argv[5]);
    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::VideoCapture capl(camid_left);
    cv::VideoCapture capr(camid_right);
    capl.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    capl.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capr.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    capr.set(CV_CAP_PROP_FRAME_WIDTH,640);
#ifdef RECTIFY
    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
#endif

    cout << endl << "-------" << endl;
    cout.precision(17);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::STEREO, true);

    cv::Mat imLeft, imRight, imLeftRect, imRightRect;
    while(capl.grab() && capr.grab())
    {
        capl.retrieve(imLeft);
        capr.retrieve(imRight);

            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: " << endl;
                return 1;
            }

            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "<< endl;
                return 1;
            }

    #ifdef RECTIFY
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Rect = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Rect = std::chrono::monotonic_clock::now();
    #endif
            cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
            cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Rect = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Rect = std::chrono::monotonic_clock::now();
    #endif

            t_rect = std::chrono::duration_cast<std::chrono::duration<double> >(t_End_Rect - t_Start_Rect).count();
            double tframe = vTimestampsCam[seq][ni];
    #endif

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif
            double tframe = 0.0;
            std::string fakestringname = "/home/shinan/Project/Calibration/210_new25.png";
            // Pass the images to the SLAM system
            SLAM.TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(),fakestringname);

    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    }
    // Stop all threads
    SLAM.Shutdown();

    bool bFileName =false;
    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

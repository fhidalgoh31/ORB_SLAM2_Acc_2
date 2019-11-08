/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Parameter.h"
#include "System.h"
#include "pangolin/pangolin.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <signal.h>
#include <cstdlib>
#include <opencv2/core/core.hpp>

using namespace std;
bool shutdown_flag=0;
void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
void kill_handler(int s);

int main(int argc, char **argv)
{
    int beg_Images, end_Images;
    if(argc < 4)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }
    if(argc>4)
        beg_Images=atoi(argv[4]);
    if(argc>5)
        end_Images=atoi(argv[5]);



    DLOG(INFO) << "Logging is active.";

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();
    ORB_SLAM2::SystemLogger::totalFrameCount = nImages;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    SLAM.setFrameCount(nImages);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    //handler kill signal
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = kill_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);


    // Main loop
    pangolin::Var<bool> pause("menu.Pause", false,  true);
    pangolin::Var<bool> nextFrame("menu.Next frame", false, false);
    pangolin::Var<std::string> fastForward("menu.Fast forward", "0");
    int forwardCounter = 0;
    cv::Mat im;
    int ni=0;
    if (beg_Images>1)
        ni=beg_Images;
    for(ni; ni<nImages; ni++)
    {
        ORB_SLAM2::SystemLogger::currentFrameNum = ni + 1;

        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }
        int fastForwardVar = std::stoi(fastForward.Get());
        if(fastForwardVar != 0)
        {
            LOG(INFO) << "Fast forwarding by " << fastForwardVar << " frames.";
            SLAM.ignoreFPS(true);
            forwardCounter = fastForwardVar;
            fastForward.operator=("0");
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        bool frame_played = false;
        while(pause.Get())
        {
            if(nextFrame.Get())
            {
                SLAM.TrackMonocular(im,tframe);
                frame_played = true;
                nextFrame.operator=(false);
                break;
            }

            usleep(10000);
        }
        if(!frame_played)
        {
            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe);
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        if(forwardCounter != 0)
        {
            // Go as fast as you can
            forwardCounter--;   //ZZZ
            if(forwardCounter == 0)
            {
                LOG(INFO) << "Fast forwarding done.";
                SLAM.ignoreFPS(false);
            }
        }
        else
        {
            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }

        if(shutdown_flag)
             {cout << "shutdown" << endl;
            //   SLAM.ReleaseVideo();      
              }
        
        if (ni>end_Images)
            {
                pause = true;
                SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
            }
    }
    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    std::cout << "Press Enter to continue" << std::endl;
    if((char)cv::waitKey(1) == 13) std::cout << "Enter pressed" << std::endl;  // Pause ZZZ
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
            std::cout<<ss.str();            //zzz
        }
    }
}

void kill_handler(int s){
    shutdown_flag=1;
    printf("Caught signal %d\n",s);
    exit(1); 

}

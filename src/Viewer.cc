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

#include "Viewer.h"
#include "Parameter.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
    , mIgnoreFPS(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    // standart view by orb slam
    auto& menuPanel = pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    ParameterManager::createPangolinEntries("menu", ParameterGroup::VISUAL);

    // create the panels for the sub parameters
    std::map<std::string, pangolin::View*> subPanels;
    auto& extractorPanel = pangolin::CreatePanel("extractor").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("extractor", ParameterGroup::ORBEXTRACTOR);
    subPanels["extractor"] = (&extractorPanel);
    extractorPanel.ToggleShow();

    auto& initializePanel = pangolin::CreatePanel("initialization").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("initialization", ParameterGroup::INITIALIZATION);
    subPanels["initialization"] = (&initializePanel);
    initializePanel.ToggleShow();

    auto& trackingPanel = pangolin::CreatePanel("tracking").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("tracking", ParameterGroup::TRACKING);
    subPanels["tracking"] = &trackingPanel;
    trackingPanel.ToggleShow();

    auto& relocalizationPanel = pangolin::CreatePanel("relocalization").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("relocalization", ParameterGroup::RELOCALIZATION);
    subPanels["relocalization"] = &relocalizationPanel;
    relocalizationPanel.ToggleShow();

    auto& localMappingPanel = pangolin::CreatePanel("localMapping").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("localMapping", ParameterGroup::LOCAL_MAPPING);
    subPanels["localMapping"] = &localMappingPanel;
    localMappingPanel.ToggleShow();

    auto& loopClosingPanel = pangolin::CreatePanel("loopClosing").SetBounds(0.0,1.0,pangolin::Attach::Pix(-200),1.0);
    ParameterManager::createPangolinEntries("loopClosing", ParameterGroup::LOOP_CLOSING);
    subPanels["loopClosing"] = &loopClosingPanel;
    loopClosingPanel.ToggleShow();

    // create a toggle function for each panel
    auto toggleBoundsCamera = [&](bool showingParamPanel){
        if (!showingParamPanel)
        {
            d_cam.SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f);
        }
        else
        {
            d_cam.SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), pangolin::Attach::Pix(-200), -1024.0f/768.0f);
        }
    };

    std::map<std::string, std::function<void(void)> > toggleFunctionMap;
    for(std::map<std::string, pangolin::View*>::iterator it = subPanels.begin(); it != subPanels.end(); it++)
    {
        toggleFunctionMap[it->first] = [=]{
            for(std::map<std::string,pangolin::View*>::const_iterator it2 = subPanels.begin(); it2 != subPanels.end(); it2++)
            {
                if(it2->first == it->first)
                {
                    if(it2->second->IsShown())
                    {
                        it2->second->ToggleShow();
                        toggleBoundsCamera(false);
                    }
                    else
                    {
                        it2->second->ToggleShow();
                        toggleBoundsCamera(true);
                    }
                }
                else
                {
                    if(it2->second->IsShown())
                    {
                        it2->second->ToggleShow();
                    }
                }
            }
        };
    }

    // parameter view shows buttons which open parameters for every subcategory
    auto& parameterPanel = pangolin::CreatePanel("parameters").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(200));
    pangolin::Var<std::function<void(void)>> extractorButton("parameters.Extractor", toggleFunctionMap["extractor"]);
    pangolin::Var<std::function<void(void)>> initializationButton("parameters.Initialization", toggleFunctionMap["initialization"]);
    pangolin::Var<std::function<void(void)>> trackingButton("parameters.Tracking", toggleFunctionMap["tracking"]);
    pangolin::Var<std::function<void(void)>> relocalizationButton("parameters.Relocalization", toggleFunctionMap["relocalization"]);
    pangolin::Var<std::function<void(void)>> localMappingButton("parameters.Local mapping", toggleFunctionMap["localMapping"]);
    pangolin::Var<std::function<void(void)>> loopClosingButton("parameters.Loop closing", toggleFunctionMap["loopClosing"]);
    ParameterManager::createPangolinEntries("parameters", ParameterGroup::GENERAL);
    parameterPanel.ToggleShow();

    // open parameter pane when pressing Ctrl+p
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'p', [&]{
                                                                          menuPanel.ToggleShow();
                                                                          parameterPanel.ToggleShow();
                                                                      });
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'k', [&]{
                                                                          menuPanel.ToggleShow();
                                                                      });
    bool toggle = false;
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'b', [&]{
            ParameterManager::getParameter<bool>(ParameterGroup::GENERAL, "menu.Pause")->setValue(!toggle);
            if(toggle)
            {
                d_cam.SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1024.0f/768.0f);
            }
            else
            {
                d_cam.SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f/768.0f);
            }
            toggle = !toggle;
                                                                      });

    // // open inidividual parameter pane when pressing Ctrl+letter
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'a', toggleFunctionMap["extractor"]);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 's', toggleFunctionMap["initialization"]);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'd', toggleFunctionMap["tracking"]);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'f', toggleFunctionMap["relocalization"]);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'g', toggleFunctionMap["localMapping"]);
    pangolin::RegisterKeyPressCallback(pangolin::PANGO_CTRL + 'h', toggleFunctionMap["loopClosing"]);

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    bool bFollow = true;
    bool bLocalizationMode = false;

    cv::namedWindow("ORB-SLAM2: Current Frame", CV_WINDOW_NORMAL);
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints();

        pangolin::FinishFrame();

        cv::Mat im = mpFrameDrawer->DrawFrame();
        cv::imshow("ORB-SLAM2: Current Frame",im);
        mIgnoreFPS ? cv::waitKey(1) : cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            LOG_SCOPE("System");
            DLOG(STATUS) << "RESET!";
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}


void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

void Viewer::ignoreFPS(const bool& ignore)
{
    mIgnoreFPS = ignore;
}

}

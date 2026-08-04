#include <spdlog/spdlog.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps);

int main(int argc, char **argv)

{  
    spdlog::set_level(spdlog::level::debug);
    if(argc < 5)
    {
        cerr << endl << "Usage: ./stereo_euroc path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;

        return 1;
    }

    const int num_seq = (argc-3)/2;
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-3) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-1]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    vector< vector<string> > vstrImageLeft;
    vector< vector<string> > vstrImageRight;
    vector< vector<double> > vTimestampsCam;
    vector<int> nImages;

    vstrImageLeft.resize(num_seq);
    vstrImageRight.resize(num_seq);
    vTimestampsCam.resize(num_seq);
    nImages.resize(num_seq);

    int tot_images = 0;
    for (seq = 0; seq<num_seq; seq++)
    {
        cout << "Loading images for sequence " << seq << "...";
        cout << "Need to rebuild" <<endl;
        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);
        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathCam1 = pathSeq + "/mav0/cam1/data";
        LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft[seq], vstrImageRight[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;
        nImages[seq] = vstrImageLeft[seq].size();
        tot_images += nImages[seq];
    }

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);
    cout << endl << "-------" << endl;
    cout.precision(17);

    // ============================================================================
    // SETUP UNDISTORTION AND RECTIFICATION MAPS (DO THIS ONCE BEFORE THE SLAM SYSTEM)
    // ============================================================================
    // Camera intrinsics (ORIGINAL FULL RESOLUTION)
    cv::Mat K1 = (cv::Mat_<double>(3,3) << 
        1061.90838, 0, 778.19368,
        0, 1065.08245, 553.85699,
        0, 0, 1);

    cv::Mat K2 = (cv::Mat_<double>(3,3) << 
        1060.22456, 0, 782.09116,
        0, 1063.51251, 582.52751,
        0, 0, 1);

    // Distortion coefficients
    cv::Mat D1 = (cv::Mat_<double>(1,4) << -0.259389, 0.203687, 0.00066, 0.003063);
    cv::Mat D2 = (cv::Mat_<double>(1,4) << -0.263264, 0.215493, -0.003977, 0.000629);

    // Stereo transformation
    cv::Mat T_c1_c2 = (cv::Mat_<double>(4,4) << 
        9.99724649e-01, -1.97948923e-02,  1.26011608e-02, 5.52256532e-02,
        2.02534145e-02,  9.99096454e-01, -3.73640900e-02, -5.09111677e-04,
       -1.18501570e-02,  3.76090183e-02,  9.99222265e-01, 1.32659033e-03,
        0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00);

    cv::Mat R = T_c1_c2(cv::Rect(0, 0, 3, 3)).clone();
    cv::Mat t = T_c1_c2(cv::Rect(3, 0, 1, 3)).clone();

    cv::Size imageSize(1440, 1080);

    // STEP 1: Create undistortion maps (no rectification yet)
    cv::Mat undist_map1x, undist_map1y, undist_map2x, undist_map2y;
    cv::initUndistortRectifyMap(K1, D1, cv::Mat(), K1, imageSize, CV_32FC1, undist_map1x, undist_map1y);
    cv::initUndistortRectifyMap(K2, D2, cv::Mat(), K2, imageSize, CV_32FC1, undist_map2x, undist_map2y);

    // STEP 2: Compute rectification for UNDISTORTED images (D1=0, D2=0 now)
    cv::Mat D_zero = cv::Mat::zeros(1, 4, CV_64F);  // No distortion after step 1
    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D_zero, K2, D_zero, imageSize, R, t, 
                      R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0, imageSize);

    // Create rectification maps for undistorted images
    cv::Mat rect_map1x, rect_map1y, rect_map2x, rect_map2y;
    cv::initUndistortRectifyMap(K1, D_zero, R1, P1, imageSize, CV_32FC1, rect_map1x, rect_map1y);
    cv::initUndistortRectifyMap(K2, D_zero, R2, P2, imageSize, CV_32FC1, rect_map2x, rect_map2y);

    cout << "\n=== RECTIFIED CAMERA PARAMETERS (after undistort + rectify) ===" << endl;
    cout << "Camera1.fx: " << P1.at<double>(0,0) << " -> downsampled (0.5): " << P1.at<double>(0,0) * 0.5 << endl;
    cout << "Camera1.fy: " << P1.at<double>(1,1) << " -> downsampled (0.5): " << P1.at<double>(1,1) * 0.5 << endl;
    cout << "Camera1.cx: " << P1.at<double>(0,2) << " -> downsampled (0.5): " << P1.at<double>(0,2) * 0.5 << endl;
    cout << "Camera1.cy: " << P1.at<double>(1,2) << " -> downsampled (0.5): " << P1.at<double>(1,2) * 0.5 << endl;
    cout << "Camera2.fx: " << P2.at<double>(0,0) << " -> downsampled (0.5): " << P2.at<double>(0,0) * 0.5 << endl;
    cout << "Camera2.fy: " << P2.at<double>(1,1) << " -> downsampled (0.5): " << P2.at<double>(1,1) * 0.5 << endl;
    cout << "Camera2.cx: " << P2.at<double>(0,2) << " -> downsampled (0.5): " << P2.at<double>(0,2) * 0.5 << endl;
    cout << "Camera2.cy: " << P2.at<double>(1,2) << " -> downsampled (0.5): " << P2.at<double>(1,2) * 0.5 << endl;
    cout << "Stereo.b: " << abs(-P2.at<double>(0,3) / P2.at<double>(0,0)) << endl;
    cout << "===================================================\n" << endl;
    // ============================================================================

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    auto exSLAM = ORB_SLAM3::SystemFactory::create(argv[1], argv[2], ORB_SLAM3::SensorType::STEREO, true);
    if (!exSLAM) { std::cerr << "Failed to init SLAM" << std::endl; return 1; }
    auto& SLAM = exSLAM.value();

    cv::Mat imLeft, imRight;
    for (seq = 0; seq<num_seq; seq++)
    {
        // Seq loop
        double t_resize = 0;
        double t_rect = 0;
        double t_track = 0;
        int num_rect = 0;
        int proccIm = 0;
        int start_frame = 0;
        int max_frames = 10000;//exclusive upper bound
        
        for(int ni = start_frame; ni < nImages[seq] && ni < max_frames; ni++, proccIm++)
        {
            // Read left and right images from file
            imLeft = cv::imread(vstrImageLeft[seq][ni],cv::IMREAD_UNCHANGED);
            imRight = cv::imread(vstrImageRight[seq][ni],cv::IMREAD_UNCHANGED);
            
            if(imLeft.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageLeft[seq][ni]) << endl;
                return 1;
            }
            if(imRight.empty())
            {
                cerr << endl << "Failed to load image at: "
                     << string(vstrImageRight[seq][ni]) << endl;
                return 1;
            }
            
            // ============================================================================
            // STEP 1: UNDISTORT (at original resolution)
            // ============================================================================
            cv::Mat imLeftUndist, imRightUndist;
            cv::remap(imLeft, imLeftUndist, undist_map1x, undist_map1y, cv::INTER_LINEAR);
            cv::remap(imRight, imRightUndist, undist_map2x, undist_map2y, cv::INTER_LINEAR);

            // ============================================================================
            // STEP 2: RECTIFY (undistorted images)
            // ============================================================================
            cv::Mat imLeftRect, imRightRect;
            cv::remap(imLeftUndist, imLeftRect, rect_map1x, rect_map1y, cv::INTER_LINEAR);
            cv::remap(imRightUndist, imRightRect, rect_map2x, rect_map2y, cv::INTER_LINEAR);
            
            // ============================================================================
            // STEP 3: DOWNSAMPLE (after rectification)
            //Why not use cv::pyrDown (img1, img2)? maybe should ... 
            // ============================================================================
            double downsample_factor = 0.5;  // 0.5 = half resolution
            if (downsample_factor != 1.0) {
                cv::resize(imLeftRect, imLeftRect, cv::Size(), downsample_factor, downsample_factor, cv::INTER_LINEAR);
                cv::resize(imRightRect, imRightRect, cv::Size(), downsample_factor, downsample_factor, cv::INTER_LINEAR);
                cout << "Undistorted, rectified, and downsampled to: " << imLeftRect.cols << "x" << imLeftRect.rows << endl;
            }
            
            // Now use the processed images
            imLeft = imLeftRect;
            imRight = imRightRect;
            
            double tframe = vTimestampsCam[seq][ni];

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

            // Pass the images to the SLAM system
            SLAM->TrackStereo(imLeft,imRight,tframe, vector<ORB_SLAM3::IMU::Point>(), vstrImageLeft[seq][ni]);
            
            // --- Get tracking state and feature counts ---
            int state = SLAM->GetTrackingState();
            vector<cv::KeyPoint> kps_left = SLAM->GetTrackedKeyPointsUn();
            int n_features = kps_left.size();

            cout << "Frame " << ni 
                 << " | State: " << state 
                 << " | Extracted ORB features: " << n_features << endl;

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#endif

#ifdef REGISTER_TIMES
            t_track = t_resize + t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM->InsertTrackTime(t_track);
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
        }

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM->ChangeDataset();
        }
    }
    
    // Stop all threads
    SLAM->Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
        SLAM->SaveTrajectoryEuRoC(f_file);
        SLAM->SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM->SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    spdlog::set_level(spdlog::level::debug);
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
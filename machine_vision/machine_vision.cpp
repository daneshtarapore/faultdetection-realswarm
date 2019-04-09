#include "machine_vision.h"


static const size_t DRIVERPATHSIZE = 256;

static UInt32 TIME_BETWEEN_TWO_SEND = 10000; // in microseconds

#define NUM_ROBOTS_IN_EXPT 7

/****************************************/
/****************************************/

Mat cvb_to_ocv_nocopy(IMG cvbImg)
{
    // Construct an appropriate OpenCV image
    Size size(ImageWidth(cvbImg), ImageHeight(cvbImg));
    void* ppixels = nullptr;
    intptr_t xInc = 0;
    intptr_t yInc = 0;
    GetLinearAccess(cvbImg, 0, &ppixels, &xInc, &yInc);
    Mat image(size, CV_8UC3, ppixels, yInc);

    return image;
}

/****************************************/
/****************************************/

void projectpoint_image_to_world(cv::Mat cameraMatrix, double world_z, cv::Point_<float> undistort_imgpoint, cv::Point_<float> &undistort_worldpoint)
{
    double image_x = undistort_imgpoint.x;
    double image_y = undistort_imgpoint.y;

    double c_x = cameraMatrix.at<double>(0,2);
    double c_y = cameraMatrix.at<double>(1,2);

    double f_x = cameraMatrix.at<double>(0,0);
    double f_y = cameraMatrix.at<double>(1,1);

    /*
   * ref: http://stackoverflow.com/questions/12007775/to-calculate-world-coordinates-from-screen-coordinates-with-opencv
     x_screen = (x_world/z_world)*f_x + c_x
     y_screen = (y_world/z_world)*f_y + c_y
     x_world = (x_screen - c_x) * z_world / f_x
     y_world = (y_screen - c_y) * z_world / f_y*/

    double world_x = (image_x - c_x) * world_z / f_x;
    double world_y = (image_y - c_y) * world_z / f_y;

    undistort_worldpoint.x = world_x;
    undistort_worldpoint.y = world_y;
}

/****************************************/
/****************************************/

int main(int argc, char* argv[])
{
    if(InitNetwork())
    {
        std::cerr << "Error in preparing server connection " << std::endl;
        return 1;
    }

    if(InitThreads())
    {
        std::cerr << "Error in preparing threads to be used to send data to robots " << std::endl;
        return 1;
    }

    signal(SIGALRM, handle_alarm); // Install the handler for the SIGALRM signal


    /*****Camera calibration parameters**********/
    /*****Done on 31/08/2016 *********/
    /*https://github.com/daneshtarapore/apriltags-cpp/blob/optimisation/out_camera_data.xml*/

    double tmp_cameraMatrix[3][3] = {{1.6523377095739027e+03, 0.0, 7.9950000000000000e+02}, {0.0, 1.6523377095739027e+03, 7.9950000000000000e+02}, {0.0, 0.0, 1}};
    cv::Mat cameraMatrix = Mat(3, 3, CV_64FC1, &tmp_cameraMatrix);
    double tmp_distCoeffs[5][1] = {-1.9494404472059521e-01, 2.9965832643639467e-01, 0.0, 0.0, -3.4329528058097419e-01};
    cv::Mat distCoeffs = Mat(5, 1, CV_64FC1, &tmp_distCoeffs);
    /********************************************/



    // Load the camera
    char driverPath[DRIVERPATHSIZE] = { 0 };
    TranslateFileName("%CVB%/drivers/GenICam.vin", driverPath, DRIVERPATHSIZE);
    IMG hCamera = NULL;

    bool success = LoadImageFile(driverPath, hCamera);

    if(!success)
    {
        cout << "Error loading " << driverPath << " driver!" << endl;
        return 1;
    }

    cout << "Load " << driverPath << " successful." << endl;

    // Start grab with ring buffer
    cvbres_t result = G2Grab(hCamera);

    std::string input_swarm_behav = "", input_fault = "", rnd_seed = "", m_strOutput;

    std::cout << "Please enter the robot swarm behaviour, one of Aggreation -> Dispersion -- [AgDp], Aggregation -- [Ag] / Dispersion -- [Dp] / Homing to moving beacon -- [Hm]:\n";
    getline(std::cin, input_swarm_behav);

    std::cout << "Please enter the behaviour transition probability [0.0 - 1.0], or the robot fault type, one of [PMIN] / [PMAX] / [ROFS] / [LACT] / [RACT] / [BACT] otherwise:\n";
    getline(std::cin, input_fault);

    std::cout << "Please enter the random seed, one of  [111], [222], [333], [444], [555]:\n";
    getline(std::cin, rnd_seed);

    m_strOutput = input_swarm_behav + "_" + input_fault + "_" + rnd_seed + ".robpos";
    std::ofstream m_cOutput; m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);


    int counter = 0;

    if(result >= 0)
    {
        // Wait for sync. message from all the e-puck clients
        int robsync_socket_desc , robsync_new_socket[NUM_TAGS], robsync_c;
        struct sockaddr_in robsync_server , robsync_client[NUM_TAGS];

        //Create socket
        robsync_socket_desc = socket(AF_INET , SOCK_STREAM , 0);
        if (robsync_socket_desc == -1)
        {
            printf("Could not create socket for robot sync");
            return 1;
        }

        //Prepare the sockaddr_in structure
        robsync_server.sin_family = AF_INET;
        robsync_server.sin_addr.s_addr = INADDR_ANY;
        robsync_server.sin_port = htons(ROBOTS_SYNC_PORT);

        //Bind
        if(bind(robsync_socket_desc,(struct sockaddr *)&robsync_server , sizeof(robsync_server)) < 0)
        {
            puts("Bind failed for robot sync.");
            return 1;
        }
        puts("Bind done for robot sync.");

        //Listen
        listen(robsync_socket_desc , 3);

        //Accept and incoming connection
        puts("Waiting for incoming connections for rob. sync. ...");
	m_cOutput << "Waiting for incoming connections for rob. sync. ..." << std::endl;
        robsync_c = sizeof(struct sockaddr_in);
        int robsync_client_index = 0;
        while((robsync_new_socket[robsync_client_index] = accept(robsync_socket_desc, (struct sockaddr *)&(robsync_client[robsync_client_index]), (socklen_t*)&robsync_c)))
        {
            char fromipaddress[100];

            if (robsync_new_socket[robsync_client_index] < 0)
            {
                perror("accept connection failed during rob. sync");
                return 1;
            }

            std::cout << "Connection accepted from e-puck with IP address" << inet_ntop(AF_INET, &(robsync_client[robsync_client_index].sin_addr), fromipaddress, INET_ADDRSTRLEN) << std::endl;;
	    m_cOutput << "Connection accepted from e-puck with IP address" << inet_ntop(AF_INET, &(robsync_client[robsync_client_index].sin_addr), fromipaddress, INET_ADDRSTRLEN) << std::endl;
            robsync_client_index++;
            if(robsync_client_index == NUM_ROBOTS_IN_EXPT)
                break;
        }

        for(int i = 0; i < NUM_ROBOTS_IN_EXPT; ++i)
        {
            //Message robots to start expt
            char robsync_message[] = "Start experiment";
            if(write(robsync_new_socket[i] , robsync_message , strlen(robsync_message)) < 0)
            {
                perror("Error writing expt start message to robot during rob. sync");
                return 1;
            }
	    }

	std::cout << "STARTING EXPERIMENT" << std::endl;
        m_cOutput << "STARTING EXPERIMENT" << std::endl;
        alarm(330); // schedule alarm to go off after argument seconds -- set to 360s for 5 min experiment time + 30 sec. grace. 

        trackingloopstep = 0;
        while(true)
        {
            /*
       * Sleep for a while to avoid overflowing the internal buffer of the sockets with range-and-bearing data packets
       * The value is set in microseconds: 10.000 = 10 millisec => 10 times per control step
       */
            usleep(TIME_BETWEEN_TWO_SEND);

            // Wait for next image to be acquired
            // (returns immediately if unprocessed images are in the ring buffer)
            result = G2Wait(hCamera);

            if(result < 0)
                cout << setw(3) << " Error with G2Wait: " << CVC_ERROR_FROM_HRES(result) << endl;
            else
            {
                // Create an attached OpenCV image
                Mat distorted_image = cvb_to_ocv_nocopy(hCamera);

                // Swap blue and red channels
                vector<Mat> channels(3);
                split(distorted_image, channels);
                merge(vector<Mat>{channels[2], channels[1], channels[0]}, distorted_image);

                // Undistort the image
                Mat image;
                //        undistort(distorted_image, image, cameraMatrix, distCoeffs);
                distorted_image.copyTo(image);

                // Check for user input
                int k = cvWaitKey(1) % 256;

                if(k == 27 || alarm_flag) // Escape key pressed or end of experiment alarm goes off
                    break;

                if(k == 10) // Enter key
                {
                    printf("Capturing image %d\n", counter);

                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(0);

                    ostringstream os;
                    os << "image_" << counter << ".png";

                    imwrite(os.str(), image, compression_params);
                    counter++;
                }

                // Detect ArUco markers
                vector<int> marker_ids;
                vector<vector<Point2f> > marker_corners, rejected_candidates;
                //Point2f projected_centres[NUM_TAGS], projected_centres_prev[NUM_TAGS];
                //double  projected_angles[NUM_TAGS],  projected_angles_prev[NUM_TAGS];


                Ptr<aruco::DetectorParameters> parameters = aruco::DetectorParameters::create();
                Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_50);

                aruco::detectMarkers(image, dictionary, marker_corners, marker_ids, parameters, rejected_candidates);

                if(marker_ids.size() > 0)
                {
                    for(size_t marker_index = 0; marker_index < marker_ids.size(); marker_index++)
                    {
                        assert(marker_ids[marker_index] < NUM_TAGS);
                        // if(marker_ids[marker_index] == 3 || marker_ids[marker_index] == 6)
                        if(true)
                            //if(marker_ids[marker_index] == 2)
                        {
                            putText(image,
                                    "o",
                                    marker_corners[marker_index][3],
                                    FONT_HERSHEY_SIMPLEX,
                                    1,
                                    CV_RGB(255, 0, 0),
                                    2,
                                    CV_AA); // bottom-left corner

                            putText(image,
                                    "*",
                                    marker_corners[marker_index][2],
                                    FONT_HERSHEY_SIMPLEX,
                                    1,
                                    CV_RGB(255, 0, 0),
                                    2,
                                    CV_AA); // bottom-right corner

			    Point2f MC = (marker_corners[marker_index][0] +  marker_corners[marker_index][1] +  marker_corners[marker_index][2] +  marker_corners[marker_index][3])/4;

                            putText(image,
                                    to_string(marker_ids[marker_index]),
                                    MC,
                                    FONT_HERSHEY_SIMPLEX,
                                    3,
                                    CV_RGB(255, 0, 0),
                                    4,
                                    CV_AA); // bottom-right corner


                            double bottom_line_angle = atan2(marker_corners[marker_index][2].y - marker_corners[marker_index][3].y,
                                    marker_corners[marker_index][2].x - marker_corners[marker_index][3].x)
                                    * 180.0f / M_PI; // atan2 returns angle in radians [-pi, pi]

                            if(std::isnan(bottom_line_angle))
                            {
                                cout << " angle is nan for tag points " << marker_corners[marker_index] << endl;
                                exit(-1);
                            }

                            Mat_<Point2f> points(1, 4);
                            points(0) = marker_corners[marker_index][0];
                            points(1) = marker_corners[marker_index][1];
                            points(2) = marker_corners[marker_index][2];
                            points(3) = marker_corners[marker_index][3];

                            // Mat undistorted;
                            Mat_<Point2f> undistorted(1, 4);
                            undistortPoints(points, undistorted, cameraMatrix, distCoeffs, noArray(), cameraMatrix);


                            Point2f undistorted_centre = (undistorted(0) +
                                                          undistorted(1) +
                                                          undistorted(2) +
                                                          undistorted(3)) / 4;

                            double undistorted_angle = atan2(undistorted(2).y - undistorted(3).y,
                                                             undistorted(2).x - undistorted(3).x); // atan2 returns angle in radians [-pi, pi]
                            //undistorted_angle -= EPUCK_ETCHEDTAG_ANGLEOFFSET;

                            time_t* ptr_simple_time; time_t simple_time;
                            ptr_simple_time = &simple_time;
                            struct tm* ptr_local_time;
                            simple_time = time(ptr_simple_time);
                            ptr_local_time = localtime(ptr_simple_time);

                            struct timespec spec; clock_gettime(CLOCK_REALTIME, &spec);


                            m_cOutput << "ImageCoordinates_TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6);
			    m_cOutput << "\tStep: "  << trackingloopstep; 
                            m_cOutput << "\tTagId: " << marker_ids[marker_index]
                                         << "\tXPos: " << undistorted_centre.x << ", " << std::setprecision(2)
                                         << "\tYPos: " << undistorted_centre.y << ", " << std::setprecision(2)
                                         << "\tAngle: " << std::right<<  undistorted_angle * 180.0f / M_PI << std::fixed << std::endl;




                            // double world_z = CAMERA_TO_FLOOR_HEIGHT;
                            double world_z = CAMERA_TO_FLOOR_HEIGHT - ARENA_HEIGHT  - EPUCKWITHLINUXBOARD_HEIGHT; //EPUCK_HEIGHT; // approx. distance of of e-puck tag to camera image plane in cm
                            //                            projectpoint_image_to_world(cameraMatrix, world_z, marker_centre, markerinworld_center);
                            //                            std::cout << " world center of tag 0 is " << markerinworld_center << std::endl;

                            cv::Point_<float> undistorted_projected_center;
                            projectpoint_image_to_world(cameraMatrix, world_z, undistorted_centre, undistorted_projected_center);
                            cv::Point_<float> undistorted_projected_corner[4];
                            projectpoint_image_to_world(cameraMatrix, world_z, (Point2f)(undistorted(2)), undistorted_projected_corner[2]);
                            projectpoint_image_to_world(cameraMatrix, world_z, (Point2f)(undistorted(3)), undistorted_projected_corner[3]);


                            double undistorted_projected_angle = atan2(undistorted_projected_corner[2].y - undistorted_projected_corner[3].y,
                                    undistorted_projected_corner[2].x - undistorted_projected_corner[3].x); // atan2 returns angle in radians [-pi, pi]
                            //undistorted_projected_angle       -= EPUCK_ETCHEDTAG_ANGLEOFFSET;


                            projected_centres[marker_ids[marker_index]] = undistorted_projected_center;
                            projected_angles[marker_ids[marker_index]]  = undistorted_projected_angle; //undistorted_angle;


                            // Computing the robot's angular acceleration
                            // TODO: Check if robot id is undetected; For now if it is undetected then the m_vecRobotBearing_Prev value used will be outdated

                            //struct timespec spec; clock_gettime(CLOCK_REALTIME, &spec);
                            double sec_prev, sec = (double)(spec.tv_nsec) / 1.0e9; // Convert nanoseconds to seconds

                            sec_prev                                                = m_vecRobotBearingAtTime[marker_ids[marker_index]];
                            m_vecRobotBearingAtTime[marker_ids[marker_index]]       = sec;
                            m_vecRobotBearing_Prev[marker_ids[marker_index]]        = m_vecRobotBearing[marker_ids[marker_index]];
                            m_vecRobotBearing[marker_ids[marker_index]]             = projected_angles[marker_ids[marker_index]];
                            double AngularVelocity_Prev                             = m_vecRobotAngularVelocity[marker_ids[marker_index]];
                            m_vecRobotAngularVelocity[marker_ids[marker_index]]     = (m_vecRobotBearing[marker_ids[marker_index]] -
                                    m_vecRobotBearing_Prev[marker_ids[marker_index]]) / ((double)abs(sec - sec_prev));
                            m_vecRobotAngularAcceleration[marker_ids[marker_index]] = m_vecRobotAngularVelocity[marker_ids[marker_index]] - AngularVelocity_Prev;

                            std::cout   << "ID = "<< marker_ids[marker_index]
                                           << "\tX = " << projected_centres[marker_ids[marker_index]].x << ", " << std::setprecision(2)
                                           << "\tY = " << projected_centres[marker_ids[marker_index]].y << ", " << std::setprecision(2)
                                           << "\tAngle = " << std::right<<  projected_angles[marker_ids[marker_index]] * 180.0f / M_PI << ", " << std::fixed
                                           << "\tSpeed = " << std::right<<  norm(projected_centres[marker_ids[marker_index]] - projected_centres_prev[marker_ids[marker_index]]) /
                                    ((double)abs(sec - sec_prev)) << std::setprecision(2)
                                                                  << "\tAngleAcc = " << std::right<<  m_vecRobotAngularAcceleration[marker_ids[marker_index]] * 180.0f / M_PI << std::setprecision(2) << std::endl;


                            /*time_t* ptr_simple_time; time_t simple_time;
                            ptr_simple_time = &simple_time;
                            struct tm* ptr_local_time;
                            simple_time = time(ptr_simple_time);
                            ptr_local_time = localtime(ptr_simple_time);*/

                            m_cOutput << "TimeStamp: " << ptr_local_time->tm_hour << ":" << ptr_local_time->tm_min << ":" << ptr_local_time->tm_sec << ":" << round(spec.tv_nsec / 1.0e6);
			    m_cOutput << "\tStep: "  << trackingloopstep; 
                            m_cOutput << "\tTagId: " << marker_ids[marker_index]
                                         << "\tXPos: " << projected_centres[marker_ids[marker_index]].x << ", " << std::setprecision(2)
                                         << "\tYPos: " << projected_centres[marker_ids[marker_index]].y << ", " << std::setprecision(2)
                                         << "\tAngle = " << std::right<<  projected_angles[marker_ids[marker_index]] * 180.0f / M_PI << ", " << std::fixed
                                         << "\tSpeed = " << std::right<<  norm(projected_centres[marker_ids[marker_index]] - projected_centres_prev[marker_ids[marker_index]]) /
                                    ((double)abs(sec - sec_prev)) << std::setprecision(2)
                                                                  << "\tAngleAcc = " << std::right<<  m_vecRobotAngularAcceleration[marker_ids[marker_index]] * 180.0f / M_PI << std::setprecision(2) << std::endl;



                            projected_centres_prev[marker_ids[marker_index]] = projected_centres[marker_ids[marker_index]];
                            projected_angles_prev[marker_ids[marker_index]]  = projected_angles[marker_ids[marker_index]];
                        }
                    }
                    std::cout << std::endl;
		    m_cOutput << std::endl;

                    for(int marker_id = 0; marker_id < NUM_TAGS; marker_id++)
                    {
                        bool b_current_pos_present = std::find(marker_ids.begin(), marker_ids.end(), marker_id) != marker_ids.end();

                        b_bothpos_available[marker_id] = TrackRobotDisplacement(displacement_pos_index[marker_id], marker_id, projected_centres[marker_id],
                                                                                vec_RobPos_ShortRangeTimeWindow[marker_id],
                                                                                b_current_pos_present,
                                                                                pos_startinterval[marker_id], pos_endinterval[marker_id]);
                    }
                    ComputeLineOfSight(marker_ids, projected_centres);
                    for(size_t marker_index = 0; marker_index < marker_ids.size(); marker_index++)
                    {
                        pthread_mutex_lock(&m_tBufferQueueMutex[marker_ids[marker_index]]); // mutex lock on shared resource m_vecRangeAndBearingPackets[marker_id] where marker_id = marker_ids[marker_index]
                        ComputeRangesAndBearing(marker_index, marker_ids, projected_centres, projected_angles, m_vecRangeAndBearingPackets[marker_ids[marker_index]]);
                        pthread_mutex_unlock(&m_tBufferQueueMutex[marker_ids[marker_index]]);
                    }
                }

                resize(image, image, Size(image.cols * 0.25, image.rows * 0.25), 0, 0, INTER_CUBIC);
                imshow("JAI GO-5000C-PGE", image);

                //Stroing the image
                vector<int> compression_params;
                compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                compression_params.push_back(0);
                ostringstream os;
                string m_strOutputImageFileName;
                m_strOutputImageFileName = "img_" + input_swarm_behav + "_" + input_fault + "_" + rnd_seed + "_";
                os << m_strOutputImageFileName << trackingloopstep << ".png";
                imwrite(os.str(), image, compression_params);

                trackingloopstep++;
            }
        }

        m_cOutput.close(); // close file of robot positions

        cout << "Stop acquisition" << endl;

        // Stop the grab (kill = false: wait for ongoing frame acquisition to stop)
        result = G2Freeze(hCamera, true);
    }
    else
        cout << "Error starting acquisition!" << endl;

    // Free camera
    cout << "Free camera" << endl;
    ReleaseObject(hCamera);


    threads_terminate = true;
    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        //Now join the thread; Wait for it to terminate
        if(pthread_join(sendto_thread[marker_id], NULL))
        {
            std::cerr << "Error joining sendto_thread: " << ::strerror(errno);
            return 1;
        }
    }

    return 0;
}

/****************************************/
/****************************************/

bool TrackRobotDisplacement(unsigned long long& step, int marker_id, Point2f current_pos,
                            std::vector<RobotGlobalPosData>& displacement_vector,
                            bool b_current_pos_present,
                            Point2f& start_pos, Point2f& end_pos)
{
    bool b_both_pos_present;
    if(step >= displacement_vector.size())
        step = ((step)%displacement_vector.size());

    if(!b_current_pos_present)
    {
        displacement_vector[step].b_PosAvailable = false;
        b_both_pos_present = false;
        step++;
        return b_both_pos_present;
    }

    if(displacement_vector[step].b_PosAvailable)
    {
        start_pos = displacement_vector[step].pos;
        end_pos   = current_pos;
        b_both_pos_present = true;
    }
    else
        b_both_pos_present = false;

    displacement_vector[step].b_PosAvailable = b_current_pos_present;
    if (displacement_vector[step].b_PosAvailable)
        displacement_vector[step].pos = current_pos;

    step++;

    return b_both_pos_present;
}

/****************************************/
/****************************************/

int ComputeRangesAndBearing(int src_marker_index, std::vector<int> &marker_ids, Point2f (&projected_centres)[NUM_TAGS], double (&projected_angles)[NUM_TAGS],
                            std::vector<struct RangeAndBearingPacket>& m_vecRAB)
{
    double range, bearing;

    m_vecRAB.resize(0);

    struct RangeAndBearingPacket NewRangeAndBearingPacket;
    Point2f dst_translated, dst_translated_rotated;


    for(size_t dst_marker_index = 0; dst_marker_index < marker_ids.size(); dst_marker_index++)
    {
        if(src_marker_index == dst_marker_index)
            continue;

        if(!InLineOfSight(src_marker_index, dst_marker_index, marker_ids))
            continue;

        dst_translated = projected_centres[marker_ids[dst_marker_index]] - projected_centres[marker_ids[src_marker_index]];
        range = norm(dst_translated) - EPUCK_DIAMETER;
        if (range < 0.0f)
            range = 0.0f;

        float theta =  projected_angles[marker_ids[src_marker_index]];
        dst_translated_rotated.x =  dst_translated.x * cos(theta) + dst_translated.y * sin(theta);
        dst_translated_rotated.y = -dst_translated.x * sin(theta) + dst_translated.y * cos(theta);

        // atan2 returns angle in radians [-pi, pi]
        bearing  = atan2(-dst_translated_rotated.y, dst_translated_rotated.x); // -dst_translated_rotated.y as -ve y-axis in opencv is +ve y-axis in standard notation (and also in argos)
        //bearing  = atan2(dst_translated_rotated.y, dst_translated_rotated.x);
        bearing  = (M_PI/2.0f - bearing) * 180.0f / M_PI; // bearing subtracted from pi/2 as we want the bearing angle wrt the +ve y-axis (not +ve x-axis)

        bearing = -bearing; // flip to ARGOS's angular sign notation where clockwise is -ve and counter-clockwise is +ve

        // Need bearing in range 0 to 360 degrees for transmission
        //  1. Convert to range [-180, 180]
        while (bearing >  180.0f)
            bearing = bearing - 360.0f;
        while (bearing < -180.0f)
            bearing = bearing + 360.0f;

        //if(marker_ids[src_marker_index] == 1)
        //	std::cout << "Src Id " << marker_ids[src_marker_index] << " Dst Id " << marker_ids[dst_marker_index] << " range " << range << " bearing " << bearing << std::endl;

        //  2. Convert to range [0, 360] before transmission
        bearing = bearing + 180.0f;

        NewRangeAndBearingPacket.Range   = (UInt16)(round(range   * 10.0f));
        NewRangeAndBearingPacket.Bearing = (UInt16)(round(bearing * 10.0f));

        //NewRangeAndBearingPacket.RobotId = (UInt8)marker_ids[dst_marker_index];

        // Send the ARGoS Robot Id (last three digits of e-puck's IP Address) instead of the ArUco Tag Id
        std::string::size_type sz;   // alias of size_t
        std::string ip_addr = TagID_IPAddress_Map[marker_ids[dst_marker_index]];
        ip_addr.erase(0, 10);
        unsigned u_id = std::stoi(ip_addr, &sz);
        NewRangeAndBearingPacket.RobotId = (UInt8)u_id;
        //std::cout << "IP Address for tag " << TagID_IPAddress_Map[marker_ids[dst_marker_index]];
        //printf(" with Robot Id %d\n",NewRangeAndBearingPacket.RobotId);

        // Add angular acceleration data pertaining to the robot with id marker_ids[dst_marker_index] to the range-and-bearing packet
        AddAngularAcceleration(marker_ids[dst_marker_index], NewRangeAndBearingPacket, m_vecRobotAngularAcceleration);

        // Add relative displacement vector data pertaining to the robot with id marker_ids[dst_marker_index] to the range-and-bearing packet. Relative wrt robot with id marker_ids[src_marker_index]
        AddDisplacementVector(marker_ids[dst_marker_index], NewRangeAndBearingPacket,
                              projected_centres[marker_ids[src_marker_index]], projected_angles[marker_ids[src_marker_index]],
                b_bothpos_available[marker_ids[dst_marker_index]],
                pos_startinterval[marker_ids[dst_marker_index]], pos_endinterval[marker_ids[dst_marker_index]]);

        m_vecRAB.push_back(NewRangeAndBearingPacket);
    }

    return m_vecRAB.size();
}

/****************************************/
/****************************************/

void AddAngularAcceleration(int marker_id, struct RangeAndBearingPacket& NewRangeAndBearingPacket, std::vector<double>& AngularAcceleration)
{
    double NormalizedAngularAcceleration;

    /*normalises to range [-1 to 1] */
    if ((AngularAcceleration[marker_id] / MAXANGULARACCLERATION) >= 0.0f)
        NormalizedAngularAcceleration = std::min(AngularAcceleration[marker_id] / MAXANGULARACCLERATION,   1.0);
    else
        NormalizedAngularAcceleration = std::max(AngularAcceleration[marker_id] / MAXANGULARACCLERATION,  -1.0);

    NewRangeAndBearingPacket.Data[0] = (UInt8)(((NormalizedAngularAcceleration + 1.0) / 2.0) * 255.0f);
}

/****************************************/
/****************************************/

void AddDisplacementVector(int marker_id, struct RangeAndBearingPacket& NewRangeAndBearingPacket, 
                           Point2f focal_robot_pos, double focal_robot_angle,
                           bool nbr_robot_pos_available, Point2f nbr_robot_startpos, Point2f nbr_robot_endpos)
{
    // flag to inform the receiving robot if the displacement vector is available in the data packet
    if(nbr_robot_pos_available)
        NewRangeAndBearingPacket.Data[1] = (UInt8)1;
    else
    {
        NewRangeAndBearingPacket.Data[1] = (UInt8)0;
        return;
    }
    
    Point2f rel_start_pos, rel_end_pos;
    Point2f tmp_translated, tmp_translated_rotated;

    tmp_translated = nbr_robot_startpos - focal_robot_pos;
    tmp_translated_rotated.x =  tmp_translated.x * cos(focal_robot_angle) + tmp_translated.y * sin(focal_robot_angle);
    tmp_translated_rotated.y = -tmp_translated.x * sin(focal_robot_angle) + tmp_translated.y * cos(focal_robot_angle);
    rel_start_pos = tmp_translated_rotated;

    tmp_translated = nbr_robot_endpos - focal_robot_pos;
    tmp_translated_rotated.x =  tmp_translated.x * cos(focal_robot_angle) + tmp_translated.y * sin(focal_robot_angle);
    tmp_translated_rotated.y = -tmp_translated.x * sin(focal_robot_angle) + tmp_translated.y * cos(focal_robot_angle);
    rel_end_pos = tmp_translated_rotated;

    Point2f rel_displacement_vector = (rel_end_pos - rel_start_pos) / norm(rel_end_pos - rel_start_pos);

    NewRangeAndBearingPacket.Data[2] = (UInt8)round(rel_displacement_vector.x * 10.0f);
    NewRangeAndBearingPacket.Data[3] = (UInt8)round(rel_displacement_vector.y * 10.0f);
}

/****************************************/
/****************************************/

void SerializeRangeAndBearingPackets(int marker_id, std::vector<struct RangeAndBearingPacket>& m_vecRAB, UInt8 *pun_Buffer, int& serializedbuffersize)
{
    //pthread_mutex_lock(&m_tBufferQueueMutex[marker_id]);

    UInt8* index = pun_Buffer;
    for(int i = 0; i < m_vecRAB.size(); ++i)
    {
        memcpy(index, (void*)&(m_vecRAB[i].RobotId), sizeof(UInt8)); index += sizeof(UInt8);
        //assert(*(index-sizeof(UInt8)) != marker_id); // catch error when two tags with same id are present in the arena
        memcpy(index, (void*)&(m_vecRAB[i].Range),   sizeof(UInt16)); index += sizeof(UInt16);
        memcpy(index, (void*)&(m_vecRAB[i].Bearing), sizeof(UInt16)); index += sizeof(UInt16);
        memcpy(index, (void*)&(m_vecRAB[i].Data[0]), sizeof(UInt8));  index += sizeof(UInt8);
        memcpy(index, (void*)&(m_vecRAB[i].Data[1]), sizeof(UInt8));  index += sizeof(UInt8);
        memcpy(index, (void*)&(m_vecRAB[i].Data[2]), sizeof(UInt8));  index += sizeof(UInt8);
        memcpy(index, (void*)&(m_vecRAB[i].Data[3]), sizeof(UInt8));  index += sizeof(UInt8);
    }

    serializedbuffersize = m_vecRAB.size() * (sizeof(UInt8) * 5 + sizeof(UInt16) + sizeof(UInt16));
    //pthread_mutex_unlock(&m_tBufferQueueMutex[marker_id]);
}

/****************************************/
/****************************************/

void Create_TagID_IPAddress_Map()
{
    assert(NUM_TAGS == 9);

    // TagID_IPAddress_Map is indexed by the marker id

    TagID_IPAddress_Map[0] = "192.168.1.201";
    TagID_IPAddress_Map[1] = "192.168.1.206";
    TagID_IPAddress_Map[2] = "192.168.1.202";
    TagID_IPAddress_Map[3] = "192.168.1.244";
    TagID_IPAddress_Map[4] = "192.168.1.210";

    // added to prevent permission denied error for sendto
    TagID_IPAddress_Map[5] = "192.168.1.208";
    TagID_IPAddress_Map[6] = "192.168.1.207";
    TagID_IPAddress_Map[7] = "192.168.1.244";
    TagID_IPAddress_Map[8] = "192.168.1.209";
}

/****************************************/
/****************************************/

unsigned InitNetwork()
{
    Create_TagID_IPAddress_Map();

    // Reserve data buffer memory and clear the data buffer
    //for(auto elem : pun_databuffer)
    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        pun_databuffer[marker_id] = new UInt8[un_maxdatabuffersize];
        bzero((UInt8 *) pun_databuffer[marker_id], un_maxdatabuffersize);
    }

    // Create sockets -- one for each client so we can parallelize send operations
    //for(auto elem : socket_desc)
    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        socket_desc[marker_id] = socket(AF_INET , SOCK_DGRAM , 0); // UDP communication
        if (socket_desc[marker_id] == -1)
        {
            std::cout << "Could not create socket " << std::endl;
            return 1;
        }
    }

    // UDP communication, provide client details.
    assert(client.size() == NUM_TAGS);
    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        bzero((char *) &(client[marker_id]), sizeof(client[marker_id]));
        client[marker_id].sin_family      = AF_INET;
        client[marker_id].sin_port        = htons(CLIENT_PORT);
        client[marker_id].sin_addr.s_addr = inet_addr((TagID_IPAddress_Map[marker_id]).c_str()); // std::string ipaddr     = TagID_IPAddress_Map[i];
    }


    assert(server.size() == NUM_TAGS);
    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        //Prepare the server for binding
        bzero((char *) &(server[marker_id]), sizeof(server[marker_id]));
        server[marker_id].sin_family = AF_INET;
        server[marker_id].sin_addr.s_addr = INADDR_ANY;
        server[marker_id].sin_port = htons(SERVER_PORTS + marker_id);

        //Bind the server to a specific port
        int bindresult = bind(socket_desc[marker_id], (struct sockaddr *) &(server[marker_id]), sizeof(struct sockaddr_in));
        if(bindresult < 0)
        {
            std::cout << "bind failed for server port " << SERVER_PORTS + marker_id << " with error " << strerror(errno) << std::endl;
            return 1;
        }
    }

    return 0;
}

/****************************************/
/****************************************/

unsigned InitThreads()
{
    threads_terminate = false;

    assert(m_tBufferQueueMutex.size() == NUM_TAGS);
    assert(sendto_thread.size()       == NUM_TAGS);
    assert(sendto_thread_args.size()  == NUM_TAGS);

    for(size_t marker_id = 0; marker_id < NUM_TAGS; ++marker_id)
    {
        // Initialize the mutex
        pthread_mutex_init(&(m_tBufferQueueMutex[marker_id]), NULL);

        sendto_thread_args[marker_id] = new UInt8;
        *(sendto_thread_args[marker_id]) = marker_id;

        // Start the thread to send data to client
        if(pthread_create(&sendto_thread[marker_id], NULL, &Sendto_Handler, (void*)(sendto_thread_args[marker_id])) < 0)
        {
            std::cerr << "Could not create sendto_thread thread: " << ::strerror(errno);
            return 1;
        }
    }
    return 0;
}

/****************************************/
/****************************************/

void *Sendto_Handler(void *thread_arg)
{
    //lock
    //make a copy of data in buffer and set size to 0
    //unlock
    // if data in buffer, send it

    UInt8 marker_id = *(UInt8*)thread_arg;

    while(!threads_terminate)
    {
        pthread_mutex_lock(&m_tBufferQueueMutex[marker_id]); // mutex lock on shared resource m_vecRangeAndBearingPackets[marker_id]


        if(m_vecRangeAndBearingPackets[marker_id].size() > 0)
        {
            SerializeRangeAndBearingPackets(marker_id,
                                            m_vecRangeAndBearingPackets[marker_id],
                                            pun_databuffer[marker_id],
                                            in_filleddatabuffersize[marker_id]);

            m_vecRangeAndBearingPackets[marker_id].resize(0); // we don't want to send the same range and bearing data multiple times

        }
        pthread_mutex_unlock(&m_tBufferQueueMutex[marker_id]);

        if(in_filleddatabuffersize[marker_id] > 0)
        {

            int n = sendto(socket_desc[marker_id], pun_databuffer[marker_id], in_filleddatabuffersize[marker_id], 0, (const struct sockaddr *)(&(client[marker_id])), sizeof(struct sockaddr_in));
            if(n < 0)  printf("\nsendto error: %s\n", strerror(errno));

            if(false)
            {
                std::cout << "Sending data to client with marker id " << marker_id << " in_filleddatabuffersize_copy = " << in_filleddatabuffersize[marker_id] << " ip address " << TagID_IPAddress_Map[marker_id] << ". Returns " << n << std::endl;


                UInt8* end   = pun_databuffer[marker_id] + in_filleddatabuffersize[marker_id];
                UInt8* index = pun_databuffer[marker_id];
                UInt8 RobotId; UInt16 Range, Bearing;
                while(index < end)
                {
                    memcpy((void*)&(RobotId), (void*)index, sizeof(UInt8));  index = index + sizeof(UInt8);
                    memcpy((void*)&(Range),   (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);
                    memcpy((void*)&(Bearing), (void*)index, sizeof(UInt16)); index = index + sizeof(UInt16);

                    printf("Id %d Range %d Bearing %d \n", RobotId, Range, Bearing);
                }

            }
            in_filleddatabuffersize[marker_id] = 0;
        }
    }

    close(socket_desc[marker_id]);
}

/****************************************/
/****************************************/

void ComputeLineOfSight(std::vector<int> &marker_ids, Point2f (&projected_centres)[NUM_TAGS])
{
    // Initialize values to 0
    //for(auto elem : m_vec2dLineOfSightMatrix) std::fill(elem.begin(), elem.end(), 0);
    for(size_t src_marker_index = 0; src_marker_index < NUM_TAGS; src_marker_index++)
        for(size_t dst_marker_index = 0; dst_marker_index < NUM_TAGS; dst_marker_index++)
            m_vec2dLineOfSightMatrix[src_marker_index][dst_marker_index] = 0;

    /** src_marker_id is the range-and-bearing emitter robot, and dst_marker_id is the receiver robot.
   *  IsOccluded is true if there exists any robot that obstructs line-of-sight between src_marker_id and src_marker_id robots.
   *  This is checked with a circle line-segment collision detection algorith.
   *  Ref: http://stackoverflow.com/a/1501725; Proof: http://stackoverflow.com/a/15187473
   */

    bool IsOccluded; size_t src_marker_id, dst_marker_id, occlusion_marker_id;
    for(size_t src_marker_index = 0; src_marker_index < marker_ids.size(); src_marker_index++)
    {
        for(size_t dst_marker_index = 0; dst_marker_index < marker_ids.size(); dst_marker_index++)
        {
            src_marker_id = marker_ids[src_marker_index];
            dst_marker_id = marker_ids[dst_marker_index];

            if(src_marker_id == dst_marker_id)
                continue;

            // Have already check for occlusion for this pair
            if(m_vec2dLineOfSightMatrix[src_marker_id][dst_marker_id] != 0)
                continue;

            IsOccluded = false;
            for(size_t occlusion_marker_index = 0; occlusion_marker_index < marker_ids.size(); occlusion_marker_index++)
            {
                occlusion_marker_id = marker_ids[occlusion_marker_index];

                if((occlusion_marker_id == src_marker_id) || (occlusion_marker_id == dst_marker_id))
                    continue;

                Point2f src_pos = projected_centres[src_marker_id];    // position of emitter robot
                Point2f dst_pos = projected_centres[dst_marker_id];    // position of receiver robot
                Point2f occ_pos = projected_centres[occlusion_marker_id]; // position of the possibly occluding robot. check if it is blocking line-of-sight of src--dst robots

                // compute vector projection of src_pos-occ_pos on src_pos-dst_pos
                // if projected point is in between src_pos and dst_pos
                // compute sqr dist of projected point from occ_pos
                // if dist from projected point to occ_pos is less than robot radius, set IsOccluded to true.

                //float l2 =  norm(dst_pos - src_pos, NORM_L2SQR);
                float l2 =  (dst_pos.x - src_pos.x) * (dst_pos.x - src_pos.x) + (dst_pos.y - src_pos.y) * (dst_pos.y - src_pos.y);
                float t  = ((occ_pos.x - src_pos.x) * (dst_pos.x - src_pos.x) + (occ_pos.y - src_pos.y) * (dst_pos.y - src_pos.y)) / l2;
                if((t < 0.0f) || (t > 1.0f)) // the projection point lies out of the line segment
                {
                    //if(src_marker_id == 1 && dst_marker_id == 8)
                    //  std::cout << "The projection point lies out of the line segment. t=" << t << std::endl;
                    continue;
                }
                Point2f projection = src_pos + t * (dst_pos - src_pos);  // Projection point falls on the segment at projection at Point2f projection

                // the distance from the projection point to the occlusion point. This is the nearest distance from the occlusion point to a point on the src-dst line segment
                float nearest_dist_sq = (occ_pos.x - projection.x) * (occ_pos.x - projection.x) + (occ_pos.y - projection.y) * (occ_pos.y - projection.y);
                if(nearest_dist_sq < EPUCK_RADIUS_SQUARE)
                {
                    //if(src_marker_id == 1 && dst_marker_id == 8)
                    //  std::cout << " nearest_dist_sq of " << occlusion_marker_id << " from " << " line segment " << src_marker_id << " - " << dst_marker_id << " is " <<  nearest_dist_sq << std::endl;
                    IsOccluded = true; break; // dist from projected point to occ_pos is less than robot radius
                }
            }
            if(IsOccluded)
            {
                m_vec2dLineOfSightMatrix[src_marker_id][dst_marker_id] = -1; m_vec2dLineOfSightMatrix[dst_marker_id][src_marker_id] = -1;
            }
            else
            {
                m_vec2dLineOfSightMatrix[src_marker_id][dst_marker_id] =  1; m_vec2dLineOfSightMatrix[dst_marker_id][src_marker_id] = 1;
            }
        }
    }
}

/****************************************/
/****************************************/

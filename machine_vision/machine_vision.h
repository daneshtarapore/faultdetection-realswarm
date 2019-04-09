#ifndef MACHINE_VISION_H
#define MACHINE_VISION_H


#include <iostream>
#include <iomanip>
#include <vector>
#include <iCVCDriver.h>
#include <iCVCImg.h>
#include <iCVGenApi.h>
#include <CVCError.h>
#include <iCVCUtilities.h>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <time.h>

#include <pthread.h>

#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <fstream>
#include <sstream>
#include <string>

#include <csignal>

#define NUM_TAGS 9
#define CAMERA_TO_FLOOR_HEIGHT 245.75f+3.0f  // in cm // added offset of 3.0cm to reduce error
#define ARENA_HEIGHT 75.0f // table: 75cm; circular arena mounted on wooden platform: 1.2cm
#define EPUCK_HEIGHT 4.2f // height of base model in cm
#define EPUCKWITHLINUXBOARD_HEIGHT 8.0f // height in cm of epuck + linux board + tag hat on 30mm standoffs
#define EPUCK_RADIUS 3.5f
#define EPUCK_RADIUS_SQUARE 12.25f
#define EPUCK_DIAMETER 2 * EPUCK_RADIUS
#define EPUCK_ETCHEDTAG_ANGLEOFFSET 30.0f * M_PI / 180.0f //30 degrees in radians
#define MAXWHEELSPEED 10.0f //cm/s
#define INTERWHEELDISTANCE 5.3f //cm
#define MAXANGULARSPEED (MAXWHEELSPEED + MAXWHEELSPEED) / INTERWHEELDISTANCE // rad/s
#define MAXANGULARACCLERATION MAXANGULARSPEED * 2.0f // rad/s/s

typedef unsigned char  UInt8;  // 8-bit unsigned integer.
typedef unsigned short UInt16; // 16-bit unsigned integer.
typedef unsigned int   UInt32; // 32-bit unsigned integer.

using namespace std;
using namespace cv;

/**
 * Stores the step number of the tracking loop control cycle. Each step is approximately equal to 10 ms.
 */
unsigned long long trackingloopstep;

/**
 * Flag indicating the end of the experiment.
 */
volatile sig_atomic_t alarm_flag = false;

/**
 * Map ArUco tag id to IP address
 */
std::map<size_t, std::string> TagID_IPAddress_Map;

/**
 * Client socket descriptor
 */
std::vector<int> socket_desc(NUM_TAGS);

/**
 * Server and client details
 */
//struct sockaddr_in server;
std::vector<struct sockaddr_in> server(NUM_TAGS), client(NUM_TAGS);
static const unsigned int CLIENT_PORT  = 9000; // Client receives data on this port
static const unsigned int SERVER_PORTS = 8800; // Server receives data on ports from  SERVER_PORTS to SERVER_PORTS+NUM_TAGS
static const unsigned int ROBOTS_SYNC_PORT = 10021; // Port used for robot synchronozation messages

/**
 * Data buffer used to send data packets
 */
unsigned int        un_maxdatabuffersize = 1024; //bytes
std::vector<int>    in_filleddatabuffersize(NUM_TAGS, 0);
//std::vector<UInt8*> pun_databuffer(NUM_TAGS, new UInt8[un_maxdatabuffersize]);
std::vector<UInt8*> pun_databuffer(NUM_TAGS);

/**
 * Thread to send data to client
 */
std::vector<pthread_t> sendto_thread(NUM_TAGS); bool threads_terminate;
//std::vector<UInt8*> sendto_thread_args(NUM_TAGS, new UInt8);
std::vector<UInt8*> sendto_thread_args(NUM_TAGS);

/**
 * This mutex protects the concurrent access to the data buffer of packets to send (pun_databuffer)
 */
std::vector<pthread_mutex_t> m_tBufferQueueMutex(NUM_TAGS);

/**
 * This struct defines the range and bearing data packet to send.  It
 * holds the id of the robot, the distance to it in cms (multiplied by 10) and
 * the bearing to it in degrees (multiplied by 10)
 */
struct RangeAndBearingPacket
{
    /**
   * Id of the robot.
    */
    UInt8 RobotId;

    /**
   * Distance in cm.
   */
    UInt16 Range;

    /**
   * Angle with respect to the robot heading.
   */
    UInt16 Bearing;

    /**
   * Four byte data packet.
   */
    UInt8 Data[4];
};

/**
 * The vector of range and bearing data to send. This vector comprises the collection of range-and-bearing data packets.
 */
//std::vector<struct RangeAndBearingPacket> m_vecRangeAndBearingPackets;
std::vector<std::vector<struct RangeAndBearingPacket> > m_vecRangeAndBearingPackets(NUM_TAGS);

/**
 * For the detection and storing the coordinates of ArUco markers
 */
//std::vector<int> marker_ids;
//std::vector<std::vector<Point2f> > marker_corners, rejected_candidates;
Point2f projected_centres[NUM_TAGS], projected_centres_prev[NUM_TAGS];
double  projected_angles[NUM_TAGS],  projected_angles_prev[NUM_TAGS];

/**
 * The RobotGlobalPosData struct stores the coordinates of each tag over a pre-specified time interval. It is used to generate the robot's displacement vector which is then sent to neighbouring robots for the flocking behavior. All positions are in global coordinates
 */
struct RobotGlobalPosData
{
    Point2f pos;
    bool b_PosAvailable; // true if the position is available (tag detected) at the start of the pre-specified time interval (1s). False otherwise.

    RobotGlobalPosData()
    {
        b_PosAvailable = false;
    };
};

/**
 * @brief pos_startinterval and pos_endinterval: The coordinates at the start and end of the pre-specified time interval.
 * b_bothpos_available is set true if the robot positions are available (tag detected) at the start and end of the pre-specified time interval.
 */
std::vector<Point2f> pos_startinterval(NUM_TAGS); std::vector<Point2f> pos_endinterval(NUM_TAGS); std::vector<bool> b_bothpos_available(NUM_TAGS);

/**
 * vec_RobPos_ShortRangeTimeWindow stores the robot coordinates over 10 consecutive steps. Each step is approximately equal to 10 ms.
 */
std::vector<std::vector<RobotGlobalPosData> > vec_RobPos_ShortRangeTimeWindow(NUM_TAGS, std::vector<RobotGlobalPosData>(10));

/**
 * An index to the vec_RobPos_ShortRangeTimeWindow vector. The index is incremented approx. every 10ms
 */
std::vector <unsigned long long> displacement_pos_index(NUM_TAGS, 0);

/**
 * The robot bearing, angular velocity and angular acceleration (all in radians)
 */
std::vector<double> m_vecRobotBearing(NUM_TAGS, 0.0f), m_vecRobotBearing_Prev(NUM_TAGS, 0.0f), m_vecRobotAngularVelocity(NUM_TAGS, 0.0f), m_vecRobotAngularAcceleration(NUM_TAGS, 0.0f);

/**
 * Records the time at which
 */
std::vector<double> m_vecRobotBearingAtTime(NUM_TAGS, 0.0f);

/**
 * The matrix stores 1 if robots indexed by their Ids are in line of sight (no robot intersects line segment between the two robots center coordinates), stores -1 otherwise.
 * Occlusion check: Occluding robot -- a circle with radius EPUCK_RADIUS intersects line segment between emitter and receiver robots.
 */
std::vector<std::vector<int>> m_vec2dLineOfSightMatrix(NUM_TAGS , std::vector<int>(NUM_TAGS, 0));

/**
 * @brief TrackRobotDisplacement Tracks the positions of the robot over a pre-specified interval of approx. 1 second. All positions are in global coordinates.
 * This function will store the robot's coordinates in displacement_vector.It will also extract the start (pre-interval) and end (post-interval) global coordinates.
 * @param step: Current step number of the tracking loop (approx. 10ms)
 * @param marker_id: Id of the robot being tracked
 * @param displacement_vector: Vector storing the robot coordinates
 * @param src_pos: Extracted source coordinates of the robot -- at the start of the pre-specified interval
 * @param dst_pos: Extracted end coordinates of the robot -- at the end of the pre-specified interval
 * Function returns true if the displacement vector of the robots motion over approx. 1 second is available.
 */
bool TrackRobotDisplacement(unsigned long long& step, int marker_id, Point2f current_pos,
                            std::vector<RobotGlobalPosData>& displacement_vector,
                            bool b_current_pos_present,
                            Point2f& src_pos, Point2f& dst_pos);

/**
 * Function computes the range and bearing data for robot with ArUco Id marker_ids[marker_index]. Computed range and bearing data is stored in m_vecRAB.
 * Returns the number of range-and-bearing data packets
 */
int ComputeRangesAndBearing(int marker_index, std::vector<int> &marker_ids, Point2f (&projected_centres)[NUM_TAGS], double (&projected_angles)[NUM_TAGS],
                            std::vector<RangeAndBearingPacket> &m_vecRAB);
/**
 * Function adds the angular acceleration data to the range and bearing packet for robot with ArUco Id marker_ids[marker_index].
 */
void AddAngularAcceleration(int marker_id, struct RangeAndBearingPacket& NewRangeAndBearingPacket, std::vector<double>& AngularAcceleration);

/**
 * Function adds the relative displacement vector data to the range and bearing packet for a focal robot with ArUco Id marker_ids[marker_index].
 * The data is the displacement vector V=(nbr_robot_endpos-nbr_robot_startpos) of a neighbouring robot to the focal robot.
 * The displacement vector V is converted from global coordinate frame to the local coordinate frame of the focal robot.
 */
void AddDisplacementVector(int marker_id, struct RangeAndBearingPacket& NewRangeAndBearingPacket, 
                           Point2f focal_robot_pos, double focal_robot_angle,
                           bool nbr_robot_pos_available, Point2f nbr_robot_startpos, Point2f nbr_robot_endpos);

/**
 * Check if robots are in line-of-sight.
 * Result stored in global variable m_vec2dLineOfSightMatrix
 */
void ComputeLineOfSight(std::vector<int> &marker_ids, Point2f (&projected_centres)[NUM_TAGS]);

/**
 * Check if robots with ArUco Id marker_ids[src_marker_index] and marker_ids[dst_marker_index] are in line-of-sight.
 * Returns true if in line of sight, false otherwise
 */
bool InLineOfSight(int src_marker_index, int dst_marker_index, std::vector<int> &marker_ids)
{
    //return true;
    /**
   * We disable line-of-sight check for now
      */
    if(m_vec2dLineOfSightMatrix[marker_ids[src_marker_index]][marker_ids[dst_marker_index]] == 1)
        return true;
    else
        return false;
}

/**
 * Serialize the range and bearing data packet in m_vecRAB for sending. Serialized bytes pointed to by pun_Buffer with size in serializedbuffersize
 */
void SerializeRangeAndBearingPackets(int marker_id, std::vector<struct RangeAndBearingPacket>& m_vecRAB, UInt8* pun_Buffer, int &serializedbuffersize);

/**
 * Prepare the network connection interface for sending data
 * Returns 0 on success, 1 on failure
 */
unsigned InitNetwork();

/**
 * Prepare the threads used to send data
 * Returns 0 on success, 1 on failure
 */
unsigned InitThreads();

/**
 * Function sends the data in pun_databuffer to robot with ArUco Id marker_id and IP address at client[marker_id].
 */
void *Sendto_Handler(void *marker_id);

/**
 * @brief handle_alarm
 * @param sig
 * Handler of the SIGALRM signal
 */
void handle_alarm( int sig )
{
    alarm_flag = true;
}

///**
// * Function sends the data in m_vecRangeAndBearingPackets to robot with ArUco Id marker_ids[marker_index] and IP address TagID_IPAddress_Map[marker_ids[marker_index]].
// * Returns the number of bytes sent. Returns -1 if there is an error
// */
//int SendRangeAndBearingData(int marker_index, std::vector<int> &marker_ids);

#endif

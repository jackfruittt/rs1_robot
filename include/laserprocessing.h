#ifndef LASERPROCESSING_H
#define LASERPROCESSING_H

#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <math.h>
#include <mutex>

class LaserProcessing
{
public:
  /*! @brief Constructor that allocates internals
   *
   *  @param[in]    laserScan - laserScan to be processed
   */
  LaserProcessing(sensor_msgs::msg::LaserScan laserScan);

  /*! @brief Count number of readings belonging to objects (not infinity, nan or max range) from the last
  * laser scan provided (either via @sa newScan or @sa LaserProcessing constructor)
  * thread-safe function, internally creates a copy fo laserScan_ 
  *
  * @return the number of laser readings that belong to objects
  */
  unsigned int countObjectReadings();


  /*! @brief Accepts a new laserScan, threadsafe function
   *  @param[in]    laserScan  - laserScan supplied
   */
  void newScan(sensor_msgs::msg::LaserScan laserScan);


private:
  /*! @brief Returns the cartesian position of laer reading at specific index
   * converted from polar coordinats stored in the #laserScan_
   *  @param[in] index - the reading needing conversion
   *  @return position cartesian values
   */
   geometry_msgs::msg::Point polarToCart(unsigned int index);

   /*! @brief Given two points (only x,y are used), returns the slope slope of the lines connecting them
    *  @param[in] p1 - first point
    *  @param[in] p2 - second point
    *  @return slope of line between the points in radians
    */
  double angleConnectingPoints(geometry_msgs::msg::Point p1, geometry_msgs::msg::Point p2);

private:
  sensor_msgs::msg::LaserScan laserScan_;
  std::mutex mtx; //!< Mutex to protect the laserScan_ from being accessed by multiple threads
  unsigned int objectReadings_; //!< Number of readings belonging to objects
};

#endif // LASERPROCESSING_H
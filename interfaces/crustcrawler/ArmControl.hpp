/*!
 * \class ArmControl
 * \brief Convert requested joint angles into servo control commands.
 * 
 * Manage the rate and joint limits allowed for movement of the arm.
 * Convert commands from absolute or delta joint angles in radians to
 * PWM command strings that the SSC32Controller can handle.  Apply
 * maximum rate (in microseconds per second of PWM) or minimum time
 * (in seconds to complete move) constraints as applicable.
 */

/*
 * VisualServoing is a tutorial program for introducing students to
 * robotics.
 *
 * Copyright 2009, 2010 Kevin Quigley <kevin.quigley@gmail.com> and
 * Marsette Vona <vona@ccs.neu.edu>
 *
 * VisualServoing is free software: you can redistribute it andor modify 
 * it under the terms of the GNU General Public License as published by 
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version. 
 *
 * VisualServoing is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
 * GNU General Public License for more details. 
 *
 * You should have received a copy of the GNU General Public License 
 * as the file COPYING along with VisualServoing.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef ARM_CONTROL_HPP
#define ARM_CONTROL_HPP

#include "Command.hpp"
#include "SSC32Controller.hpp"
#include <boost/numeric/ublas/vector.hpp>
#include <map>

using namespace boost::numeric;

#define NUM_JOINTS 5 //!< Number of joint in arm

namespace Arm {
  //! Servo port numbers.  Change here if configuration differs.
  enum {
    BASE     = 4,
    SHOULDER = 3,
    ELBOW    = 2,
    WRIST    = 1,
    GRIP     = 0,
  };
}

/*!
 * Class for handling control commands and translating into commands
 * used by the SSC32Controller.
 */
class ArmControl {

public:
  ArmControl(SSC32Controller& s, bool slave_theta3 = true);
  ~ArmControl(void);

  void slaveWrist(bool s) { slave_theta3 = s; };
  void setRateLimit(unsigned int r) { max_rate_set = true; max_rate = r; };
  void clearRateLimit(void) {max_rate_set = false;};

  void moveDelta(const ublas::vector<double>& dtheta);
  const ublas::vector<double>& getCurrentAngles(void) {return position;};

  void moveToPosition(const ublas::vector<double>& pos,
		      unsigned int time = 0);

  void setOffset(const ublas::vector<int> off) {offset = off;};
  const ublas::vector<int>& getOffset(void) {return offset;};

  void setMaxAngle(const ublas::vector<double>& max) {max_angle = max;};
  void setMinAngle(const ublas::vector<double>& min) {min_angle = min;};
  const ublas::vector<double>& getMaxAngle(void) {return max_angle;};
  const ublas::vector<double>& getMinAngle(void) {return min_angle;};

  void grabMarker(void);
  void grabBall(void);
  void openGrip(void);

  
  void stop(void);
  bool busy(void) { return /*!(control->isMoveCompleted()) || TBD*/ set_busy; };
  void park(void) { control->move(park_cmd); position = park_position; };
  void home(void);

  // mutators
  static void degreesToRadians(ublas::vector<double>& v);
  static void radiansToDegrees(ublas::vector<double>& v);
  ublas::vector<double> position; //!< Current position

private:
  unsigned int thetaToPwm(double theta, int offset = 0);
  double pwmToTheta(unsigned int pwm, int offset = 0);

  SSC32Controller* control;
  bool slave_theta3; //!< If true, theta3 is not directly commandable
  ublas::vector<int> offset; //!< Calibration offsets
  ublas::vector<double> max_angle; //!< Maximum joint limits
  ublas::vector<double> min_angle; //!< Minimum joint limits
  
  bool set_busy; //!< Report arm busy processing another command

  bool max_rate_set; //!< TRUE = use max rate, FALSE = no software limit
  unsigned int max_rate; //!< max microsecond/second change in PWM pulse
  
  Command park_cmd; //!< saved command
  ublas::vector<double> park_position; //!< angles to command for park
				       //!  position
};

#endif

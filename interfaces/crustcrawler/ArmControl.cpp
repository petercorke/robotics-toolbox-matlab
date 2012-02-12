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
#include "ArmControl.hpp"

#include <boost/numeric/ublas/io.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <math.h>
#include <limits.h>
#include <stdexcept>
#include <unistd.h>
#include <stdio.h>

/*!
 * Initialize control using the SSC32Controller.  Setup the saved park
 * command and the arm to park.
 *
 * \param s The SSC32Controller.
 * \param slave If true, the WRIST joint depends on the values of
 *              SHOULDER and ELBOW to remain horizontal.
 */
ArmControl::ArmControl(SSC32Controller& s, bool slave)
  : control(&s),
    slave_theta3(slave),
    position(NUM_JOINTS), offset(NUM_JOINTS),
    max_angle(NUM_JOINTS), min_angle(NUM_JOINTS),
    set_busy(false), max_rate_set(false),
    park_position(NUM_JOINTS)
{

  for (unsigned int i = 0; i < position.size(); i++) position(i) = 0.0;
  for (unsigned int i = 0; i < offset.size(); i++) offset(i) = 0;

  park_position(0) = pwmToTheta(1500);
  park_position(1) = pwmToTheta(2100);
  park_position(2) = pwmToTheta(2000);
  park_position(3) = pwmToTheta(2400);
  park_position(4) = pwmToTheta(1800);
  std::cout << "park position: " << park_position << std::endl;

  park_cmd.addMove(Arm::BASE,     1500);  // values on p31
  park_cmd.addMove(Arm::SHOULDER, 2100);
  park_cmd.addMove(Arm::ELBOW,    2000);
  park_cmd.addMove(Arm::WRIST,    2400);
  park_cmd.addMove(Arm::GRIP,     1800);
  park_cmd.minTime(4000); // 4 sec

  control->move(park_cmd);
  //park(); // TBD - same, but sets position?
}

/*!
 * Destroy this objects, releasing all servos from control on exit.
 */
ArmControl::~ArmControl(void) {
  //  printf("parking\n"); //TBD this causes ugly delay on app shutdown
  //  control->move(park_cmd);
  //  sleep(5);
  printf("release servos\n");
  control->releaseAll();
  //  sleep(1);
}

/*!
 * Move by the specified delta angle with relation to the current
 * angular position of the joints.
 *
 * \param[in] dtheta The requested change to the current joint angles
 *                    (radians).
 */
void
ArmControl::moveDelta(const ublas::vector<double>& dtheta) {

  for (unsigned int i = 0; i < dtheta.size(); i++)
    if (dtheta(i) != dtheta(i))
      throw std::runtime_error(std::string("commanded dtheta NaN at index ")+
                               boost::lexical_cast<std::string>(i));

  for (unsigned int i = 0; i < dtheta.size(); i++)
    if(fabs(dtheta(i)) == std::numeric_limits<double>::infinity())
      throw std::runtime_error(std::string("commanded dtheta inf at index ")+
                               boost::lexical_cast<std::string>(i));
  
  for (unsigned int i = 0; i < (dtheta.size() < position.size()
                                ? dtheta.size() : position.size()); i++)
    position(i) += dtheta(i);
  
  moveToPosition(position, 50);
}

/*!
 * Move to the specified joint angle position taking at least time to
 * move.  The move may take longer than time to complete.
 *
 * \param[in] pos The requested joint position (radians).
 * \param[in] time The minimum time of movement (milliseconds).
 */
void
ArmControl::moveToPosition(const ublas::vector<double>& pos,
                           unsigned int time) {

  for (unsigned int i = 0; i < pos.size(); i++)
    if (pos(i) != pos(i))
      throw std::runtime_error(std::string("commanded position NaN at index ")+
                               boost::lexical_cast<std::string>(i));

  for (unsigned int i = 0; i < pos.size(); i++)
    if(fabs(pos(i)) == std::numeric_limits<double>::infinity())
      throw std::runtime_error(std::string("commanded position inf at index ")+
                               boost::lexical_cast<std::string>(i));

  position = pos;

  printf("commanded position [");
  for (unsigned int i = 0; i < position.size(); i++)
    printf("%10f,", position(i));
  printf("]\n");
  
  if (slave_theta3)
    position(3) = M_PI_2 - position(1) - position(2);

  for (unsigned int i = 0; i < position.size(); i++) {
    if (position(i) < min_angle(i))
      position(i) = min_angle(i);
    else if (position(i) > max_angle(i))
      position(i) = max_angle(i);
  }

  printf("actual position [");
  for (unsigned int i = 0; i < position.size(); i++)
    printf("%10f,", position(i));
  printf("]\n");
  
  std::cout << "offsets: " << offset << std::endl;
  Command cmd;
  cmd.addMove(Arm::BASE,
              thetaToPwm(position(0), offset(Arm::BASE)),
              max_rate_set ? max_rate : Command::ANY_SPEED);
  cmd.addMove(Arm::SHOULDER,
              thetaToPwm(position(1), offset(Arm::SHOULDER)),
              max_rate_set ? max_rate : Command::ANY_SPEED);
  cmd.addMove(Arm::ELBOW,
              thetaToPwm(position(2), offset(Arm::ELBOW)),
              max_rate_set ? max_rate : Command::ANY_SPEED);
  if (slave_theta3)
    cmd.addMove(Arm::WRIST,
                thetaToPwm(M_PI_2 - position(1) - position(2),
                           offset(Arm::WRIST)),
                max_rate_set ? max_rate : Command::ANY_SPEED);
  else cmd.addMove(Arm::WRIST,  thetaToPwm(position(3), offset(Arm::WRIST)));
  //cmd.addMove(Arm::GRIP,     thetaToPwm(position(4), offset(Arm::GRIP)));
  
  if (time != 0) cmd.minTime(time);

  control->move(cmd);
}

/*!
 * Convert an angle to a PWM length, accounting for a calibration offset.
 *
 * \param[in] theta The angle to convert (radians).
 * \param[in] offset Calibration PWM offset (microseconds).
 *
 * \return The equivalent PWM length (microseconds).
 */
unsigned int
ArmControl::thetaToPwm(double theta, int offset) {
  // scale theta to -PI:PI
  //while (theta < -M_PI) theta -= ((int)theta%(int)M_PI)*M_PI;
  //while (theta >  M_PI) theta -= ((int)theta%(int)M_PI)*M_PI;

  // PI/2:-PI/2 == 600:2400
  // negate theta because +theta is low PWM
  unsigned int pwm = trunc((-theta + M_PI_2) * 1800 / M_PI) + 600 + offset;
  if (pwm < 500) return 500;
  if (pwm > 2500) return 2500;
  return pwm;
}

/*!
 * Convert the PWM length to an angle, accounting for a calibration
 * offset.
 *
 * \param[in] pwm The PWM length (microseconds).
 * \param[in] offset Calibration PWM offset (microseconds).
 *
 * \return The equivalent angle (radians).
 */
double
ArmControl::pwmToTheta(unsigned int pwm, int offset) {
  // PI/2:-PI/2 == 600:2400
  // theta value is negative because low PWM is +theta
  return -((((double)pwm - 600.0 - offset) * M_PI / 1800.0) - M_PI_2);
}

/*!
 * Command the arm to a saved \a home position.
 */
void
ArmControl::home(void) {
  bool temp = slave_theta3;
  slave_theta3 = true;
  ublas::vector<double>pos(ublas::zero_vector<double>(NUM_JOINTS));
  pos(0) = 0.0;
  pos(1) = -M_PI_4;
  pos(2) = M_PI_2;
  moveToPosition(pos,1000);
  slave_theta3 = temp;
}

/*!
 * Command the grip to close to a position which would grab the ball.
 */
void
ArmControl::grabBall(void) {
  Command cmd;
  cmd.addMove(Arm::GRIP, 1350 + offset(Arm::GRIP), 500);
  control->move(cmd);
}

/*!
 * Command the grip to close to a position which would grab a
 * specially designed marker-holder for the class exercise.
 */
void
ArmControl::grabMarker(void) {
  Command cmd;
  cmd.addMove(Arm::GRIP, 1350 + offset(Arm::GRIP) - 200, 500);
  control->move(cmd);
}

/*!
 * Open the gripper.
 */
void
ArmControl::openGrip(void) {
  Command cmd;
  cmd.addMove(Arm::GRIP, 1350 + offset(Arm::GRIP) + 500, 500);
  control->move(cmd);
}

/*!
 * Stop movement of the arm.
 * \b NOTE: Although the SSC-32 documentation lists: <tt>"<esc> =
 * Cancel the current command, ASCII 27."</tt> (e.g.0x1b) this does
 * not stop single or group moves in progress.  The only way to
 * effectively do this is to read each servo position and then command
 * that position again to override any existing command.  Since
 * querying is not currently implemented, calling stop() releases all
 * servo power.
 */
void
ArmControl::stop(void) {
  control->releaseAll();
}

/*!
 * \param[in,out] v The provided vector of degree values, converted in
 * place to radian values.
 */
void
ArmControl::degreesToRadians(ublas::vector<double>& v) { v *= M_PI / 180.0; }

/*!
 * \param[in,out] v The provided vector of radian values, converted in
 * place to degree values.
 */
void
ArmControl::radiansToDegrees(ublas::vector<double>& v) { v *= 180.0 / M_PI; }

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
#ifndef SSC32CONTROLLER_HPP
#define SSC32CONTROLLER_HPP

#include "Command.hpp"

#include <list>
#include <string>
#include <termios.h>


class SSC32Controller {

public:
  static const unsigned int NUM_SERVOS = 32;
  static const unsigned int NUM_BANKS = 4;

  struct SPS {
    unsigned int servo;
    unsigned int position;
    unsigned int speed;
  };

  enum BAUD {
    B_2400 = B2400,
    B_9600 = B9600,
    B_38_4K = B38400,
    B_115_2K= B115200,
  };
  
  SSC32Controller(const std::string& port_name = "/dev/ttyS0",
                  //BAUD baud = B_9600);
                  BAUD baud = B_115_2K);
  ~SSC32Controller(void);

  
  unsigned int getMaxServos(void) { return NUM_SERVOS; }


  /// movements
  void release(unsigned int servo);
  void release(std::list<unsigned int> servos);
  void releaseAll(void);

  /* bool isMoveCompleted(void); TBD */

  /* unsigned int getPosition(unsigned int servo); TBD */

  void move(const Command& cmd);
  void movePWM(unsigned int servo,
               unsigned int pwm,
               unsigned int speed = Command::ANY_SPEED,
               unsigned int time = Command::NO_TIME);

  void movePWM(std::list<SPS> servos, unsigned int time = Command::NO_TIME);


private:
  void write(const std::string s);

  int port;
  Command cmd;
  struct termios newtio;
  struct termios oldtio;

};


#endif //#ifndef SSC32CONTROLLER_HPP

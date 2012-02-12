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
#ifndef COMMAND_HPP
#define COMMAND_HPP

#include <list>
#include <sstream>

class Command {

public:
  static const unsigned int NO_TIME = 0;
  static const unsigned int ANY_SPEED = 0;

  Command(const char c = 0);
  
  void addMove(unsigned int servo,
               unsigned int pwm,
               unsigned int speed = ANY_SPEED,
               unsigned int time = NO_TIME);
  
  void minTime(unsigned int msec);

  void reset(void);

  operator const std::string() const { return cmd.str(); }

  static const Command& release(unsigned int servo);
  static const Command& release(const std::list<unsigned int> servos);
  static const Command& releaseAll(unsigned int num_servos);
  static const Command& queryMove(void);
  static const Command& queryPosition(unsigned int servo);

private:
  static const char CR = '\r';
  static const char CANCEL = 0x1b;

  std::stringstream cmd;
};


#endif

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
#include "Command.hpp"


Command::Command(const char c) {
  if (c != 0) cmd << c;
}


void Command::addMove(unsigned int servo,
                      unsigned int pwm,
                      unsigned int speed,
                      unsigned int time) {
  cmd << "#" << servo
      << "P" << pwm;

  if (speed != ANY_SPEED)
    cmd << "S" << speed;

  if (time != NO_TIME)
    minTime(time);
}

void Command::minTime(unsigned int time) {
  if (time == NO_TIME) {
    //perror("Time commanded is not possible");
    return;
  }

  cmd << "T" << time;
}

void Command::reset(void) {
  cmd.str("");
  cmd.clear();
}


const Command& Command::release(unsigned int servo) {
  static Command cmd;
  cmd.reset();
  cmd.addMove(servo,0); // zero move releases power
  return cmd;
}

const Command& Command::release(std::list<unsigned int> servos) {
  static Command cmd;
  cmd.reset();
  for (std::list<unsigned int>::const_iterator c = servos.begin();
       c != servos.end();
       c++) {
    cmd.addMove(*c,0); // zero move releases power
  }
  return cmd;
}


const Command& Command::releaseAll(unsigned int num_servos) {
  static Command cmd;
  cmd.reset();
  for (unsigned int i = 0; i < num_servos; i++) {
    cmd.addMove(i,0);
  }
  return cmd;
}

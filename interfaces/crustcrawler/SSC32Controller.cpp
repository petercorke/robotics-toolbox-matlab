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
#include "SSC32Controller.hpp"

#include <iostream>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>

SSC32Controller::SSC32Controller(const std::string& port_name, BAUD baud) {

    std::cerr << "opening port \"" << port_name << "\""<< std::endl;
  port = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (port < 0) {

    perror(port_name.c_str());
    //    exit(port);

  } else {

    std::cerr << "opened port \"" << port_name << "\""<< std::endl;

    tcgetattr(port, &oldtio);  // save current serial port settings
    
    memset(&newtio, 0x00, sizeof(newtio));
    
    // set baud rate
    cfsetispeed(&newtio, baud); // input
    cfsetospeed(&newtio, baud); // output
    
    // CRTSCTS/CNEW_RTSCTS: output hardware flow control
    // set to NONE
#if defined(CRTSCTS)
    newtio.c_cflag &= ~CRTSCTS;
#elif defined(CNEW_RTCTS)
    newtio.c_cflag &= ~CNEW_RTSCTS;
#endif
    
    // CLOCAL  : local connection, no modem contol
    // CREAD   : enable receiving characters
    newtio.c_cflag |= CLOCAL | CREAD;
 
    // set parity 8N1
    newtio.c_cflag &= ~CSIZE;  // data mask
    newtio.c_cflag |= CS8;     // 8
    newtio.c_cflag &= ~PARENB; // N
    newtio.c_cflag &= ~CSTOPB; // 1
    
    // use raw data, no processing or signals
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    newtio.c_oflag &= ~OPOST;
    
    newtio.c_iflag |= ICRNL;  // map CR to NL
    
    
    newtio.c_oflag = 0; // raw output
    
    tcflush(port, TCIOFLUSH); // clear all data on line
    tcsetattr(port, TCSANOW, &newtio); // set up port
  }
}

SSC32Controller::~SSC32Controller(void) {
  if (port >= 0) {
    std::cout << "closing port" << std::endl;
    sleep(1);
    tcsetattr(port, TCSANOW, &oldtio); // reset port to old settings
    if (close(port) < 0) {
      perror("error closing port");
    }
    port = -1;
  }
}

void SSC32Controller::release(unsigned int servo) {
  write(Command::release(servo));
}

void SSC32Controller::release(std::list<unsigned int> servos) {
  write(Command::release(servos));
}

void SSC32Controller::releaseAll(void) {
  write(Command::releaseAll(NUM_SERVOS));
}

void SSC32Controller::move(const Command& cmd) {
  write(cmd);
}

void SSC32Controller::movePWM(unsigned int servo,
                              unsigned int pwm, unsigned int speed,
                              unsigned int time) {
  cmd.reset();
  cmd.addMove(servo, pwm, speed);
  cmd.minTime(time);
  write(cmd);
}

void SSC32Controller::movePWM(std::list<SPS> servos, unsigned int time) {
  cmd.reset();
  for (std::list<SPS>::const_iterator c = servos.begin();
       c != servos.end();
       c++ ) {
    cmd.addMove(c->servo, c->position, c->speed);
  }
  cmd.minTime(time);
  write(cmd);
}

/*
bool SSC32Controller::isMoveCompleted(void) {

  //write(Command::queryMove());

  //TBD read

  return true;
}
*/


/*
unsigned int SSC32Controller::getPosition(unsigned int servo) {
  cmd.reset();

  //cmd.queryPosition(servo);

  //write(cmd);

  //TBD read

  return 0;
}
*/

void SSC32Controller::write(const std::string cmd) {
  printf("sending command: %s\n", cmd.c_str());
  if (port >= 0) {
    int n = ::write(port, cmd.c_str(), cmd.length());
    if (n < 0) {
      perror("error writing command");
      return;
    }
    char CR = '\r';
    if (::write(port, &CR, sizeof(CR)) != sizeof(CR))
      perror("error writing command terminator");
    else
      printf("ok\n");
  }
}

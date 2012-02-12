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

// system headers
#include <cstdio>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <unistd.h>

// CTY arm project
#include "ArmControl.hpp"
#include "ArmGui.hpp"
#include "ArmGuiGTK.hpp"
#include "IK.hpp"
#include "ImageProcessing.hpp"
#include "Params.hpp"

// OpenCV
#include <cv.h>
#include <highgui.h>

// Boost
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

using namespace boost::numeric;

#if RUN_THREADED
#include <errno.h>
#include <pthread.h>
#endif

// constants
#define FOCAL_LENGTH  481.0 // calc p65 //TBD - add calculation info
#define DIAMETER      .038  //!< Diameter of ball in meters (measurexd)
#define MIN_TRACKING_RADIUS_PIXELS 2.0 //!< Minimum tracking radius required

void mark_images(const ublas::vector<double>& target, const CvSeq* circles,
                 const Params& params, Images& images);

void calibrate_offsets(std::string& file, ublas::vector<double>& offsets);
void update_gui_position (ArmControl& ctl, Params& params); 

void handler(int sig);
ArmControl* sig_ctl = 0; //!< Pointer to ArmControl for stopping arm
			 //!  movement upon received signal.


/*!
 * \Brief Starting function containing main control loop.
 * Start up and configure all objects, spin off the GUI, and continue
 * in main loop until told to stop.
 */
int main(int argc, char** argv) {
  // set signal handling
  struct sigaction action;
  action.sa_handler = &handler;
  
  if (sigaction(SIGHUP, &action, NULL) < 0)
    printf("Error setting action for SIGHUP\n");
  if (sigaction(SIGINT, &action, NULL) < 0)
    printf("Error setting action for SIGINT\n");
  if (sigaction(SIGQUIT, &action, NULL) < 0)
    printf("Error setting action for SIGQUIT\n");
  if (sigaction(SIGILL, &action, NULL) < 0)
    printf("Error setting action for SIGILL\n");
  if (sigaction(SIGABRT, &action, NULL) < 0)
    printf("Error setting action for SIGABRT\n");
  if (sigaction(SIGFPE, &action, NULL) < 0)
    printf("Error setting action for SIGFPE\n");
  if (sigaction(SIGSEGV, &action, NULL) < 0)
    printf("Error setting action for SIGSEGV\n");
  if (sigaction(SIGTERM, &action, NULL) < 0)
    printf("Error setting action for SIGTERM\n");

  Images images;
  images.set = false;
  images.bgr = 0;
  images.filtered_bgr = 0;
  images.filtered_hls = 0;

  Params params;
  init_params(params);
  CvSeq* circles = 0;

  unsigned int cameraID(0);
  std::string port("/dev/ttyS0");
  std::string config_file;

  std::string flags = "hp:f:";
  int opt;
  bool help = false;
  while ((opt = getopt(argc, argv, flags.c_str())) > 0) {
    switch (opt) {
    case 'h': help = true;          break;
    case 'p': port = optarg;        break;
    case 'f': config_file = optarg; break;
    default: break;
    }
  }

  if (help) {
    printf("Visual Servo Arm Options:\n"
	   "  -h         Print this help menu\n"
	   "  -f <file>  Use a calibration file to set joint offsets\n"
	   "  -p <port>  Use an alternate serial port (default: /dev/ttyS0\n");
    exit(0);
  }
  
  CvCapture* capture(0);
  IplImage* frame(0);

  ImageProcessing ip;

  ublas::vector<double> features(3);
  ublas::vector<double> delta_angles(3);
  ublas::vector<double> target_pos(3);
  ublas::vector<double> grab_target(3);

  target_pos(0) = 0.0; //x
  target_pos(1) = 0.0; //y
  target_pos(2) = 0.2; //Z

  grab_target(0) = 0.0; //x
  grab_target(1) = 0.0; //y
  grab_target(2) = 0.05; //Z

  // div by focal_length to normalize target x,y
  ublas::vector<double> target_pos_norm(target_pos);
  target_pos_norm(0) /= FOCAL_LENGTH;
  target_pos_norm(1) /= FOCAL_LENGTH;

  IK ik;
  ik.setTarget(target_pos_norm);
  ik.setLengths(0.0, .152, 0.122, 0.075);
  ik.setV(.015, -.150, .25); //m, m, rad

  ArmGuiGTK* gui = ArmGuiGTK::instance();
  gui->update(images, params);

#if RUN_THREADED
  pthread_t guiTID;
  switch (pthread_create(&guiTID, 0, ArmGui::threadRun, gui)) {
  case EAGAIN: printf("Max threads reached\n"); return -1;
  case EINVAL: printf("Invalid thread attributes\n"); return -1;
  case EPERM: printf("Invalid permissions\n"); return -1;
  default: break;
 }
#endif

  SSC32Controller ssc(port);
  ArmControl ctl(ssc);
  sig_ctl = &ctl;
  ctl.setRateLimit(500);

  ublas::vector<double> off(ublas::zero_vector<double>(NUM_JOINTS));
  calibrate_offsets(config_file, off);
  ctl.setOffset(off);

  ublas::vector<double> angle_limits(ublas::vector<double>(NUM_JOINTS));

  // max limits
  angle_limits(0) = 3.0/8.0 * M_PI;
  angle_limits(1) = M_PI_2;
  angle_limits(2) = M_PI - .70; // off arm brace
  angle_limits(3) = M_PI_2;
  std::cout << "max limits: " << angle_limits << std::endl;
  ctl.setMaxAngle(angle_limits);

  ArmControl::radiansToDegrees(angle_limits);
  for (int i = 0; i < NUM_JOINTS; i++)
    params.ctl.max_limits[i] = angle_limits(i);
  params.limits_changed = true;
  
  // min limits
  angle_limits(0) = -3.0/8.0 * M_PI;
  angle_limits(1) = -M_PI_2 + 0.35; // off spring pedestal
  //  angle_limits(2) =  0;
   angle_limits(2) =  -50.0*2.0*M_PI/360.0;
  angle_limits(3) = -M_PI_2;
  ctl.setMinAngle(angle_limits);
  std::cout << "min limits: " << angle_limits << std::endl;

  ArmControl::radiansToDegrees(angle_limits);
  
  for (int i = 0; i < NUM_JOINTS; i++)
    params.ctl.min_limits[i] = angle_limits(i);
  params.limits_changed = true;

  ctl.park();
  update_gui_position(ctl, params);
  params.current_mode = PARK;
  

  while (params.run) { //mainloop

    gui->update(images, params);
    
#if !RUN_THREADED
    gui->run();
#endif

    if (!params.run) continue; //to next mainloop iteration

    if (params.gui.estop) {
      params.gui.estop = false;
      printf("ESTOP received\n");
      ctl.stop();
      if (params.current_mode != ESTOP) {
        params.current_mode = ESTOP;
      }
    }

    // all activities respond to these new modes
    switch (params.new_mode) {
    case HOME:
      params.new_mode = NONE;
      printf("*** -> HOME\n");
      ctl.home();
      update_gui_position(ctl, params);
      params.current_mode = READY;
      break;
    case PARK:
      printf("park request\n");
      params.new_mode = NONE;
      printf("*** -> PARK\n");
      ctl.park();
      update_gui_position(ctl, params);
      params.current_mode = PARK;
      break;
    default:
      break;
    }

    // all activities respond to these current modes
    switch (params.current_mode) {
    case HOME:
      printf("HOME->READY\n");
      params.current_mode = READY;
      break;
    case PARK:
      // getting out of PARK handled above
      usleep(10000); // 10ms
    case BUSY:
      printf("BUSY -> READY\n");
      if (!ctl.busy())
        params.current_mode = READY;
      break;
    default:
      break;
    }
    
    if (params.activity == KINEMATICS) {

      usleep(10000); // 10ms

      ctl.slaveWrist(false);
      ublas::vector<double> new_position(NUM_JOINTS);
      
      if (params.current_mode == READY) {
        switch (params.new_mode) {
        case MOVE:
          params.new_mode = NONE;
          printf("Moving\n");
          for (int i = 0; i < NUM_JOINTS; i++ )
            new_position(i) = params.gui.new_theta[i];
          ArmControl::degreesToRadians(new_position);
          ctl.moveToPosition(new_position);
          update_gui_position(ctl, params);
          break;

        case DRAW:
          params.new_mode = NONE;
          printf("Drawing\n");
          if (params.ctl.holding_marker) {
            //ctl.drawX();
          } else {
            params.new_mode = ERROR;
            params.error = "Must hold marker to draw.";
          }
          break;
          
          // end movement modes
        case GRAB:
          params.new_mode = NONE;
          printf("Grab marker\n");
          if (!params.ctl.holding_marker) {
            ctl.grabMarker();
            //sleep(1);
            params.ctl.holding_marker = true;
          } else {
            printf("error set\n");
            params.error_set = true;
            params.error = "Marker already held\n";
          }
          break;

        case RELEASE:
          params.new_mode = NONE;
          printf("Release marker\n");
          if (params.ctl.holding_marker) {
            ctl.openGrip();
            params.ctl.holding_marker = false;
          } else {
            params.error_set = true;
            params.error = "Marker not being held\n";
          }
          break;

	default:
	  break;
        }
      }
      
      // update param struct

      continue; //to next mainloop iteration

    } //end of kinematics

    //
    // Setup code for Image Processing and Visual Servoing
    //

    if (capture == 0) {

      capture = cvCreateCameraCapture(cameraID);

      if (capture == 0) {
        printf("failed to init capture device\n");
        sleep(1); continue; //to next mainloop iteration
      }

      printf("initialized capture device\n");
      printf("allocating images\n");

      images.bgr = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_8U, 3);

#if FLOAT_HLS
      images.bgr32 = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_32F, 3);
      images.hls = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_32F, 3);
#else
      images.hls = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_8U, 3);
#endif

      images.filtered_bgr = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_8U, 1);
      images.filtered_hls = cvCreateImage(IMAGE_SIZE, IPL_DEPTH_8U, 1);

      if (images.bgr == 0 || images.hls == 0
#if FLOAT_HLS
          || images.bgr32 == 0
#endif
          || images.filtered_bgr == 0 || images.filtered_hls == 0) {
        params.current_mode = ERROR;
        params.error = "Cannot create image holders";
        std::cout << params.error << std::endl;
        params.run = false;
        continue; //to next mainloop iteration
      }

      //some images might be displayed before being initialized

      cvSet(images.bgr, cvScalar(0,0,0));
#if FLOAT_HLS
      cvSet(images.bgr32, cvScalar(0,0,0));
#endif
      cvSet(images.hls, cvScalar(0,0,0));
      cvSet(images.filtered_bgr, cvScalar(0));
      cvSet(images.filtered_hls, cvScalar(0));

      images.set = true;

    } //capture was 0

    //
    // Image Processing
    //
    frame = cvQueryFrame(capture);
    
    if (frame == 0) {
      params.current_mode = ERROR;
      params.error = "Null frame";
      std::cout << params.error << std::endl;
      params.run = false;
      continue; //to next mainloop iteration
    }

    cvResize(frame, images.bgr);

    ctl.slaveWrist(true);

    ip.filterImages(images, params);

    if (!params.gui.target_set) continue;

    if (params.activity == VS ||
	params.activity == IP) {
      //find ball
      
      circles = ip.findBall(images, params);
            
      mark_images(target_pos, circles, params, images);

    } //find ball

    if (params.activity != VS) {
      usleep(1000);
      continue; //to next mainloop iteration
    }

    //
    // Visual Servoing code
    //

    switch (params.new_mode) {
    case GRAB: params.new_mode = NONE; ctl.grabBall(); break;
    case RELEASE: params.new_mode = NONE; ctl.openGrip(); break;
    default: break;
    }
    
    printf("current_mode = %d\n", params.current_mode);
    switch (params.current_mode) {
    case READY:
      printf("old: READY\t");
      switch (params.new_mode) {
      case MOVE:
        printf("new: MOVE\n");
        params.new_mode = NONE;
        params.current_mode = MOVE;
        break;
      case PAUSE:
        printf("new: PAUSE\n");
        params.new_mode = NONE;
        params.current_mode = PAUSE;
        continue; //to next mainloop iteration
      default:
	break;
      }
      break;

    case PAUSE:
      printf("old: PAUSE\t");
      if (params.new_mode == MOVE) {
        printf("new: MOVE\n");
        params.new_mode = NONE;
        params.current_mode = MOVE;
        break;
      }
      break;

    case MOVE:
      printf("old: MOVE\t");
      if (params.new_mode == PAUSE) {
        printf("new: PAUSE\n");
        params.new_mode = NONE;
        //ctl.stop();
        params.current_mode = PAUSE;
        continue; //to next mainloop iteration
      }
      break;

    default:
      break;
    }
    
    if (circles != 0 && circles->total > 0 &&
        params.gui.target_set &&
        (params.current_mode == MOVE || params.current_mode == GRAB)) {
      ublas::vector<double> features(3);
      
      float* p = (float*) cvGetSeqElem(circles, 0);

      printf("first circle at (%d,%d) radius %d\n", 
             cvRound(p[0]), cvRound(p[1]), cvRound(p[2]));

      features(0) = p[0]; features(1) = p[1]; features(2) = p[2];

      if (features(2) >= MIN_TRACKING_RADIUS_PIXELS) {
        
        // rotate/translate to center origin, x left, y up
        features(0) = (images.hls->width / 2.0) - features(0);    // x
        if (images.hls->origin == 0) // top left origin
          features(1) = (images.hls->height / 2.0) - features(1); // y
        
        // normalize x & y
        features(0) /= FOCAL_LENGTH; features(1) /= FOCAL_LENGTH;
        
        // circular approximation of Z
        // Z = D*f / radius*2
        features(2) = DIAMETER * FOCAL_LENGTH / (features(2) * 2.0);
        
        printf("Norm features  x,y = (%3f, %3f),  Z = %3f\n",
               features(0), features(1), features(2));
        printf("Norm target    x,y = (%3f, %3f),  Z = %3f\n",
               target_pos_norm(0), target_pos_norm(1), target_pos_norm(2));
        
        std::cout << "current angles: " << ctl.getCurrentAngles() << std::endl;
        
        bool dls = ik.damped_least_squares(features, ctl.getCurrentAngles(),
                                           params, delta_angles);
        
        if (dls && params.current_mode != PARK) {
          std::cout << "commanded angle deltas: " << delta_angles << std::endl;
          ctl.moveDelta(delta_angles);
        }

      } else {
        std::cout <<
          "radius below tracking enable threshold " <<
          MIN_TRACKING_RADIUS_PIXELS;
      }
    } //tracking ball

  } //mainloop
  
#if RUN_THREADED
  switch (pthread_join(guiTID, 0)) {
  case 0: break; // all ok
  case EINVAL:
    printf("pthread_join: Invalid thread id %d\n", (int) guiTID); break;
  case ESRCH:
    printf("pthread_join: Thread ID %d not found\n", (int) guiTID); break;
  case EDEADLK:
    printf("pthread_join: Deadlock detected\n"); break;
  default:
    break;
  }
#endif

  if (images.set) {
    printf("releasing images\n");
    cvReleaseImage(&(images.bgr));
    cvReleaseImage(&(images.hls));
    cvReleaseImage(&(images.filtered_hls));
    cvReleaseImage(&(images.filtered_bgr));
#ifdef FLOAT_HLS
    cvReleaseImage(&(images.bgr32));
#endif
  }

  if (gui != 0) {
    printf("destroying gui\n");
    gui->destroy();
    gui = 0;
  }
 
  if (capture != 0) {
    printf("releasing capture device\n");
    cvReleaseCapture(&capture);
  }

} //main()


/*!
 * \brief Markup images with circles and lines.
 * Used for giving feedback to the user on the location of the  visual
 * servo target and where the ball is detected in the image.
 *
 * \param[in] target Cartesian coordinates of the target in [pixel,
 * pixel, meter] units.
 * \param[in] circles Sequence of detected circles (u,v,r) in pixels
 * \param[in] params  Params struct
 * \param[in,out] images  Images struct
 */
void mark_images(const ublas::vector<double>& target, const CvSeq* circles,
                 const Params& params, Images& images) {
  
  // draw target cross
  if (params.gui.target_set && params.activity == VS) {
    // fl * D / Z = apparent diameter, so div by 2 to get apparent radius
    double radius = (FOCAL_LENGTH * DIAMETER / target(2)) / 2.0;
    
    // rescale since target(x,y) was normalized using FOCAL_LENGTH
    double ih = images.bgr->height/2.0;
    double iw = images.bgr->width/2.0;
    CvPoint v1 = cvPoint(cvRound(target(0) + iw         ),
			 cvRound(target(1) + ih - radius)); // up
    CvPoint v2 = cvPoint(cvRound(target(0) + iw         ),
			 cvRound(target(1) + ih + radius)); // down
    CvPoint h1 = cvPoint(cvRound(target(0) + iw - radius),
			 cvRound(target(1) + ih         )); // left
    CvPoint h2 = cvPoint(cvRound(target(0) + iw + radius),
			 cvRound(target(1) + ih         )); // right

    // Draw target cross for sighting.
    cvLine(images.bgr, h1, h2, CV_RGB(0x00, 0x00, 0xff));
    cvLine(images.bgr, v1, v2, CV_RGB(0x00, 0x00, 0xff));
  }

  int num_circles = /*params.activity == VS ? 1 :*/ circles->total;

  // draw the ball
  for (int i = 0; i < num_circles; i++ ) {
    float* p = (float*) cvGetSeqElem(circles, i);
    CvPoint pt = cvPoint(cvRound(p[0]),cvRound(p[1]));
    cvCircle(images.bgr, pt, cvRound(p[2]), CV_RGB(0xff, 0x00, 0x00));
    cvCircle(images.filtered_hls, pt, cvRound(p[2]), cvScalar(192)); //greyscale
    //TBD mark filtered_bgr if using that to find the ball
  }
}

/*!
 * \brief Uses calibration file to set offsets.
 * Reads servo numbers and calibration positions from the provided
 * file.  Offsets are calculated from calibration position differences
 * to ideal positions.
 */
void
calibrate_offsets(std::string& file, ublas::vector<double>& offsets){
  if (file.empty()) {
    offsets(Arm::ELBOW) = 400;
  } else {
    std::fstream input(file.c_str());
    int servo, val;
    ublas::vector<double> calibration_position(NUM_JOINTS);
    calibration_position(Arm::GRIP)     = 1350;
    calibration_position(Arm::WRIST)    = 1500;
    calibration_position(Arm::ELBOW)    = 1500;
    calibration_position(Arm::SHOULDER) = 1500;
    calibration_position(Arm::BASE)     = 1500;
    std::cout << "cal: " << calibration_position << std::endl;
    std::cout << "grip: " << Arm::GRIP << std::endl;
    while (!input.eof()) {
      input >> std::skipws >> servo >> val;
      printf("servo: %d, val: %d, cal: %g\t",
             servo, val, calibration_position(servo));
      offsets[servo] = val - calibration_position(servo);
      printf("offset: %g\n", offsets(servo));
    }
    std::cout << "off: " << offsets << std::endl;
  }
}

/*!
 * \brief Update params with current angles.
 * Sets current angles of ctl in struct params to be picked up by GUI.
 */
void
update_gui_position (ArmControl& ctl, Params& params) {
  ublas::vector<double> current_theta(ctl.getCurrentAngles());
  ArmControl::radiansToDegrees(current_theta);
  for (int i = 0; i < NUM_JOINTS; i++) {
    params.ctl.current_theta[i] = current_theta(i);
  }
  params.position_changed = true;
}

/*! 
 * \brief Signal handler function for stopping threads.
 *
 * \param[in] sig The received signal.
 */
void handler(int sig) {
  if (sig_ctl == 0) {
    printf("No control object for emergency shutdown!\n");
  } else {
    sig_ctl->stop();
  }
  exit(sig);
}

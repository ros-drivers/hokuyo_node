/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2008-2010  Willow Garage
 *                      
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <termios.h>
#include <math.h>
#include <poll.h>

#include "hokuyo.h"

#include <time.h>

#include <fcntl.h>

#include "ros/console.h"

#if POSIX_TIMERS <= 0
#include <sys/time.h>
#endif


//! Macro for throwing an exception with a message
#define HOKUYO_EXCEPT(except, msg) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "hokuyo::Laser::%s: " msg, __FUNCTION__); \
    throw except(buf); \
  }

//! Macro for throwing an exception with a message, passing args
#define HOKUYO_EXCEPT_ARGS(except, msg, ...) \
  { \
    char buf[100]; \
    snprintf(buf, 100, "hokuyo::laser::%s: " msg, __FUNCTION__, __VA_ARGS__); \
    throw except(buf); \
  }


//! Helper function for querying the system time
static uint64_t timeHelper()
{
#if POSIX_TIMERS > 0
  struct timespec curtime;
  clock_gettime(CLOCK_REALTIME, &curtime);
  return (uint64_t)(curtime.tv_sec) * 1000000000 + (uint64_t)(curtime.tv_nsec);  
#else
  struct timeval timeofday;
  gettimeofday(&timeofday,NULL);
  return (uint64_t)(timeofday.tv_sec) * 1000000000 + (uint64_t)(timeofday.tv_usec) * 1000;  
#endif
}


///////////////////////////////////////////////////////////////////////////////
hokuyo::Laser::Laser() :
                      dmin_(0), dmax_(0), ares_(0), amin_(0), amax_(0), afrt_(0), rate_(0),
                      wrapped_(0), last_time_(0), time_repeat_count_(0), offset_(0),
                      laser_port_(NULL), laser_fd_(-1)
{ }


///////////////////////////////////////////////////////////////////////////////
hokuyo::Laser::~Laser ()
{
  if (portOpen())
    close();
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::open(const char * port_name)
{
  if (portOpen())
    close();
  
  laser_port_ = fopen(port_name, "r+");
  if (laser_port_ == NULL)
    HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "Failed to open port: %s -- error = %d: %s", port_name, errno, strerror(errno));

  try
  {
    laser_fd_ = fileno (laser_port_);
    if (laser_fd_ == -1)
      HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "Failed to get file descriptor --  error = %d: %s", errno, strerror(errno));

    struct flock fl;
    fl.l_type   = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start = 0;
    fl.l_len   = 0;
    fl.l_pid   = getpid();

    if (fcntl(laser_fd_, F_SETLK, &fl) != 0)
      HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "Device %s is already locked.", port_name);

    // Settings for USB?
    struct termios newtio;
    memset (&newtio, 0, sizeof (newtio));
    newtio.c_cflag = CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = ICANON;
    
    // activate new settings
    tcflush (laser_fd_, TCIFLUSH);
    tcsetattr (laser_fd_, TCSANOW, &newtio);
    usleep (200000);

    // Some models (04LX) need to be told to go into SCIP2 mode...
    laserFlush();
    setToSCIP2();

    // Just in case a previous failure mode has left our Hokuyo
    // spewing data, we send the TM2 and QT commands to be safe.
    laserFlush();
    sendCmd("TM2", 1000);
    sendCmd("QT", 1000);
    laserFlush();

    querySensorConfig();

    getVersionInformation(); // In preparation for calls to get various parts of the version info.
  }
  catch (hokuyo::Exception& e)
  {
    // These exceptions mean something failed on open and we should close
    if (laser_port_ != NULL)
      fclose(laser_port_);
    laser_port_ = NULL;
    laser_fd_ = -1;
    throw e;
  }
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::close ()
{
  int retval = 0;

  if (portOpen()) {
    //Try to be a good citizen and turn off the laser before we shutdown communication
    try
    {
      sendCmd("QT",1000);
      laserFlush();
    }
    catch (hokuyo::Exception& e) {
      //Exceptions here can be safely ignored since we are closing the port anyways
    }

    retval = fclose(laser_port_); // Automatically releases the lock.
  }

  laser_port_ = NULL;
  laser_fd_ = -1;

  if (retval != 0)
    HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}

///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::setToSCIP2()
{
	if (!portOpen())
	    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");
	const char * cmd = "SCIP2.0";
	char buf[100];
	laserWrite(cmd);
  	laserWrite("\n");

	laserReadline(buf, 100, 1000);
	ROS_DEBUG("Laser comm protocol changed to %s \n", buf);
	//printf ("Laser comm protocol changed to %s \n", buf);
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo::Laser::sendCmd(const char* cmd, int timeout)
{
if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  char buf[100]; 

  laserWrite(cmd);
  laserWrite("\n");

  laserReadlineAfter(buf, 100, cmd, timeout);
  laserReadline(buf,100,timeout);

  if (!checkSum(buf,4))
    HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on status code.");

  buf[2] = 0;
  
  if (buf[0] - '0' >= 0 && buf[0] - '0' <= 9 && buf[1] - '0' >= 0 && buf[1] - '0' <= 9)
    return (buf[0] - '0')*10 + (buf[1] - '0');
  else
    HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "Hokuyo error code returned. Cmd: %s --  Error: %s", cmd, buf);
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::getConfig(LaserConfig& config)
{
  config.min_angle  =  (amin_ - afrt_) * (2.0*M_PI)/(ares_);
  config.max_angle  =  (amax_ - afrt_) * (2.0*M_PI)/(ares_);
  config.ang_increment =  (2.0*M_PI)/(ares_);
  config.time_increment = (60.0)/(double)(rate_ * ares_);
  config.scan_time = 60.0/((double)(rate_));
  config.min_range  =  dmin_ / 1000.0;
  config.max_range  =  dmax_ / 1000.0;
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo::Laser::laserWrite(const char* msg)
{
  int retval = fputs(msg, laser_port_);
  if (retval != EOF)
    return retval;
  else
    HOKUYO_EXCEPT(hokuyo::Exception, "fputs failed");
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo::Laser::laserFlush()
{
  int retval = tcflush(laser_fd_, TCIOFLUSH);
  if (retval != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "tcflush failed");
  
  return retval;
} 


///////////////////////////////////////////////////////////////////////////////
int 
hokuyo::Laser::laserReadline(char *buf, int len, int timeout)
{
  char* ret;
  int current=0;

  struct pollfd ufd[1];
  int retval;
  ufd[0].fd = laser_fd_;
  ufd[0].events = POLLIN;

  while (current < len - 1)
  {
    if (current > 0)
      if (buf[current-1] == '\n')
	return current;

    if(timeout >= 0)
    {
      if ((retval = poll(ufd, 1, timeout)) < 0)
        HOKUYO_EXCEPT_ARGS(hokuyo::Exception, "poll failed   --  error = %d: %s", errno, strerror(errno));

      if (retval == 0)
        HOKUYO_EXCEPT(hokuyo::TimeoutException, "timeout reached");
    }

    ret = fgets(&buf[current], len-current, laser_port_);

    if (ret != &buf[current])
      HOKUYO_EXCEPT(hokuyo::Exception, "fgets failed");

    current += strlen(&buf[current]);
  }
  HOKUYO_EXCEPT(hokuyo::Exception, "buffer filled without end of line being found");
}


char*
hokuyo::Laser::laserReadlineAfter(char* buf, int len, const char* str, int timeout)
{
  buf[0] = 0;
  char* ind = &buf[0];

  int bytes_read = 0;
  int skipped = 0;

  while ((strncmp(buf, str, strlen(str))) != 0) {
    bytes_read = laserReadline(buf,len,timeout);

    if ((skipped += bytes_read) > MAX_SKIPPED)
      HOKUYO_EXCEPT(hokuyo::Exception, "too many bytes skipped while searching for match");
  }

  return ind += strlen(str);
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::querySensorConfig()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (sendCmd("PP",1000) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting configuration information");

  char buf[100];
  char* ind;
    
  ind = laserReadlineAfter(buf,100,"DMIN:",-1);
  sscanf(ind, "%d", &dmin_);
    
  ind = laserReadlineAfter(buf,100,"DMAX:",-1);
  sscanf(ind, "%d", &dmax_);
    
  ind = laserReadlineAfter(buf,100,"ARES:",-1);
  sscanf(ind, "%d", &ares_);
    
  ind = laserReadlineAfter(buf,100,"AMIN:",-1);
  sscanf(ind, "%d", &amin_);
    
  ind = laserReadlineAfter(buf,100,"AMAX:",-1);
  sscanf(ind, "%d", &amax_);
    
  ind = laserReadlineAfter(buf,100,"AFRT:",-1);
  sscanf(ind, "%d", &afrt_);
    
  ind = laserReadlineAfter(buf,100,"SCAN:",-1);
  sscanf(ind, "%d", &rate_);
    
  return;
}


///////////////////////////////////////////////////////////////////////////////
bool
hokuyo::Laser::checkSum(const char* buf, int buf_len)
{
  char sum = 0;
  for (int i = 0; i < buf_len - 2; i++)
    sum += (unsigned char)(buf[i]);

  if ((sum & 63) + 0x30 == buf[buf_len - 2])
    return true;
  else
    return false;
}


///////////////////////////////////////////////////////////////////////////////
uint64_t
hokuyo::Laser::readTime(int timeout)
{
  char buf[100];

  laserReadline(buf, 100, timeout);
  if (!checkSum(buf, 6))
    HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on time stamp.");

  unsigned int laser_time = ((buf[0]-0x30) << 18) | ((buf[1]-0x30) << 12) | ((buf[2]-0x30) << 6) | (buf[3] - 0x30);

  if (laser_time == last_time_)
  {
    if (++time_repeat_count_ > 2)
    {
      HOKUYO_EXCEPT_ARGS(hokuyo::RepeatedTimeException, "The timestamp has not changed for %d reads", time_repeat_count_);
    }
    else if (time_repeat_count_ > 0)
      ROS_DEBUG("The timestamp has not changed for %d reads. Ignoring for now.", time_repeat_count_);
  }
  else
  {
    time_repeat_count_ = 0;
  }

  if (laser_time < last_time_)
    wrapped_++;
  
  last_time_ = laser_time;
  
  return (uint64_t)((wrapped_ << 24) | laser_time)*(uint64_t)(1000000);
}


///////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::readData(hokuyo::LaserScan& scan, bool has_intensity, int timeout)
{
  scan.ranges.clear();
  scan.intensities.clear();

  int data_size = 3;
  if (has_intensity)
    data_size = 6;

  char buf[100];

  int ind = 0;

  scan.self_time_stamp = readTime(timeout);

  int bytes;

  float range;
  float intensity;

  for (;;)
  {
    bytes = laserReadline(&buf[ind], 100 - ind, timeout);
    
    if (bytes == 1)          // This is \n\n so we should be done
      return;
    
    if (!checkSum(&buf[ind], bytes))
      HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Checksum failed on data read.");
    
    bytes += ind - 2;
    
    // Read as many ranges as we can get
    for (int j = 0; j < bytes - (bytes % data_size); j+=data_size)
    {
      if (scan.ranges.size() < MAX_READINGS)
      {
        range = (((buf[j]-0x30) << 12) | ((buf[j+1]-0x30) << 6) | (buf[j+2]-0x30)) / 1000.0;
	scan.ranges.push_back(range);

        if (has_intensity)
        {
	  intensity = (((buf[j+3]-0x30) << 12) | ((buf[j+4]-0x30) << 6) | (buf[j+5]-0x30));
          scan.intensities.push_back(intensity);
        }
      }
      else
      {
        HOKUYO_EXCEPT(hokuyo::CorruptedDataException, "Got more readings than expected");
      }
    }
    // Shuffle remaining bytes to front of buffer to get them on the next loop
    ind = 0;
    for (int j = bytes - (bytes % data_size); j < bytes ; j++)
      buf[ind++] = buf[j];
  }
}


///////////////////////////////////////////////////////////////////////////////
int
hokuyo::Laser::pollScan(hokuyo::LaserScan& scan, double min_ang, double max_ang, int cluster, int timeout)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  int status;

  // Always clear ranges/intensities so we can return easily in case of erro
  scan.ranges.clear();
  scan.intensities.clear();

  // clustering of 0 and 1 are actually the same
  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  sprintf(cmdbuf,"GD%.4d%.4d%.2d", min_i, max_i, cluster);
  
  status = sendCmd(cmdbuf, timeout);
  
  scan.system_time_stamp = timeHelper() + offset_;
  
  if (status != 0)
    return status;
  
  // Populate configuration
  scan.config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan.config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan.config.scan_time = 0.0;
  scan.config.min_range  =  dmin_ / 1000.0;
  scan.config.max_range  =  dmax_ / 1000.0;
  
  readData(scan, false, timeout);
  
  long long inc = (long long)(min_i * scan.config.time_increment * 1000000000);
  
  scan.system_time_stamp += inc;
  scan.self_time_stamp += inc;
  
  return 0;
}

int
hokuyo::Laser::laserOn() {
  int res = sendCmd("BM",1000);
  if (res == 1)
    HOKUYO_EXCEPT(hokuyo::Exception, "Unable to control laser due to malfunction");
  return res;
}

int
hokuyo::Laser::laserOff() {
  return sendCmd("QT",1000);
}

int
hokuyo::Laser::stopScanning() {
  return laserOff();
}

///////////////////////////////////////////////////////////////////////////////
int
hokuyo::Laser::requestScans(bool intensity, double min_ang, double max_ang, int cluster, int skip, int count, int timeout)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  int status;

  if (cluster == 0)
    cluster = 1;
  
  int min_i = (int)(afrt_ + min_ang*ares_/(2.0*M_PI));
  int max_i = (int)(afrt_ + max_ang*ares_/(2.0*M_PI));
  
  char cmdbuf[MAX_CMD_LEN];
  
  char intensity_char = 'D';
  if (intensity)
    intensity_char = 'E';
  
  sprintf(cmdbuf,"M%c%.4d%.4d%.2d%.1d%.2d", intensity_char, min_i, max_i, cluster, skip, count);
  
  status = sendCmd(cmdbuf, timeout);
  
  return status;
}


int
hokuyo::Laser::serviceScan(hokuyo::LaserScan& scan, int timeout)
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  // Always clear ranges/intensities so we can return easily in case of erro
  scan.ranges.clear();
  scan.intensities.clear();

  char buf[100];

  bool intensity = false;
  int min_i;
  int max_i;
  int cluster;
  int skip;
  int left;

  char* ind;

  int status = -1;

  do {
    ind = laserReadlineAfter(buf, 100, "M",timeout);
    scan.system_time_stamp = timeHelper() + offset_;

    if (ind[0] == 'D')
      intensity = false;
    else if (ind[0] == 'E')
      intensity = true;
    else
      continue;

    ind++;

    sscanf(ind, "%4d%4d%2d%1d%2d", &min_i, &max_i, &cluster, &skip, &left);  
    laserReadline(buf,100,timeout);

    buf[4] = 0;

    if (!checkSum(buf, 4))
      HOKUYO_EXCEPT_ARGS(hokuyo::CorruptedDataException, "Checksum failed on status code: %s", buf);

    sscanf(buf, "%2d", &status);

    if (status != 99)
      return status;
    
  } while(status != 99);

  scan.config.min_angle  =  (min_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.max_angle  =  (max_i - afrt_) * (2.0*M_PI)/(ares_);
  scan.config.ang_increment =  cluster*(2.0*M_PI)/(ares_);
  scan.config.time_increment = (60.0)/(double)(rate_ * ares_);
  scan.config.scan_time = (60.0 * (skip + 1))/((double)(rate_));
  scan.config.min_range  =  dmin_ / 1000.0;
  scan.config.max_range  =  dmax_ / 1000.0;

  readData(scan, intensity, timeout);

  long long inc = (long long)(min_i * scan.config.time_increment * 1000000000);

  scan.system_time_stamp += inc;
  scan.self_time_stamp += inc;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
void
hokuyo::Laser::getVersionInformation()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (sendCmd("VV",1000) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting version information");
  
  char buf[100];
  vendor_name_ = laserReadlineAfter(buf, 100, "VEND:");
  vendor_name_ = vendor_name_.substr(0,vendor_name_.length() - 3);
  
  product_name_ = laserReadlineAfter(buf, 100, "PROD:");
  product_name_ = product_name_.substr(0,product_name_.length() - 3);
  
  firmware_version_ = laserReadlineAfter(buf, 100, "FIRM:");
  firmware_version_ = firmware_version_.substr(0,firmware_version_.length() - 3);

  protocol_version_ = laserReadlineAfter(buf, 100, "PROT:");
  protocol_version_ = protocol_version_.substr(0,protocol_version_.length() - 3);
  
  serial_number_ = laserReadlineAfter(buf, 100, "SERI:");
  serial_number_ = serial_number_.substr(0,serial_number_.length() - 3);
}


//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo::Laser::getID()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return serial_number_;
}


//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo::Laser::getFirmwareVersion()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return firmware_version_;
}


//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo::Laser::getProtocolVersion()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return protocol_version_;
}


//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo::Laser::getVendorName()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return vendor_name_;
}


//////////////////////////////////////////////////////////////////////////////
std::string 
hokuyo::Laser::getProductName()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  return product_name_;
}


//////////////////////////////////////////////////////////////////////////////
std::string
hokuyo::Laser::getStatus()
{
  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  if (sendCmd("II",1000) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting device information information");
  
  char buf[100];
  char* stat = laserReadlineAfter(buf, 100, "STAT:");

  std::string statstr(stat);
  statstr = statstr.substr(0,statstr.length() - 3);

  return statstr;
}


//////////////////////////////////////////////////////////////////////////////
long long
hokuyo::Laser::calcLatency(bool intensity, double min_ang, double max_ang, int clustering, int skip, int num, int timeout)
{
  ROS_DEBUG("Entering calcLatency.");

  if (!portOpen())
    HOKUYO_EXCEPT(hokuyo::Exception, "Port not open.");

  offset_ = 0;

  uint64_t comp_time = 0;
  uint64_t laser_time = 0;
  long long diff_time = 0;
  long long drift_time = 0;
  long long tmp_offset1 = 0;
  long long tmp_offset2 = 0;

  int count = 0;
 
  sendCmd("TM0",timeout);
  count = 100;

  for (int i = 0; i < count;i++)
  {
    usleep(1000);
    sendCmd("TM1",timeout);
    comp_time = timeHelper();
    try 
    {
      laser_time = readTime();

      diff_time = comp_time - laser_time;

      tmp_offset1 += diff_time / count;
    } catch (hokuyo::RepeatedTimeException &e)
    {
      // We expect to get Repeated Time's when hammering on the time server
      continue;
    }
  }

  uint64_t start_time = timeHelper();
  usleep(5000000);
  sendCmd("TM1;a",timeout);
  sendCmd("TM1;b",timeout);
  comp_time = timeHelper();
  drift_time = comp_time - start_time;
  laser_time = readTime() + tmp_offset1;
  diff_time = comp_time - laser_time;
  double drift_rate = double(diff_time) / double(drift_time);

  sendCmd("TM2",timeout);
  
  if (requestScans(intensity, min_ang, max_ang, clustering, skip, num, timeout) != 0)
    HOKUYO_EXCEPT(hokuyo::Exception, "Error requesting scans during latency calculation");

  hokuyo::LaserScan scan;

  count = 200;
  for (int i = 0; i < count;i++)
  {
    try
    {
      serviceScan(scan, 1000);
    } catch (hokuyo::CorruptedDataException &e) {
      continue;
    }

    comp_time = scan.system_time_stamp;
    drift_time = comp_time - start_time;
    laser_time = scan.self_time_stamp + tmp_offset1 + (long long)(drift_time*drift_rate);
    diff_time = laser_time - comp_time;

    tmp_offset2 += diff_time / count;
  }

  offset_ = tmp_offset2;

  stopScanning();
  
  ROS_DEBUG("Leaving calcLatency.");

  return offset_;
}

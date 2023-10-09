#ifndef _REGISTER_H_
#define _REGISTER_H_

enum I2Creg
{
  drive_version = 0,
  drive_mode = 1,

  cmd_speed = 10,
  cmd_pos = 11,
  

  status_cpu = 20,
  status_drive = 21

};

#endif

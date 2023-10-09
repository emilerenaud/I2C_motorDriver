#ifndef _REGISTER_H_
#define _REGISTER_H_

enum I2Creg
{
  drive_version = 0x000,
  drive_mode = 0x001,

  cmd_speed = 0x100,
  cmd_pos = 0x101,

  status_cpu = 0x200,
  status_drive = 0x201

};

#endif

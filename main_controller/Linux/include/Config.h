/*
 * Config.h
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#if 1
// Baby Overlord
#define PADSP

#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin"

#define INI_FILE_PATH       "../../../Data/config.ini"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// DARwIn
#define WEBOTS

#define FLIP_FRAME

#define FSR_FEET

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin.darwin"
#else
#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin.darwin"
#endif

#define INI_FILE_PATH       "../../../Data/config.ini.darwin"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// HROS1
#define MX28_1024

#define MPLAYER

#define MOTION_FILE_PATH    "../../../Data/motion_1024.bin.hros1"

#define INI_FILE_PATH       "../../../Data/config.ini.hros1"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// HROS5
#define MPLAYER

#define MOTION_FILE_PATH    "../../../Data/motion_4096.bin.hros5"

#define INI_FILE_PATH       "../../../Data/config.ini.hros5"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#endif /* CONFIG_H_ */

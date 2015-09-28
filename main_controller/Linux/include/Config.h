/*
 * Config.h
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#if 1
// Baby Overlord
#define MPG123

#define MOTION_FILE_PATH    "../../../Data/baby_overlord/motion_4096.bin"

#define INI_FILE_PATH       "../../../Data/baby_overlord/config.ini"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// DARwIn
#define WEBOTS

#define FLIP_FRAME

#define FSR_FEET

#ifdef MX28_1024
#define MOTION_FILE_PATH    "../../../Data/darwin/motion_1024.bin"
#else
#define MOTION_FILE_PATH    "../../../Data/darwin/motion_4096.bin"
#endif

#define INI_FILE_PATH       "../../../Data/darwin/config.ini"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// HROS1
#define MX28_1024

#define MPLAYER

#define MOTION_FILE_PATH    "../../../Data/hros1/motion_4096.bin"

#define INI_FILE_PATH       "../../../Data/hros1/config.ini"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#if 0
// HROS5
#define MPLAYER

#define MOTION_FILE_PATH    "../../../Data/hros5/motion_4096.bin"

#define INI_FILE_PATH       "../../../Data/hros5/config.ini"

#define CM730_DEV_NAME      "/dev/ttyUSB0"
#endif

#endif /* CONFIG_H_ */

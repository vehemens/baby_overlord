#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "Config.h"

#include "LinuxDARwIn.h"
#include "cmd_process.h"


#define PROGRAM_VERSION		"v1.00"
#define MAXNUM_INPUTCHAR	(128)

using namespace Robot;

LinuxCM730 linux_cm730(CM730_DEV_NAME);
CM730 cm730(&linux_cm730);

int gID = CM730::ID_CM;

void sighandler(int sig)
{
	exit(0);
}

int main()
{
	signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
	signal(SIGQUIT, &sighandler);
	signal(SIGINT, &sighandler);

	char input[MAXNUM_INPUTCHAR];
	char *token;
	int input_len;
	char cmd[80];
	char param[20][30];
	int num_param;
	int iparam[20];	

	printf( "\n[Dynamixel Monitor for DARwIn %s]\n", PROGRAM_VERSION);

	if(cm730.Connect() == true)
	{
		Scan(&cm730, CM730::ID_CM);

		for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
			Scan(&cm730, i);

		while(1)
		{
			Prompt(gID);
			//gets(input);
			fgets(input, MAXNUM_INPUTCHAR, stdin);
			fflush(stdin);
			input_len = strlen(input);
			if(input_len == 0)
				continue;

			token = strtok( input, " \n" );
			if(token == 0)
				continue;

			strcpy( cmd, token );
			token = strtok( 0, " \n" );
			num_param = 0;
			while(token != 0)
			{
				strcpy( param[num_param++], token );
				token = strtok( 0, " \n" );
			}

			if(strcmp(cmd, "exit") == 0)
				break;
			else if(strcmp(cmd, "scan") == 0)
			{
				Scan(&cm730, CM730::ID_CM);

				for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
					Scan(&cm730, i);
			}
			else if((strcmp(cmd, "h") == 0) || (strcmp(cmd, "help") == 0))
				Help();
			else if(strcmp(cmd, "id") == 0)
			{
				if(num_param != 1)
				{
					printf(" Invalid parameter!\n");
					continue;
				}
				
				iparam[0] = atoi(param[0]);
	            		if(cm730.Ping(iparam[0], 0) == CM730::SUCCESS)
	            		{
	                		gID = iparam[0];
	            		}
	            		else
	            		{
                    		printf(" Invalid ID(%d)!\n", iparam[0]);
                    		continue;
	            		}
			}
			else if(strcmp(cmd, "on") == 0)
			{
				if(num_param == 0)
				{
					cm730.WriteByte(gID, MX28::P_TORQUE_ENABLE, 1, 0);
					if(gID == CM730::ID_CM)
						printf(" Dynamixel power on\n");
				}
				else if(num_param == 1)
				{
					if(strcmp(param[0], "all") == 0)
					{
						for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
							cm730.WriteByte(i, MX28::P_TORQUE_ENABLE, 1, 0);
					}
					else
					{
						printf(" Invalid parameter!\n");
						continue;
					}
				}
				else
				{
					printf(" Invalid parameter!\n");
					continue;
				}
			}
			else if(strcmp(cmd, "off") == 0)
			{
				if(num_param == 0)
				{
					cm730.WriteByte(gID, MX28::P_TORQUE_ENABLE, 0, 0);
					if(gID == CM730::ID_CM)
						printf(" Dynamixel power off\n");
				}
				else if(num_param == 1)
				{
					if(strcmp(param[0], "all") == 0)
					{
						for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
							cm730.WriteByte(i, MX28::P_TORQUE_ENABLE, 0, 0);
					}
					else
					{
						printf(" Invalid parameter!\n");
						continue;
					}
				}
				else
				{
					printf(" Invalid parameter!\n");
					continue;
				}
			}
			else if((strcmp(cmd, "d") == 0) || (strcmp(cmd, "dump") == 0))
			{
				if(num_param == 0)
				{
					Dump(&cm730, gID);
				}
				else if(num_param == 1)
				{
					if(strcmp(param[0], "all") == 0)
					{
						for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
							Dump(&cm730, i);
					}
					else
					{
						printf(" Invalid parameter!\n");
						continue;
					}
				}
				else
				{
					printf(" Invalid parameter!\n");
					continue;
				}
			}
			else if(strcmp(cmd, "reset") == 0)
			{
			    int firm_ver = 0;
			    if(cm730.ReadByte(JointData::ID_HEAD_PAN, MX28::P_VERSION, &firm_ver, 0)  != CM730::SUCCESS)
			    {
			        fprintf(stderr, "Can't read firmware version from Dynamixel ID %d!! \n\n", JointData::ID_HEAD_PAN);
			        exit(0);
			    }

			    if(0 < firm_ver && firm_ver < 27)
			    {
			        fprintf(stderr, "\n MX-28's firmware is not support 4096 resolution!! \n");
			        fprintf(stderr, " Upgrade MX-28's firmware to version 27(0x1B) or higher.\n\n");
			        continue;
			    }

				if(num_param == 0)
				{
					Reset(&cm730, gID);
				}
				else if(num_param == 1)
				{
					if(strcmp(param[0], "all") == 0)
					{
						for(int i=JointData::ID_R_SHOULDER_PITCH; i<JointData::NUMBER_OF_JOINTS; i++)
							Reset(&cm730, i);
					}
					else
					{
						printf(" Invalid parameter!\n");
						continue;
					}
				}
				else
				{
					printf(" Invalid parameter!\n");
					continue;
				}
			}
			else if(strcmp(cmd, "wr") == 0)
			{
				if(num_param == 2)
				{
					Write(&cm730, gID, atoi(param[0]), atoi(param[1]));
				}
				else
				{
					printf(" Invalid parameter!\n");
					continue;
				}
			}
			else
				printf(" Bad command! please input 'help'.\n");
		}
	}
	else
		printf("Failed to connect CM-730!");

	printf("\nTerminated DXL Manager.\n");
	return 0;
}

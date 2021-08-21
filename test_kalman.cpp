#include "fast_kalman_filter.h"
#include <arpa/inet.h>
#include <assert.h>
#include <cassert>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <semaphore.h>
#include <queue>
#include <sched.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/sem.h>
#include <unistd.h>
#include <wiringPi.h>
#include <typeinfo>

extern "C" {
	#include "spi_if.h"
	#include "icm20948.h"
}

typedef struct icm_data{
	int sensor_id;
	struct timeval tp;
	float ax,ay,az;
	float wx,wy,wz;
	float mx,my,mz;
	float temp;
	float qw,qx,qy,qz;
	float roll,pitch,yaw;
}icm_data_t;

#define SERV_PORT 11206
// #define SERV_IP "192.168.9.107"
#define BUFF_SIZE 64
#define ENABLE_INTERRUPUT	0
#define ICM_INT_EVENT     0x01
#define NEW_FILE_EVENT    0x03
#define FILE_CLOSE_EVENT  0x04
#define LOG_START_EVENT   0x05
#define LOG_STOP_EVENT    0x06
#define APP_EXIT_EVENT    0x07
#define SENORS_NUMBER     12

#define ICM_DATA_LEN	  sizeof(icm_data_t)
#define ICM_DATA_SAMPLES  10000

using namespace std;

pthread_mutex_t lock; 
std::queue<char> icm_sensor_queue,icm_record_queue;
ofstream icm_logfile;
string icm_filename;
int cs[SENORS_NUMBER] = {
	SENSOR_ID_1,
	SENSOR_ID_2,
	SENSOR_ID_9,
	SENSOR_ID_10,
	SENSOR_ID_11,
	SENSOR_ID_6,
	SENSOR_ID_7,
	SENSOR_ID_8,
	SENSOR_ID_3,
	SENSOR_ID_4,
	SENSOR_ID_5,
	1
};

unsigned char *icm_data_buffer;
uint64_t  icm_data_buffer_wpos;
uint64_t  icm_data_buffer_rpos;
sem_t     free_mutex;
sem_t     used_mutex;

#if ENABLE_INTERRUPUT
void icm_handler(void)
{
	icm_queue.push(ICM_INT_EVENT);
}
#endif
int sock_client_fd;
void *icm_sensor_thread(void *threadid)
{
	pthread_mutex_lock(&lock); 
	size_t len[SENORS_NUMBER];
	int imu_exist[SENORS_NUMBER];
	int mag_exist[SENORS_NUMBER];
	for(int i=0; i<SENORS_NUMBER;i++)
	{
		len[i] = 0;
	}
	fast_kalman_filter fkf[SENORS_NUMBER];
	sensor_data data[SENORS_NUMBER];
	icm_data_t icm_sensor_data;
	float ax,ay,az;
	float wx,wy,wz;
	float mx,my,mz;
	float temp;
	for(int i=0; i<SENORS_NUMBER;i++)
	{
		sensor_object_init(&data[i]);
		fast_kalman_filter_init(&fkf[i]);
	}
	long tid;
	float ax_sensor,ay_sensor,az_sensor;
	float gx_sensor,gy_sensor,gz_sensor;
	float mx_sensor,my_sensor,mz_sensor;
	float temp_sensor;
	struct timeval tp;
	tid = (long)threadid;
	
	printf("\r\nInitialize sensor:\r\n");
	printf("Sensor ID: ");
	for(int i = 0; i < 11; i++)
	{
		sensor_select(cs[i]);
		if(configMPU())
		{
			imu_exist[i] = 1;
		}
		else
		{
			imu_exist[i] = 0;
		}
		if(configAK09916())
		{
			mag_exist[i] = 1;
		}
		else
		{
			mag_exist[i] = 0;
		}
		printf("%d ", i);
	}
	printf("\r\n");
	printf("IMU      : ");
	for(int i = 0; i < 11; i++)
	{
		if(imu_exist[i])
		{
			printf("+ ");
		}
		else
		{
			printf("- ");
	    }
	}
	printf("\r\n");
	printf("MAG      : ");
	for(int i = 0; i < 11; i++)
	{
		if(mag_exist[i])
		{
			printf("+ ");
		}
		else
		{
			printf("- ");
	    }
	}
	pthread_mutex_unlock(&lock); 
	while(1)
	{
#if !ENABLE_INTERRUPUT
		sensor_select(cs[0]);
		if(checkDataReady())
		{
			icm_sensor_queue.push(ICM_INT_EVENT);
		}
#endif
		if(!icm_sensor_queue.empty())
		{
			switch(icm_sensor_queue.front())
			{
				case ICM_INT_EVENT:
				gettimeofday(&(icm_sensor_data.tp),NULL);

				for(int i=0; i<SENORS_NUMBER; i++)
				{
					sensor_select(cs[i]);
					if(checkExisting() != imu_exist[i] && imu_exist[i])
					{
						printf("\r\n[ERROR] sensor %d lost\r\n");
						imu_exist[i] = 0;
					}
					else if(imu_exist[i])
					{	
						readSensor(&ax_sensor,&ay_sensor,&az_sensor,&gx_sensor,&gy_sensor,&gz_sensor,&mx_sensor,&my_sensor,&mz_sensor,&temp_sensor);
		
						icm_sensor_data.sensor_id = i;
						icm_sensor_data.ax = ax_sensor;
						icm_sensor_data.ay = ay_sensor;
						icm_sensor_data.az = az_sensor;
						icm_sensor_data.wx = gx_sensor*M_PI/180.0;
						icm_sensor_data.wy = gy_sensor*M_PI/180.0;
						icm_sensor_data.wz = gz_sensor*M_PI/180.0;
						icm_sensor_data.mx = mx_sensor;
						icm_sensor_data.my = my_sensor;
						icm_sensor_data.mz = mz_sensor;
						icm_sensor_data.temp = temp_sensor;

						len[i]++;
						data[i].fAccel[X] = -ax;
						data[i].fAccel[Y] = ay;
						data[i].fAccel[Z] = -az;

						data[i].fGyro[X] = -wx;
						data[i].fGyro[Y] = wy;
						data[i].fGyro[Z] = -wz;

						data[i].fMag[X] = -mx;
						data[i].fMag[Y] = my;
						data[i].fMag[Z] = -mz;
						
						//fast_kalman_filter_update(&fkf[i],&data[i]);
						//quat2euler_f64(&(fkf[i].q_k), &(fkf[i].euler));
						
						if(sem_trywait(&free_mutex) == 0)
						{
							memcpy(icm_data_buffer+icm_data_buffer_wpos,(unsigned char*)&icm_sensor_data,ICM_DATA_LEN);
							icm_data_buffer_wpos += ICM_DATA_LEN;
							icm_data_buffer_wpos %= (ICM_DATA_LEN*ICM_DATA_SAMPLES);	
							sem_post(&used_mutex);
						}
						else
						{
							cout<<"buffer is full";
						}
					}
				}
				break;
				case NEW_FILE_EVENT:
				case FILE_CLOSE_EVENT:
				case LOG_START_EVENT:
				case LOG_STOP_EVENT:
				break;
				case APP_EXIT_EVENT:
				printf("\r\n[INFO] exit ICM sensor thread.\r\n");
				pthread_exit(NULL);
				break;
				default:

				break;
			}
			icm_sensor_queue.pop();
		}
		usleep(10);
	}
}

void *icm_record_thread(void *threadid)
{
	icm_data_t record_data;
	while(1)
	{
		while(sem_trywait(&used_mutex) == 0)
		{
			memcpy((unsigned char*)&record_data,icm_data_buffer+icm_data_buffer_rpos,ICM_DATA_LEN);
			icm_data_buffer_rpos += ICM_DATA_LEN;
			icm_data_buffer_rpos %= (ICM_DATA_LEN*ICM_DATA_SAMPLES);	
			sem_post(&free_mutex);
			
			if(icm_logfile.is_open())
			{
				icm_logfile<<record_data.sensor_id<<",";
				icm_logfile<<record_data.tp.tv_sec<<","<<record_data.tp.tv_usec<<",";
				icm_logfile<<record_data.wx<<","<<record_data.wy<<","<<record_data.wz<<",";
				icm_logfile<<record_data.ax<<","<<record_data.ay<<","<<record_data.az<<",";
				icm_logfile<<record_data.mx<<","<<record_data.my<<","<<record_data.mz<<",";
				//icm_logfile<<record_data.qw<<","<<record_data.qx<<","<<record_data.qy<<","<<record_data.qz<<",";
				//icm_logfile<<record_data.roll<<","<<record_data.pitch<<","<<record_data.yaw<<",";
				icm_logfile<<record_data.temp;
				icm_logfile<<endl;
			}
		}
		
		if(!icm_record_queue.empty())
		{
			switch(icm_record_queue.front())
			{
				case NEW_FILE_EVENT:
				printf("\r\n[INFO] create new ICM log file.\r\n");
				if(icm_logfile.is_open())
					icm_logfile.close();
				icm_logfile.open(icm_filename.c_str());
				assert(!icm_logfile.fail());
			    //write headers
				if(icm_logfile.is_open())
				{
				/*icm_logfile<<"seconds.milliseconds,";
				icm_logfile<<"gx(dps),gy,gz,";
				icm_logfile<<"ax(g),ay,az,";
				icm_logfile<<"mx(uT),my,mz,";
				icm_logfile<<"temperature(degree),";
				icm_logfile<<endl;
				*/
				}
				break;
				case FILE_CLOSE_EVENT:
				if(icm_logfile.is_open())
				{
					icm_logfile.close();
				}
				printf("\r\n[INFO] close ICM log file.\r\n");
				break;
				case LOG_START_EVENT:
				printf("\r\n[INFO] ICM log start.\r\n");	
				break;
				case LOG_STOP_EVENT:
				printf("\r\n[INFO] ICM log stop.\r\n");	
				break;
				case APP_EXIT_EVENT:
		
				if(icm_logfile.is_open())
				{
					icm_logfile.close();
				}
				printf("\r\n[INFO] exit ICM record thread.\r\n");
				pthread_exit(NULL);
				break;
				default:

				break;
			}
			icm_record_queue.pop();
		}
		
		usleep(1000);
	}
}

int main()
{
	char key = 0;
	pthread_t icm_sensor_thrd,icm_record_thrd;
	pthread_attr_t icm_sensor_tattr,icm_record_tattr;
	sched_param icm_sensor_param,icm_record_param;
	
	spi_init();

#if ENABLE_INTERRUPUT
	if(wiringPiISR(ICM_INTERRUPT_PIN,INT_EDGE_RISING,&icm_handler) < 0)
	{
		printf("[ERROR] Unable to setup ICM ISR.\r\n");
	}
#endif
	if (pthread_mutex_init(&lock, NULL) != 0) { 
        printf("\n mutex init has failed\n"); 
        return 1; 
    } 
    
    sem_init(&free_mutex, 0, ICM_DATA_SAMPLES);
    sem_init(&used_mutex, 0, 0);
    icm_data_buffer_rpos = 0;
    icm_data_buffer_wpos = 0;
    icm_data_buffer = (unsigned char*) malloc(ICM_DATA_LEN*ICM_DATA_SAMPLES);
    
	pthread_attr_init(&icm_sensor_tattr);
	pthread_attr_getschedparam (&icm_sensor_tattr, &icm_sensor_param);
	icm_sensor_param.sched_priority = 99;
	pthread_attr_setschedparam (&icm_sensor_tattr, &icm_sensor_param);
	pthread_create(&icm_sensor_thrd,&icm_sensor_tattr,icm_sensor_thread,(void *)NULL);
	
	pthread_attr_init(&icm_record_tattr);
	pthread_attr_getschedparam (&icm_record_tattr, &icm_record_param);
	icm_record_param.sched_priority = 99;
	pthread_attr_setschedparam (&icm_record_tattr, &icm_record_param);
	pthread_create(&icm_record_thrd,&icm_record_tattr,icm_record_thread,(void *)NULL);
	
	sleep(1);
	pthread_mutex_lock(&lock); 
	pthread_mutex_unlock(&lock);
	while(1)
	{
		cout<<endl;
		cout<<"1: create new file for logging data"<<endl;
		cout<<"2: close and save data file"<<endl;
		cout<<"3: pause data logging"<<endl;
		cout<<"4: start data logging"<<endl;
		cout<<"5: save data file and exit the application"<<endl;
		cout<<"Input your selection:";
		key = getchar();
		if(key == '1')
		{	
			string str;
			cout<<"Input the new filename:";
				//getline(cin,filename);
			cin>>str;
			icm_filename = "icm_";
			icm_filename.append(str);
			icm_filename.append(".txt");
			icm_sensor_queue.push(NEW_FILE_EVENT);
			icm_record_queue.push(NEW_FILE_EVENT);
		}
		else if(key == '2')
		{
			icm_sensor_queue.push(FILE_CLOSE_EVENT);
			icm_record_queue.push(FILE_CLOSE_EVENT);
		}
		else if(key == '3')
		{
			icm_sensor_queue.push(LOG_START_EVENT);
			icm_record_queue.push(LOG_START_EVENT);
		}
		else if(key == '4')
		{
			icm_sensor_queue.push(LOG_STOP_EVENT);
			icm_record_queue.push(LOG_STOP_EVENT);
		}
		else if(key == '5')
		{
			icm_sensor_queue.push(APP_EXIT_EVENT);
			icm_record_queue.push(APP_EXIT_EVENT);
			pthread_join(icm_sensor_thrd, NULL);
			pthread_join(icm_record_thrd, NULL);
			pthread_mutex_destroy(&lock); 
			exit(0);
		}
	}
	
	free(icm_data_buffer);
	
	return 0;
}
		

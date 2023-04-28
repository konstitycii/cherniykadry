#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <math.h>
#include <typeinfo>
#include <iostream>
#include <pthread.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>

#include <cstring>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

extern "C" {
    #include <linux/i2c-dev.h>
    #include "i2c/smbus.h"
}

#define CLEAR(x) memset(&(x), 0, sizeof(x))//очистка
#define   WIDTH   1920                       // The width of the picture
#define   HEIGHT  1080                       // The height of the picture
#define   FMT     V4L2_PIX_FMT_SGRBG8 // формат поддерживаемый матрицей
#define mydelay usleep (1000)// минимальная возможная задержка шага 100 мкс
#define pwmmydelay usleep (1000) // задержка между шагами
#define I2C4_PATH "/dev/i2c-3"
#define I2C2_PATH "/dev/i2c-1"
#define INPUT_ADDR 0x22
#define INPUT_REG_P0 0x04
#define dev_name "/dev/video0"
cv::VideoWriter writer;
using namespace cv;
static void xioctl(int fh, int request, void *arg)//настройка параметров камеры
{
        int r;

        do {
                r = ioctl(fh, request, arg);
        } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

        if (r == -1) {
                fprintf(stderr, "error %d, %s\\n", errno, strerror(errno), request);
                exit(EXIT_FAILURE);
        }
}
// структура для получения значения из потока с плавающей запятой
typedef struct someArgs_tag {
    int id;
    const char *msg;
    double out[2];
} someArgs_t;

//структура для получения кадра
struct buffer {
        void   *start;
        int64_t length;
};



int main (void){

        /*char*                  	BUFIMAGEA = new char [WIDTH*HEIGHT];
        char*                    	BUFIMAGER = new char [WIDTH*HEIGHT];
        char*                    	BUFIMAGEG = new char [WIDTH*HEIGHT];
        char*                    	BUFIMAGEB = new char [WIDTH*HEIGHT];
        char*                    	BUFIMAGEY = new char [WIDTH*HEIGHT];*/


        v4l2_format			fmt; // формат видео
        v4l2_buffer              	buf; //
        v4l2_requestbuffers      	req; //
        v4l2_capability          	device_params; //параметры видеомодуля
        v4l2_buf_type            	type;
        fd_set                   	fds;
        timeval                  	tv;

        int				r, fd = -1;

        unsigned int			i, n_buffers;

        buffer*			buffers;

        char*				val = (char*)malloc(sizeof(char)*1*WIDTH*HEIGHT);



        using namespace cv;
        using namespace std;



	printf("BEGIN\n");

        fd = open(dev_name, O_RDWR | O_NONBLOCK, 0);//открытие камеры

        if (fd < 0) {
                perror("Cannot open device");
                exit(EXIT_FAILURE);
        }

        CLEAR(device_params);

        if (ioctl(fd, VIDIOC_QUERYCAP, &device_params) == -1)// получение параметров видеомодуля из драйвера
        {
          printf ("\"VIDIOC_QUERYCAP\" error %d, %s\n", errno, strerror(errno));
          exit(EXIT_FAILURE);
        }

        CLEAR(fmt);

//настройка формата
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = WIDTH;
        fmt.fmt.pix.height      = HEIGHT;
        fmt.fmt.pix.pixelformat = FMT;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;
        xioctl(fd, VIDIOC_S_FMT, &fmt);
        if (fmt.fmt.pix.pixelformat != FMT) {
                printf("Libv4l didn't accept ЭТОТ format. Can't proceed.\\n");
                exit(EXIT_FAILURE);
        }

        CLEAR(req);//очистка структуры запросов

        req.count = 4;//количество буферов
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        xioctl(fd, VIDIOC_REQBUFS, &req);

        buffers =(buffer*)calloc(req.count, sizeof(*buffers));//выделение памяти под буферы
        printf("buffers\n");
        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {//подготовка буферов к захвату
                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                xioctl(fd, VIDIOC_QUERYBUF, &buf);

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start = mmap(NULL, buf.length,
                              PROT_READ | PROT_WRITE, MAP_SHARED,
                              fd, buf.m.offset);
                if (MAP_FAILED == buffers[n_buffers].start) {
                        printf("MAP fail. Can't proceed.\\n");perror("mmap");
                        printf("MAP fail. Can't proceed.\\n");
                        exit(EXIT_FAILURE);
                }

        }

	printf("cycle\n");
        for (i = 0; i < n_buffers ; ++i) {// поставка буферов в очередь на захват кадров
                CLEAR(buf);
		printf("qbuf\n");
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;
                buf.index = i;
                xioctl(fd, VIDIOC_QBUF, &buf);
        }

	printf("on\n");
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        printf("V4L2_BUF_TYPE_VIDEO_CAPTURE\n");
        xioctl(fd, VIDIOC_STREAMON, &type);


        printf("endon\n");

i=0;
	Mat frame(HEIGHT, WIDTH, CV_8UC1);

	VideoWriter writer("output777.avi", VideoWriter::fourcc('H','2','6','4'), 25, Size(WIDTH, HEIGHT), true);//открытие файла для записи
	if (!writer.isOpened()) {
	         std::cerr << "Failed to open output file" << std::endl;
	         return -1;
	            }

	time_t start_time = time(nullptr);// получаем текущее время в секундах
	//while (difftime(time(nullptr), start_time) < 10.0) { //цикл записи видео в файл

		while (difftime(time(nullptr), start_time) < 10.0) { //цикл записи видео в файл

                do {
                        FD_ZERO(&fds);
                        FD_SET(fd, &fds);

                        // задаем время ожидания доступности буфера
                        tv.tv_sec = 2;
                        tv.tv_usec = 0;//100

                        // проверяем доступность буфера
                        r = select(fd + 1, &fds, NULL, NULL, &tv);//проверка доступности буфера
                } while ((r == -1 && (errno = EINTR)));
                if (r == -1) {
                        perror("select");
                        return 0;
                }
i++;
cout<<"KADR: "<< i << endl;
                CLEAR(buf);//макрос для очистки структуры buf от мусора и установки всех ее полей в 0.
                printf("dqbuf\n");
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory = V4L2_MEMORY_MMAP;//установка способа доступа к буферу

               if (ioctl(fd, VIDIOC_DQBUF, &buf)==-1){//удаление буфера из очереди
            	   perror("Error dequeuing buffer");
            	                 return 1;
               }
                   printf("memcpy\n");
                    memcpy(val, buffers[buf.index].start, sizeof(char)*WIDTH*HEIGHT);//копирование содержимого буфера, на который указывает buf.index, в память по указателю val
                    printf("memcpy proshel \n");

        printf("zapis poshla\n");

        cv::Mat frame(HEIGHT, WIDTH, CV_8UC1, val);
        cv::Mat framer;
        cv::cvtColor(frame, framer, cv::COLOR_BayerRG2BGR);
        writer.write(framer);

        if (framer.empty()) {//проверка на пустоту файла
                    cerr << "ERROR! blank frame grabbed\n";
                    break;
                }

        if (ioctl(fd, VIDIOC_QBUF, &buf)==-1){// Помещаем буфер обратно в очередь захвата
                    	   perror("Error dequeuing buffer");
                    	                 return 1;
                       }

        printf("Kadr zapisan\n\n\n\n\n\n");


	}


	CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

	        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	        xioctl(fd, VIDIOC_STREAMOFF, &type);//отключение потока видеозахвата
	        printf("xioctl\n");

	        for (i = 0; i < n_buffers; ++i){
	            munmap(buffers[i].start, buffers[i].length);//Эта строка используется для освобождения памяти, которая была выделена для буферов.
	            //Она вызывает функцию munmap для каждого буфера, которая освобождает ранее выделенную память.

	    }


	 printf("video saved\n");

                close(fd);
                delete[] buffers;
                printf("close fd\n");
        return 0;
}



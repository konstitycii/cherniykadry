#include <unistd.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <fstream>
#include <cstring> // memset()
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

#if 0 // изменить 1 на 0 если нужно подключаться к отличному от localhost компу
const char * camera_ip = "127.0.0.1"; // для тестов если сервер локально
#else
const char * camera_ip = "192.168.3.11";
#endif

const double default_fps = 25;       // должно совпадать с соответствующим значением в server.cpp
const unsigned default_port = 3425;  // порт сервера
const int default_codec  = VideoWriter::fourcc('a', 'v', 'c', '1'); // = VideoWriter::fourcc('H', '2', '6', '4');

int main() {
    int sock = socket(AF_INET, SOCK_STREAM, 0);//создание сокета
    if (sock < 0) {
        perror("socket");
        exit(1);
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(default_port);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_addr.s_addr = inet_addr(camera_ip);

    if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {//подключение к сокету
        perror("connect");
        exit(2);
    }

    int rc = 0;
    std::vector<char> buffer;//хранение данных кадров
    VideoWriter writer;
    string filename("./receivee.avi"); // имя файла, под которым будет сохранено видео
    uint32_t frames_count = 0;// количество кадров 
    int grows = 0, gcols = 0;

    // получаем кол-во кадров
    // высоту и ширину одного кадра
    // соответственно

    rc = recv(sock, (char*)&frames_count, sizeof(frames_count), 0);
    rc = recv(sock, (char*)&grows, sizeof(grows), 0);
    rc = recv(sock, (char*)&gcols, sizeof(gcols), 0);
    if (rc != sizeof(uint32_t) || frames_count == 0 || grows == 0 || gcols == 0) {
        cerr << "ERR: frames_count is empty" << endl;
        exit(3);
    }

    cout << "frames_count=" << frames_count << endl;
    cout << "grows=" << grows << endl;
    cout << "gcols=" << gcols << endl;

    while (frames_count > 0) {
        // читаем размер чанка
        uint32_t chunk_size = 0;
        rc = recv(sock, (char*)&chunk_size, sizeof(chunk_size), 0);
        if (rc == -1 || rc != sizeof(uint32_t)) {
            // cerr << "ERR: can't read chunk_size" << endl;
            break;
        }
        // выделяем память для чанка если нужно
        if (chunk_size >= buffer.size()) {
            buffer.resize(chunk_size);
        }
        // читаем чанк до конца
        int total = 0;
        while (total < chunk_size) {
            rc = recv(sock, buffer.data() + total, chunk_size - total, 0);//прием кадров в буфер
            if (rc == -1) {
                cerr << "ERR: while chunk transmission" << endl;
                break;
            }
            total += rc;//прибавляем к total полученный размер rc  
        }
        // пишем в файл
        if (!writer.isOpened()) {
            writer.open(filename, default_codec, default_fps, Size2i(gcols, grows));// открываем файл для записи
            if (!writer.isOpened()) {
                cerr << "ERR: could not open the output video file for write." << endl;
                exit(4);
            }
        }
        Mat mat(grows, gcols, CV_8UC3, buffer.data());
        writer.write(mat);//запись в файл
        frames_count -= 1;//отнимаем  1 количество кадров
    }
    close(sock);//закрываем сокет

    return 0;
}


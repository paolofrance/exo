#include <iostream>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <cstdio>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <deque>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>

using namespace std;

#define BUFLEN 1024     // Buffer Length
#define PORT 9090       // The Port on which to listen for incoming data

struct timestampstrc{
    int year, month, day, hour, minute;
    float second;
};
#define SHM_KEY 12345   // shared memory key
#define SHM_SIZE (sizeof(bool) + sizeof(timestampstrc))      // shared memory size
// function to check if the stream works
bool checkstream(const timestampstrc& current, const timestampstrc& frame) {
    if ((current.year>frame.year)||(current.month>frame.month)||(current.day>frame.day)||(current.hour>frame.hour)||(current.minute>frame.minute))
        return 0;
    else if ((current.minute==frame.minute)&&((current.second-frame.second)>0.6))
        return 0;
    else
        return 1;
}
// function to check if the object is detected continuously
bool checkdetection(std::deque<bool>& detbuf, bool current_sit)
{
    int sum = 0;
    for (int i = 0; i < detbuf.size(); i++)
    {
        sum += detbuf[i];   
    }
    if (current_sit == false)
    {
        if (sum>20)
            return 1;
        else
            return 0;
    }
    else
    {
        if (sum>1)
            return 1;
        else
            return 0;        
    }

}
int main() {
    struct sockaddr_in si_me, si_other;
    int s, i, slen = sizeof(si_other), recv_len;
    char buf[BUFLEN];
    bool detectionstatus,streamstatus, current_situation;
    char timestamp;
    timestampstrc nowtimestamp;
    string strbuf,datetime;

    std::deque<bool> detectionbuffer;
    int windowsize_detection = 30;

    int shmid;
    void *shmaddr;

    // Create shared memory segment
    shmid = shmget(SHM_KEY, SHM_SIZE, IPC_CREAT | 0666);
    if (shmid == -1) {
        perror("shmget");
        return 1;
    }

    // Attach the shared memory segment to the process's address space
    shmaddr = shmat(shmid, nullptr, 0);
    if (shmaddr == (void *)-1) {
        perror("shmat");
        return 1;
    }
    // initialize the boolean variable and the timestampstcr
    bool *detectiontosend = (bool *)shmaddr; 
    
    timestampstrc *frametimestamp = (timestampstrc *)((char *)shmaddr + sizeof(bool)); 
    //Create a UDP  socket
    if ((s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
        perror("socket");
        return 1;
    } 
    //Zero out the structures
    memset((char *)&si_me, 0, sizeof(si_me));

    si_me.sin_family = AF_INET;
    si_me.sin_port = htons(PORT);
    si_me.sin_addr.s_addr = htonl(INADDR_ANY);

    //Bind the socket to the port
    if (bind(s, (struct sockaddr *)&si_me, sizeof(si_me)) == -1) {
        perror("bind");
        close(s);
        return 1;
    }

    std::cout << "Listening for UDP packets on port " << PORT << std::endl;

    // Keep listening for data
    while (true) {
        // std::cout << "waiting to recieve..." << std::endl;
        fflush(stdout);

        //Try to recieve some data, tis is a blocking call
        if ((recv_len = recvfrom(s, buf, BUFLEN, 0, (struct sockaddr *)&si_other, (socklen_t*)&slen)) == -1) {
            perror("recvfrom");
            close(s);
            return 1;
        }
        // Print the recieved message
        // std::cout << "Received packet from " << inet_ntoa(si_other.sin_addr) << ":" << ntohs(si_other.sin_port) << std::endl;
        //date start
        strbuf = string(buf);
        std::size_t datestart = strbuf.find("Data:") + 5;
        //detection status
        std::size_t statuspos = strbuf.find("not",datestart);
        detectionstatus = !(statuspos != std::string::npos);
        //date & time
        datetime = strbuf.substr(0,26);
        //cout<<datetime<<std::endl;
        sscanf(datetime.c_str(),"%d-%d-%d %d:%d:%f",&frametimestamp->year,&frametimestamp->month,&frametimestamp->day,&frametimestamp->hour,&frametimestamp->minute,&frametimestamp->second);


        std::cout << "current detection: "<<detectionstatus<< std::endl;
        std::cout << frametimestamp->year<<"-"<<frametimestamp->month<<"-"<<frametimestamp->day<<" "<<frametimestamp->hour<<":"<<frametimestamp->minute<<":"<<frametimestamp->second<<std::endl;


        //  // Get the current time as a time_point
        // auto now = std::chrono::system_clock::now();

        // //Convert to time_t to get calendar time
        // std::time_t now_c = std::chrono::system_clock::to_time_t(now);

        // //Convert to local time format
        // std::tm* localtime = std::localtime(&now_c);

        // //Get the milliseconds part
        // auto ms = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
        // //split the parameters
        // nowtimestamp.year = localtime->tm_year+1900;
        // nowtimestamp.month = localtime->tm_mon+1;
        // nowtimestamp.day = localtime->tm_mday;
        // nowtimestamp.hour = localtime->tm_hour;
        // nowtimestamp.minute = localtime->tm_min;
        // nowtimestamp.second = localtime->tm_sec;
        // nowtimestamp.second+=ms.count();
        
        // //compare the times to check if the stream is working
        // streamstatus = checkstream(nowtimestamp,frametimestamp);
        detectionbuffer.push_back(detectionstatus);
        if (detectionbuffer.size()>windowsize_detection)
        {
            detectionbuffer.pop_front();
        }
        current_situation = checkdetection(detectionbuffer,current_situation);
        *detectiontosend = current_situation;
        std::cout<<"detection window: "<<*detectiontosend<<std::endl;
        //std::cout<<"stream status: "<<streamstatus;
        std::cout<<"---------------------------------------------------------------------------"<<std::endl;

        
    }
    // Detatch the shared memory segment
    if (shmdt(shmaddr)== -1) {
        perror("shmdt");
        return 1;
    }
    close(s);
    return 0;
}
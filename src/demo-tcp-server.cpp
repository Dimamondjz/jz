/*
Demo
变量说明
x0:状态输入(x,y,航向角,速度,里程)
control:控制输出(速度 加速度 舵轮转角)
 */
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include<iostream>
#include <chrono>
#include <string>
#include "mpcc_planner/mpcc_planner.hpp"
#define QUEUE 20
#define PORT 6667
void split(const std::string& s, std::vector<double>& sv,
           const char flag = ',') {
  sv.clear();
  std::istringstream iss(s);
  std::string temp;

  while (getline(iss, temp, flag)) {
    sv.push_back(stod(temp));
  }
  return;
}
int main(int argc, char** argv) {
  // close(STDOUT_FILENO);
  mpcc::MpccPlanner mpccPlanner;
  int init_flag=0;
  kinematic_model::VectorX x0;
  std::vector<double> state;
  //
  fd_set rfds;
  struct timeval tv;
  int retval, maxfd;  //选择器

  /*创建socket*/
  int ss =
      socket(AF_INET, SOCK_STREAM, 0);  // AF_INET   IPV4   ;SOCK_STREAM   TCP
  struct sockaddr_in server_sockaddr;
  server_sockaddr.sin_family = AF_INET;
  server_sockaddr.sin_port = htons(PORT);
  server_sockaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  /*bind*/
  if (bind(ss, (struct sockaddr*)&server_sockaddr, sizeof(server_sockaddr)) ==
      -1) {
    perror("bind");
    exit(1);
  }
  /*listen*/
  if (listen(ss, QUEUE) == -1) {
    perror("listen");
    exit(1);
  }
  /*connect*/
  struct sockaddr_in client_addr;
  socklen_t length = sizeof(client_addr);
  // 成功返回非负描述字，出错返回-1
  int conn = accept(ss, (struct sockaddr*)&client_addr,
                    &length);  //目测需要客户端部分的addr
  if (conn < 0) {
    perror("connect");
    exit(1);
  }
  while (1) {
    /*把可读文件描述符的集合清空*/
    FD_ZERO(&rfds);
    /*把标准输入的文件描述符加入到集合中*/
    FD_SET(0, &rfds);
    maxfd = 0;
    /*把当前连接的文件描述符加入到集合中*/
    FD_SET(conn, &rfds);
    /*找出文件描述符集合中最大的文件描述符*/
    if (maxfd < conn) maxfd = conn;
    /*设置超时时间*/
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    /*等待聊天*/
    retval = select(maxfd + 1, &rfds, NULL, NULL, &tv);
    std::cout << "tcp连接flag" << retval << std::endl;
    if (retval == -1) {
      std::cout << "select出错，客户端程序退出\n" << std::endl;
      break;
    } else if (retval == 0) {
      std::cout
          << "服务端没有任何输入信息，并且客户端也没有信息到来，waiting..."
          << std::endl;
      continue;
    } else {
      /*客户端发来了消息*/
      if (FD_ISSET(conn, &rfds)) {
        char buffer[1024];
        memset(buffer, 0, sizeof(buffer));
        int len = recv(conn, buffer, sizeof(buffer), 0);
        if (strcmp(buffer, "exit\n") == 0) break;
        std::cout << buffer << std::endl;
        split(buffer, state);
        /*参数解析*/
        for (int i = 0; i < state.size()-1; ++i) {
          x0.coeffRef(i) =state[i];
          std::cout<<"state:"<<state[i]<<std::endl;  
        }
        if (init_flag==0){
          mpccPlanner.delta_last_=state[state.size()-1];
          mpccPlanner.init();
          init_flag=1;
          continue;
        }
        std::vector<double> control = mpccPlanner.solveQP(x0, state[state.size()-1]);
        std::string s_control = "";
        for (int i = 0; i < 3; i++) {
          std::cout << control[i] << std::endl;
          s_control += std::to_string(control[i]);
          if (i < 2) {
            s_control += ",";
          }
        }
        const char* p = s_control.data();
        std::cout << p << std::endl;
        send(conn, p, s_control.size(), 0);
      }
      /*用户输入信息了,开始处理信息并发送*/
      // if (FD_ISSET(0, &rfds)) {
      //   char buf[1024];
      //   fgets(buf, sizeof(buf), stdin);
      //   // printf("you are send %s", buf);
      //   send(conn, buf, sizeof(buf), 0);
      // }
    }
  }
  close(conn);
  close(ss);
  return 0;
}
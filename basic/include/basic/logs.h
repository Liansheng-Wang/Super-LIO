/**
 * @file logs.h
 * @author Liansheng Wang (lswang@mail.ecust.edu.cn)
 * @version 0.1
 * @date 2023-10-06
 * @copyright Copyright (c) 2023
 */

#ifndef CUSTOM_LOG_H_
#define CUSTOM_LOG_H_

#include <ctime>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <filesystem>
#include <glog/logging.h>
#include <gflags/gflags.h>


using namespace google;

namespace BASIC {

#define RESET "\033[0m"
#define RED "\033[31m"
#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define BLUE "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN "\033[36m"
#define WHITE "\033[37m"

#define gINFO(msg) LOG(INFO) << GREEN << " ---> "<< msg << RESET
#define gWARNING(msg) LOG(WARNING) << YELLOW << " ---> "<< msg << RESET
#define gERROR(msg) LOG(ERROR) << RED << " ---> "<< msg << RESET


void init_log();

void init_log_dir(std::string log_dir);


}  // namespace LI2Sup
#endif
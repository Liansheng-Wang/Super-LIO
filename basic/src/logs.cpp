#include "basic/logs.h"

namespace BASIC {

void init_log(){
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_alsologtostderr = true;
}


void init_log_dir(std::string log_dir){
  FLAGS_logbufsecs = 0;
  FLAGS_minloglevel = google::INFO;
  FLAGS_alsologtostderr = true;
  FLAGS_log_dir = log_dir;
}

}
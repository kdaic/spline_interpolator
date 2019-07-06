#include "test/util/gnuplot_realtime.hpp"

GnuplotServer::GnuplotServer() : gp_(), mulitplot_frag_(false) {

  // gp_ << "set terminal postscript eps enhanced color\n";
  gp_ << "set terminal png\n";
  gp_.flush();

}



GnuplotServer::~GnuplotServer() {
  if(mulitplot_frag_){
    gp_ << "set nomultiplot\n";
  }
  gp_.flush();
}

GnuplotServer& GnuplotServer::set_multiplot() {
  mulitplot_frag_ = true;
  gp_ << "set multiplot\n";
  return *this;
}

GnuplotServer& GnuplotServer::set(const std::string& param) {
  gp_ << "set " << param << "\n";
  gp_.flush();
  return *this;
}

void GnuplotServer::flush() {
  gp_.flush();
}

void GnuplotServer::pause(const int& time) {
  gp_ << "pause " << time << "\n";
  gp_.flush();
}


GnuplotServer& GnuplotServer::plot(
                 const std::vector<std::pair<double, double> > &data,
                 const plot_options_t &options){
  gp_ << "plot '-' ";
  for(plot_options_t::const_iterator option(options.begin());
      option != options.end();
      ++option) {
    gp_ << option->first << " " << option->second << " ";
  }
  if(options.find("title") == options.end())  {
    gp_ << "notitle " ;
  }
  gp_ << "\n";

  for(std::vector<std::pair<double, double> >::const_iterator it(data.begin());
      it != data.end();
      ++it) {
    gp_ << it->first << " " << it->second << "\n";
  }
  gp_ << "e\n";

  return *this;
}

GnuplotServer& GnuplotServer::plot(const std::vector<std::pair<double, double> > &data) {
  return plot(data, plot_options_t());
}

GnuplotServer& GnuplotServer::plot(const container_t &data_list) {
  gp_ << "plot ";
  for(container_t::const_iterator it(data_list.begin()); true; ) {
    gp_ << "'-' ";
    for(plot_options_t::const_iterator option(it->second.begin());
        option != it->second.end();
        ++option) {
      gp_ << option->first << " " << option->second << " ";
    }
    if(it->second.find("title") == it->second.end()) {
      gp_ << "notitle " ;
    }

    if((++it) != data_list.end()) {
      gp_ << ", ";
    } else {
      break;
    }
  }
  gp_ << "\n";

  for( container_t::const_iterator it(data_list.begin()); true; ) {
    for( container_t::value_type::first_type::const_iterator it2(it->first.begin());
        it2 != it->first.end();
        ++it2){

      gp_ << it2->first << " " << it2->second << "\n";
    }
    if((++it) != data_list.end()){
      gp_ << "ee\n";
    }else{
      break;
    }
  }
  gp_ << "e\n";

  return *this;
}

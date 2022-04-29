#include <joint_filter.h>
#include <vector>
#include <cmath>

using namespace std;

Filter::Filter(double spike_M_, int spike_n_, int max_seq_len_){
  setData(spike_M_, spike_n_, max_seq_len_);
}

void Filter::setData(double spike_M_, int spike_n_, int max_seq_len_){
  spike_M = spike_M_;
  spike_n = spike_n_;
  max_seq_len = max_seq_len_;
}

double Filter::setRawSeq1(double angle){
  setRawSeq(c1, raw_seq_1, spike_filtered_seq_1, angle);
  return spike_filtered_seq_1.back();
}
double Filter::setRawSeq2(double angle){
  setRawSeq(c2, raw_seq_2, spike_filtered_seq_2, angle);
  return spike_filtered_seq_2.back();
}
double Filter::setRawSeq3(double angle){
  setRawSeq(c3, raw_seq_3, spike_filtered_seq_3, angle);
  return spike_filtered_seq_3.back();
}
double Filter::setRawSeq4(double angle){
  setRawSeq(c4, raw_seq_4, spike_filtered_seq_4, angle);
  return spike_filtered_seq_4.back();
}
double Filter::setRawSeq5(double angle){
  setRawSeq(c5, raw_seq_5, spike_filtered_seq_5, angle);
  return spike_filtered_seq_5.back();
}
double Filter::setRawSeq6(double angle){
  setRawSeq(c6, raw_seq_6, spike_filtered_seq_6, angle);
  return spike_filtered_seq_6.back();
}
double Filter::setRawSeq7(double angle){
  setRawSeq(c7, raw_seq_7, spike_filtered_seq_7, angle);
  return spike_filtered_seq_7.back();
}
double Filter::setRawSeq8(double angle){
  setRawSeq(c8, raw_seq_8, spike_filtered_seq_8, angle);
  return spike_filtered_seq_8.back();
}

void Filter::setRawSeq(int& c, vector<double> &raw_seq, vector<double> &spike_filtered_seq, double angle){
  raw_seq.push_back(angle);
  if(raw_seq.size() > max_seq_len)
    raw_seq.erase(raw_seq.begin());

  if(spike_filtered_seq.empty()){
    spike_filtered_seq.push_back(raw_seq.back());
  }else{
    if(abs(raw_seq.back()-spike_filtered_seq.back())>spike_M && c<spike_n){
      c = c+1;
      spike_filtered_seq.push_back(spike_filtered_seq.back());
    }else{
      c = 0;
      spike_filtered_seq.push_back(raw_seq.back());
    }
    if(spike_filtered_seq.size() > max_seq_len)
      spike_filtered_seq.erase(spike_filtered_seq.begin());
  }
}

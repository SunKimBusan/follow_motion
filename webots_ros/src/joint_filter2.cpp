#include "joint_filter.h"
#include <vector>
#include <cmath>

using namespace std;

class Filter {
  private:
    // M is a maximum change parameter.
    double spike_M;
    // The parameter n is the number of extreme changes in a row that will be rejected before finally accepting a large change, with possible values n=0,1,2,…
    int spike_n;
    int max_seq_len;

    int c1;
    int c2;
    int c3;
    int c4;
    int c5;
    int c6;
    int c7;
    int c8;
    vector<double> raw_seq_1;
    vector<double> raw_seq_2;
    vector<double> raw_seq_3;
    vector<double> raw_seq_4;
    vector<double> raw_seq_5;
    vector<double> raw_seq_6;
    vector<double> raw_seq_7;
    vector<double> raw_seq_8;
    vector<double> spike_filtered_seq_1;
    vector<double> spike_filtered_seq_2;
    vector<double> spike_filtered_seq_3;
    vector<double> spike_filtered_seq_4;
    vector<double> spike_filtered_seq_5;
    vector<double> spike_filtered_seq_6;
    vector<double> spike_filtered_seq_7;
    vector<double> spike_filtered_seq_8;
    //vector<double> kalman_filtered_seq_1;
    //vector<double> kalman_filtered_seq_2;
    //vector<double> kalman_filtered_seq_3;
    //vector<double> kalman_filtered_seq_4;
    //vector<double> kalman_filtered_seq_5;
    //vector<double> kalman_filtered_seq_6;
    //vector<double> kalman_filtered_seq_7;
    //vector<double> kalman_filtered_seq_8;

  public:
    Filter(double spike_M_, int spike_n_, int max_seq_len_){
      setData(spike_M_, spike_n_, max_seq_len_);
    }
    void setData(double spike_M_, int spike_n_, int max_seq_len_){
      spike_M = spike_M_;
      spike_n = spike_n_;
      max_seq_len = max_seq_len_;
    }
    double setRawSeq1(double angle){
      setRawSeq(c1, raw_seq_1, spike_filtered_seq_1);
      return spike_filtered_seq_1.back();
    }
    double setRawSeq2(double angle){
      setRawSeq(c2, raw_seq_2, spike_filtered_seq_2);
      return spike_filtered_seq_2.back();
    }
    double setRawSeq3(double angle){
      setRawSeq(c3, raw_seq_3, spike_filtered_seq_3);
      return spike_filtered_seq_3.back();
    }
    double setRawSeq4(double angle){
      setRawSeq(c4, raw_seq_4, spike_filtered_seq_4);
      return spike_filtered_seq_4.back();
    }
    double setRawSeq5(double angle){
      setRawSeq(c5, raw_seq_5, spike_filtered_seq_5);
      return spike_filtered_seq_5.back();
    }
    double setRawSeq6(double angle){
      setRawSeq(c6, raw_seq_6, spike_filtered_seq_6);
      return spike_filtered_seq_6.back();
    }
    double setRawSeq7(double angle){
      setRawSeq(c7, raw_seq_7, spike_filtered_seq_7);
      return spike_filtered_seq_7.back();
    }
    double setRawSeq8(double angle){
      setRawSeq(c8, raw_seq_8, spike_filtered_seq_8);
      return spike_filtered_seq_8.back();
    }
    void setRawSeq(int& c, vector<double>& raw_seq, vector<double>& spike_filtered_seq){
      raw_seq.insert(angle);
      if(raw_seq.size() > max_seq_len)
        raw_seq.erase(raw_seq.begin());

      if(spike_filtered_seq.empty()){
        spike_filtered_seq.insert(raw_seq.back());
      }else{
        if(abs(raw_seq.back()-spike_filtered_seq.back())>spike_M && c<spike_n){
          c = c+1;
          spike_filtered_seq.insert(spike_filtered_seq.back());
        }else{
          c = 0;
          spike_filtered_seq.insert(raw_seq.back());
        }
        if(spike_filtered_seq.size() > max_seq_len)
          spike_filtered_seq.erase(spike_filtered_seq.begin());
      }
    }
};

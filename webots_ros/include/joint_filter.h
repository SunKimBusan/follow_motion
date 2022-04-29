#include <vector>

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
    Filter(double spike_M_, int spike_n_, int max_seq_len_);
    void setData(double spike_M_, int spike_n_, int max_seq_len_);
    double setRawSeq1(double angle);
    double setRawSeq2(double angle);
    double setRawSeq3(double angle);
    double setRawSeq4(double angle);
    double setRawSeq5(double angle);
    double setRawSeq6(double angle);
    double setRawSeq7(double angle);
    double setRawSeq8(double angle);
    void setRawSeq(int& c, vector<double> &raw_seq, vector<double> &spike_filtered_seq, double angle);
};

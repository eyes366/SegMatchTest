#include <iostream>
#include <unistd.h>
#include "OpenCVInc.h"

using namespace std;
using namespace cv;

void split1(std::string s, std::string delim,std::vector< std::string >* ret)
{
    size_t last = 0;
    size_t index=s.find_first_of(delim,last);
    while (index!=std::string::npos)
    {
        ret->push_back(s.substr(last,index-last));
        last=index+1;
        index=s.find_first_of(delim,last);
    }
    if (index-last>0)
    {
        ret->push_back(s.substr(last,index-last));
    }
}

Mat compare_feature(Mat& feature0, Mat& feature1)
{
    const unsigned int f_dim = 640u;
    const unsigned int f_dim_out = 10u;
    const unsigned int bin_size = 64u;
    Mat out = Mat::zeros(1,f_dim_out,CV_32FC1);
    for (size_t i = 0; i < f_dim_out; ++i)
    {
        Mat h1 = feature0(Range::all(), Range(i*bin_size,(i+1)*bin_size));
        Mat h2 = feature1(Range::all(), Range(i*bin_size,(i+1)*bin_size));
        Scalar intersection = cv::sum(h1+h2-cv::abs(h1-h2))/2.0;
        out.at<float>(0,i) = intersection.val[0];
    }
}

Mat read_features(std::string szLog)
{
    ifstream fs;
    fs.open(szLog);
    if (!fs.is_open())
    {
        break;
    }
    string szLine;
    vector<cv::Vec<float,640> > line_numbers;
    while( getline(fs, szLine) )
    {
        cv::Vec<float,640> line_number;
        vector<string> szNumbers;
        split1(szLine, string(","), &szNumbers);
        if (szNumbers.size() < 640)
        {
            break;
        }
        for (unsigned int j = 0; j < 640; j++)
        {
            line_number[j] = atof(szNumbers[j].c_str());
        }
        line_numbers.push_back(line_number);
    }
    if (line_numbers.size() <= 0)
    {
        break;
    }
    Mat feature = Mat(line_numbers.size(), 640, CV_32FC1, line_numbers.data()).clone();
}

int main1()
{
    char szFilePath[1024] = {0};
    string szRoot("/home/xinzi/qt_project/build-VelodyneCalib-Desktop-Debug/");
    vector<Mat> features;
    for (unsigned int i = 1; i < 10000; i++)
    {
        sprintf(szFilePath, "%ssignatures%06d.csv",szRoot.c_str(),i);
        ifstream fs;
        fs.open(szFilePath);
        if (!fs.is_open())
        {
            break;
        }
        string szLine;
        vector<cv::Vec<float,640> > line_numbers;
        while( getline(fs, szLine) )
        {
            cv::Vec<float,640> line_number;
            vector<string> szNumbers;
            split1(szLine, string(","), &szNumbers);
            if (szNumbers.size() < 640)
            {
                break;
            }
            for (unsigned int j = 0; j < 640; j++)
            {
                line_number[j] = atof(szNumbers[j].c_str());
            }
            line_numbers.push_back(line_number);
        }
        if (line_numbers.size() <= 0)
        {
            break;
        }
        Mat feature = Mat(line_numbers.size(), 640, CV_32FC1, line_numbers.data()).clone();
        FileStorage fs_cv;
        char szSaveName[1024] = {0};
        sprintf(szSaveName, "Mat%06d.yml", i);
        fs_cv.open(szSaveName, FileStorage::WRITE);
        fs_cv << string("Mat") + to_string(i) << feature;
        fs_cv.release();

        features.push_back(feature);
    }

    Mat data5 = features[40];
    Mat data10 = features[9];
    Mat data_out = Mat::zeros(data5.rows, data10.rows, CV_32FC1);
    for (unsigned int i = 0; i < data5.rows; i++)
    {
        for (unsigned int j = 0; j < data10.rows; j++)
        {
            data_out.at<float>(i,j) = norm(data5.row(i), data10.row(j));
        }
    }
    cout << data_out ;

    return 0;
}

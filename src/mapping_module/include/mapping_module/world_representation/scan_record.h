#pragma once

#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>

#include <Eigen/Dense>

class Record {
 public:
  uint32_t seq;
  ros::Time stamp;
  Eigen::Isometry3d pose;
  std::string file;

  Record() {}

  Record(uint32_t _seq, ros::Time _stamp, Eigen::Isometry3d _pose,
         std::string _file) {
    seq = _seq;
    stamp = _stamp;
    pose = _pose;
    file = _file;
  }

  std::string getRecordString() {  // NOLINT
    Eigen::Quaterniond q(pose.rotation());
    std::stringstream ss;
    ss << file                            // 当前地图路径
       << "\t" << seq                     // id
       << "\t" << stamp.toNSec()          // 时间戳
       << "\t" << pose.translation().x()  //
       << "\t" << pose.translation().y()  //
       << "\t" << pose.translation().z()  //
       << "\t" << q.w()                   //
       << "\t" << q.x()                   //
       << "\t" << q.y()                   //
       << "\t" << q.z()                   //
       << std::endl;
    std::cout << "<recordToStream>: " << ss.str();
    //   std::cout << "position: " << pose.translation().transpose() <<
    //   std::endl; std::cout << "rotation: " << q.coeffs().transpose() <<
    //   std::endl;
    return ss.str();
  }
};

class RecordManager {
 public:
  RecordManager() {}

  void setPath(std::string _path) { file_path = _path; }
  std::string getPath() { return file_path; }

  void writeOneRecordToFile(Record &record) {     // NOLINT
    std::ofstream out(file_path, std::ios::app);  // 追加写入
    std::cout << "<writeOneRecordToFile>: " << std::endl;
    if (!out.fail()) {
      out << record.getRecordString();
      return;
    }
    out.close();
  }

  void writeAllRecordsToFile(std::vector<Record> &records) {  // NOLINT
    std::ofstream out(file_path);                             // 覆盖写入
    std::cout << "<writeAllRecordsToFile>: " << std::endl;

    for (int i = 0; i < records.size(); i++) {
      out << records[i].getRecordString();
    }

    if (!out.fail()) {
      return;
    }
    out.close();
  }

  void readAllRecordsFromFile(std::vector<Record> &records) {  // NOLINT
    std::ifstream fin(file_path);

    if (fin.fail()) {
      std::cout << "<readAllRecordsFromFile>: Open file failed." << std::endl;
      return;
    }

    std::stringstream istr;
    std::vector<double> tmpvec;
    std::string str_line;
    int line_count = 0;

    Record rec;
    while (std::getline(fin, str_line)) {  // 读取一行
      std::cout << "ReadLine [" << line_count++ << "]: ";
      {
        istr.str(str_line);  // inputstd::stringStream
        istr >> rec.file;
        std::cout << rec.file << std::endl;

        istr >> rec.seq;
        std::cout << "\t" << rec.seq << std::endl;

        uint64_t t;
        istr >> t;
        rec.stamp.fromNSec(t);
        std::cout << "\t" << rec.stamp.toNSec() << std::endl;
      }

      std::cout << "\t";
      double tmpf;
      while (istr >> tmpf) {
        tmpvec.push_back(tmpf);
        std::cout << tmpf << "\t";
      }
      std::cout << std::endl;
      rec.pose = Eigen::Isometry3d::Identity();
      rec.pose.rotate(Eigen::Quaterniond(tmpvec[3], tmpvec[4], tmpvec[5],
                                         tmpvec[6]));  // rotate first!!!!
      rec.pose.pretranslate(Eigen::Vector3d(tmpvec[0], tmpvec[1], tmpvec[2]));

      // std::cout << "[position]: " << rec.pose.translation().transpose()
      //           << std::endl;

      // std::cout << "[rotation]: " <<
      //           Eigen::Quaterniond(rec.pose.rotation()).coeffs().transpose()
      //           << std::endl
      //           << std::endl;

      records.push_back(rec);
      tmpvec.clear();
      istr.clear();
    }
    fin.close();
  }

  bool queryRecordWithSeq(uint32_t seq, std::vector<Record> &records,  // NOLINT
                          Record &record, uint32_t *curr_id) {         // NOLINT
    for (int i = *curr_id; i < records.size(); i++) {
      if (records[i].seq == seq) {
        record = records[i];
        *curr_id = i;
        return true;
      }
    }
    return false;
  }

 private:
  string file_path;  // NOLINT
};
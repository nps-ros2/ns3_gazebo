#include <vector>
#include <string>
#include <sstream> // strngstream
#include <fstream>
#include <locale> // tolower
#include <cassert>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

#include "setup_reader.hpp"

//https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
std::vector<std::string> _split(const std::string& str,
                                       const std::string& delim) {
  std::vector<std::string> tokens;
  size_t prev=0, pos=0;
  do {
    pos = str.find(delim, prev);
    if (pos == std::string::npos) pos = str.length();
    std::string token = str.substr(prev, pos-prev);
    if (!token.empty()) tokens.push_back(token);
    prev = pos + 1;
  }
  while (pos < str.length() && prev < str.length());
  return tokens;
}

std::vector<std::string> _range(std::string range) {

  range.erase(0,1); // remove "^r"

  std::vector<std::string> endpoints = _split(range, "-");

  unsigned int start = 0;
  unsigned int stop = 0;
  try {
    if(endpoints.size() == 1) {
      start = stoi(endpoints[0]);
      stop = start;
    } else if (endpoints.size() == 2) {
      start = stoi(endpoints[0]);
      stop = stoi(endpoints[1]);
    } else {
      std::cerr << "range error in '" << range << "'\n";
      assert(0);
    }
  } catch (const std::invalid_argument& ia) {
    std::cerr << "invalid range '" << range << "'\n";
    assert(0);
  }

  std::vector<std::string> nodes;
  for(unsigned int i=start; i<=stop; i++) {
    std::stringstream ss;
    ss << "R" << i;
    nodes.push_back(ss.str());
  }
  return nodes;
}

void _validate_header(std::string line, std::string mode) {
  if(mode == "publish") {
    if(line != "node,subscription,frequency,size,history,depth,reliability,durability") {
      std::cout << "invalid publish line: '" << line << "'" << std::endl;
      assert(0);
    }
  }
  if(mode == "subscribe") {
    if(line != "node,subscription,history,depth,reliability,durability,,") {
      std::cout << "invalid subscribe line: '" << line << "'" << std::endl;
      assert(0);
    }
  }
}

rmw_qos_profile_t _qos_profile(std::string history, std::string depth,
                             std::string reliability, std::string durability) {

  rmw_qos_profile_t profile = rmw_qos_profile_default;
  if(history == "keep_last") {
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  } else if (history == "keep_all") {
    profile.history = RMW_QOS_POLICY_HISTORY_KEEP_ALL;
  } else {
    assert(0);
  }

  depth = stoi(depth);

  if(reliability == "reliable") {
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  } else if (reliability == "best_effort") {
    profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  } else {
    assert(0);
  }

  if(durability == "transient_local") {
    profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  } else if (durability == "volatile") {
    profile.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
  } else {
    assert(0);
  }

  return profile;
}

publish_record_t::publish_record_t(const std::vector<std::string>& row) :
           robot_name(row[0]), subscription(row[1]),
           frequency(stoi(row[2])), size(stoi(row[3])), 
           history(row[4]), depth(stoi(row[5])),
                   reliability(row[6]), durability(row[7]),
           qos_profile(_qos_profile(row[4],row[5],row[6],row[7])) {
}

subscribe_record_t::subscribe_record_t(const std::vector<std::string>& row) :
           robot_name(row[0]), subscription(row[1]),
           history(row[2]), depth(stoi(row[3])),
                   reliability(row[4]), durability(row[5]),
           qos_profile(_qos_profile(row[2],row[3],row[4],row[5])) {
}

publishers_subscribers_t::publishers_subscribers_t(std::string filename,
                                                   bool verbose) :
            publishers(), subscribers() {

  // open CSV file
  std::string line;
  std::vector<std::string> row;
  std::string mode = "start";
  std::ifstream f(filename);
  if(f.fail()) {
    std::cout << "CSV file open error on file '" << filename << "'\n";
    assert(0);
  }
  while (std::getline(f, line)) {

    // line to lower case
    std::transform(line.begin(), line.end(), line.begin(), 
                   [](unsigned char c){ return std::tolower(c); });

    std::vector<std::string> row = _split(line, ",");

    // verbose
    if(verbose) {
      std::cout << "Row: '" << line << "'\n";
    }

    // no first column
    if(row.size() == 0) {
      continue;
    }

    // blank first column
    if(row[0] == "") {
      continue;
    }

//    std::cerr << "row length: " << row.size() << "\n";
//    std::cerr << "row[0]: '" << row[0] << "'\n";

    // mode publish
    if (row[0] == "publish") {
      mode = "publish";
      continue;
    }

    // mode subscribe
    if (row[0] == "subscribe") {
      mode = "subscribe";
      continue;
    }

    // row [0][0] is not R so it must be a header
    std::string a=row[0];

    if(row[0][0]!='r') {
      // validate header
      _validate_header(line, mode);
      continue;
    }

    // consume valid entry

    // range, either r<n> or r<n1-n2>
    std::vector<std::string> nodes = _range(row[0]);
    for (std::vector<std::string>::const_iterator it = nodes.begin();
            it != nodes.end(); ++it) {
      row[0] = *it;
      try {
        if (mode == "publish") {
          publishers.push_back(publish_record_t(row));
        } else if (mode == "subscribe") {
          subscribers.push_back(subscribe_record_t(row));
        }
      } catch (const std::invalid_argument& ia) {
        std::cerr << "invalid argument in mode " << mode
                  << " on line '" << line << "'\n";
        assert(0);
      }
    }
  }
}


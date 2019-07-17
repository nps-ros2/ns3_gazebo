#ifndef SETUP_READER_HPP
#define SETUP_READER_HPP

#include <vector>
#include <string>
#include <sstream> // strngstream
#include <fstream>
#include <locale> // tolower

#include "rclcpp/rclcpp.hpp"

/*
// https://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
inline std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
    std::vector<std::string>   result;
    std::string                line;
    std::getline(str,line);

    std::stringstream          lineStream(line);
    std::string                cell;

    while(std::getline(lineStream,cell, ','))
    {
        result.push_back(cell);
    }
    // This checks for a trailing comma with no data after it.
    if (!lineStream && cell.empty())
    {
        // If there was a trailing comma then add an empty element.
        result.push_back("");
    }
    return result;
}
*/

// no, use split instead:
//https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c
inline std::vector<std::string> _split(const std::string& str) {
  std::vector<std::string> tokens;
  size_t prev=0, pos=0;
  do {
    pos = str.find(',', prev);
    if (pos == std::string::npos) pos = str.length();
    std::string token = str.substr(prev, pos-prev);
    if (!token.empty()) tokens.push_back(token);
    prev = pos + 1;
  }
  while (pos < str.length() && prev < str.length());
  return tokens;
}

inline std::vector<std::string> _multiple(std::string node) {
  std::vector<std::string> entries;

  std::size_t index = node.find("-");
  if(index != std::string::npos) {
    // has range

    // maybe start,stop in node
    std::stringstream node_stream(node);
    std::string token;
    std::getline(node_stream, token, '-');
    unsigned int start = stoi(token);
    std::getline(node_stream, token, '-');
    unsigned int stop = stoi(token);

    for(unsigned int i=start; i<=stop; i++) {
      std::stringstream ss;
      ss << "R" << i;
      entries.push_back(ss.str());
    }

  } else {
    // no range
    entries.push_back(node);
  }

  return entries;
}

inline void _validate_header(std::string line, std::string mode) {
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

inline rmw_qos_profile_t _qos_profile(std::string history, std::string depth,
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

class publish_record_t {
  public:
  const std::string robot_name;
  const std::string subscription;
  const unsigned int frequency;
  const unsigned int size;

  const unsigned int history;    // keep_last|keep_all
  const unsigned int depth;      // used if using keep_last
  const std::string reliability; // reliable|best_effort
  const std::string durability;  // transient_local|volatile
  const rmw_qos_profile_t qos_profile;

  publish_record_t(const std::vector<std::string>& row) :
           robot_name(row[0]), subscription(row[1]),
           frequency(stoi(row[2])), size(stoi(row[3])), 
           history(stoi(row[4])), depth(stoi(row[5])),
                   reliability(row[6]), durability(row[7]),
           qos_profile(_qos_profile(row[4],row[5],row[6],row[7])) {
  }
};

class subscribe_record_t {
public:

  const std::string robot_name;
  const std::string subscription;

  const unsigned int history;    // keep_last|keep_all
  const unsigned int depth;      // used if using keep_last
  const std::string reliability; // reliable|best_effort
  const std::string durability;  // transient_local|volatile
  const rmw_qos_profile_t qos_profile;

  subscribe_record_t(const std::vector<std::string>& row) :
           robot_name(row[0]), subscription(row[1]),
           history(stoi(row[2])), depth(stoi(row[3])),
                   reliability(row[4]), durability(row[5]),
           qos_profile(_qos_profile(row[2],row[3],row[4],row[5])) {
  }
};

class publishers_subscribers_t {
public:
  std::vector<publish_record_t> publishers;
  std::vector<subscribe_record_t> subscribers;

  publishers_subscribers_t(std::string filename, bool verbose) :
            publishers(), subscribers() {

    // open CSV file
    std::string line;
    std::vector<std::string> row;
    std::string mode = "start";
    std::ifstream f(filename);
    while (std::getline(f, line)) {

      // line to lower case
      std::transform(line.begin(), line.end(), line.begin(), 
                     [](unsigned char c){ return std::tolower(c); });

      std::vector<std::string> row = _split(line);

      // verbose
      if(verbose) {
        std::cout << "Row: " << line;
      }

      // blank first column
      if(row[0] == "") {
        continue;
      }

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

      if(row[0][0]!='R') {
        // validate header
        _validate_header(line, mode);
        continue;
      }

      // consume valid entry
      std::vector<std::string> entries = _multiple(row[0]);
      for (std::vector<std::string>::iterator it = entries.begin();
             it != entries.end(); ++it) {
        row[0] = *it;
        if (mode == "publish") {
          publishers.push_back(publish_record_t(row));
        } else if (mode == "subscribe") {
          subscribers.push_back(subscribe_record_t(row));
        }
      }
    }
  }
};

#endif

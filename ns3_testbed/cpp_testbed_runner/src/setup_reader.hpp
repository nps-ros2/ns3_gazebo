#ifndef SETUP_READER_HPP
#define SETUP_READER_HPP

#include "rclcpp/rclcpp.hpp"

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

inline std::vector<std::string> _multiple(std::string node) {
  entries = std::vector<std::string>;

  std::size_t index = node.find("-");
  if(index != std::string::npos) {
    // has range

    // maybe start,stop in node
    std::stringstream node_stream(node);
    std::string token;
    std::getline(node_stream, token, "-");
    unsigned int start = stoi(token);
    std::getline(node_stream, token, "-");
    unsigned int stop = stoi(token);

    for(int i=start; i<=stop; i++) {
      std::stringstream ss;
      ss << "R" << i;
      entries.push_back(ss.str());
    }

  } else {
    // no range
    entries.push_back(node);
  }
}

inline void _validate_header(std::string line, std::string mode) {
  if(mode == "publish") {
    if(line != "node,subscription,frequency,size,history,depth,reliability,durability") {
      assert(0);
    }
  }
  if(mode == "subscribe") {
    if(line != "node,subscription,history,depth,reliability,durability,,") {
      assert(0);
    }
  }
}

inline rmw_qos_profile_t _qos_profile(std::string history, std::string depth,
                             std::string reliability, std::string durability) {

  rmw_qos_profile_t profile = rmw_qos_profile_default;
//  std::transform(history.begin(), history.end(), histogy.begin(), 
//                 [](unsigned char c){ return std::tolower(c); });
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

class publish_record_t {
  public:
  unsigned int node;
  std::string subscription;
  unsigned int frequency;
  unsigned int size;

  unsigned int history;    // keep_last|keep_all
  unsigned int depth;      // used if using keep_last
  std::string reliability; // reliable|best_effort
  std::string durability;  // transient_local|volatile
  rmw_qos_profile_t qos_profile;

  publish_record_t(std::string c0, std::string c1, std::string c2,
                   std::string c3, std::string c4, std::string c5,
                   std::string c6, std::string c7):
           node(stoi(c0)), subscription(c1),
           frequency(stoi(c2)), size(stoi(c3)), 
           history(stoi(c4)), depth(stoi(c5)), reliability(c6), durability(c7),
           qos_profile(_qos_profile(c4, c5, c6, c7)) {
  }
}

class subscribe_record_t {
public:

  unsigned int node;
  std::string subscription;

  unsigned int history;    // keep_last|keep_all
  unsigned int depth;      // used if using keep_last
  std::string reliability; // reliable|best_effort
  std::string durability;  // transient_local|volatile
  rmw_qos_profile_t qos_profile;

  subscribe_record_t(std::string c0, std::string c1, std::string c2,
                   std::string c3, std::string c4, std::string c5):
           history(stoi(c0)), depth(stoi(c1)), reliability(c2),
           durability(c3), qos_profile(_qos_profile(c4, c5, c6, c7)) {
  }
}

class publishers_subscribers_t {
public:
  std::vector<publish_record_t> publishers;
  std::vector<subscribe_record_t> subscribers;

  publishers_subscribers_t(std:string filename, bool verbose) :
            publishers(new std::vector<publish_record_t),
            subscribers(new std::vector<subscribe_record_t) {

    // open CSV file
    std::string line;
    std::vector<std::string> csv;
    std::string mode = "start";
    std::ifstream f(filename);
    while (std::getline(f, line)) {

      // line to lower case
      std::transform(line.begin(), line.end(), histogy.begin(), 
                     [](unsigned char c){ return std::tolower(c); });

      csv = getNextLineAndSplitIntoTokens(line);

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
      if(row[0][0]!="R":
        // validate header
        _validate_header(row, mode)
        continue

      // consume valid entry
      std::vector<std::string> entries = _multiple(row[0]);
      for (std::vector<std::string>::iterator it = entries.begin();
             it != entries.end(); ++it) {
        row[0] = entry
        if (mode == "publish") {
          publishers.push_back(publish_record_t(row))
        } else if (mode == "subscribe") {
          subscribers.push_back(subscribe_record_t(row))
        }
      }
    }
  }

  ~publishers_subscribers_t() {
    delete publishers;
    delete subscribers;
  }


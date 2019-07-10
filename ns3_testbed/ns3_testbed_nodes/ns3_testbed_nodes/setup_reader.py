import csv
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, \
     QoSReliabilityPolicy
from rclpy.qos import QoSProfile

PUBLISH_HEADER = "node,subscription,frequency,size," \
                 "history,depth,reliability,durability"
SUBSCRIBE_HEADER = "node,subscription," \
                   "history,depth,reliability,durability,,"

def _multiple(node):
    entries = list()
    if node[0] == "R" and "-" in node:
        start,stop = node[1:].split("-")
        for i in range(int(start), int(stop)+1):
            entries.append("R%d"%i)
    else:
        entries.append(node)
    return entries

def _validate_header(row, mode):
    csv_row = ",".join(row).lower()
    if mode == "publish":
        if csv_row != PUBLISH_HEADER:
            raise RuntimeError("Invalid row in publish section: %s"%csv_row)
    elif mode == "subscribe":
        if csv_row != SUBSCRIBE_HEADER:
            raise RuntimeError("Invalid row in subscribe section: %s"%csv_row)
    else:
        # treat this as a comment
        print("Table comment: %s"%csv_row)

# ref. https://github.com/ros2/demos/blob/master/topic_monitor/topic_monitor/scripts/data_publisher.py
def _qos_profile(history, depth, reliability, durability):
    # depth
    profile = QoSProfile(depth = depth)

    # history
    if history.lower() == "keep_all":
        profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL
    elif history.lower() == "keep_last":
        profile.history = QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST
    else:
        raise RuntimeError("Invalid history policy: %s"%history)

    # reliability
    if reliability.lower() == "reliable":
        profile.reliability = \
                   QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    elif reliability.lower() == "best_effort":
        profile.reliability = \
                   QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
    else:
        raise RuntimeError("Invalid reliability policy: %s"%reliability)

    # durability
    if durability.lower() == "transient_local":
        profile.durability = \
                   QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
    elif durability.lower() == "volatile":
        profile.durability = \
                   QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
    else:
        raise RuntimeError("Invalid durability policy: %s"%durability)

    return profile

class PublishRecord():
    def __init__(self, row):
        self.node = row[0]
        self.subscription = row[1]
        self.frequency = int(row[2])
        self.size = int(row[3])

        history = row[4]             # keep_last|keep_all
        depth = int(row[5])          # used if using keep_last
        reliability = row[6]         # reliable|best_effort
        durability = row[7]          # transient_local|volatile
        self.qos_profile = _qos_profile(history, depth, reliability, durability)

class SubscribeRecord():
    def __init__(self, row):
        self.node = row[0]
        self.subscription = row[1]
        history = row[2]             # keep_last|keep_all
        depth = int(row[3])          # used if using keep_last
        reliability = row[4]         # reliable|best_effort
        durability = row[5]          # transient_local|volatile
        self.qos_profile = _qos_profile(history, depth, reliability, durability)

def read_setup(filename, verbose):
    publishers = list()
    subscribers = list()

    with open(filename) as f:
        mode="start"
        reader = csv.reader(f)
        for row in reader:
            if verbose:
                print("Row: %s"%",".join(row))

            # blank first column
            if not row[0]:
                continue

            # mode publish
            if row[0].lower()=="publish":
                mode = "publish"
                continue

            # mode subscribe
            if row[0].lower()=="subscribe":
                mode = "subscribe"
                continue

            # row 0 is not R* and not GS so it must be a header
            if row[0][0]!="R" and row[0]!="GS":
                # validate header
                _validate_header(row, mode)
                continue

            # valid entry
            entries = _multiple(row[0])
            for entry in entries:
                row[0]=entry
                if mode == "publish":
                    publishers.append(PublishRecord(row))
                elif mode == "subscribe":
                    subscribers.append(SubscribeRecord(row))
                else:
                    print("invalid mode '%s' for row '%s'"%(
                                                mode, ",".join(row)))
                    raise RuntimeError("Invalid table.  Aborting")

        return publishers, subscribers


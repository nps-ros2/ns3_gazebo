import csv

# publish: "robot_name","subscription_name","frequency","size","policy"
# subscribe: "robot_name","subscription_name","policy"

class PublishRecord():
    def __init__(self, row):
        self.robot_name = row[0]
        self.subscription_name = row[1]
        self.frequency = int(row[2])
        self.size = int(row[3])
#        self.policy = row[4]

class SubscribeRecord():
    def __init__(self, row):
        self.robot_name = row[0]
        self.subscription_name = row[1]
#        self.policy = row[2]

def read_setup(filename="../csv_setup/example1.csv"):
    publishers = list()
    subscribers = list()

    with open(filename) as f:
        mode="start"
        reader = csv.reader(f)
        for row in reader:
#            print(row)

            # mode publish
            if row[0]=="publish":
                mode = "publish"
                continue

            # mode subscribe
            if row[0]=="subscribe":
                mode = "subscribe"
                continue

            # blank first column
            if not row[0]:
                continue

            # comment is not R* and not GS
            if row[0][0]!="R" and row[0]!="GS":
                print("Comment: %s"%",".join(row))
                continue

            # valid entry
            if mode == "publish":
                publishers.append(PublishRecord(row))
                continue
            if mode == "subscribe":
                subscribers.append(SubscribeRecord(row))

        return publishers, subscribers

def robot_names(publishers, subscribers):
    names = set()
    for publisher in publishers:
        names.add(publisher.robot_name)
    for subscriber in subscribers:
        names.add(subscriber.robot_name)


sub = rossubscriber('/MyTopic2');

for i=1:10
    msg = sub.receive();
    msg.Data
end
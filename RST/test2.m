rosinit('172.16.9.141');

sub = rossubscriber('/MyTopic', 'std_msgs/String', @rxcallback)
pub = rospublisher('/MyTopic', 'std_msgs/String')

msg = rosmessage(pub.MessageType);
msg.Data = 'Hello world (from MATLAB)!';

pub.publish(msg);
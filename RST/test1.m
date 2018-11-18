%roscore=rosmatlab.roscore()

%node = rosmatlab.node('MyNode', '10.0.0.76')
% publisher = node.addPublisher('MyTopic', 'std_msgs/String');
% subscriber = node.addSubscriber('MyTopic', 'std_msgs/String', 1);
% msg = node.newMessage('std_msgs/String');
% msg.setData('hello world!');
% subscriber.setOnNewMessageListeners(@function3)
% publisher.publish(msg)

msg = rosmessage('std_msgs/String');
msg.Data = 'hello world!';

% pub = rospublisher('/MyTopic');
% pub.send(msg);


rospublisher('/MyTopic', msg);



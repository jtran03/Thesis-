function myFunction(data)
    
    pub = rospublisher('/zlac8015d/rpm','std_msgs/Float32MultiArray');
    msg = rosmessage(pub);
    msg.Data = [data(1,1), data(2,1)];
    send(pub,msg);
    
end 
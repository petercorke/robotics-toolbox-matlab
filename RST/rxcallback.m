function rxcallback(src, msg)
    disp([char(msg.Data()),sprintf('\n  Message received: %s',datestr(now))]);

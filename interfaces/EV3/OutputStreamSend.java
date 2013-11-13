import java.io.*;

public class OutputStreamSend {
  public static byte[] buffer = null;
  public static int counter = 0;

  public OutputStreamSend(int length) {
    buffer = new byte[length];
    clear();
  }

  public OutputStreamSend() {
    this(128);
  }
  
  public void addtoBuffer(char v) {
      buffer[counter] = (byte)v;
      counter++;
  }
  
  public void addtoBufferN(char[] v, int len) {
        for(int i=0; i<len; i++)
        {
            buffer[counter] = (byte)v[i];
            counter++;
        }
  }
  
  public void display() {
      for(int i=0;i<counter;i++)
      {
        System.out.print(buffer[i]);
      }
  }
  
  public void clear() {
       for(int i=0;i<counter;i++)
      {
        buffer[i] = 0;
      }
      counter = 0;
  }

  public void send(OutputStream output) throws IOException {
      try {
        output.write(buffer,0,counter);
		output.flush();
        this.clear();
            
      } catch (Exception e) {}
  }

}
